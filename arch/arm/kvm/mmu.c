/*
 * Copyright (C) 2012 - Virtual Open Systems and Columbia University
 * Author: Christoffer Dall <c.dall@virtualopensystems.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/mman.h>
#include <linux/kvm_host.h>
#include <linux/io.h>
#include <asm/idmap.h>
#include <asm/pgalloc.h>
#include <asm/kvm_arm.h>
#include <asm/kvm_mmu.h>
#include <asm/kvm_asm.h>
#include <asm/mach/map.h>

static DEFINE_MUTEX(kvm_hyp_pgd_mutex);

static int mmu_topup_memory_cache(struct kvm_mmu_memory_cache *cache,
				  int min, int max)
{
	void *page;

	BUG_ON(max > KVM_NR_MEM_OBJS);
	if (cache->nobjs >= min)
		return 0;
	while (cache->nobjs < max) {
		page = (void *)__get_free_page(PGALLOC_GFP);
		if (!page)
			return -ENOMEM;
		cache->objects[cache->nobjs++] = page;
	}
	return 0;
}

static void mmu_free_memory_cache(struct kvm_mmu_memory_cache *mc)
{
	while (mc->nobjs)
		free_page((unsigned long)mc->objects[--mc->nobjs]);
}

static void *mmu_memory_cache_alloc(struct kvm_mmu_memory_cache *mc)
{
	void *p;

	BUG_ON(!mc || !mc->nobjs);
	p = mc->objects[--mc->nobjs];
	return p;
}

static void free_ptes(pmd_t *pmd, unsigned long addr)
{
	pte_t *pte;
	unsigned int i;

	for (i = 0; i < PTRS_PER_PMD; i++, addr += PMD_SIZE) {
		if (!pmd_none(*pmd) && pmd_table(*pmd)) {
			pte = pte_offset_kernel(pmd, addr);
			pte_free_kernel(NULL, pte);
		}
		pmd++;
	}
}

/**
 * free_hyp_pmds - free a Hyp-mode level-2 tables and child level-3 tables
 *
 * Assumes this is a page table used strictly in Hyp-mode and therefore contains
 * only mappings in the kernel memory area, which is above PAGE_OFFSET.
 */
void free_hyp_pmds(void)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	unsigned long addr;

	mutex_lock(&kvm_hyp_pgd_mutex);
	for (addr = PAGE_OFFSET; addr != 0; addr += PGDIR_SIZE) {
		pgd = hyp_pgd + pgd_index(addr);
		pud = pud_offset(pgd, addr);

		if (pud_none(*pud))
			continue;
		BUG_ON(pud_bad(*pud));

		pmd = pmd_offset(pud, addr);
		free_ptes(pmd, addr);
		pmd_free(NULL, pmd);
		pud_clear(pud);
	}
	mutex_unlock(&kvm_hyp_pgd_mutex);
}

/*
 * Create a HYP pte mapping.
 *
 * If pfn_base is NULL, we map kernel pages into HYP with the virtual
 * address. Otherwise, this is considered an I/O mapping and we map
 * the physical region starting at *pfn_base to [start, end[.
 */
static void create_hyp_pte_mappings(pmd_t *pmd, unsigned long start,
				    unsigned long end, unsigned long *pfn_base)
{
	pte_t *pte;
	unsigned long addr;
	pgprot_t prot;

	if (pfn_base)
		prot = __pgprot(get_mem_type_prot_pte(MT_DEVICE) | L_PTE_USER);
	else
		prot = PAGE_HYP;

	for (addr = start & PAGE_MASK; addr < end; addr += PAGE_SIZE) {
		pte = pte_offset_kernel(pmd, addr);
		if (pfn_base) {
			BUG_ON(pfn_valid(*pfn_base));
			set_pte_ext(pte, pfn_pte(*pfn_base, prot), 0);
			(*pfn_base)++;
		} else {
			struct page *page;
			BUG_ON(!virt_addr_valid(addr));
			page = virt_to_page(addr);
			set_pte_ext(pte, mk_pte(page, prot), 0);
		}

	}
}

static int create_hyp_pmd_mappings(pud_t *pud, unsigned long start,
				   unsigned long end, unsigned long *pfn_base)
{
	pmd_t *pmd;
	pte_t *pte;
	unsigned long addr, next;

	for (addr = start; addr < end; addr = next) {
		pmd = pmd_offset(pud, addr);

		BUG_ON(pmd_sect(*pmd));

		if (pmd_none(*pmd)) {
			pte = pte_alloc_one_kernel(NULL, addr);
			if (!pte) {
				kvm_err("Cannot allocate Hyp pte\n");
				return -ENOMEM;
			}
			pmd_populate_kernel(NULL, pmd, pte);
		}

		next = pmd_addr_end(addr, end);
		create_hyp_pte_mappings(pmd, addr, next, pfn_base);
	}

	return 0;
}

static int __create_hyp_mappings(void *from, void *to, unsigned long *pfn_base)
{
	unsigned long start = (unsigned long)from;
	unsigned long end = (unsigned long)to;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	unsigned long addr, next;
	int err = 0;

	BUG_ON(start > end);
	if (start < PAGE_OFFSET)
		return -EINVAL;

	mutex_lock(&kvm_hyp_pgd_mutex);
	for (addr = start; addr < end; addr = next) {
		pgd = hyp_pgd + pgd_index(addr);
		pud = pud_offset(pgd, addr);

		if (pud_none_or_clear_bad(pud)) {
			pmd = pmd_alloc_one(NULL, addr);
			if (!pmd) {
				kvm_err("Cannot allocate Hyp pmd\n");
				err = -ENOMEM;
				goto out;
			}
			pud_populate(NULL, pud, pmd);
		}

		next = pgd_addr_end(addr, end);
		err = create_hyp_pmd_mappings(pud, addr, next, pfn_base);
		if (err)
			goto out;
	}
out:
	mutex_unlock(&kvm_hyp_pgd_mutex);
	return err;
}

/**
 * create_hyp_mappings - map a kernel virtual address range in Hyp mode
 * @from:	The virtual kernel start address of the range
 * @to:		The virtual kernel end address of the range (exclusive)
 *
 * The same virtual address as the kernel virtual address is also used in
 * Hyp-mode mapping to the same underlying physical pages.
 *
 * Note: Wrapping around zero in the "to" address is not supported.
 */
int create_hyp_mappings(void *from, void *to)
{
	return __create_hyp_mappings(from, to, NULL);
}

/**
 * create_hyp_io_mappings - map a physical IO range in Hyp mode
 * @from:	The virtual HYP start address of the range
 * @to:		The virtual HYP end address of the range (exclusive)
 * @addr:	The physical start address which gets mapped
 */
int create_hyp_io_mappings(void *from, void *to, phys_addr_t addr)
{
	unsigned long pfn = __phys_to_pfn(addr);
	return __create_hyp_mappings(from, to, &pfn);
}

/**
 * kvm_alloc_stage2_pgd - allocate level-1 table for stage-2 translation.
 * @kvm:	The KVM struct pointer for the VM.
 *
 * Allocates the 1st level table only of size defined by PGD2_ORDER (can
 * support either full 40-bit input addresses or limited to 32-bit input
 * addresses). Clears the allocated pages.
 *
 * Note we don't need locking here as this is only called when the VM is
 * created, which can only be done once.
 */
int kvm_alloc_stage2_pgd(struct kvm *kvm)
{
	pgd_t *pgd;

	if (kvm->arch.pgd != NULL) {
		kvm_err("kvm_arch already initialized?\n");
		return -EINVAL;
	}

	pgd = (pgd_t *)__get_free_pages(GFP_KERNEL, PGD2_ORDER);
	if (!pgd)
		return -ENOMEM;

	memset(pgd, 0, PTRS_PER_PGD2 * sizeof(pgd_t));
	kvm->arch.pgd = pgd;

	return 0;
}

static void free_guest_pages(pte_t *pte, unsigned long addr)
{
	unsigned int i;
	struct page *page, *pte_page;

	pte_page = virt_to_page(pte);

	for (i = 0; i < PTRS_PER_PTE; i++) {
		if (pte_present(*pte)) {
			unsigned long pfn = pte_pfn(*pte);

			if (pfn_valid(pfn)) { /* Skip over device memory */
				page = pfn_to_page(pfn);
				put_page(page);
			}
			put_page(pte_page);
		}
		pte++;
	}
}

static void free_stage2_ptes(pmd_t *pmd, unsigned long addr)
{
	unsigned int i;
	pte_t *pte;
	struct page *page, *pmd_page;

	pmd_page = virt_to_page(pmd);

	for (i = 0; i < PTRS_PER_PMD; i++, addr += PMD_SIZE) {
		BUG_ON(pmd_sect(*pmd));
		if (!pmd_none(*pmd) && pmd_table(*pmd)) {
			pte = pte_offset_kernel(pmd, addr);
			free_guest_pages(pte, addr);
			page = virt_to_page((void *)pte);
			WARN_ON(page_count(page) != 1);
			pte_free_kernel(NULL, pte);

			put_page(pmd_page);
		}
		pmd++;
	}
}

/**
 * kvm_free_stage2_pgd - free all stage-2 tables
 * @kvm:	The KVM struct pointer for the VM.
 *
 * Walks the level-1 page table pointed to by kvm->arch.pgd and frees all
 * underlying level-2 and level-3 tables before freeing the actual level-1 table
 * and setting the struct pointer to NULL.
 *
 * Note we don't need locking here as this is only called when the VM is
 * destroyed, which can only be done once.
 */
void kvm_free_stage2_pgd(struct kvm *kvm)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	unsigned long long i, addr;
	struct page *page, *pud_page;

	if (kvm->arch.pgd == NULL)
		return;

	/*
	 * We do this slightly different than other places, since we need more
	 * than 32 bits and for instance pgd_addr_end converts to unsigned long.
	 */
	addr = 0;
	for (i = 0; i < PTRS_PER_PGD2; i++) {
		addr = i * (unsigned long long)PGDIR_SIZE;
		pgd = kvm->arch.pgd + i;
		pud = pud_offset(pgd, addr);
		pud_page = virt_to_page(pud);

		if (pud_none(*pud))
			continue;

		BUG_ON(pud_bad(*pud));

		pmd = pmd_offset(pud, addr);
		free_stage2_ptes(pmd, addr);
		page = virt_to_page((void *)pmd);
		WARN_ON(page_count(page) != 1);
		pmd_free(NULL, pmd);
		put_page(pud_page);
	}

	WARN_ON(page_count(pud_page) != 1);
	free_pages((unsigned long)kvm->arch.pgd, PGD2_ORDER);
	kvm->arch.pgd = NULL;
}

/*
 * Clear a stage-2 PTE, lowering the various ref-counts. Also takes
 * care of invalidating the TLBs.  Must be called while holding
 * pgd_lock, otherwise another faulting VCPU may come in and mess
 * things behind our back.
 */
static void stage2_clear_pte(struct kvm *kvm, phys_addr_t addr)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;
	struct page *page;

	kvm_debug("Clearing PTE&%08llx\n", addr);
	pgd = kvm->arch.pgd + pgd_index(addr);
	pud = pud_offset(pgd, addr);
	BUG_ON(pud_none(*pud));

	pmd = pmd_offset(pud, addr);
	BUG_ON(pmd_none(*pmd));

	pte = pte_offset_kernel(pmd, addr);
	set_pte_ext(pte, __pte(0), 0);

	page = virt_to_page(pte);
	put_page(page);
	if (page_count(page) != 1) {
		__kvm_tlb_flush_vmid(kvm);
		return;
	}

	/* Need to remove pte page */
	pmd_clear(pmd);
	__kvm_tlb_flush_vmid(kvm);
	pte_free_kernel(NULL, (pte_t *)((unsigned long)pte & PAGE_MASK));

	page = virt_to_page(pmd);
	put_page(page);
	if (page_count(page) != 1)
		return;

	/*
	 * Need to remove pmd page. This is the worst case, and we end
	 * up invalidating the TLB twice. No big deal.
	 */
	pud_clear(pud);
	__kvm_tlb_flush_vmid(kvm);
	pmd_free(NULL, (pmd_t *)((unsigned long)pmd & PAGE_MASK));

	page = virt_to_page(pud);
	put_page(page);
}

static void stage2_set_pte(struct kvm *kvm, struct kvm_mmu_memory_cache *cache,
			   phys_addr_t addr, const pte_t *new_pte)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	/* Create 2nd stage page table mapping - Level 1 */
	pgd = kvm->arch.pgd + pgd_index(addr);
	pud = pud_offset(pgd, addr);
	if (pud_none(*pud)) {
		if (!cache)
			return; /* ignore calls from kvm_set_spte_hva */
		pmd = mmu_memory_cache_alloc(cache);
		pud_populate(NULL, pud, pmd);
		pmd += pmd_index(addr);
		get_page(virt_to_page(pud));
	} else
		pmd = pmd_offset(pud, addr);

	/* Create 2nd stage page table mapping - Level 2 */
	if (pmd_none(*pmd)) {
		if (!cache)
			return; /* ignore calls from kvm_set_spte_hva */
		pte = mmu_memory_cache_alloc(cache);
		clean_pte_table(pte);
		pmd_populate_kernel(NULL, pmd, pte);
		pte += pte_index(addr);
		get_page(virt_to_page(pmd));
	} else
		pte = pte_offset_kernel(pmd, addr);

	/* Create 2nd stage page table mapping - Level 3 */
	BUG_ON(pte_none(pte));
	set_pte_ext(pte, *new_pte, 0);
	get_page(virt_to_page(pte));
}

/**
 * kvm_phys_addr_ioremap - map a device range to guest IPA
 *
 * @kvm:	The KVM pointer
 * @guest_ipa:	The IPA at which to insert the mapping
 * @pa:		The physical address of the device
 * @size:	The size of the mapping
 */
int kvm_phys_addr_ioremap(struct kvm *kvm, phys_addr_t guest_ipa,
			  phys_addr_t pa, unsigned long size)
{
	phys_addr_t addr, end;
	pgprot_t prot;
	int ret = 0;
	unsigned long pfn;
	struct kvm_mmu_memory_cache cache = { 0, };

	end = (guest_ipa + size + PAGE_SIZE - 1) & PAGE_MASK;
	prot = __pgprot(get_mem_type_prot_pte(MT_DEVICE) | L_PTE_USER |
			L_PTE2_READ | L_PTE2_WRITE);
	pfn = __phys_to_pfn(pa);

	for (addr = guest_ipa; addr < end; addr += PAGE_SIZE) {
		pte_t pte = pfn_pte(pfn, prot);

		ret = mmu_topup_memory_cache(&cache, 2, 2);
		if (ret)
			goto out;
		spin_lock(&kvm->arch.pgd_lock);
		stage2_set_pte(kvm, &cache, addr, &pte);
		spin_unlock(&kvm->arch.pgd_lock);

		pfn++;
	}

out:
	mmu_free_memory_cache(&cache);
	return ret;
}

int kvm_handle_guest_abort(struct kvm_vcpu *vcpu, struct kvm_run *run)
{
	return -EINVAL;
}

static bool hva_to_gpa(struct kvm *kvm, unsigned long hva, gpa_t *gpa)
{
	struct kvm_memslots *slots;
	struct kvm_memory_slot *memslot;
	bool found = false;

	slots = kvm_memslots(kvm);

	/* we only care about the pages that the guest sees */
	kvm_for_each_memslot(memslot, slots) {
		unsigned long start = memslot->userspace_addr;
		unsigned long end;

		end = start + (memslot->npages << PAGE_SHIFT);
		if (hva >= start && hva < end) {
			gpa_t gpa_offset = hva - start;
			*gpa = (memslot->base_gfn << PAGE_SHIFT) + gpa_offset;
			found = true;
			/* no overlapping memslots allowed: break */
			break;
		}
	}

	return found;
}

int kvm_unmap_hva(struct kvm *kvm, unsigned long hva)
{
	bool found;
	gpa_t gpa;

	if (!kvm->arch.pgd)
		return 0;

	found = hva_to_gpa(kvm, hva, &gpa);
	if (found) {
		spin_lock(&kvm->arch.pgd_lock);
		stage2_clear_pte(kvm, gpa);
		spin_unlock(&kvm->arch.pgd_lock);
	}
	return 0;
}

int kvm_unmap_hva_range(struct kvm *kvm,
			unsigned long start, unsigned long end)
{
	unsigned long addr;
	int ret;

	BUG_ON((start | end) & (~PAGE_MASK));

	for (addr = start; addr < end; addr += PAGE_SIZE) {
		ret = kvm_unmap_hva(kvm, addr);
		if (ret)
			return ret;
	}

	return 0;
}

void kvm_set_spte_hva(struct kvm *kvm, unsigned long hva, pte_t pte)
{
	gpa_t gpa;
	bool found;

	if (!kvm->arch.pgd)
		return;

	found = hva_to_gpa(kvm, hva, &gpa);
	if (found) {
		spin_lock(&kvm->arch.pgd_lock);
		stage2_set_pte(kvm, NULL, gpa, &pte);
		spin_unlock(&kvm->arch.pgd_lock);
		__kvm_tlb_flush_vmid(kvm);
	}
}

void kvm_mmu_free_memory_caches(struct kvm_vcpu *vcpu)
{
	mmu_free_memory_cache(&vcpu->arch.mmu_page_cache);
}
