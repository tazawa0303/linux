/*
 * arch/arm/kernel/kgdb.c
 *
 * ARM KGDB support
 *
 * Copyright (c) 2002-2004 MontaVista Software, Inc
 * Copyright (c) 2008 Wind River Systems, Inc.
 * Copyright (c) 2013 Texas Instruments.
 *
 * Authors:  George Davis <davis_g@mvista.com>
 *           Deepak Saxena <dsaxena@plexity.net>
 *           Vincent Stehlé <v-stehle@ti.com> [transposed single step method from sh]
 */
#include <linux/irq.h>
#include <linux/kdebug.h>
#include <linux/kgdb.h>
#include <linux/io.h>
#include <asm/cacheflush.h>
#include <asm/traps.h>

/* Calculate if opcode condition is true */
static bool cond_is_true(unsigned cond, const struct pt_regs *linux_regs)
{
	u32 cpsr = linux_regs->ARM_cpsr;
	int n = (cpsr >> 31) & 1;
	int z = (cpsr >> 30) & 1;
	int c = (cpsr >> 29) & 1;
	int v = (cpsr >> 28) & 1;

	/* TODO! Coding style */
	switch (cond) {
	/* EQ */ case 0: return (z == 1);
	/* NE */ case 1: return (z == 0);
	/* CS */ case 2: return (c == 1);
	/* CC */ case 3: return (c == 0);
	/* MI */ case 4: return (n == 1);
	/* PL */ case 5: return (n == 0);
	/* VS */ case 6: return (v == 1);
	/* VC */ case 7: return (v == 0);
	/* HI */ case 8: return ((c == 1) && (z == 0));
	/* LS */ case 9: return ((c == 0) || (z == 1));
	/* GE */ case 10: return (n == v);
	/* LT */ case 11: return (n != v);
	/* GT */ case 12: return ((z == 0) && (n == v));
	/* LE */ case 13: return ((z == 1) || (n != v));
	/* None */ case 14: default: return true;
	}
}

/* Get Nth 32b word on the stack (following increasing addresses, starting at zero) */
static u32 get_word_on_stack(unsigned n, const struct pt_regs *linux_regs)
{
	u32 sp = linux_regs->ARM_sp;
	return __raw_readl((u32 *)sp + n);
}

/* Compute how many bits are set in a 16b word */
static unsigned num_bits_set(u16 w)
{
	int i;
	unsigned r = 0;

	for (i = 0; i < 16; i++)
		if (w & (1 << i))
			r++;

	return r;
}

/* Macros for single step instruction identification */
#define OPCODE_B(op)		(((op) & 0x0f000000) == 0x0a000000)
#define OPCODE_BL(op)		(((op) & 0x0f000000) == 0x0b000000)
#define OPCODE_IMM24_SIGNED(op)	(long)(((op) & 0x00ffffff) | (((op) & (1 << 23)) ? 0xff000000 : 0))
#define OPCODE_B_DISP(op)	(OPCODE_IMM24_SIGNED(op) << 2)
#define OPCODE_BLX_IMM(op)	(((op) & 0xfe000000) == 0xfa000000)
#define OPCODE_BLX_REG(op)	(((op) & 0x0ff000f0) == 0x01200030)
#define OPCODE_BX_BLX_RM(op)	((op) & 0xf)
#define OPCODE_BX(op)		(((op) & 0x0ff000f0) == 0x01200010)
#define ALIGN4(pc)		((pc) & 0xfffffffc)
#define OPCODE_POP(op)		(((op) & 0x0fff0000) == 0x08bd0000)
#define OPCODE_REG_LIST(op)	((op) & 0x0000ffff)
#define REG_LIST_PC		0x8000
#define OPCODE_POP_PC(op)	(((op) & 0x0fffffff) == 0x049df004)

/* Calculate the new address for after a step */
/* TODO! Handle thumb, too */
#ifdef CONFIG_THUMB2_KERNEL
# warn kgdb single stepping may not work properly with thumb2 kernel
#endif
static unsigned long get_step_address(const struct pt_regs *linux_regs)
{
	u32 pc = linux_regs->ARM_pc;
	u32 op = __raw_readl((void *)pc);
	unsigned cond = (op >> 28) & 0xf;
	/* By default, assume the instruction continues just after */
	unsigned long addr = pc + 4;

	/* B, BL */
	if (OPCODE_B(op) || OPCODE_BL(op)) {
		if (cond_is_true(cond, linux_regs)) {
			addr = pc + 8 + OPCODE_B_DISP(op);
		}

	/* BLX (imm) */
	} else if (OPCODE_BLX_IMM(op)) {
		addr = pc + 8 + OPCODE_B_DISP(op);

	/* BX, BLX (reg) */
	} else if (OPCODE_BX(op) || OPCODE_BLX_REG(op)) {
		if (cond_is_true(cond, linux_regs)) {
			addr = linux_regs->uregs[OPCODE_BX_BLX_RM(op)];
		}

	/* POP */
	} else if (OPCODE_POP(op)) {
		if (cond_is_true(cond, linux_regs)) {
			u16 reg_list = OPCODE_REG_LIST(op);

			if (reg_list & REG_LIST_PC) {
				unsigned n = num_bits_set(reg_list) - 1;
				addr = get_word_on_stack(n, linux_regs);
			}
		}

	/* POP pc */
	} else if (OPCODE_POP_PC(op)) {
		if (cond_is_true(cond, linux_regs)) {
			addr = get_word_on_stack(0, linux_regs);
		}
	}
	/* TODO! Handle <op> R15, unpredictables, endianness, etc... */

	addr = ALIGN4(addr);
	return addr;
}

/*
 * Replace the instruction immediately after the current instruction
 * (i.e. next in the expected flow of control) with a trap instruction,
 * so that returning will cause only a single instruction to be executed.
 */
static unsigned long stepped_address;
static u32 stepped_opcode;

/* Replace a kernel instruction */
static u32 poke_insn(u32 insn, unsigned long addr)
{
	u32 replaced_insn = __raw_readl((void *)addr);
	__raw_writel(insn, (void *)addr);

	/* Flush dcache */
	if (current->mm && current->mm->mmap_cache) {
		flush_cache_range(current->mm->mmap_cache,
				  addr, addr + sizeof(u32));
	}

	/* Flush icache */
	flush_icache_range(addr, addr + sizeof(u32));
	return replaced_insn;
}

static void do_single_step(struct pt_regs *linux_regs)
{
	/* Determine where the target instruction will send us to */
	unsigned long addr = get_step_address(linux_regs);

	stepped_address = addr;

	/* Replace it */
	/* TODO! h/w break? */
	/* TODO! handle 16/32b opcode size? */
	stepped_opcode = poke_insn(KGDB_BREAKINST, addr);
}

/* Undo a single step */
static void undo_single_step(struct pt_regs *linux_regs)
{
	/* If we have stepped, put back the old instruction */
	/* Use stepped_address in case we stopped elsewhere */
	if (stepped_opcode != 0)
		poke_insn(stepped_opcode, stepped_address);

	stepped_opcode = 0;
}

struct dbg_reg_def_t dbg_reg_def[DBG_MAX_REG_NUM] =
{
	{ "r0", 4, offsetof(struct pt_regs, ARM_r0)},
	{ "r1", 4, offsetof(struct pt_regs, ARM_r1)},
	{ "r2", 4, offsetof(struct pt_regs, ARM_r2)},
	{ "r3", 4, offsetof(struct pt_regs, ARM_r3)},
	{ "r4", 4, offsetof(struct pt_regs, ARM_r4)},
	{ "r5", 4, offsetof(struct pt_regs, ARM_r5)},
	{ "r6", 4, offsetof(struct pt_regs, ARM_r6)},
	{ "r7", 4, offsetof(struct pt_regs, ARM_r7)},
	{ "r8", 4, offsetof(struct pt_regs, ARM_r8)},
	{ "r9", 4, offsetof(struct pt_regs, ARM_r9)},
	{ "r10", 4, offsetof(struct pt_regs, ARM_r10)},
	{ "fp", 4, offsetof(struct pt_regs, ARM_fp)},
	{ "ip", 4, offsetof(struct pt_regs, ARM_ip)},
	{ "sp", 4, offsetof(struct pt_regs, ARM_sp)},
	{ "lr", 4, offsetof(struct pt_regs, ARM_lr)},
	{ "pc", 4, offsetof(struct pt_regs, ARM_pc)},
	{ "f0", 12, -1 },
	{ "f1", 12, -1 },
	{ "f2", 12, -1 },
	{ "f3", 12, -1 },
	{ "f4", 12, -1 },
	{ "f5", 12, -1 },
	{ "f6", 12, -1 },
	{ "f7", 12, -1 },
	{ "fps", 4, -1 },
	{ "cpsr", 4, offsetof(struct pt_regs, ARM_cpsr)},
};

char *dbg_get_reg(int regno, void *mem, struct pt_regs *regs)
{
	if (regno >= DBG_MAX_REG_NUM || regno < 0)
		return NULL;

	if (dbg_reg_def[regno].offset != -1)
		memcpy(mem, (void *)regs + dbg_reg_def[regno].offset,
		       dbg_reg_def[regno].size);
	else
		memset(mem, 0, dbg_reg_def[regno].size);
	return dbg_reg_def[regno].name;
}

int dbg_set_reg(int regno, void *mem, struct pt_regs *regs)
{
	if (regno >= DBG_MAX_REG_NUM || regno < 0)
		return -EINVAL;

	if (dbg_reg_def[regno].offset != -1)
		memcpy((void *)regs + dbg_reg_def[regno].offset, mem,
		       dbg_reg_def[regno].size);
	return 0;
}

void
sleeping_thread_to_gdb_regs(unsigned long *gdb_regs, struct task_struct *task)
{
	struct pt_regs *thread_regs;
	int regno;

	/* Just making sure... */
	if (task == NULL)
		return;

	/* Initialize to zero */
	for (regno = 0; regno < GDB_MAX_REGS; regno++)
		gdb_regs[regno] = 0;

	/* Otherwise, we have only some registers from switch_to() */
	thread_regs		= task_pt_regs(task);
	gdb_regs[_R0]		= thread_regs->ARM_r0;
	gdb_regs[_R1]		= thread_regs->ARM_r1;
	gdb_regs[_R2]		= thread_regs->ARM_r2;
	gdb_regs[_R3]		= thread_regs->ARM_r3;
	gdb_regs[_R4]		= thread_regs->ARM_r4;
	gdb_regs[_R5]		= thread_regs->ARM_r5;
	gdb_regs[_R6]		= thread_regs->ARM_r6;
	gdb_regs[_R7]		= thread_regs->ARM_r7;
	gdb_regs[_R8]		= thread_regs->ARM_r8;
	gdb_regs[_R9]		= thread_regs->ARM_r9;
	gdb_regs[_R10]		= thread_regs->ARM_r10;
	gdb_regs[_FP]		= thread_regs->ARM_fp;
	gdb_regs[_IP]		= thread_regs->ARM_ip;
	gdb_regs[_SPT]		= thread_regs->ARM_sp;
	gdb_regs[_LR]		= thread_regs->ARM_lr;
	gdb_regs[_PC]		= thread_regs->ARM_pc;
	gdb_regs[_CPSR]		= thread_regs->ARM_cpsr;
}

void kgdb_arch_set_pc(struct pt_regs *regs, unsigned long pc)
{
	regs->ARM_pc = pc;
}

static int compiled_break;

int kgdb_arch_handle_exception(int exception_vector, int signo,
			       int err_code, char *remcom_in_buffer,
			       char *remcom_out_buffer,
			       struct pt_regs *linux_regs)
{
	unsigned long addr;
	char *ptr;

	/* Undo any stepping we may have done */
	undo_single_step(linux_regs);

	switch (remcom_in_buffer[0]) {
	case 'c':
	case 's':
		/*
		 * Try to read optional parameter, pc unchanged if no parm.
		 * If this was a compiled breakpoint, we need to move
		 * to the next instruction or we will just breakpoint
		 * over and over again.
		 */
		ptr = &remcom_in_buffer[1];
		if (kgdb_hex2long(&ptr, &addr))
			linux_regs->ARM_pc = addr;
		else if (compiled_break == 1)
			linux_regs->ARM_pc += 4;

		compiled_break = 0;

	case 'D':
	case 'k':
		atomic_set(&kgdb_cpu_doing_single_step, -1);

		/* single step */
		if (remcom_in_buffer[0] == 's') {
			do_single_step(linux_regs);
			kgdb_single_step = 1;

			atomic_set(&kgdb_cpu_doing_single_step,
				   raw_smp_processor_id());
		}

		return 0;
	}

	return -1;
}

static int kgdb_brk_fn(struct pt_regs *regs, unsigned int instr)
{
	kgdb_handle_exception(1, SIGTRAP, 0, regs);

	return 0;
}

static int kgdb_compiled_brk_fn(struct pt_regs *regs, unsigned int instr)
{
	compiled_break = 1;
	kgdb_handle_exception(1, SIGTRAP, 0, regs);

	return 0;
}

static struct undef_hook kgdb_brkpt_hook = {
	.instr_mask		= 0xffffffff,
	.instr_val		= KGDB_BREAKINST,
	.fn			= kgdb_brk_fn
};

static struct undef_hook kgdb_compiled_brkpt_hook = {
	.instr_mask		= 0xffffffff,
	.instr_val		= KGDB_COMPILED_BREAK,
	.fn			= kgdb_compiled_brk_fn
};

static void kgdb_call_nmi_hook(void *ignored)
{
       kgdb_nmicallback(raw_smp_processor_id(), get_irq_regs());
}

void kgdb_roundup_cpus(unsigned long flags)
{
       local_irq_enable();
       smp_call_function(kgdb_call_nmi_hook, NULL, 0);
       local_irq_disable();
}

static int __kgdb_notify(struct die_args *args, unsigned long cmd)
{
	struct pt_regs *regs = args->regs;

	if (kgdb_handle_exception(1, args->signr, cmd, regs))
		return NOTIFY_DONE;
	return NOTIFY_STOP;
}
static int
kgdb_notify(struct notifier_block *self, unsigned long cmd, void *ptr)
{
	unsigned long flags;
	int ret;

	local_irq_save(flags);
	ret = __kgdb_notify(ptr, cmd);
	local_irq_restore(flags);

	return ret;
}

static struct notifier_block kgdb_notifier = {
	.notifier_call	= kgdb_notify,
	.priority	= -INT_MAX,
};


/**
 *	kgdb_arch_init - Perform any architecture specific initalization.
 *
 *	This function will handle the initalization of any architecture
 *	specific callbacks.
 */
int kgdb_arch_init(void)
{
	int ret = register_die_notifier(&kgdb_notifier);

	if (ret != 0)
		return ret;

	register_undef_hook(&kgdb_brkpt_hook);
	register_undef_hook(&kgdb_compiled_brkpt_hook);

	return 0;
}

/**
 *	kgdb_arch_exit - Perform any architecture specific uninitalization.
 *
 *	This function will handle the uninitalization of any architecture
 *	specific callbacks, for dynamic registration and unregistration.
 */
void kgdb_arch_exit(void)
{
	unregister_undef_hook(&kgdb_brkpt_hook);
	unregister_undef_hook(&kgdb_compiled_brkpt_hook);
	unregister_die_notifier(&kgdb_notifier);
}

/*
 * Register our undef instruction hooks with ARM undef core.
 * We regsiter a hook specifically looking for the KGB break inst
 * and we handle the normal undef case within the do_undefinstr
 * handler.
 */
struct kgdb_arch arch_kgdb_ops = {
#ifndef __ARMEB__
	.gdb_bpt_instr		= {0xfe, 0xde, 0xff, 0xe7}
#else /* ! __ARMEB__ */
	.gdb_bpt_instr		= {0xe7, 0xff, 0xde, 0xfe}
#endif
};
