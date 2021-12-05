/* ssm_ops.c: opcodes for sisvm
 * Copyright (c) 2014, tenpertur
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following
 * conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with
 *    the distribution.
 *
 * 3. Neither the name of the sisvm nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "sisvm.h"
#include "sisvm_ops.h"
#include "opc_tab.h"

/* bit 0-1: load/store/push/pop: 00 load, 01 store, 10 push, 11 pop
 * bit 2-   immediate / address: 0 immediate, 1 address (direct/indirect when branch)
 * bit 3-5  add/sub/mul/div/branch cond/branch unc/control
 *          000 shifts
 *          001 arithmetic
 *          010 logic
 *          011 control
 *          110 branch cond
 *          111 branch unc
 * bit 6-7  options:
 *          load/store:
 *              0 0 direct mem
 *              0 1 indirect mem
 *              1 0 flags
 *              1 1 immediate
 *          push/pop:
 *          	when bit 2 set:
 *          	  push only:
 *          	    0 0 direct
 *                  0 1 indirect
 *              when bit 2 not set:
 *                push / pop:
 *                  0 0 acc
 *                  0 1 tmp
 *                  1 0 ip
 *                push only:
 *                  1 1 immediate
  *          arithmetic:
 *              0 0 add
 *              0 1 sub
 *              1 0 mul
 *              1 1 div
 *          logic:
 *              0 0 and
 *              0 1 not
 *              1 0 or
 *              1 1 xor
 *          branch cond:
 *              bit 6: 0/1 eq/neq
 *              bit 7: 0/1 gr/lw
 *          branch unconditionally:
 *              1 1 call function
 *              1 0 interrupt
 *          control:
 *              0 0 nop
 *              0 1 hlt
 *              1 0 ret
 *              1 1 rst
 *          shift:
 *              bit 6 : 0 left   1 right
 *              bit 7 : 0 cyclic 1 non cyclic
 * note:
 * 1) bit 0: indirection (ip+addr)
 * 2) bit 3 means that next 4 bytes will be fetched
 *
 *
 * 0 1 2 3 4 5 6 7
 * 0 0 0 1 0 0 1 1  ldv	imm 13	load immediate value to the accumulator                acc<-mem(ip);        ip+=5
 * 0 0 1 1 0 0 0 0  ldd	adr 30	load to accumulator contents from mem loc. addr        acc<-mem(addr);      ip+=5;
 * 0 1 1 1 0 0 0 0  std adr 70  store contents from acc to memory under loc addr       mem(addr)<-acc;      ip+=5;
 * 0 0 1 1 0 0 0 1  ldi	adr 31	load to accumulator contents from mem loc. addr + ip   acc<-mem(mem(addr)); ip+=5;
 * 0 1 1 1 0 0 0 1  sti adr 71  store contents from acc to memory under loc addr + ip  mem(mem(addr))<-acc; ip+=5;
 * 0 0 1 0 0 0 1 0  ldf --- 22  loads contents of flag reg to acc                      acc<-flags           ip+=1;
 * 0 1 1 0 0 0 1 0  stf --- 62  stores contents of acc to flags                        flg<-acc             ip+=1;
 *
 * 1 0 0 0 0 0 0 0  psh --- 80  push acc on the stack                                  stack(sp)<-acc
 * 1 0 0 0 0 0 0 1  pst --- 81  push tmp                                               stack(sp)<-tmp
 * 1 0 0 0 0 0 1 0  psp --- 82  push ip                                                stack(sp)<-ip
 * 1 0 0 1 0 0 0 0  psv imm 90  push value to stack                                    stack(sp)<-mem(ip)
 * 1 0 1 1 0 0 0 0  psd add E0  push address                                           stack(sp)<-mem(addr)
 * 1 0 1 1 0 0 0 1  psi add E1  push address direct                                    stack(sp)<-mem(mem(addr))

 * 1 1 0 0 0 0 0 0  pop --- C0  pops acc                                               acc<-stack(sp)
 * 1 1 0 0 0 0 0 1  pot --- C1  pop tmp                                                tmp<-stack(sp)
 * 1 1 0 0 0 0 1 0  poi --- C2  pops ip                                                ip<-stack(sp);
 *
 * 0 0 0 0 0 1 0 0  add --- 04  adds to acc first element from stack
 * 0 0 0 0 0 1 0 1  sub --- 05  subtracts from acc first element from stack
 * 0 0 0 0 0 1 1 0  mul --- 06  multiplies
 * 0 0 0 0 0 1 1 1  div --- 07  div
 *
 * 0 0 0 0 1 0 0 0  and --- 08  acc->acc AND (sp)
 * 0 0 0 0 1 0 0 1  not --- 09  acc-> ~acc
 * 0 0 0 0 1 0 1 0  lor --- 0A  acc-> acc OR (sp)
 * 0 0 0 0 1 0 1 1  xor --- 0B  acc-> acc XOR (sp)
 *
 * 0 0 0 0 1 1 0 0  nop --- 0C  nop
 * 0 0 0 0 1 1 0 1  hlt --- 0D  halt
 * 0 0 0 0 1 1 1 0  ret --- 0E  return from function
 * 0 0 0 0 1 1 1 1  rst --- 0F  reset
 *
 * 0 0 0 1 1 0 0 0  bea adr 18  branch if equal (acc and stack)
 * 0 0 1 1 1 0 0 0  bei adr 38  branch if equal (acc and stack) indirect
 * 0 0 0 1 1 0 0 1  bna adr 19  branch if not equal
 * 0 0 1 1 1 0 0 1  bni adr 39  branch if not equal indirect
 * 0 0 0 1 1 0 1 0  bge adr 1A  branch if greater
 * 0 0 0 1 1 0 1 1  blw adr 1B  branch if lower
 * 0 0 0 1 1 1 0 0  bra adr 1C  branch unconditionally
 * 0 0 0 1 1 1 0 1  brl adr 1D  branch to absolute location
 *
 * 0 0 0 0 0 0 0 0  scl --- 00  shift cyclic left
 * 0 0 0 0 0 0 0 1  scr --- 01  shift cyclic rigt
 * 0 0 0 0 0 0 1 0  snl --- 02  shift non cyclic left
 * 0 0 0 0 0 0 1 1  snr --- 03  shift non cyclic rigt
 *
 *
 * 0 0 0 1 1 1 1 0  int adr 1E  interrupt
 * 0 0 0 1 1 1 1 1  exe adr 1F  execute function

 *
 */
/*
 * Architecture
 * Registers:
 * 1. Accumulator (acc) - performs all arithmetic operations
 * 2. Temporary (
 */

/** Halts the machine.
 *  Standalone version for ease of use inside other functions which may need to stop the execution.
 *  \param m machine object to operate on
 *  \return operation status
 *  \retval 0 if success
 *  \retval 1 when failed
 */
int hlt(struct sisvm *m)
{
	m->cpu->flag->halt = 1;
	return 0;
}
/** Fetch one byte from memory location pointed by IP.
 *  Byte is stored into TMP reg.
 *  \return operation status code
 *  \retval 0 success
 *  \retval 1 fail
 */
static int
fetch_byte(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	struct memory *mm = m->memory;

	if (r->ip == mm->ram_s)
		CHECK_HALT(m, 1, "fetch_byte failed, ram size exceeded");
	r->tmp = *(mm->ram + r->ip);
	r->ip++;
	return 0;
}

/** Fetch one word from memory location pointed by IP.
 *  Word is stored into TMP reg.
 *  \return operation status code
 *  \retval 0 success
 *  \retval 1 fail
 */
static int
fetch_word(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	int rv;
	word_t tmp = 0;
	int n = sizeof(r->acc);
	while (n--)
	{
		rv = fetch_byte(m);
		CHECK_HALT(m, rv, "fetch_word failed, ram size exceeded");
		tmp |= r->tmp << (n << 3);
	}
	r->tmp = tmp;
	return 0;
}

/** Stores one byte in memory under location pointed by ADR reg.
 *
 *  \return operation status code
 *  \retval 0 success
 *  \retval 1 fail
 */
static int
store_byte(struct sisvm *m, uint8_t byte)
{
	struct regs *r = m->cpu->reg;
	struct memory *mm = m->memory;

	if (r->adr == mm->ram_s)
		CHECK_HALT(m, 1, "store_byte failed, ram size exceeded");
	*(mm->ram + r->adr) = byte;

	return 0;
}

/** Stores one word, location pointed by TMP reg
 *
 *  \return operation status code
 *  \retval 0 success
 *  \retval 1 fail
 */
static int
store_word(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	int rv;
	int n = sizeof(r->acc);
	uint8_t byte;
	while (n--)
	{
		byte = r->acc >> (n << 3);
		rv = store_byte(m, byte);
		CHECK_HALT(m, rv, "store_word failed, ram size exceeded");
		r->tmp++;
	}
	return 0;
}

/** Stores register or value under given memory location.
 *
 *
 *  \return operation status code
 *  \retval 0 success
 *  \retval 1 fail
 */
static int
store_operation(struct sisvm *m)
{
	int rv;
	word_t opcode = m->cpu->reg->tmp;
	struct regs *r = m->cpu->reg;
	word_t ip;
	rv = fetch_word(m);
	CHECK_HALT(m, rv, "store operation failed, cannot get operand");
	switch (opcode & 0x03)
	{
	case 0x00:
		r->adr = r->tmp;
		rv = store_word(m);
		break;
	case 0x01:
		r->adr = r->tmp;
		ip = r->ip;
		rv = fetch_word(m);
		CHECK_HALT(m, rv, "store operation failed, cannot get operand");
		r->adr = r->tmp;
		r->ip = ip;
		rv = store_word(m);
		break;
		/* TODO flags manipulation */
	case 0x03:
	default:
		CHECK_HALT(m, rv, "store operation failed, unknown error");
		break;
	}
	CHECK_HALT(m, rv,
			   "store operation failed, cannot store data under given memory location");
	return 0;
}

static int
pop(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;

	int rv;
	word_t reg = 0;

	reg = r->ip, r->ip = r->stk;

	rv = fetch_word(m);
	CHECK_HALT(m, rv, "pop failed, cannot fetch stack");

	r->stk = r->ip, r->ip = reg;

	r->acc = r->tmp;

	return 0;
}

static int
pop_operation(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	uint8_t op = r->tmp & 0x0f;
	int rv;

	rv = pop(m);
	CHECK_HALT(m, rv, "pop operation failed, cannot fetch from stack");

	switch (op)
	{
	case 0:
	case 1:
		break;
	case 2:
		r->ip = r->acc;
		break;
	default:
		CHECK_HALT(m, 1, "pop failed, unknown op?");
		break;
	}
	return 0;
}

/*
 * should not be called directly
 */
static int
push_word(struct sisvm *m, word_t word)
{
	struct regs *r = m->cpu->reg;
	struct memory *mm = m->memory;
	int j = 0;
	uint8_t byte;
	size_t n = sizeof(r->acc);
	if (r->stk - n < mm->ram_s - mm->stack_s)
		CHECK_HALT(m, 1, "push_word failed, stack space exhausted");
	r->stk -= n;
	while (n--)
	{
		byte = word >> (n << 3);
		*(mm->ram + (ptrdiff_t)(r->stk + j++)) = byte;
	}
	return 0;
}

static int
psh(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	int rv;

	rv = push_word(m, r->acc);

	return rv;
}

static int
push_operation(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	word_t reg, opcode = r->tmp;
	int rv;
	/* push reg */
	if ((opcode & 0xF0) == 0x80)
	{
		switch (opcode & 0x0F)
		{
		case 0:
			reg = r->acc;
			break;
		case 1:
			reg = r->tmp;
			break;
		case 2:
			reg = r->ip;
			break;
		default:
			CHECK_HALT(m, 1, "push failed, unknown reg requested");
			break;
		}
	}
	else
	{
		/* push from memory location or direct value */
		rv = fetch_word(m);
		CHECK_HALT(m, rv, "push failed, cannot fetch operand");
		reg = r->tmp;
		word_t ip;
		if ((opcode & 0xF0) == 0xE0)
		{
			switch (opcode & 0x0F)
			{
			case 0x00:
				r->adr = r->tmp;
				break;
			case 0x01:
				r->adr = r->tmp;
				ip = r->ip;
				rv = fetch_word(m);
				CHECK_HALT(m, rv,
						   "push indirect operation failed, cannot get content");
				r->ip = ip;
				r->adr = r->tmp;
				break;
			default:
				CHECK_HALT(m, 1,
						   "push failed, cannot select addessing");
			}
			rv = fetch_word(m);
			CHECK_HALT(m, rv,
					   "push failed, cannot fetch value from given address");
			reg = r->tmp;
		}
	}
	rv = push_word(m, reg);
	CHECK_HALT(m, rv, "push failed, stack exhausted");
	return 0;
}

static int
ldv(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;

	int rv;
	rv = fetch_word(m);
	CHECK_HALT(m, rv, "ldv failed, cannot fetch data to be loaded\0");
	r->acc = r->tmp;
	return 0;
}

static int
ldx(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	int rv;
	int is_indirect = r->tmp & 0x80;
	char err[] = {"ldx failed, cannot fetch data\0"};
	*(err + 2) = (is_indirect) ? 'i' : 'd';
	/* load addr bytes */
	rv = fetch_word(m);
	CHECK_HALT(m, rv, err);

	/* calulate new address */
	if (!is_indirect)
		r->adr = r->tmp;
	else
	{
		word_t ip = r->ip;
		r->adr = r->tmp;
		rv = fetch_word(m);
		CHECK_HALT(m, rv, "ldi operation failed, cannot get operand");
		r->adr = r->tmp;
		r->ip = ip;
	}

	rv = ldv(m);
	CHECK_HALT(m, rv, err);

	return 0;
}

static int
ldf(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	struct flags *f = m->cpu->flag;
	r->acc = 0;
	r->acc |= f->halt;
	r->acc |= (f->halt) << 1;

	return 0;
}

static int
aritmetic_op(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	struct flags *f = m->cpu->flag;
	int64_t res;
	int rv;
	word_t tmp;
	uint8_t sel = r->tmp & 0x0f;
	tmp = r->acc;
	rv = pop(m);
	CHECK_HALT(m, rv, "aritmetic operation failed");
	r->tmp = r->acc;
	r->acc = tmp;
	switch (sel)
	{
	case 0x04:
		res = r->tmp + r->acc;
		break;
	case 0x05:
		res = r->acc - r->tmp;
		break;
	case 0x06:
		res = r->acc * r->tmp;
		break;
	case 0x07:
		if (r->tmp == 0)
		{
			f->exception = 1;
			return 1;
		}
		res = r->acc / r->tmp;
		break;
	default:
		CHECK_HALT(m, 1, "error internal in aritmetic");
		break;
	}
	f->overflow = (res < INT32_MIN) ? 1 : 0;
	f->overflow = (res > INT32_MAX) ? 1 : 0;
	r->acc = (int32_t)res;
	f->zero = (r->acc == 0) ? 1 : 0;
	return 0;
}

static int
logic_op(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	struct flags *f = m->cpu->flag;
	int rv;
	uint8_t sel = r->tmp & 0x0f;
	r->tmp = r->acc;
	rv = pop(m);
	CHECK_HALT(m, rv, "LOGIC failed");
	switch (sel)
	{
	case 0x08:
		r->acc = r->tmp & r->acc;
		break;
	case 0x09:
		r->acc = ~r->tmp;
		break;
	case 0x0a:
		r->acc = r->tmp | r->acc;
		break;
	case 0x0b:
		r->acc = r->tmp ^ r->acc;
		break;
	default:
		CHECK_HALT(m, rv, "error internal in logic");
		break;
	}
	f->zero = (r->acc == 0) ? 1 : 0;
	return 0;
}

static int
shift_op(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	struct flags *f = m->cpu->flag;
	int rv;
	uint8_t sel = r->tmp & 0x0f;
	r->tmp = r->acc;
	rv = pop(m);
	CHECK_HALT(m, rv, "LOGIC failed");
	switch (sel)
	{
	case 0x04:
		r->acc = r->tmp << (r->acc % (sizeof(r->acc) << 3));
		break;
	case 0x05:
		r->acc = r->tmp >> (r->acc % (sizeof(r->acc) << 3));
		break;
	case 0x06:
		r->acc = r->tmp << r->acc;
		break;
	case 0x07:
		r->acc = r->tmp >> r->acc;
		break;
	default:
		CHECK_HALT(m, rv, "error internal in logic");
		break;
	}
	f->zero = (r->acc == 0) ? 1 : 0;
	return 0;
}

static int
branch_op(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	struct memory *mm = m->memory;
	int is_indirect = r->tmp & 0x80;
	uint8_t sel = r->tmp & 0x0f;
	int rv;
	uint8_t cmptab[0xf] = {0};

	if (sel < 0x0c)
	{
		word_t t = r->acc;
		/* get to the acc 1st element from stack */
		rv = pop(m);
		CHECK_HALT(m, rv, "branch cannot pop.");
		rv = psh(m);
		CHECK_HALT(m, rv, "branch cannot push.");
		r->tmp = r->acc;
		r->acc = t;
	}

	cmptab[0x8] = r->acc == r->tmp;
	cmptab[0x9] = r->acc != r->tmp;
	cmptab[0x9] = r->acc != r->tmp;
	cmptab[0xA] = r->acc > r->tmp;
	cmptab[0xB] = r->acc < r->tmp;
	cmptab[0xC] = 1;
	cmptab[0xD] = 1;

	rv = fetch_word(m);
	r->adr = r->tmp;
	CHECK_HALT(m, rv, "branch cannot fetch");

	if (cmptab[sel])
	{
		if (is_indirect)
		{
			word_t ip = r->ip;
			rv = fetch_word(m);
			CHECK_HALT(m, rv,
					   "indirect branch tried to read outside of memory");
			r->adr = r->tmp;
			r->ip += ip;
		}

		if (r->ip + r->adr >= mm->ram_s - mm->stack_s)
			CHECK_HALT(m, 1, "branch tried to jump outside of memory");
		r->ip += r->adr;
	}

	return 0;
}

static int
control_op(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	struct flags *f = m->cpu->flag;
	uint8_t sel = r->tmp & 0x0f;
	int rv;
	switch (sel)
	{
	case 0x0C:
		m->cpu->reg->ip++;
		break;
	case 0x0D:
		f->halt = 1;
		break;
	case 0x0E:
		rv = pop(m);
		CHECK_HALT(m, rv, "ret error");
		r->ip = r->acc;
		break;
	case 0x0F:
		memset(r, 0, sizeof(*r));
		memset(f, 0, sizeof(*f));
		r->stk = m->memory->ram_s;
		break;
	default:
		CHECK_HALT(m, 1, "control error");
		break;
	}
	return 0;
}

static int
exec_int_op(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	int rv;
	uint8_t sel = r->tmp & 0x0f;
	rv = fetch_byte(m);
	CHECK_HALT(m, rv, "exe/int cannot fetch");
	switch (sel)
	{
	case 0x0D:
		break;
	case 0x0F:
		r->acc = r->ip;
		rv = psh(m);
		CHECK_HALT(m, 1, "exe fail");
		r->ip = r->tmp;
		break;
	default:
		break;
	}
	return 0;
}

uint8_t
opc_to_hex(const char **opt, const char *op)
{
	size_t loc = 0;

	while ((opt + loc) && strcmp(*(opt + loc), op) != 0)
		loc++;
	return (uint8_t)loc;
}

static fptr_opcf_t *
getoptab()
{
	fptr_opcf_t *r;
	const char **ot = (void *)opc_tab;
	int rv = 0xff;
	r = malloc(rv * sizeof(*r));
	if (NULL == r)
		return NULL;
	/* execution of not suported opcode will stop the machine */
	while (rv--)
		r[rv] = hlt;

	r[opc_to_hex(ot, "add")] = aritmetic_op;
	r[opc_to_hex(ot, "and")] = logic_op;
	r[opc_to_hex(ot, "bea")] = branch_op;
	r[opc_to_hex(ot, "bei")] = branch_op;
	r[opc_to_hex(ot, "bna")] = branch_op;
	r[opc_to_hex(ot, "bni")] = branch_op;
	r[opc_to_hex(ot, "bra")] = branch_op;
	r[opc_to_hex(ot, "bri")] = branch_op;
	r[opc_to_hex(ot, "brl")] = branch_op;
	r[opc_to_hex(ot, "div")] = aritmetic_op;
	r[opc_to_hex(ot, "exe")] = exec_int_op;
	r[opc_to_hex(ot, "hlt")] = control_op;
	r[opc_to_hex(ot, "int")] = exec_int_op;
	r[opc_to_hex(ot, "ldd")] = ldx;
	r[opc_to_hex(ot, "ldf")] = ldf;
	r[opc_to_hex(ot, "ldi")] = ldx;
	r[opc_to_hex(ot, "ldv")] = ldv;
	r[opc_to_hex(ot, "lor")] = logic_op;
	r[opc_to_hex(ot, "mul")] = aritmetic_op;
	r[opc_to_hex(ot, "nop")] = control_op;
	r[opc_to_hex(ot, "not")] = logic_op;
	r[opc_to_hex(ot, "poi")] = pop_operation;
	r[opc_to_hex(ot, "pop")] = pop_operation;
	r[opc_to_hex(ot, "pot")] = pop_operation;
	r[opc_to_hex(ot, "psd")] = push_operation;
	r[opc_to_hex(ot, "psh")] = push_operation;
	r[opc_to_hex(ot, "psi")] = push_operation;
	r[opc_to_hex(ot, "psp")] = push_operation;
	r[opc_to_hex(ot, "pst")] = push_operation;
	r[opc_to_hex(ot, "psv")] = push_operation;
	r[opc_to_hex(ot, "ret")] = control_op;
	r[opc_to_hex(ot, "rst")] = control_op;
	r[opc_to_hex(ot, "scl")] = shift_op;
	r[opc_to_hex(ot, "scr")] = shift_op;
	r[opc_to_hex(ot, "snl")] = shift_op;
	r[opc_to_hex(ot, "snr")] = shift_op;
	r[opc_to_hex(ot, "std")] = store_operation;
	r[opc_to_hex(ot, "sti")] = store_operation;
	r[opc_to_hex(ot, "sub")] = aritmetic_op;
	r[opc_to_hex(ot, "xor")] = logic_op;

	return r;
}

/** Reads word from given location in memory.
 *  Used for debug puposes
 */
static int
get_mem_wrd(struct sisvm *m, size_t idx, word_t *wrd)
{
	size_t i = 0, n = sizeof(m->cpu->reg->acc);
	*wrd = 0;
	while (n-- && idx + i < m->memory->ram_s)
	{
		*wrd = *wrd | *(m->memory->ram + idx + i) << (n << 3);
		i++;
	}
	if (i < n)
		return 1;
	else
		return 0;
}

static int
dump_machine_state(struct sisvm *m)
{
	fprintf(stderr, "PC: %08x\tIP: %08x\tSP: %08x\tACC: %08x\n", m->cpu->reg->cnt,
			m->cpu->reg->ip, m->cpu->reg->stk, m->cpu->reg->acc);
	return 0;
}

int decode(struct sisvm *m)
{
	struct regs *r = m->cpu->reg;
	int rv;
	fptr_opcf_t *btable;
	uint8_t opcode;

	btable = getoptab();
	if (NULL == btable)
		return 1;
	while (!m->cpu->flag->halt)
	{
		rv = fetch_byte(m);
		if (rv)
			return 1;
		opcode = r->tmp;
		if (opcode & 0x10)
		{
			word_t val;
			get_mem_wrd(m, r->ip, &val);
			printf("PC: %x\t%08x: %s\t%08x\n", r->cnt,
				   r->ip - 1, opc_tab[opcode], val);
		}
		else
			printf("PC: %x\t%08x: %s\n",
				   r->cnt, r->ip - 1, opc_tab[opcode]);
		rv = btable[opcode](m);
		if (rv)
			dump_machine_state(m);
		CHECK_HALT(m, rv, "execution failed");
		r->cnt++;
		//sleep(1);
	}
	return 0;
}
