/* ssm.h: declarations for sisvm
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

#ifndef SISSVM_H
#define SISSVM_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <memory.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define HALTF 0
#define OVERF 1

	typedef int32_t word_t;

	struct flags
	{
		uint8_t exception;
		uint8_t halt;
		uint8_t overflow;
		uint8_t zero;
	};

	struct regs
	{
		word_t acc;	  /* accumulator */
		word_t tmp;	  /* additional register */
		uint32_t ip;  /* instruction pointer */
		uint32_t cnt; /* program counter */
		uint32_t stk; /* stack pointer */
		uint32_t adr; /* memory cell adress */
	};

	struct cpu
	{
		struct regs *reg;
		struct flags *flag;
	};

	struct memory
	{
		uint8_t *ram;
		uint8_t *stack;
		size_t ram_s;
		size_t stack_s;
	};

	struct sisvm
	{
		struct cpu *cpu;
		struct memory *memory;
	};

	typedef int (*fptr_opcf_t)(struct sisvm *);

	extern int hlt(struct sisvm *);

#define CHECK(m, r, text)                                                                     \
	do                                                                                        \
	{                                                                                         \
		int _ret = (r);                                                                       \
		if (_ret < 0)                                                                         \
		{                                                                                     \
			fprintf(stderr, "%s: %d (%s) -> %s\n", __FILE__, __LINE__, __FUNCTION__, (text)); \
			return 1;                                                                         \
		}                                                                                     \
	} while (0)

#define CHECK_HALT(m, r, text)                                                                        \
	do                                                                                                \
	{                                                                                                 \
		int _ret = (r);                                                                               \
		if (_ret < 0)                                                                                 \
		{                                                                                             \
			hlt(m);                                                                                   \
			fprintf(stderr, "HALT in %s: %d (%s) -> %s\n", __FILE__, __LINE__, __FUNCTION__, (text)); \
			return 1;                                                                                 \
		}                                                                                             \
	} while (0)

#ifdef __cplusplus
}
#endif

#endif /* SISSVM_H */
