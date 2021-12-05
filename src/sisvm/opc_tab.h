/* op_tab.h: opcodes table for sisvm
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
 * 3. Neither the name of the simple stack machine nor the names of its
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
#ifndef SISVM_OPC_TAB_H
#define SISVM_OPC_TAB_H

#ifdef __cplusplus
extern "C"
{
#endif

	static const char *opc_tab[] =
		{
			/* 0       1       2       3       4       5       6       7       8       9       A       B       C       D       E       F */
			"scl", "scr", "snl", "snr", "add", "sub", "mul", "div", "and", "not", "lor", "xor", "nop", "hlt", "ret", "rst", /* 0 */
			"---", "---", "---", "ldv", "---", "---", "---", "---", "bea", "bna", "bge", "blw", "bra", "brl", "int", "exe", /* 1 */
			"---", "---", "ldf", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", /* 2 */
			"ldd", "ldi", "---", "---", "---", "---", "---", "---", "bei", "bni", "bgi", "bli", "bri", "---", "---", "---", /* 3 */
			"pop", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", /* 4 */
			"---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", /* 5 */
			"---", "hlt", "stf", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", /* 6 */
			"std", "sti", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", /* 7 */
			"psh", "pst", "psp", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", /* 8 */
			"psv", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", /* 9 */
			"---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", /* A */
			"---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", /* B */
			"pop", "pot", "poi", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", /* C */
			"---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", /* D */
			"psd", "psi", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", /* E */
			"---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", "---", /* F */
			NULL};

#ifdef __cplusplus
}
#endif

#endif
