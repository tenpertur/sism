/* sisvm.c: Small Instruction Set Virtual Machine (sisvm)
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
#define STACK_S 0x10
#define RAM_S 0x30

void free_cpu(struct cpu *cpu)
{
	if (cpu)
	{
		free(cpu->reg);
		cpu->reg = NULL;
		free(cpu->flag);
		cpu->flag = NULL;
		free(cpu);
		cpu = NULL;
	}
}

int init_cpu(struct cpu *cpu)
{
	cpu->reg = calloc(1, sizeof *cpu->reg);
	if (NULL == cpu->reg)
		return -1;
	cpu->flag = calloc(1, sizeof *cpu->flag);
	if (NULL == cpu->flag)
	{
		free(cpu->reg);
		cpu->reg = NULL;
		return -1;
	}

	return 0;
}

int init_memory(struct memory *m, size_t rs, size_t ss)
{
	m->ram_s = rs;
	m->stack_s = ss;
	m->ram = calloc(m->ram_s, sizeof *m->ram);
	if (NULL == m->ram)
		return -1;
	m->stack = m->ram + (ptrdiff_t)(m->ram_s - m->stack_s);
	return 0;
}
void free_memory(struct memory *m)
{
	free(m->ram);
	free(m);
}

void free_vm(struct sisvm *m)
{
	if (m)
	{
		free_memory(m->memory);
		free_cpu(m->cpu);
	}
	free(m);
}

int init_vm(struct sisvm **machine, size_t ram_s, size_t stack_s)
{
	struct sisvm *m;
	int r;
	m = calloc(1, sizeof *m);
	if (NULL == m)
		return -1;

	m->cpu = calloc(1, sizeof *m->cpu);
	if (NULL == m->cpu)
		goto error;
	r = init_cpu(m->cpu);
	if (r)
		goto error;
	m->memory = calloc(1, sizeof *m->memory);
	if (NULL == m->memory)
		goto error;
	r = init_memory(m->memory, ram_s, stack_s);
	if (r)
		goto error;
	/* set stack ptr to the end of ram */
	m->cpu->reg->stk = m->memory->ram_s;
	m->cpu->reg->cnt = 0;

	*machine = m;
	return 0;
error:
	free_vm(m);
	return -1;
}

int load_binary_program(const char *name, struct sisvm *m)
{
	FILE *f;
	size_t size, readed;
	f = fopen(name, "rb");
	if (NULL == f)
		return -1;
	fseek(f, 0, SEEK_END);
	size = ftell(f);
	rewind(f);
	readed = fread(m->memory->ram, 1, size, f);
	if (readed != size)
		return -1;
	fclose(f);
	return 0;
}

int main()
{
	struct sisvm *machine;
	int r;
	r = init_vm(&machine, RAM_S, STACK_S);
	if (r)
	{
		free_vm(machine);
		return -1;
	}
	machine->cpu->reg->ip = 0;
	r = load_binary_program("src/stp/test4.bin", machine);
	if (r)
	{
		free_vm(machine);
		return -1;
	}
	r = decode(machine);

	free_vm(machine);
	return 0;
}
