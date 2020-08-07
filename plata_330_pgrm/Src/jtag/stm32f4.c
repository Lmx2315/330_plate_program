/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * Copyright (C) 2017, 2018  Uwe Bonnes
 *                           <bon@elektron.ikp.physik.tu-darmstadt.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements STM32F4 target specific functions for detecting
 * the device, providing the XML memory map and Flash memory programming.
 *
 * Refereces:
 * ST doc - RM0090
 *   Reference manual - STM32F405xx, STM32F407xx, STM32F415xx and STM32F417xx
 *   advanced ARM-based 32-bit MCUs
 * ST doc - PM0081
 *   Programming manual - STM32F40xxx and STM32F41xxx Flash programming
 *    manual
 */
#include "main.h"
#include "target.h"
#include "target_internal.h"
#include "adiv5.h"
#include <stdlib.h>
//#include "cortexm.h"

static bool stm32f4_cmd_erase_mass(target *t, int argc, const char **argv);
static bool stm32f4_cmd_option(target *t, int argc, char *argv[]);
static bool stm32f4_cmd_psize(target *t, int argc, char *argv[]);

const struct command_s stm32f4_cmd_list[] = {
	{"erase_mass", (cmd_handler)stm32f4_cmd_erase_mass,
	 "Erase entire flash memory"},
	{"option", (cmd_handler)stm32f4_cmd_option, "Manipulate option bytes"},
	{"psize", (cmd_handler)stm32f4_cmd_psize,
	 "Configure flash write parallelism: (x8|x16|x32(default)|x64)"},
	{NULL, NULL, NULL}
};

static bool stm32f4_attach(target *t);
static int stm32f4_flash_erase(struct target_flash *f, target_addr addr,
							   size_t len);
static int stm32f4_flash_write(struct target_flash *f,
                               target_addr dest, const void *src, size_t len);

/* Flash Program ad Erase Controller Register Map */
#define FPEC_BASE	0x40023C00
#define FLASH_ACR	(FPEC_BASE+0x00)
#define FLASH_KEYR	(FPEC_BASE+0x04)
#define FLASH_OPTKEYR	(FPEC_BASE+0x08)
#define FLASH_SR	(FPEC_BASE+0x0C)
#define FLASH_CR	(FPEC_BASE+0x10)
#define FLASH_OPTCR	(FPEC_BASE+0x14)

#define FLASH_CR_PG		(1 << 0)
#define FLASH_CR_SER		(1 << 1)
#define FLASH_CR_MER		(1 << 2)
#define FLASH_CR_PSIZE8		(0 << 8)
#define FLASH_CR_PSIZE16	(1 << 8)
#define FLASH_CR_PSIZE32	(2 << 8)
#define FLASH_CR_PSIZE64	(3 << 8)
#define FLASH_CR_MER1		(1 << 15)
#define FLASH_CR_STRT		(1 << 16)
#define FLASH_CR_EOPIE		(1 << 24)
#define FLASH_CR_ERRIE		(1 << 25)
#define FLASH_CR_STRT		(1 << 16)
#define FLASH_CR_LOCK		(1 << 31)

#define FLASH_SR_BSY		(1 << 16)

#define FLASH_OPTCR_OPTLOCK	(1 << 0)
#define FLASH_OPTCR_OPTSTRT	(1 << 1)
#define FLASH_OPTCR_WDG_SW	(1 << 5)
#define FLASH_OPTCR_nDBANK	(1 << 29)
#define FLASH_OPTCR_DB1M	(1 << 30)

#define FLASH_OPTCR_PROT_MASK	0xff00
#define FLASH_OPTCR_PROT_L0  	0xaa00
#define FLASH_OPTCR_PROT_L1  	0xbb00

#define KEY1 0x45670123
#define KEY2 0xCDEF89AB

#define OPTKEY1 0x08192A3B
#define OPTKEY2 0x4C5D6E7F

#define SR_ERROR_MASK	0xF2
#define SR_EOP		0x01

#define F4_FLASHSIZE	0x1FFF7A22
#define F7_FLASHSIZE	0x1FF0F442
#define F72X_FLASHSIZE	0x1FF07A22
#define DBGMCU_IDCODE	0xE0042000
#define DBGMCU_CR		0xE0042004
#define DBG_SLEEP		(1 <<  0)
#define ARM_CPUID	0xE000ED00

#define AXIM_BASE 0x8000000
#define ITCM_BASE 0x0200000

struct stm32f4_flash {
	struct target_flash f;
	enum align psize;
	uint8_t base_sector;
	uint8_t bank_split;
};

static bool stm32f4_option_write(target *t, uint32_t *val, int count)
{
	val[0] &= ~(FLASH_OPTCR_OPTSTRT | FLASH_OPTCR_OPTLOCK);
	uint32_t optcr = target_mem_read32(t, FLASH_OPTCR);
	/* Check if watchdog and read protection is active.
	 * When both are active, watchdog will trigger when erasing
	 * to get back to level 0 protection and operation aborts!
	 */
	if (!(optcr & FLASH_OPTCR_WDG_SW) &&
		((optcr & FLASH_OPTCR_PROT_MASK) != FLASH_OPTCR_PROT_L0) &&
		((val[0] & FLASH_OPTCR_PROT_MASK) != FLASH_OPTCR_PROT_L1)) {
		val[0] &= ~FLASH_OPTCR_PROT_MASK;
		val[0] |=  FLASH_OPTCR_PROT_L1;
		tc_printf(t, "Keeping L1 protection while HW Watchdog fuse is set!\n");
	}
	target_mem_write32(t, FLASH_OPTKEYR, OPTKEY1);
	target_mem_write32(t, FLASH_OPTKEYR, OPTKEY2);
	while (target_mem_read32(t, FLASH_SR) & FLASH_SR_BSY)
		if(target_check_error(t))
			return -1;

	/* WRITE option bytes instruction */
	
	target_mem_write32(t, FLASH_OPTCR, val[0]);
	target_mem_write32(t, FLASH_OPTCR, val[0] | FLASH_OPTCR_OPTSTRT);
	const char spinner[] = "|/-\\";
	int spinindex = 0;
	tc_printf(t, "Erasing flash... This may take a few seconds.  ");
	/* Read FLASH_SR to poll for BSY bit */
	while(target_mem_read32(t, FLASH_SR) & FLASH_SR_BSY) {
		Delay(100);
		tc_printf(t, "\b%c", spinner[spinindex++ % 4]);
		if(target_check_error(t)) {
			tc_printf(t, " failed\n");
			return false;
		}
	}
	tc_printf(t, "\n");
	target_mem_write32(t, FLASH_OPTCR, FLASH_OPTCR_OPTLOCK);
	/* Reset target to reload option bits.*/
	target_reset(t);
	return true;
}

static bool stm32f4_option_write_default(target *t)
{
	uint32_t val[3];
	switch (t->idcode) {
	default:
		val[0] = 0x0FFFAAED;
		return stm32f4_option_write(t, val, 1);
	}
}



static void stm32f4_add_flash(target *t,
                              uint32_t addr, size_t length, size_t blocksize,
                              unsigned int base_sector, int split)
{
	if (length == 0) return;
	struct stm32f4_flash *sf = calloc(1, sizeof(*sf));
	struct target_flash *f;
	if (!sf) {			/* calloc failed: heap exhaustion */
		Transf("calloc: failed in s\n\r");
		return;
	}

	f = &sf->f;
	f->start = addr;
	f->length = length;
	f->blocksize = blocksize;
	f->erase = stm32f4_flash_erase;
	f->write = stm32f4_flash_write;
	f->buf_size = 1024;
	f->erased = 0xff;
	sf->base_sector = base_sector;
	sf->bank_split = split;
	sf->psize = ALIGN_WORD;
	target_add_flash(t, f);
}



static void stm32f4_flash_unlock(target *t)
{
	if (target_mem_read32(t, FLASH_CR) & FLASH_CR_LOCK) {
		/* Enable FPEC controller access */
		target_mem_write32(t, FLASH_KEYR, KEY1);
		target_mem_write32(t, FLASH_KEYR, KEY2);
	}
}

static int stm32f4_flash_erase(struct target_flash *f, target_addr addr,
							   size_t len)
{
	target *t = f->t;
	struct stm32f4_flash *sf = (struct stm32f4_flash *)f;
	uint32_t sr;
	/* No address translation is needed here, as we erase by sector number */
	uint8_t sector = sf->base_sector + (addr - f->start)/f->blocksize;
	stm32f4_flash_unlock(t);

	enum align psize = ALIGN_WORD;
	for (struct target_flash *f = t->flash; f; f = f->next) {
		if (f->write == stm32f4_flash_write) {
			psize = ((struct stm32f4_flash *)f)->psize;
		}
	}
	while(len) {
		uint32_t cr = FLASH_CR_EOPIE | FLASH_CR_ERRIE | FLASH_CR_SER |
			(psize * FLASH_CR_PSIZE16) | (sector << 3);
		/* Flash page erase instruction */
		target_mem_write32(t, FLASH_CR, cr);
		/* write address to FMA */
		target_mem_write32(t, FLASH_CR, cr | FLASH_CR_STRT);

		/* Read FLASH_SR to poll for BSY bit */
		while(target_mem_read32(t, FLASH_SR) & FLASH_SR_BSY)
			if(target_check_error(t)) {
				Transf("stm32f4 flash erase: comm error\n\r");
				return -1;
			}
		if (len > f->blocksize)
			len -= f->blocksize;
		else
			len = 0;
		sector++;
		if ((sf->bank_split) && (sector == sf->bank_split))
			sector = 16;
	}

	/* Check for error */
	sr = target_mem_read32(t, FLASH_SR);
	if(sr & SR_ERROR_MASK) {
		x_out("stm32f4 flash erase: sr error:", sr);
		return -1;
	}
	return 0;
}


static bool stm32f4_cmd_erase_mass(target *t, int argc, const char **argv)
{
	(void)argc;
	(void)argv;
	const char spinner[] = "|/-\\";
	int spinindex = 0;
	struct target_flash *f = t->flash;
	struct stm32f4_flash *sf = (struct stm32f4_flash *)f;

	tc_printf(t, "Erasing flash... This may take a few seconds.  ");
	stm32f4_flash_unlock(t);

	/* Flash mass erase start instruction */
	uint32_t cr =  FLASH_CR_MER;
	if (sf->bank_split)
		cr |=  FLASH_CR_MER1;
	target_mem_write32(t, FLASH_CR, cr);
	target_mem_write32(t, FLASH_CR, cr | FLASH_CR_STRT);

	/* Read FLASH_SR to poll for BSY bit */
	while (target_mem_read32(t, FLASH_SR) & FLASH_SR_BSY) {
		tc_printf(t, "\b%c", spinner[spinindex++ % 4]);
		if(target_check_error(t)) {
			tc_printf(t, "\n");
			return false;
		}
	}
	tc_printf(t, "\n");

	/* Check for error */
	uint32_t sr = target_mem_read32(t, FLASH_SR);
	if ((sr & SR_ERROR_MASK) || !(sr & SR_EOP))
		return false;

	return true;
}


static bool stm32f4_cmd_option(target *t, int argc, char *argv[])
{
	uint32_t val[3];
	int count = 0, readcount = 1;
	
	Transf ("stm32f4_cmd_option\r\n");
/*

	if ((argc == 2) && !strncasecmp(argv[1], "erase", 1)) {
		stm32f4_option_write_default(t);
	}
	else if ((argc > 2) && !strncasecmp(argv[1], "write", 1)) {
		val[0] = strtoul(argv[2], NULL, 0);
		count++;
		if (argc > 3) {
			val[1] = strtoul(argv[3], NULL, 0);
			count ++;
		}
		if (argc > 4) {
			val[2] = strtoul(argv[4], NULL, 0);
			count ++;
		}
		if (optcr_mask(t, val))
			stm32f4_option_write(t, val, count);
		else
			tc_printf(t, "error\n");
	} else {
		tc_printf(t, "usage: monitor option erase\n");
		tc_printf(t, "usage: monitor option write <OPTCR>");
		if (readcount > 1)
			tc_printf(t, " <OPTCR1>");
		if (readcount > 2)
			tc_printf(t, " <OPTCR2>");
		tc_printf(t, "\n");
	}

	val[0]  = target_mem_read32(t, FLASH_OPTCR);
	if (readcount > 1)
		val[1] = target_mem_read32(t, FLASH_OPTCR + 4);
	if (readcount > 2)
		val[2] = target_mem_read32(t, FLASH_OPTCR + 8);
	optcr_mask(t, val);
	tc_printf(t, "OPTCR: 0x%08X ", val[0]);
	if (readcount > 1)
		tc_printf(t, "OPTCR1: 0x%08lx ", val[1]);
	if (readcount > 2)
		tc_printf(t, "OPTCR2: 0x%08lx" , val[2]);
	tc_printf(t, "\n");
	return true;
	*/
}

static bool stm32f4_cmd_psize(target *t, int argc, char *argv[])
{
	if (argc == 1) {
		enum align psize = ALIGN_WORD;
		for (struct target_flash *f = t->flash; f; f = f->next) {
			if (f->write == stm32f4_flash_write) {
				psize = ((struct stm32f4_flash *)f)->psize;
			}
		}
		tc_printf(t, "Flash write parallelism: %s\n",
		          psize == ALIGN_DWORD ? "x64" :
		          psize == ALIGN_WORD ? "x32" :
				  psize == ALIGN_HALFWORD ? "x16" : "x8");
	} else {
		enum align psize;
		if (!strcmp(argv[1], "x8")) {
			psize = ALIGN_BYTE;
		} else if (!strcmp(argv[1], "x16")) {
			psize = ALIGN_HALFWORD;
		} else if (!strcmp(argv[1], "x32")) {
			psize = ALIGN_WORD;
		} else if (!strcmp(argv[1], "x64")) {
			psize = ALIGN_DWORD;
		} else {
			tc_printf(t, "usage: monitor psize (x8|x16|x32|x32)\n");
			return false;
		}
		for (struct target_flash *f = t->flash; f; f = f->next) {
			if (f->write == stm32f4_flash_write) {
				((struct stm32f4_flash *)f)->psize = psize;
			}
		}
	}
	return true;
}

static int stm32f4_flash_write(struct target_flash *f,
                               target_addr dest, const void *src, size_t len)
{
	/* Translate ITCM addresses to AXIM */
	if ((dest >= ITCM_BASE) && (dest < AXIM_BASE)) {
		dest = AXIM_BASE + (dest - ITCM_BASE);
	}
	target *t = f->t;
	uint32_t sr;
	enum align psize = ((struct stm32f4_flash *)f)->psize;
	target_mem_write32(t, FLASH_CR,
					   (psize * FLASH_CR_PSIZE16) | FLASH_CR_PG);
	cortexm_mem_write_sized(t, dest, src, len, psize);
	/* Read FLASH_SR to poll for BSY bit */
	/* Wait for completion or an error */
	do {
		sr = target_mem_read32(t, FLASH_SR);
		if(target_check_error(t)) {
		Transf("stm32f4 flash write: comm error\n\r");
			return -1;
		}
	} while (sr & FLASH_SR_BSY);

	if (sr & SR_ERROR_MASK) {
		x_out("stm32f4 flash write error :", sr);
			return -1;
	}
	return 0;
}

