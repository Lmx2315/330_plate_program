/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
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

/* This file implements the JTAG-DP specific functions of the
 * ARM Debug Interface v5 Architecure Specification, ARM doc IHI0031A.
 */

//#include "exception.h"
//#include "adiv5.h"
#include "jtag_scan.h"
#include "jtagtap.h"
#include "timing.h"
#include "platform_support.h"
#include "exception.h"
//#include "morse.h"

#define JTAGDP_ACK_OK	0x02
#define JTAGDP_ACK_WAIT	0x01

/* 35-bit registers that control the ADIv5 DP */
#define IR_ABORT	0x8
#define IR_DPACC	0xA
#define IR_APACC	0xB

#define ADIV5_APnDP       0x100
#define ADIV5_DP_REG(x)   (x)
#define ADIV5_AP_REG(x)   (ADIV5_APnDP | (x))

/* ADIv5 DP Register addresses */
#define ADIV5_DP_IDCODE   ADIV5_DP_REG(0x0)
#define ADIV5_DP_ABORT    ADIV5_DP_REG(0x0)
#define ADIV5_DP_CTRLSTAT ADIV5_DP_REG(0x4)
#define ADIV5_DP_SELECT   ADIV5_DP_REG(0x8)
#define ADIV5_DP_RDBUFF   ADIV5_DP_REG(0xC)

#define ADIV5_DP_BANK0    0
#define ADIV5_DP_BANK1    1
#define ADIV5_DP_BANK2    2
#define ADIV5_DP_BANK3    3
#define ADIV5_DP_BANK4    4

#define ADIV5_DP_VERSION_MASK 0xf000
#define ADIV5_DPv1            0x1000
#define ADIV5_DPv2            0x2000

/* AP Abort Register (ABORT) */
/* Bits 31:5 - Reserved */
#define ADIV5_DP_ABORT_ORUNERRCLR	(1 << 4)
#define ADIV5_DP_ABORT_WDERRCLR		(1 << 3)
#define ADIV5_DP_ABORT_STKERRCLR	(1 << 2)
#define ADIV5_DP_ABORT_STKCMPCLR	(1 << 1)
/* Bits 5:1 - SW-DP only, reserved in JTAG-DP */
#define ADIV5_DP_ABORT_DAPABORT		(1 << 0)

/* Control/Status Register (CTRLSTAT) */
#define ADIV5_DP_CTRLSTAT_CSYSPWRUPACK	(1u << 31)
#define ADIV5_DP_CTRLSTAT_CSYSPWRUPREQ	(1u << 30)
#define ADIV5_DP_CTRLSTAT_CDBGPWRUPACK	(1u << 29)
#define ADIV5_DP_CTRLSTAT_CDBGPWRUPREQ	(1u << 28)
#define ADIV5_DP_CTRLSTAT_CDBGRSTACK	(1u << 27)
#define ADIV5_DP_CTRLSTAT_CDBGRSTREQ	(1u << 26)
/* Bits 25:24 - Reserved */
/* Bits 23:12 - TRNCNT */
#define ADIV5_DP_CTRLSTAT_TRNCNT
/* Bits 11:8 - MASKLANE */
#define ADIV5_DP_CTRLSTAT_MASKLANE
/* Bits 7:6 - Reserved in JTAG-DP */
#define ADIV5_DP_CTRLSTAT_WDATAERR	(1u << 7)
#define ADIV5_DP_CTRLSTAT_READOK	(1u << 6)
#define ADIV5_DP_CTRLSTAT_STICKYERR	(1u << 5)
#define ADIV5_DP_CTRLSTAT_STICKYCMP	(1u << 4)
#define ADIV5_DP_CTRLSTAT_TRNMODE_MASK	(3u << 2)
#define ADIV5_DP_CTRLSTAT_STICKYORUN	(1u << 1)
#define ADIV5_DP_CTRLSTAT_ORUNDETECT	(1u << 0)


/* ADIv5 MEM-AP Registers */
#define ADIV5_AP_CSW		ADIV5_AP_REG(0x00)
#define ADIV5_AP_TAR		ADIV5_AP_REG(0x04)
/* 0x08 - Reserved */
#define ADIV5_AP_DRW		ADIV5_AP_REG(0x0C)
#define ADIV5_AP_DB(x)		ADIV5_AP_REG(0x10 + (4*(x)))
/* 0x20:0xF0 - Reserved */
#define ADIV5_AP_CFG		ADIV5_AP_REG(0xF4)
#define ADIV5_AP_BASE		ADIV5_AP_REG(0xF8)
#define ADIV5_AP_IDR		ADIV5_AP_REG(0xFC)

/* AP Control and Status Word (CSW) */
#define ADIV5_AP_CSW_DBGSWENABLE	(1u << 31)
/* Bits 30:24 - Prot, Implementation defined, for Cortex-M3: */
#define ADIV5_AP_CSW_MASTERTYPE_DEBUG	(1u << 29)
#define ADIV5_AP_CSW_HPROT1		(1u << 25)
#define ADIV5_AP_CSW_SPIDEN		(1u << 23)
/* Bits 22:12 - Reserved */
/* Bits 11:8 - Mode, must be zero */
#define ADIV5_AP_CSW_TRINPROG		(1u << 7)
#define ADIV5_AP_CSW_DEVICEEN		(1u << 6)
#define ADIV5_AP_CSW_ADDRINC_NONE	(0u << 4)
#define ADIV5_AP_CSW_ADDRINC_SINGLE	(1u << 4)
#define ADIV5_AP_CSW_ADDRINC_PACKED	(2u << 4)
#define ADIV5_AP_CSW_ADDRINC_MASK	(3u << 4)
/* Bit 3 - Reserved */
#define ADIV5_AP_CSW_SIZE_BYTE		(0u << 0)
#define ADIV5_AP_CSW_SIZE_HALFWORD	(1u << 0)
#define ADIV5_AP_CSW_SIZE_WORD		(2u << 0)
#define ADIV5_AP_CSW_SIZE_MASK		(7u << 0)

/* AP Debug Base Address Register (BASE) */
#define ADIV5_AP_BASE_BASEADDR		(0xFFFFF000u)
#define ADIV5_AP_BASE_PRESENT		(1u << 0)


/* ADIv5 Class 0x1 ROM Table Registers */
#define ADIV5_ROM_MEMTYPE			0xFCC
#define ADIV5_ROM_MEMTYPE_SYSMEM	(1u << 0)
#define ADIV5_ROM_ROMENTRY_PRESENT  (1u << 0)
#define ADIV5_ROM_ROMENTRY_OFFSET	(0xFFFFF000u)


/* Constants to make RnW parameters more clear in code */
#define ADIV5_LOW_WRITE		0
#define ADIV5_LOW_READ		1

enum align {
	ALIGN_BYTE     = 0,
	ALIGN_HALFWORD = 1,
	ALIGN_WORD     = 2,
	ALIGN_DWORD    = 3
};

//------------------------------------------------------------------------------------------------------------

uint32_t fw_adiv5_jtagdp_low_access   //процедура чтения записи на уровне JTAG , 
(jtag_dev_t *,//тут лежит ячейка массива с адресами клиентов на шине JTAG  (например jtag_devs[0])
 uint8_t ,   //чтение или запись , 
 uint16_t , // адресс доступа
 uint32_t  // значение записываемое
 );

static uint32_t adiv5_jtagdp_error(jtag_dev_t *dp);

static void adiv5_jtagdp_abort(jtag_dev_t *dp, uint32_t abort);

void adiv5_jtag_dp_handler(jtag_dev_t *dev)
{

}

uint32_t fw_adiv5_jtagdp_read   //процедура чтения из JTAG 
(jtag_dev_t *dp,				//тут лежит ячейка массива с адресами клиентов на шине JTAG  (например jtag_devs[0])
 uint16_t addr					//адрес чтения
 )  
{
	fw_adiv5_jtagdp_low_access(dp, ADIV5_LOW_READ, addr, 0);
	return fw_adiv5_jtagdp_low_access(dp, ADIV5_LOW_READ,
					ADIV5_DP_RDBUFF, 0);
}

static uint32_t adiv5_jtagdp_error(jtag_dev_t *dp)
{
	fw_adiv5_jtagdp_low_access(dp, ADIV5_LOW_READ, ADIV5_DP_CTRLSTAT, 0);
	return fw_adiv5_jtagdp_low_access(dp, ADIV5_LOW_WRITE,
				ADIV5_DP_CTRLSTAT, 0xF0000032) & 0x32;
}

uint32_t fw_adiv5_jtagdp_low_access   //процедура чтения записи на уровне JTAG , 
(jtag_dev_t *dp,//тут лежит ячейка массива с адресами клиентов на шине JTAG  (например jtag_devs[0])
 uint8_t RnW,   //чтение или запись , 
 uint16_t addr, // адресс доступа
 uint32_t value // значение записываемое
 )
{
	bool APnDP = addr & ADIV5_APnDP;
	addr &= 0xff;
	uint64_t request, response;
	uint8_t ack;
	platform_timeout timeout;

	request = ((uint64_t)value << 3) | ((addr >> 1) & 0x06) | (RnW?1:0);

	jtag_dev_write_ir
	(&jtag_proc,					//это не меняется,всегда одно и тоже - тут лежит ссылка на реализацию команд TMS,TDO,TDI,TCK
	  dp,  					//тут лежит ячейка массива с адресами клиентов на шине JTAG  (например jtag_devs[0])
	  APnDP ? IR_APACC : IR_DPACC 	//это команда для IR
	  );

	jtag_dev_shift_dr(&jtag_proc,dp, (uint8_t*)&response,(uint8_t*)&request, 35);
		ack = response & 0x07;
	

	return (uint32_t)(response >> 3);
}

static void adiv5_jtagdp_abort(jtag_dev_t *dp, uint32_t abort)
{
	uint64_t request = (uint64_t)abort << 3;
	jtag_dev_write_ir(&jtag_proc, dp, IR_ABORT);
	jtag_dev_shift_dr(&jtag_proc, dp, NULL, (const uint8_t*)&request, 35);
}


void TST (void)
{
	uint32_t data=0;
	uint16_t addr=0x8;// (страница 2-41 IHI0031C_debug_interface_as.pdf)
	
	jtag_scan(NULL);
	
	data=fw_adiv5_jtagdp_read (&jtag_devs[0],8);	
    x_out("data:",data);

	data=fw_adiv5_jtagdp_read (&jtag_devs[0],0xfc);	
    x_out("data:",data);
	
	data=fw_adiv5_jtagdp_read (&jtag_devs[1],0xfc);	
    x_out("data:",data);	
}