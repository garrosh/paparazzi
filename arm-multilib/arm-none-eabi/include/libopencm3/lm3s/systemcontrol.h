/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>
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

#ifndef LM3S_SYSTEMCONTROL_H
#define LM3S_SYSTEMCONTROL_H

#include <libopencm3/cm3/common.h>

#define SYSTEMCONTROL_DID0		MMIO32(SYSTEMCONTROL_BASE + 0x000)
#define SYSTEMCONTROL_DID1		MMIO32(SYSTEMCONTROL_BASE + 0x004)
#define SYSTEMCONTROL_DC0		MMIO32(SYSTEMCONTROL_BASE + 0x008)
#define SYSTEMCONTROL_DC1		MMIO32(SYSTEMCONTROL_BASE + 0x010)
#define SYSTEMCONTROL_DC2		MMIO32(SYSTEMCONTROL_BASE + 0x014)
#define SYSTEMCONTROL_DC3		MMIO32(SYSTEMCONTROL_BASE + 0x018)
#define SYSTEMCONTROL_DC4		MMIO32(SYSTEMCONTROL_BASE + 0x01C)
#define SYSTEMCONTROL_DC5		MMIO32(SYSTEMCONTROL_BASE + 0x020)
#define SYSTEMCONTROL_DC6		MMIO32(SYSTEMCONTROL_BASE + 0x024)
#define SYSTEMCONTROL_DC7		MMIO32(SYSTEMCONTROL_BASE + 0x028)
#define SYSTEMCONTROL_PBORCTL		MMIO32(SYSTEMCONTROL_BASE + 0x030)
#define SYSTEMCONTROL_LDORCTL		MMIO32(SYSTEMCONTROL_BASE + 0x034)
#define SYSTEMCONTROL_SRCR0		MMIO32(SYSTEMCONTROL_BASE + 0x040)
#define SYSTEMCONTROL_SRCR1		MMIO32(SYSTEMCONTROL_BASE + 0x044)
#define SYSTEMCONTROL_SRCR2		MMIO32(SYSTEMCONTROL_BASE + 0x048)
#define SYSTEMCONTROL_RIS		MMIO32(SYSTEMCONTROL_BASE + 0x050)
#define SYSTEMCONTROL_IMC		MMIO32(SYSTEMCONTROL_BASE + 0x054)
#define SYSTEMCONTROL_MISC		MMIO32(SYSTEMCONTROL_BASE + 0x058)
#define SYSTEMCONTROL_RESC		MMIO32(SYSTEMCONTROL_BASE + 0x05C)
#define SYSTEMCONTROL_RCC		MMIO32(SYSTEMCONTROL_BASE + 0x060)
#define SYSTEMCONTROL_PLLCFG		MMIO32(SYSTEMCONTROL_BASE + 0x064)
#define SYSTEMCONTROL_GPIOHBCTL		MMIO32(SYSTEMCONTROL_BASE + 0x06C)
#define SYSTEMCONTROL_RCC2		MMIO32(SYSTEMCONTROL_BASE + 0x070)
#define SYSTEMCONTROL_MOSCCTL		MMIO32(SYSTEMCONTROL_BASE + 0x07C)
#define SYSTEMCONTROL_RCGC0		MMIO32(SYSTEMCONTROL_BASE + 0x100)
#define SYSTEMCONTROL_RCGC1		MMIO32(SYSTEMCONTROL_BASE + 0x104)
#define SYSTEMCONTROL_RCGC2		MMIO32(SYSTEMCONTROL_BASE + 0x108)
#define SYSTEMCONTROL_SCGC0		MMIO32(SYSTEMCONTROL_BASE + 0x110)
#define SYSTEMCONTROL_SCGC1		MMIO32(SYSTEMCONTROL_BASE + 0x114)
#define SYSTEMCONTROL_SCGC2		MMIO32(SYSTEMCONTROL_BASE + 0x118)
#define SYSTEMCONTROL_DCGC0		MMIO32(SYSTEMCONTROL_BASE + 0x120)
#define SYSTEMCONTROL_DCGC1		MMIO32(SYSTEMCONTROL_BASE + 0x124)
#define SYSTEMCONTROL_DCGC2		MMIO32(SYSTEMCONTROL_BASE + 0x128)
#define SYSTEMCONTROL_DSLPCLKCFG	MMIO32(SYSTEMCONTROL_BASE + 0x144)

#endif

