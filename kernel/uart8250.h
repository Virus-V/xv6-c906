/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 */

#ifndef __SERIAL_UART8250_H__
#define __SERIAL_UART8250_H__

//#include <sbi/sbi_types.h>

#include "types.h"

int uart8250_getc(void);
void uart8250_putc(char ch);
int uart8250_init(unsigned long base, uint32 in_freq, uint32 baudrate, uint32 reg_shift,
		  uint32 reg_width);

#endif
