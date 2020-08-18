/*
 * Copyright (c) 2020 Trunexa Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
 
/*
 * @file imx6_truxd01_som.h 
 *
 * @brief GPIO Defination for iMx6x SM SOMs
 *
 * @ingroup Main
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx6_pins_truxd01.h>
#include <asm/arch/sys_proto.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>

#include <asm/gpio.h>
#include <asm/arch/gpio.h>

#define BSP_VERSION             "TRUX-iMX6UL-D01-Linux4.1.15-V.1.0.0"
#define SOM_VERSION             "TRUX-iMX6UL-D01"

/*  Unused GPIO pins */

#define MX6_PAD_SNVS_TAMPER0__GPIO5_IO0        IMX_GPIO_NR(5, 0)
#define MX6_PAD_SNVS_TAMPER3__GPIO5_IO3        IMX_GPIO_NR(5, 3)
#define MX6_PAD_SNVS_TAMPER7__GPIO5_IO7        IMX_GPIO_NR(5, 7)
#define MX6_PAD_SNVS_TAMPER8__GPIO5_IO8        IMX_GPIO_NR(5, 8)
#define MX6_PAD_GPIO1_IO05__GPIO1_IO5 	       IMX_GPIO_NR(1, 5)

iomux_v3_cfg_t unused_gpio_pads_ul_ull[] = {
	MX6_PAD_SNVS_TAMPER0__GPIO5_IO00 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER3__GPIO5_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER7__GPIO5_IO07 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER8__GPIO5_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int unused_gpio_pads[] = {
	MX6_PAD_SNVS_TAMPER0__GPIO5_IO0,
	MX6_PAD_SNVS_TAMPER3__GPIO5_IO3,
	MX6_PAD_SNVS_TAMPER7__GPIO5_IO7,
	MX6_PAD_SNVS_TAMPER8__GPIO5_IO8,
};
