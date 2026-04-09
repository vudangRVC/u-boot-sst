/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Defines macros and constants for Renesas RZ/G2L pin controller pin
 * muxing functions.
 */
#ifndef __DT_BINDINGS_RZV2L_PINCTRL_H
#define __DT_BINDINGS_RZV2L_PINCTRL_H

#define RZV2L_PINS_PER_PORT	8

/*
 * Create the pin index from its bank and position numbers and store in
 * the upper 16 bits the alternate function identifier
 */
#define RZV2L_PINMUX(b, p, f)	((b) * RZV2L_PINS_PER_PORT + (p) | ((f) << 16))

#endif /* __DT_BINDINGS_RZV2L_PINCTRL_H */