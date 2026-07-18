/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2026 ThreadX Shell Project
 */
/**
 * @file    cmd_wdt.c
 * @brief   `wdt` shell command (issue #4): IWDG status + starve test.
 *
 *   wdt info     show watchdog state, nominal timeout and last reset cause
 *   wdt starve   stop petting (IRQ off, spin) so the IWDG resets the board
 *
 * `wdt info` is always present: with BSP_ENABLE_IWDG off it reports
 * "disabled (build)".  `wdt starve` is a *dangerous* command AND needs the watchdog
 * actually built in, so it compiles only when both CLI_ENABLE_DANGEROUS_CMDS and
 * BSP_ENABLE_IWDG are set -- without the IWDG the spin would just hang forever with
 * nothing to reset the board.
 *
 * Clean-room design; no third-party code reused.
 */
#include "cli.h"
#include "iwdg.h"
#include "log.h"

#include "tx_api.h"          /* tx_thread_sleep */
#include "stm32h7xx_hal.h"   /* __disable_irq */

static int cmd_wdt_info(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc;
	(void)argv;

#if BSP_ENABLE_IWDG
	cli_print(sh, "IWDG:       %s\r\n",
	          iwdg_init_failed() ? "init failed (may be armed)" : "enabled");
	cli_print(sh, "  timeout:  ~3.0s typ (2.0-5.7s over LSI tol), prescaler /64 RLR=1499\r\n");
	cli_print(sh, "  pet:      ~1s by the priority-5 petter thread\r\n");
#else
	cli_print(sh, "IWDG:       disabled (build)\r\n");
#endif
	cli_print(sh, "  last reset: %s\r\n", log_reset_cause());
	return 0;
}

#if CLI_ENABLE_DANGEROUS_CMDS && BSP_ENABLE_IWDG
/*
 * starve: prove the watchdog works.  Disable interrupts (which stops the petter
 * thread) and spin -- with nothing left to refresh the IWDG, it times out (~2-6 s
 * over LSI tolerance) and resets the board.  Sleep first so the warning reaches the
 * USB CDC console before IRQs go off (same TX-drain reasoning as `reboot`).
 */
static int cmd_wdt_starve(struct cli_instance *sh, int argc, char **argv)
{
	(void)argc;
	(void)argv;

	cli_warn(sh, "wdt: starving the IWDG (IRQ off, spin) -> reset in ~timeout\r\n");
	tx_thread_sleep(50);     /* ~50 ms: let the USB CDC TX ring flush */
	__disable_irq();         /* stop the petter (and everything else) */
	for (;;)                 /* spin until the IWDG times out and resets us */
		;
	return 0;                /* unreachable */
}
#endif /* CLI_ENABLE_DANGEROUS_CMDS && BSP_ENABLE_IWDG */

CLI_SUBCMD_SET_CREATE(wdt_subcmds,
	CLI_CMD(info, NULL, "show IWDG state / timeout / last reset cause", cmd_wdt_info),
#if CLI_ENABLE_DANGEROUS_CMDS && BSP_ENABLE_IWDG
	CLI_CMD(starve, NULL, "stop petting -> IWDG resets the board (HALTS)", cmd_wdt_starve),
#endif
	CLI_SUBCMD_SET_END);

CLI_CMD_REGISTER(wdt, wdt_subcmds, "independent watchdog status / starve test",
                 NULL, 1, 0);
