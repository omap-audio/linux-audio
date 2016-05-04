#ifndef _DT_BINDINGS_TI_MCASP_H
#define _DT_BINDINGS_TI_MCASP_H

/* Source of High-frequency transmit/receive clock */
#define MCASP_CLK_HCLK_AHCLK		0 /* AHCLKX/R */
#define MCASP_CLK_HCLK_AUXCLK		1 /* Internal functional clock */
#define MCASP_CLK_HCLK_AHCLK_TXONLY	2 /* AHCLKX for TX only */
#define MCASP_CLK_HCLK_AHCLK_RXONLY	3 /* AHCLKR for RX only */
#define MCASP_CLK_HCLK_AUXCLK_TXONLY	4 /* AUXCLK for TX only */
#define MCASP_CLK_HCLK_AUXCLK_RXONLY	5 /* AUXCLK for RX only */

/* clock divider IDs */
#define MCASP_CLKDIV_AUXCLK		0 /* HCLK divider from AUXCLK */
#define MCASP_CLKDIV_BCLK		1 /* BCLK divider from HCLK */
#define MCASP_CLKDIV_BCLK_FS_RATIO	2 /* to set BCLK FS ration */
#define MCASP_CLKDIV_AUXCLK_TXONLY	3 /* AUXCLK divider for TX only */
#define MCASP_CLKDIV_AUXCLK_RXONLY	4 /* AUXCLK divider for RX only */
#define MCASP_CLKDIV_BCLK_TXONLY	5 /* BCLK divider for TX only */
#define MCASP_CLKDIV_BCLK_RXONLY	6 /* BCLK divider for RX only */
#define MCASP_CLKDIV_BCLK_FS_RATIO_TXONLY 7 /* BCLK/FS ratio for TX only */
#define MCASP_CLKDIV_BCLK_FS_RATIO_RXONLY 8 /* BCLK/FS ratio for RX only*/

#endif /* _DT_BINDINGS_TI_MCASP_H */
