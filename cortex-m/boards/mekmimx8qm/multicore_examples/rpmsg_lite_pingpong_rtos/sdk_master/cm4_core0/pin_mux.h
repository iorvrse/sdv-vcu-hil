/*
 * Copyright 2017-2020 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

#include "board.h"

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/* UART0_TX (number AV48), FTDI_UART0_TX */
#define BOARD_INITPINS_FTDI_UART0_TX_PERIPHERAL                   DMA__UART0   /*!< Peripheral name */
#define BOARD_INITPINS_FTDI_UART0_TX_SIGNAL                          uart_tx   /*!< Signal name */
#define BOARD_INITPINS_FTDI_UART0_TX_PIN_NAME                       UART0_TX   /*!< Routed pin name */
#define BOARD_INITPINS_FTDI_UART0_TX_PIN_FUNCTION_ID           SC_P_UART0_TX   /*!< Pin function id */
#define BOARD_INITPINS_FTDI_UART0_TX_LABEL                   "FTDI_UART0_TX"   /*!< Label */
#define BOARD_INITPINS_FTDI_UART0_TX_NAME                    "FTDI_UART0_TX"   /*!< Identifier */

/* UART0_RX (number AV50), FTDI_UART0_RX */
#define BOARD_INITPINS_FTDI_UART0_RX_PERIPHERAL                   DMA__UART0   /*!< Peripheral name */
#define BOARD_INITPINS_FTDI_UART0_RX_SIGNAL                          uart_rx   /*!< Signal name */
#define BOARD_INITPINS_FTDI_UART0_RX_PIN_NAME                       UART0_RX   /*!< Routed pin name */
#define BOARD_INITPINS_FTDI_UART0_RX_PIN_FUNCTION_ID           SC_P_UART0_RX   /*!< Pin function id */
#define BOARD_INITPINS_FTDI_UART0_RX_LABEL                   "FTDI_UART0_RX"   /*!< Label */
#define BOARD_INITPINS_FTDI_UART0_RX_NAME                    "FTDI_UART0_RX"   /*!< Identifier */

/* FLEXCAN0_RX (coord C5), BB_CAN0_RX/J20C[25] */
/* Routed pin properties */
#define BOARD_BB_CAN0_RX_PERIPHERAL                                DMA__FLEXCAN0   /*!< Peripheral name */
#define BOARD_BB_CAN0_RX_SIGNAL                                       flexcan_rx   /*!< Signal name */
#define BOARD_BB_CAN0_RX_PIN_NAME                                    FLEXCAN0_RX   /*!< Routed pin name */
#define BOARD_BB_CAN0_RX_PIN_FUNCTION_ID                        SC_P_FLEXCAN0_RX   /*!< Pin function id */
#define BOARD_BB_CAN0_RX_LABEL                             "BB_CAN0_RX/J20C[25]"   /*!< Label */
#define BOARD_BB_CAN0_RX_NAME                                       "BB_CAN0_RX"   /*!< Identifier */

/* FLEXCAN0_TX (coord H6), BB_CAN0_TX/J20C[26] */
/* Routed pin properties */
#define BOARD_BB_CAN0_TX_PERIPHERAL                                DMA__FLEXCAN0   /*!< Peripheral name */
#define BOARD_BB_CAN0_TX_SIGNAL                                       flexcan_tx   /*!< Signal name */
#define BOARD_BB_CAN0_TX_PIN_NAME                                    FLEXCAN0_TX   /*!< Routed pin name */
#define BOARD_BB_CAN0_TX_PIN_FUNCTION_ID                        SC_P_FLEXCAN0_TX   /*!< Pin function id */
#define BOARD_BB_CAN0_TX_LABEL                             "BB_CAN0_TX/J20C[26]"   /*!< Label */
#define BOARD_BB_CAN0_TX_NAME                                       "BB_CAN0_TX"   /*!< Identifier */


/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif


/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 * @param ipc scfw ipchandle.
 *
 */
void BOARD_InitPins(sc_ipc_t ipc);                         /*!< Function assigned for the core: Cortex-M4F[cm4_core0] */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
