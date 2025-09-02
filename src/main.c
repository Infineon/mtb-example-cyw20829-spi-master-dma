/*******************************************************************************
* File Name:   main.c
*
* Description: This example demonstrates the use of the
*              SPI Serial Communication Block (SCB) resource for CYW920829
*              MCU in master mode using DMA
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2021-2022, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "spi_master.h"
#include "interface.h"
#include "dma.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
*******************************************************************************/


/*******************************************************************************
* Global Variables
*******************************************************************************/
volatile bool tx_dma_done;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*    1. System entrance point, This function configures and initializes the SPI
*    2. SPI master sends commands to the slave to turn LED ON or OFF,
*       every one second.
*    3. SPI master polls for the data transfer completion.
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    int status =0;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Local command variable */
    uint8_t cmd = CYBSP_LED_STATE_OFF;

    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);


    printf("\x1b[2J\x1b[;H");
    printf("================================================\r\n");
    printf("================ DMA SPI MASTER ================\r\n");
    printf("================================================\r\n");

    /* Initialize the SPI Master */
    status = master_init();

    if (INIT_FAILURE == status)
    {
        /* NOTE: This function will block the CPU forever */
        handle_error();
    }
    status = tx_dma_configure( );

       if (INIT_FAILURE == status)
       {
           /* NOTE: This function will block the CPU forever */
           handle_error();
       }
       /* Enable global interrupts */
       __enable_irq();

       printf("Data is being transferred to the SPI slave... \r\n");

    for (;;)
    {

        /* Toggle the current LED state */
        cmd = (cmd == CYBSP_LED_STATE_ON) ? CYBSP_LED_STATE_OFF : CYBSP_LED_STATE_ON;

        /* Form the command packet */
        tx_buffer[4] = cmd;

        /* Pass the command packet to the master along with the number of bytes to be
         * sent to the slave.*/
        send_packet();

        /* Wait until master complete the transfer */
        while (false == tx_dma_done) {}
        tx_dma_done = false;
        cyhal_system_delay_ms(CMD_DELAY_MS);

    }
}



/* [] END OF FILE */
