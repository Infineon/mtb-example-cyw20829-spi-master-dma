/*******************************************************************************
* File Name:   dma.c

* Description: Provides initialization code for DMA.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2022-2023, Cypress Semiconductor Corporation (an Infineon company)
* or an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
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
#include "interface.h"


/*******************************************************************************
* Macros
*******************************************************************************/
#define TXDMA_INTERRUPT_PRIORITY   (7)


/*******************************************************************************
* Global Variables
*******************************************************************************/
extern volatile bool tx_dma_done;

/*Buffer to hold command packet to be sent to the slave by the master*/

uint8_t tx_buffer[]={0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x17};

/*Initialization configuration structure for interrupt channel*/
const cy_stc_sysint_t intTxDma_cfg =
{
    .intrSrc      = txDma_IRQ,
    .intrPriority = TXDMA_INTERRUPT_PRIORITY
};
/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void tx_dma_complete_isr(void);


/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: tx_dma_configure
********************************************************************************
* Summary:
*  This function is to configure the transmit DMA block
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
uint32_t tx_dma_configure()
 {
     cy_en_dma_status_t dma_init_status;

     /* Initialize descriptor */
     dma_init_status = Cy_DMA_Descriptor_Init(&txDma_Descriptor_0,
                       &txDma_Descriptor_0_config);
     if (dma_init_status!=CY_DMA_SUCCESS)
     {
         return INIT_FAILURE;
     }

     dma_init_status = Cy_DMA_Channel_Init(txDma_HW, txDma_CHANNEL,
                       &txDma_channelConfig);
     if (dma_init_status!=CY_DMA_SUCCESS)
     {
         return INIT_FAILURE;
     }

     /* Set source and destination for descriptor 1 */
     Cy_DMA_Descriptor_SetSrcAddress(&txDma_Descriptor_0, (uint8_t *)tx_buffer);
     Cy_DMA_Descriptor_SetDstAddress(&txDma_Descriptor_0,
             (void *)&mSPI_HW->TX_FIFO_WR);

     Cy_DMA_Enable(txDma_HW);

     /* Initialize and enable the interrupt from TxDma */
    if(CY_SYSINT_SUCCESS != Cy_SysInt_Init(&intTxDma_cfg, &tx_dma_complete_isr))
    {
        return INIT_FAILURE;
    }

     NVIC_EnableIRQ((IRQn_Type)intTxDma_cfg.intrSrc);

      /* Enable DMA interrupt source. */
     Cy_DMA_Channel_SetInterruptMask(txDma_HW, txDma_CHANNEL, CY_DMA_INTR_MASK);
     /* Enable DMA block to start descriptor execution process */

     return INIT_SUCCESS;
 }


/*******************************************************************************
* Function Name: tx_dma_complete_isr
********************************************************************************
* Summary:
*   Complete Transmit DMA callback handler.
*
* Parameters:
*  None
*
*******************************************************************************/
void tx_dma_complete_isr(void)
 {

     tx_dma_done = true;
     /* Clear tx DMA interrupt */
     Cy_DMA_Channel_ClearInterrupt(txDma_HW, txDma_CHANNEL);

 }


/*******************************************************************************
* Function Name: send_packet
********************************************************************************
* Summary:
* Enable DMA channel to transfer the data into mSPI TX-FIFO
*
* Parameters:
*  None
*
* Return:
*  void
*
*******************************************************************************/
void send_packet(void)
{
    /* Enable DMA channel to transfer 12 bytes of data from
       txBuffer into mSPI TX-FIFO */
    Cy_DMA_Channel_Enable(txDma_HW, txDma_CHANNEL);
}


/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*    None
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
     /* Disable all interrupts. */
    __disable_irq();

    /* Infinite loop. */
    while(1u) {}

}
