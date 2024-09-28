/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the Sine wave generation using VDAC Example
 *              for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "cy_pdl.h"
#include "cybsp.h"


/*******************************************************************************
 * Macros
 ********************************************************************************/


/*******************************************************************************
 * Global Variables
 ********************************************************************************/

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
void vdac_start(void);
void dma_start(uint32_t sineWaveLUT[], uint32_t *CTDAC_VAL_NXT);


/*******************************************************************************
 * Function Definitions
 *******************************************************************************/
/* Lookup table for a sine wave in unsigned format. */
uint32_t sineWaveLUT[] = { 0x7FF, 0x880, 0x900, 0x97F, 0x9FC, 0xA78, 0xAF1,
        0xB67, 0xBD9, 0xC48, 0xCB2, 0xD18, 0xD79, 0xDD4, 0xE29, 0xE77, 0xEC0,
        0xF01, 0xF3C, 0xF6F, 0xF9A, 0xFBE, 0xFDA, 0xFEE, 0xFFA, 0xFFF, 0xFFA,
        0xFEE, 0xFDA, 0xFBE, 0xF9A, 0xF6F, 0xF3C, 0xF01, 0xEC0, 0xE77, 0xE29,
        0xDD4, 0xD79, 0xD18, 0xCB2, 0xC48, 0xBD9, 0xB67, 0xAF1, 0xA78, 0x9FC,
        0x97F, 0x900, 0x880, 0x7FF, 0x77E, 0x6FE, 0x67F, 0x602, 0x586, 0x50D,
        0x497, 0x425, 0x3B6, 0x34C, 0x2E6, 0x285, 0x22A, 0x1D5, 0x187, 0x13E,
        0x0FD, 0x0C2, 0x08F, 0x064, 0x040, 0x024, 0x010, 0x004, 0x000, 0x004,
        0x010, 0x024, 0x040, 0x064, 0x08F, 0x0C2, 0x0FD, 0x13E, 0x187, 0x1D5,
        0x22A, 0x285, 0x2E6, 0x34C, 0x3B6, 0x425, 0x497, 0x50D, 0x586, 0x602,
        0x67F, 0x6FE, 0x77E };


/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  The main function performs the following actions:
 *   1. Initializes VDAC and DMA peripherals
 *   2. DMA continuously updates VDAC value from lookup table
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();
    /*starting the vdac*/
    vdac_start();
    /*starting the dma*/
    dma_start(sineWaveLUT, (uint32_t*) &(VDAC_HW->CTDAC_VAL_NXT));
    /* No CPU operations are required because the design uses DMA for all memory transfers. */
    for (;;)
    {

    }
}


/*******************************************************************************
 * Function Name: vdac_start
 *********************************************************************************
 * Summary:
 * This function initializes and enable the CTDAC.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void vdac_start(void)
{
    cy_en_ctdac_status_t status;
    status = Cy_CTDAC_Init(VDAC_HW, &VDAC_config);
    if (CY_CTDAC_SUCCESS == status)
    {
        /* Turn on the hardware block. */
        Cy_CTDAC_Enable(VDAC_HW);
    }
    Cy_CTDAC_SetValue(VDAC_HW,0x00);
}


/*******************************************************************************
 * Function Name: dma_start
 *********************************************************************************
 * Summary:
 * This function initializes and sets up the DMA descriptor and channel.
 * Additionally, it enables both the DMA the channel.
 *
 * Parameters:
 * uint32_t sineWaveLUT[] : lookup table array
 * uint32_t *CTDAC_VAL_NXT : next memory address
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void dma_start(uint32_t sineWaveLUT[], uint32_t *CTDAC_VAL_NXT)
{
    cy_en_dma_status_t dma_init_status;

    /* Initialize descriptor 0 */
    dma_init_status = Cy_DMA_Descriptor_Init(&DMA_Descriptor_0,
            &DMA_Descriptor_0_config);
    if (CY_DMA_SUCCESS != dma_init_status )
    {
        CY_ASSERT(0);
    }
    dma_init_status = Cy_DMA_Channel_Init(DMA_HW, DMA_CHANNEL, &DMA_channelConfig);

    if (CY_DMA_SUCCESS != dma_init_status )
    {
        CY_ASSERT(0);
    }

    /* Set source address as the LUT for descriptor 0 */
    Cy_DMA_Descriptor_SetSrcAddress(&DMA_Descriptor_0, (uint32_t*) sineWaveLUT);
    /* Set destination address as the CTDAC buffer register */
    Cy_DMA_Descriptor_SetDstAddress(&DMA_Descriptor_0,
            (uint32_t*) &(VDAC_HW->CTDAC_VAL_NXT));

    /* Enable the descriptor */
    Cy_DMA_Channel_Enable(DMA_HW, DMA_CHANNEL);
    Cy_DMA_Enable(DMA_HW);

}

/* [] END OF FILE */
