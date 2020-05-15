/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    p2_chavez_correa.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK66F18.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "GPIO.h"
/* TODO: insert other definitions and declarations here. */

/* Pin definitions */
#define CS_PIN						(bit_2)
#define SCK_PIN						(bit_3)
#define MOSI_PIN					(bit_4)
#define SPI_PORT					GPIO_B

/* Half period of SCK definition in ms */
#define SCK_HALF_PERIOD				(500U)
#define MOSI_BYTE					(0xAA)

/* Event bits definitions */
#define CHIPSELECT_EVENT			(1 << 0)
#define CLK_RISE_EDGE_EVENT			(1 << 1)
#define CLK_FALL_EDGE_EVENT			(1 << 2)

/*** TASK PRIORITIES ***/
#define TASK_CHIPSELECT_PRIO      	(configMAX_PRIORITIES-4)
#define TASK_CLK_PRIO      			(configMAX_PRIORITIES-3)
#define TASK_MOSI_PRIO      		(configMAX_PRIORITIES-2)

typedef enum {LOW, HIGH} bit_t;

typedef struct
{
	EventGroupHandle_t SPI_event;
	bit_t csBit;
	bit_t mosiBit;
}parameters_task_t;


void chipSelect_task(void* param);
void clk_task(void* param);
void mosi_task(void* param);
/*
 *
 * @brief   Application entry point.
 */
int main(void) {

	gpio_pin_control_register_t pin_control_register = GPIO_MUX1;
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    GPIO_clock_gating(SPI_PORT);
    GPIO_pin_control_register(SPI_PORT, SCK_PIN, &pin_control_register);
    GPIO_pin_control_register(SPI_PORT, CS_PIN, &pin_control_register);
    GPIO_pin_control_register(SPI_PORT, MOSI_PIN, &pin_control_register);
    GPIO_data_direction_pin(SPI_PORT, GPIO_OUTPUT, SCK_PIN);
    GPIO_data_direction_pin(SPI_PORT, GPIO_OUTPUT, CS_PIN);
    GPIO_data_direction_pin(SPI_PORT, GPIO_OUTPUT, MOSI_PIN);
    GPIO_clear_pin(SPI_PORT, SCK_PIN);
    GPIO_set_pin(SPI_PORT, CS_PIN);
    GPIO_clear_pin(SPI_PORT, MOSI_PIN);



    /* Force the counter to be placed into memory. */
    volatile static int i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
        i++ ;
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile ("nop");
    }
    return 0 ;
}


void chipSelect_task(void* param)
{

	parameters_task_t parameters_task = *((parameters_task_t*) param);

	for(;;)
	{
		if (LOW == parameters_task.csBit)
		{
			xEventGroupSetBits(parameters_task.SPI_event, CHIPSELECT_EVENT);
		}
		else
		{
			xEventGroupClearBits(parameters_task.SPI_event, CHIPSELECT_EVENT);
		}
		GPIO_tooglePIN(SPI_PORT, CS_PIN);
	}
}
void clk_task(void* param)
{
	static uint8_t newBitMask = (1<<0);
	static bit_t clkState = LOW;
	parameters_task_t parameters_task = *((parameters_task_t*) param);


	for(;;)
	{
		xEventGroupWaitBits(parameters_task.SPI_event, CHIPSELECT_EVENT, pdFALSE, pdTRUE, portMAX_DELAY);
		switch (clkState)
		{
			case LOW:

				if (newBitMask == 0x80)
				{
					newBitMask = 1;
					parameters_task.mosiBit = (MOSI_BYTE && newBitMask);
				}
				else
				{
					parameters_task.mosiBit = (MOSI_BYTE && newBitMask);
					newBitMask <<= 1;
				}

				xEventGroupSetBits(parameters_task.SPI_event, CLK_FALL_EDGE_EVENT);
				clkState = HIGH;
			break;

			case HIGH:
				xEventGroupSetBits(parameters_task.SPI_event, CLK_RISE_EDGE_EVENT);
				clkState = LOW;
			break;

			default:
			break;
		}
		GPIO_tooglePIN(SPI_PORT, SCK_PIN);
		vTaskDelay(pdMS_TO_TICKS(SCK_HALF_PERIOD));
	}

}
void mosi_task(void* param)
{
	parameters_task_t parameters_task = *((parameters_task_t*) param);
	static lastBit = LOW;
	bit_t bitToSend = LOW;
	for(;;)
	{
		xEventGroupWaitBits(parameters_task.SPI_event, CLK_FALL_EDGE_EVENT, pdTRUE, pdTRUE, portMAX_DELAY);
		bitToSend = parameters_task.mosiBit;
		xEventGroupWaitBits(parameters_task.SPI_event, CLK_RISE_EDGE_EVENT, pdTRUE, pdTRUE, portMAX_DELAY);
		if (lastBit != bitToSend)
		{
			GPIO_tooglePIN(SPI_PORT, CS_PIN);
			lastBit = bitToSend;
		}

	}
}
