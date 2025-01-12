/*****************************************************************************
 * File name: radar_task.c
 *
 * Description: This file uses RadarSensing library APIs to demonstrate
 * reading data from radar with specific configuration.
 *
 * Related Document: See README.md
 *
 * ===========================================================================
 * Copyright (C) 2022 Infineon Technologies AG. All rights reserved.
 * ===========================================================================
 *
 * ===========================================================================
 * Infineon Technologies AG (INFINEON) is supplying this file for use
 * exclusively with Infineon's sensor products. This file can be freely
 * distributed within development tools and software supporting such
 * products.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON
 * WHATSOEVER.
 * ===========================================================================
 */

/* Header file from system */
#include <malloc.h>
#include <inttypes.h>
#include <stdio.h>

/* Header file includes */
#include "cybsp.h"
#include "cyhal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "rtos_artifacts.h"
#include "resource_map.h"

/* Header file for local task */
#include "radar_config_task.h"

#include "radar_task.h"
#include "udp_server.h"
#include "xensiv_bgt60trxx_mtb.h"

#define XENSIV_BGT60TRXX_CONF_IMPL
#include <radar_settings.h>
#include "ifx_sensor_dsp.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define PIN_XENSIV_BGT60TRXX_SPI_SCLK       CYBSP_SPI_CLK
#define PIN_XENSIV_BGT60TRXX_SPI_MOSI       CYBSP_SPI_MOSI
#define PIN_XENSIV_BGT60TRXX_SPI_MISO       CYBSP_SPI_MISO
#define PIN_XENSIV_BGT60TRXX_SPI_CSN        CYBSP_SPI_CS
#define PIN_XENSIV_BGT60TRXX_IRQ            CYBSP_GPIO10
#define PIN_XENSIV_BGT60TRXX_RSTN           CYBSP_GPIO11
#define PIN_XENSIV_BGT60TRXX_LDO_EN         CYBSP_GPIO5

#define XENSIV_BGT60TRXX_SPI_FREQUENCY      (25000000UL)

#define NUM_SAMPLES_PER_FRAME               (XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP *\
                                             XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME *\
                                             XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS)

#define NUM_CHIRPS_PER_FRAME                XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME
#define NUM_SAMPLES_PER_CHIRP               XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP
#define NUM_RANGE_BINS 						(NUM_SAMPLES_PER_CHIRP / 2)
#define NUM_DOPPLER_BINS					NUM_CHIRPS_PER_FRAME

/* Interrupt priorities */
#define GPIO_INTERRUPT_PRIORITY             (6)

/* RTOS tasks */
#define MAIN_TASK_NAME                      "radar_task"
#define MAIN_TASK_STACK_SIZE                (configMINIMAL_STACK_SIZE * 2)
#define MAIN_TASK_PRIORITY                  (configMAX_PRIORITIES - 1)

#define PREPROCESSING_TASK_NAME                "preprocessing_task"
#define PREPROCESSING_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE * 20)
#define PREPROCESSING_TASK_PRIORITY            (configMAX_PRIORITIES - 2)


/*******************************************************************************
 * Global Variables
 ******************************************************************************/
TaskHandle_t radar_task_handle = NULL;
TaskHandle_t preprocessing_task_handler = NULL;
TimerHandle_t timer_handler = NULL;

static cyhal_spi_t spi_obj;
static xensiv_bgt60trxx_mtb_t bgt60_obj;
static uint16_t bgt60_buffer[NUM_SAMPLES_PER_FRAME] __attribute__((aligned(2)));

static uint32_t frame_num = 0;
static uint8_t bgt60_frame_buffer[(NUM_SAMPLES_PER_FRAME/2 * sizeof(float32_t))] __attribute__((aligned(2)));
static publisher_data_t udp_data = {
    .data = bgt60_frame_buffer,
    .cmd =  1,
    .length = (NUM_SAMPLES_PER_FRAME/2 * sizeof(float32_t))
};
static publisher_data_t * publisher_msg = &udp_data;

static uint16_t packet_num = 0;
static uint8_t bgt60_packet_buffer[((NUM_SAMPLES_PER_FRAME/2 * sizeof(float32_t))/12) + 8] __attribute__((aligned(2)));
static publisher_data_t udp_packet = {
    .data = bgt60_packet_buffer,
    .cmd =  1,
    .length = ((NUM_SAMPLES_PER_FRAME/2 * sizeof(float32_t))/12) + 8
};
static publisher_data_t * publisher_packet = &udp_packet;

static bool test_mode = false;

float32_t temp_frame[XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS][NUM_SAMPLES_PER_FRAME/XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS];
float32_t frame[NUM_SAMPLES_PER_FRAME];
cfloat32_t range[NUM_RANGE_BINS * XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME ];
cfloat32_t doppler[XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS][NUM_RANGE_BINS * NUM_DOPPLER_BINS ];
int counter = 0;
int max_prob_id = 0;
float max_prob = 0;
float prob = 0;


#define MAX_COUNT UINT64_MAX
int count = 0u;
volatile uint64_t count_time=0;

/*******************************************************************************
* Function Name: xensiv_bgt60trxx_interrupt_handler
********************************************************************************
* Summary:
* This is the interrupt handler to react on sensor indicating the availability
* of new data
*    1. Notifies radar task that there is an interrupt from the sensor
*
* Parameters:
*  args : pointer to pass parameters to callback
*  event: gpio event
* Return:
*  none
*
*******************************************************************************/
#if defined(CYHAL_API_VERSION) && (CYHAL_API_VERSION >= 2)
static void xensiv_bgt60trxx_interrupt_handler(void *args, cyhal_gpio_event_t event)
#else
static void xensiv_bgt60trxx_interrupt_handler(void *args, cyhal_gpio_irq_event_t event)
#endif
{
    CY_UNUSED_PARAMETER(args);
    CY_UNUSED_PARAMETER(event);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(radar_task_handle, &xHigherPriorityTaskWoken);

    /* Context switch needed? */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
* Function Name: init_sensor
********************************************************************************
* Summary:
* This function configures the SPI interface, initializes radar and interrupt
* service routine to indicate the availability of radar data.
*
* Parameters:
*  void
*
* Return:
*  Success or error
*
*******************************************************************************/
static int32_t init_sensor(void)
{
    if (cyhal_spi_init(&spi_obj,
                       PIN_XENSIV_BGT60TRXX_SPI_MOSI,
                       PIN_XENSIV_BGT60TRXX_SPI_MISO,
                       PIN_XENSIV_BGT60TRXX_SPI_SCLK,
                       NC,
                       NULL,
                       8,
                       CYHAL_SPI_MODE_00_MSB,
                       false) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: cyhal_spi_init failed\n");
        return RESULT_ERROR;
    }

    /* Reduce drive strength to improve EMI */
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_MOSI), CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CYHAL_GET_PIN(PIN_XENSIV_BGT60TRXX_SPI_SCLK), CY_GPIO_DRIVE_1_8);

    /* Set the data rate to 25 Mbps */
    if (cyhal_spi_set_frequency(&spi_obj, XENSIV_BGT60TRXX_SPI_FREQUENCY) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: cyhal_spi_set_frequency failed\n");
        return RESULT_ERROR;
    }

    /* Enable LDO */
    if (cyhal_gpio_init(PIN_XENSIV_BGT60TRXX_LDO_EN,
                        CYHAL_GPIO_DIR_OUTPUT,
                        CYHAL_GPIO_DRIVE_STRONG,
                        true) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: LDO_EN cyhal_gpio_init failed\n");
        return RESULT_ERROR;
    }

    /* Wait LDO stable */
    (void)cyhal_system_delay_ms(5);

    if (xensiv_bgt60trxx_mtb_init(&bgt60_obj,
                                  &spi_obj,
                                  PIN_XENSIV_BGT60TRXX_SPI_CSN,
                                  PIN_XENSIV_BGT60TRXX_RSTN,
                                  register_list,
                                  XENSIV_BGT60TRXX_CONF_NUM_REGS) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: xensiv_bgt60trxx_mtb_init failed\n");
        return RESULT_ERROR;
    }

    if (xensiv_bgt60trxx_mtb_interrupt_init(&bgt60_obj,
                                            NUM_SAMPLES_PER_FRAME,
                                            PIN_XENSIV_BGT60TRXX_IRQ,
                                            GPIO_INTERRUPT_PRIORITY,
                                            xensiv_bgt60trxx_interrupt_handler,
                                            NULL) != CY_RSLT_SUCCESS)
    {
        printf("ERROR: xensiv_bgt60trxx_mtb_interrupt_init failed\n");
        return RESULT_ERROR;
    }

    return RESULT_SUCCESS;
}
/*******************************************************************************
* Function Name: init_leds
********************************************************************************
* Summary:
* This function initializes the GPIOs for LEDs and set them to off state.
* Parameters:
*  void
*
* Return:
*  Success or error
*
*******************************************************************************/
static int32_t init_leds(void)
{

    if(cyhal_gpio_init(LED_RGB_RED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false)!= CY_RSLT_SUCCESS)
    {
        printf("ERROR: GPIO initialization for LED_RGB_RED failed\n");
        return -1;
    }

    if( cyhal_gpio_init(LED_RGB_GREEN, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false)!= CY_RSLT_SUCCESS)
    {
        printf("ERROR: GPIO initialization for LED_RGB_GREEN failed\n");
        return -1;
    }

    if( cyhal_gpio_init(LED_RGB_BLUE, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false)!= CY_RSLT_SUCCESS)
    {
        printf("ERROR: GPIO initialization for LED_RGB_BLUE failed\n");
        return -1;
    }

    return 0;
}
/*******************************************************************************
* Function Name: timer_callbak
********************************************************************************
* Summary:
* This is the timer_callback which toggles the LED
*
* Parameters:
*  void
*
* Return:
*  none
*
*******************************************************************************/
static void timer_callbak(TimerHandle_t xTimer)
{
    (void)xTimer;

#ifdef TARGET_CYSBSYSKIT_DEV_01
    cyhal_gpio_toggle(CYBSP_USER_LED);
#endif
}
/*******************************************************************************
 * Function Name: test_radar_spi_data_1rx
 *******************************************************************************
 * Summary:
 *  This function takes radar input data and verifies it for 1 rx antenna with
 *  generated test data.
 *
 * Parameters:
 *   samples : pointer to hold frame buffer containing samples
 *
 * Return:
 *   error
 *
 ******************************************************************************/
static void test_radar_spi_data_1rx(const uint16_t *samples)
{
    static uint32_t frame_idx = 0;
    static uint16_t test_word = XENSIV_BGT60TRXX_INITIAL_TEST_WORD;

    /* Check received data */
    for (int32_t sample_idx = 0; sample_idx < NUM_SAMPLES_PER_FRAME; ++sample_idx)
    {
        /*if ((sample_idx % XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS) == 0)
        {
            if (test_word != samples[sample_idx])
            {
                printf("Frame %" PRIu32 " error detected. "
                       "Expected: %" PRIu16 ". "
                       "Received: %" PRIu16 "\n",
                       frame_idx, test_word, samples[sample_idx]);
                CY_ASSERT(false);
            }
        }*/
        printf("Frame %" PRIu32 " sample " PRIu32 " . "
                                           "Expected: %f . "
                                           "Received: %f \n",
                                           frame_idx, sample_idx, test_word, samples[sample_idx]);
        // Generate next test_word
        test_word = xensiv_bgt60trxx_get_next_test_word(test_word);
    }

    sprintf((char *)publisher_msg->data, "Frame %" PRIu32 " received correctly", frame_idx);
    publisher_msg->length = strlen((const char *) publisher_msg->data);

    /* Send message back to publish queue. */
    xQueueSendToBack(radar_data_queue, &publisher_msg, 0 );

    frame_idx++;
}

/*******************************************************************************
 * Function Name: radar_task
 *******************************************************************************
 * Summary:
 *
 *   Initializes radar sensor, create configuration task and continuously
 *   process data acquired from radar.
 *
 * Parameters:
 *   pvParameters: thread
 *
 * Return:
 *   none
 ******************************************************************************/
void radar_task(void *pvParameters)
{

    printf(" 'radar_task in'\n\n");
    (void)pvParameters;

    printf("Radar Raw Data Shape: (%d, %d, %d) \r\n", XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS, NUM_CHIRPS_PER_FRAME, NUM_SAMPLES_PER_CHIRP);

    timer_handler = xTimerCreate("timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, timer_callbak);
	if (timer_handler == NULL)
	{
		CY_ASSERT(0);
	}

	if (xTimerStart(timer_handler, 0) != pdPASS)
	{
		CY_ASSERT(0);
	}

    if (xTaskCreate(preprocessing_task, PREPROCESSING_TASK_NAME, PREPROCESSING_TASK_STACK_SIZE, NULL, PREPROCESSING_TASK_PRIORITY, &preprocessing_task_handler) != pdPASS)
    {
        CY_ASSERT(0);
    }

    if (init_sensor() != RESULT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    if (init_leds () != 0)
    {
        CY_ASSERT(0);
    }


    /**
     * Create task for radar configuration. Configuration parameters come from
     * udp client task.
     */
    if (pdPASS != xTaskCreate(radar_config_task,
                              RADAR_CONFIG_TASK_NAME,
                              RADAR_CONFIG_TASK_STACK_SIZE,
                              NULL,
                              RADAR_CONFIG_TASK_PRIORITY,
                              &radar_config_task_handle))
    {
        printf("Failed to create Radar config task!\n");
        CY_ASSERT(0);
    }

    printf("Radar device initialized successfully. Waiting for start from UDP client...\n\n");

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xensiv_bgt60trxx_get_fifo_data(&bgt60_obj.dev,
        								   bgt60_buffer,
                                           NUM_SAMPLES_PER_FRAME) == XENSIV_BGT60TRXX_STATUS_OK)
        {
            if(!test_mode)
            {
                /* Data preprocessing */
            	uint16_t *bgt60_buffer_ptr = &bgt60_buffer[0];
                float32_t *frame_ptr = &frame[0];
                for (int32_t sample = 0; sample < NUM_SAMPLES_PER_FRAME; ++sample)
                {
                	*frame_ptr++ = ((float32_t)(*bgt60_buffer_ptr++) / 4095.0F);
                }
                /* Tell processing task to take over */
                xTaskNotifyGive(preprocessing_task_handler);
            }
            else
            {
                test_radar_spi_data_1rx(&bgt60_buffer);
            }

        }
    }
}

/*******************************************************************************
* Function Name: preprocessing_task
********************************************************************************
* Summary:
* This is the data preprocessing task.
*    1. Reorder fetch data
*    2. DopplerMap + ComplexToReal
*    3. Normalize into range [0, 1]
*    4. Reshape the processed frames to H x W x C to meet Tensorflow model input form
*
* Parameters:
*  void
*
* Return:
*  None
*
*******************************************************************************/
void preprocessing_task(void *pvParameters)
{
    (void)pvParameters;

    for(;;)
    {
        /* Wait for frame data available to process */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Reorder the fetch radar data into Rx x Chirps x Samples
        for (int i = 0; i < XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS; i++)
		{
        	for (int j = 0; j < NUM_CHIRPS_PER_FRAME; j++)
        	{
        		for (int k = 0; k < NUM_SAMPLES_PER_CHIRP; k++)
        		{
        			temp_frame[i][ j * NUM_SAMPLES_PER_CHIRP + k] = frame[j * (NUM_SAMPLES_PER_CHIRP * XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS) + k * XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS + i];
        		}
        	}
		}

        // Process complex doppler per channel
        for(int k = 0; k < XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS; k++)
        {
            float32_t* frame_ptr = &temp_frame[k][0];
        	// Range FFT
        	if(ifx_range_fft_f32(frame_ptr, range, true, NULL, NUM_SAMPLES_PER_CHIRP, NUM_CHIRPS_PER_FRAME) != IFX_SENSOR_DSP_STATUS_OK)
        	{
        		printf("Range FFT failed\r\n");
				abort();
        	}
        	// Range Doppler
        	if(ifx_doppler_cfft_f32(range, doppler[k], false, NULL, NUM_RANGE_BINS, NUM_DOPPLER_BINS) != IFX_SENSOR_DSP_STATUS_OK)
			{
				printf("Range Doppler failed\r\n");
				abort();
			}
        	ifx_shift_cfft_f32(doppler[k], NUM_DOPPLER_BINS, NUM_RANGE_BINS);

        	frame_ptr += NUM_CHIRPS_PER_FRAME * NUM_SAMPLES_PER_CHIRP;
        }


        // RDM of 3 Antennas is ready to be used.


		int sample_idx = 0;
		for (int k = 0; k < XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS; k++)
		{
			for (int i = 0; i < NUM_DOPPLER_BINS; i++)
			{
				for (int j = 0; j < NUM_RANGE_BINS; j++)
				{
					float32_t value = cabs(doppler[k][i * NUM_DOPPLER_BINS + j]);
					//printf(" %f \n", value);
					uint8_t* value_ptr = (uint8_t*)&value;
					for (int b = 0; b < sizeof(float32_t); b++)
					{
						publisher_msg->data[sample_idx + b] = value_ptr[b];
					}
					sample_idx += sizeof(float32_t); // 4 bytes increment
				}
			}
		}

		/*		printf("Contenuto dell'array data:\n");
				for (int i = 0; i < publisher_msg->length; i++)
				{
					printf("%02x ", publisher_msg->data[i]);
					if ((i + 1) % 16 == 0)
					{
						printf("\n"); // Aggiungi una nuova riga ogni 16 byte per leggibilità
					}
				}
				printf("\n");*/




		// Calculating size of each packet
		uint16_t packet_size = publisher_msg->length / 12;

		// Create packets dividing the entire frame and send them
		packet_num = 0;
		for (int i = 0; i < 12; i++)
		{
			publisher_packet->data[0] = RADAR_DATA_COMMAND;
			publisher_packet->data[1] = DUMMY_BYTE;
			publisher_packet->data[2] = (uint8_t)(frame_num & 0x000000ff);
			publisher_packet->data[3] = (uint8_t)((frame_num & 0x0000ff00) >> 8);
			publisher_packet->data[4] = (uint8_t)((frame_num & 0x00ff0000) >> 16);
			publisher_packet->data[5] = (uint8_t)((frame_num & 0xff000000) >> 24);
			publisher_packet->data[6] = (uint8_t)(packet_num & 0x000000ff);
			publisher_packet->data[7] = (uint8_t)((packet_num & 0x0000ff00) >> 8);
			uint8_t* packet_start = &publisher_msg->data[i * packet_size];
			uint16_t current_packet_size = (i == 11) ? (publisher_msg->length - i * packet_size) : packet_size;
			publisher_packet->length = current_packet_size + 8 ;
			memcpy(&publisher_packet->data[8], packet_start, current_packet_size);
			/* Send message back to publish queue. */
			xQueueSendToBack(radar_data_queue, &publisher_packet, 0 );
			packet_num++;
		    /* Wait */
		    (void)cyhal_system_delay_ms(5);

		}



		printf("# Frame %" PRIu32 " sent with config: #SPF %" PRIu32 " . "
				   " #Total frame message length: %" PRIu32 "\n\n\n",
				   frame_num, XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS*NUM_DOPPLER_BINS*NUM_RANGE_BINS, publisher_msg->length);

		frame_num++;

    }

}

/*******************************************************************************
 * Function Name: radar_start
 *******************************************************************************
 * Summary:
 *   to start/stop radar device.
 *
 * Parameters:
 *   start : start/stop value for radar device
 *
 * Return:
 *   error
 ******************************************************************************/
int32_t radar_start(bool start)
{
    if (xensiv_bgt60trxx_start_frame(&bgt60_obj.dev, start) != XENSIV_BGT60TRXX_STATUS_OK)
    {
        return RESULT_ERROR;
    }

    return RESULT_SUCCESS;
}

/*******************************************************************************
 * Function Name: radar_enable_test_mode
 *******************************************************************************
 * Summary:
 *   Starting/stopping radar test mode.
 *
 * Parameters:
 *   start : start/stop value for radar device in test mode
 *
 * Return:
 *   error
 ******************************************************************************/
int32_t radar_enable_test_mode(bool start)
{
    test_mode = start;

    /* Enable sensor data test mode. The data received on antenna RX1 will be overwritten by
       a deterministic sequence of data generated by the test pattern generator */
    if (xensiv_bgt60trxx_enable_data_test_mode(&bgt60_obj.dev,true) != XENSIV_BGT60TRXX_STATUS_OK)
    {
        return RESULT_ERROR;
    }

    return RESULT_SUCCESS;
}


/* [] END OF FILE */
