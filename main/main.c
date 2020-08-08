/* Electric UI Barebones Example project

   Runs electricui-embedded and exposes interfaces over the boards main serial port (usb), and hardware uart

   MIT licenced.
*/

#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "electricui.h"

#define BUF_SIZE (1024)
#define GPIO_LED 13

#define EXTERNAL_UART UART_NUM_2
#define UART_TX 17
#define UART_RX 16

void serial_write( uint8_t *data, uint16_t len );

uint8_t   blink_enable = 1; // if the blinker should be running
uint8_t   led_state  = 0;   // track if the LED is illuminated
uint16_t  glow_time  = 200; // in milliseconds
uint32_t  led_timer = 0;

char nickname[] = "ESP-IDF Example";

eui_interface_t serial_comms = EUI_INTERFACE( &serial_write ); 

// Electric UI manages variables referenced in this array
eui_message_t tracked_variables[] = 
{
  EUI_CHAR_RO_ARRAY(  "name",  nickname ),

  EUI_UINT8(  "led_blink",  blink_enable ),
  EUI_UINT8(  "led_state",  led_state ),
  EUI_UINT16( "lit_time",   glow_time ),
};

static void echo_task(void *pvParameter)
{
    // Setup the UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        // .source_clk = UART_SCLK_APB,
    };

    uart_param_config(EXTERNAL_UART, &uart_config);
    uart_driver_install(EXTERNAL_UART, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_set_pin(EXTERNAL_UART, UART_TX, UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    eui_setup_interface( &serial_comms );
    eui_setup_identifier( "hello", 5 );

    // Provide the tracked variables to the library
    EUI_TRACK( tracked_variables );
    printf("Started uart looping\n");

    while (1) {
        int rx_read = uart_read_bytes(EXTERNAL_UART, data, BUF_SIZE, 0);

        if( rx_read > 0 )
        {
            for( uint16_t i = 0; i < rx_read; i++)
            {
                eui_parse( data[i], &serial_comms );  // Ingest a byte
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }

}

void serial_write( uint8_t *data, uint16_t len )
{
    uart_write_bytes(EXTERNAL_UART, (const char *)data, len);
}

void blink_task(void *pvParameter) 
{ 
    // Configure the onboard LED to output push-pull mode
    gpio_pad_select_gpio(GPIO_LED); 
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT); 

    led_timer = xTaskGetTickCount();

    while(1) { 

        if( blink_enable )
        {
            // Check if the LED has been on for the configured duration
            uint32_t tick_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

            if( tick_time - led_timer >= glow_time )
            {
                led_state = !led_state; //invert led state
                led_timer = tick_time;
            }    
        }

        gpio_set_level(GPIO_LED, led_state); 
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    } 
} 

void app_main(void)
{
    xTaskCreate( &echo_task,  "uart_task",  4024, NULL, 5, NULL);
    xTaskCreate( &blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL); 
}
