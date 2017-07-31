/* SPI Master example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

/*
*/

#define PIN_NUM_MISO 13
#define PIN_NUM_MOSI 12
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   15

#define COMMAND_DATA_MODE_ADDRESS_AUTO    0x40
#define COMMAND_DATA_MODE_ADDRESS_FIX     0x44

#define COMMAND_ADDRESS_0       0xC0
#define COMMAND_ADDRESS_1       0xC1
#define COMMAND_ADDRESS_2       0xC2
#define COMMAND_ADDRESS_3       0xC3
#define COMMAND_ADDRESS_4       0xC4
#define COMMAND_ADDRESS_5       0xC5
#define COMMAND_ADDRESS_6       0xC6
#define COMMAND_ADDRESS_7       0xC7
#define COMMAND_ADDRESS_8       0xC8
#define COMMAND_ADDRESS_9       0xC9
#define COMMAND_ADDRESS_10      0xCA
#define COMMAND_ADDRESS_11      0xCB
#define COMMAND_ADDRESS_12      0xCC
#define COMMAND_ADDRESS_13      0xCD
#define COMMAND_ADDRESS_14      0xCE
#define COMMAND_ADDRESS_15      0xCF

#define COMMAND_DISPLAY_OFF     0x80
#define COMMAND_DISPLAY_ON      0x8C

#define NUMBER_0                0x3F
#define NUMBER_1                0x06
#define NUMBER_2                0x5B
#define NUMBER_3                0x4F
#define NUMBER_4                0x66
#define NUMBER_5                0x6D
#define NUMBER_6                0x7D
#define NUMBER_7                0x07
#define NUMBER_8                0x7F
#define NUMBER_9                0x6F

DRAM_ATTR static const uint8_t display_data[]={
    COMMAND_ADDRESS_0,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
    NUMBER_8,
};

//Send data to the TA6932. Uses spi_device_transmit, which waits until the transfer is complete.
void spi_trassfer_single_byte(spi_device_handle_t spi, const uint8_t data) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));          //Zero out the transaction
    t.length=8;                        //Command is 8 bits
    t.tx_buffer=&data;                 //command or data
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);               //Should have had no issues.
}

void spi_trassfer_2bytes(spi_device_handle_t spi, const uint8_t command, const uint8_t data) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));          //Zero out the transaction
    t.length=16;                       //2bytes is 16 bits
    t.tx_data[0]=command;              //command
    t.tx_data[1]=data;                 //data
    t.flags=SPI_TRANS_USE_TXDATA;
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);               //Should have had no issues.
}

void spi_trassfer_display(spi_device_handle_t spi) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));          //Zero out the transaction
    t.length=17*8;                      //17bytes is 17*8 bits
    t.tx_buffer=CODE;                   //command
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);               //Should have had no issues.
}

void display_numbers(spi_device_handle_t spi)
{
    uint8_t numbers[] = {
        NUMBER_0, NUMBER_1, NUMBER_2, NUMBER_3, NUMBER_4, NUMBER_5,
        NUMBER_6, NUMBER_7, NUMBER_8, NUMBER_9
    };
    spi_trassfer_single_byte(spi, COMMAND_DATA_MODE_ADDRESS_FIX);
    for (int i=0; i< 8; i++ ) {
        uint8_t address = COMMAND_ADDRESS_0 + i;
        spi_trassfer_2bytes(spi, address, numbers[i]);
    }
    spi_trassfer_single_byte(spi, COMMAND_DISPLAY_ON);
}

void app_main()
{
    printf("SPI Display!!!\n");

    esp_err_t ret;
    spi_device_handle_t spi;
    spi_bus_config_t buscfg={
        .miso_io_num=PIN_NUM_MISO,
        .mosi_io_num=PIN_NUM_MOSI,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=1000000,                //Clock out at 1MHz
        .mode=3,                                //SPI mode 3
        .spics_io_num=PIN_NUM_CS,               //CS pin
        .cs_ena_posttrans=3,                    //Keep the CS low 3 cycles after transaction, to stop slave from missing the last bit when CS has less propagation delay than CLK
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        .flags=SPI_DEVICE_TXBIT_LSBFIRST|SPI_DEVICE_RXBIT_LSBFIRST,
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
    vTaskDelay(1000/portTICK_RATE_MS);

    //Do display
    printf("Start display loop!\n");

    while(1) {
        display_numbers(spi);
        vTaskDelay(100/portTICK_RATE_MS);
    }
}
