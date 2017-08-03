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
#define COMMAND_DISPLAY_ON      0x8F

#define DECIMAL_POINT           0x80
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

/*
DRAM_ATTR static uint8_t display_data[]={
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
*/
static uint8_t display_data[]={
    COMMAND_ADDRESS_0,
    NUMBER_1,
    NUMBER_2,
    NUMBER_3,
    NUMBER_4,
    NUMBER_5,
    NUMBER_6,
    NUMBER_7,
    NUMBER_8
};

static uint8_t numbers[] = {
    NUMBER_0,
    NUMBER_1,
    NUMBER_2,
    NUMBER_3,
    NUMBER_4,
    NUMBER_5,
    NUMBER_6,
    NUMBER_7,
    NUMBER_8,
    NUMBER_9,
};

static spi_device_handle_t spi;

void setTime(uint32_t timeInSeconds)
{

}

void setDisplayInteger(uint8_t displayNum, uint32_t value)
{
    uint8_t point_pos = 0;
    uint8_t data[7];
    int i;

    // printf("setDisplayInteger(%d, %d)!!!\n", displayNum, value);

    //make sure value has 7 digits
    if (value >= 10000000) return;

    if (value == 0) {
        //set display data
        int start = 1+4*displayNum;
        for (int j=0; j<4; j++,i--) {
            display_data[start+j] = NUMBER_0;
        }
        return;
    }

    //convert to array, LSB mode
    for(i=0; i<7; i++) {
        data[i] = value % 10;
        value/=10;
    }

    for(int c=0;c<7;c++){
        // printf("data[%d] %d!!!\n", c, data[c]);
    }

    //find 4 valid number
    for(i=6; i>3; i--) {
        if (data[i] != 0) {
            break;
        }
    }

    //find decimal position, should be 0-3
    point_pos+=6-i;
    // printf("point_pos %d!!!\n", point_pos);

    //set display data
    int start = 1+4*displayNum;
    for (int j=0; j<4; j++,i--) {
        display_data[start+j] = numbers[data[i]];
    }

    //set point
    if (point_pos > 0) {
        int pos = start + 3 - point_pos;
        display_data[pos] |= DECIMAL_POINT;
    }

    for(int c = 0; c < 9; ++c) {
        // printf("display_data %02x!!!\n", display_data[c]);
    }

}


//Send data to the TA6932. Uses spi_device_transmit, which waits until the transfer is complete.
void spi_trassfer_single_byte(const uint8_t data) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));          //Zero out the transaction
    t.length=8;                        //Command is 8 bits
    t.tx_buffer=&data;                 //command or data
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);               //Should have had no issues.
}

void spi_trassfer_2bytes(const uint8_t command, const uint8_t data) 
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

void spi_trassfer_display() 
{
    esp_err_t ret;
    spi_transaction_t trans[3];        //total 3 transactions
    spi_transaction_t *rtrans;
    memset(trans, 0, sizeof(trans));   //Zero out the transaction

    //AUTO address command
    trans[0].length=8;                                      //Command is 8 bits
    trans[0].tx_data[0]=COMMAND_DATA_MODE_ADDRESS_AUTO;     //command
    trans[0].flags=SPI_TRANS_USE_TXDATA;
    ret=spi_device_queue_trans(spi, &trans[0], portMAX_DELAY);
    assert(ret==ESP_OK);               //Should have had no issues.

    //Address + data
    trans[1].length=9*8;                     //17bytes is 17*8 bits
    trans[1].tx_buffer=display_data;          //command + data
    ret=spi_device_queue_trans(spi, &trans[1], portMAX_DELAY);
    assert(ret==ESP_OK);               //Should have had no issues.

    //Turn on display command
    trans[2].length=8;                          //Command is 8 bits
    trans[2].tx_data[0]=COMMAND_DISPLAY_ON;     //command
    trans[2].flags=SPI_TRANS_USE_TXDATA;
    ret=spi_device_queue_trans(spi, &trans[2], portMAX_DELAY);
    assert(ret==ESP_OK);               //Should have had no issues.

    //Wait for all 2 transactions to be done and get back the results.
    for (int x=0; x<3; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
    }

}

void display_numbers()
{
    uint8_t numbers[] = {
        NUMBER_0, NUMBER_1, NUMBER_2, NUMBER_3, NUMBER_4, NUMBER_5,
        NUMBER_6, NUMBER_7, NUMBER_8, NUMBER_9
    };
    spi_trassfer_single_byte(COMMAND_DATA_MODE_ADDRESS_FIX);
    for (int i=0; i< 8; i++ ) {
        uint8_t address = COMMAND_ADDRESS_0 + i;
        spi_trassfer_2bytes(address, numbers[i]);
    }
    spi_trassfer_single_byte(COMMAND_DISPLAY_ON);
}

void display_loop()
{
    while(1) {
        // display_numbers(spi);
        spi_trassfer_display();
        vTaskDelay(500/portTICK_RATE_MS);
    }
}

void display_init()
{
    printf("SPI Display!!!\n");

    esp_err_t ret;
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
        .queue_size=5,                          //We want to be able to queue 5 transactions at a time
        .flags=SPI_DEVICE_TXBIT_LSBFIRST|SPI_DEVICE_RXBIT_LSBFIRST,
    };
    //Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret==ESP_OK);
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);

    //Create task
    //xTaskCreate(&display_loop, "display_task", 4096, NULL, 5, NULL);
}
