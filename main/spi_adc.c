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
#include "esp_log.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

/*
*/
#define TAG = "ADC";

#define PIN_NUM_DATA 23
#define PIN_NUM_CLK  18

#define DATA_PIN_FUNC_SPI   FUNC_GPIO23_VSPID
#define DATA_PIN_FUNC_GPIO  FUNC_GPIO23_GPIO23

#define REFO_ON               0x0<<6
#define REFO_OFF              0x1<<6

#define SPEED_SEL_10HZ        0x0<<4
#define SPEED_SEL_40HZ        0x1<<4
#define SPEED_SEL_640HZ       0x2<<4
#define SPEED_SEL_1280HZ      0x3<<4

#define PGA_SEL_1             0x0<<2
#define PGA_SEL_2             0x1<<2
#define PGA_SEL_64            0x2<<2
#define PGA_SEL_128           0x3<<2

#define CH_SEL_A              0x0
#define CH_SEL_B              0x1
#define CH_SEL_TEMP           0x2
#define CH_SEL_SHORT          0x3

#define VREF_HALF             1650000       //voltage in uv
#define CONVERT_INPUT(X)      ((VREF_HALF*X)/0x7FFFFF)           //8388607 make sure we can read 4 digit after decimal

static const uint8_t channels[]={
    0x00|REFO_ON|SPEED_SEL_40HZ|PGA_SEL_1|CH_SEL_A,
    0x00|REFO_ON|SPEED_SEL_40HZ|PGA_SEL_1|CH_SEL_B,
};

//The semaphore indicating the data is ready.
static SemaphoreHandle_t rdySem = NULL;
static spi_device_handle_t spi;
static uint8_t channelNum = 0;
uint32_t channel_values[2] = {0,0};

/*
This ISR is called when the data line goes low.
*/
static void IRAM_ATTR gpio_data_isr_handler(void* arg)
{
    //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
    //looking at the time between interrupts and refusing any interrupt too close to another one.
    static uint32_t lastDataReadyTime;
    uint32_t currtime=xthal_get_ccount();
    uint32_t diff=currtime-lastDataReadyTime;
    if (diff<1200000) return; //ignore everything <5ms after an earlier irq
    lastDataReadyTime=currtime;
    //Give the semaphore.
    BaseType_t mustYield=false;
    xSemaphoreGiveFromISR(rdySem, &mustYield);
    if (mustYield) portYIELD_FROM_ISR();
}

void gpio_spi_switch(uint8_t mode)
{
    if (mode == DATA_PIN_FUNC_GPIO) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_DATA], mode);
        gpio_intr_enable(PIN_NUM_DATA);
    }else if (mode == DATA_PIN_FUNC_SPI) {
        gpio_intr_disable(PIN_NUM_DATA);
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_DATA], mode);
    }
}

void spi_init()
{
    printf("%s: spi_init !!!\n", TAG);
    esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=-1,
        .mosi_io_num=PIN_NUM_DATA,
        .sclk_io_num=PIN_NUM_CLK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=1000000,                //Clock out at 1MHz
        .mode=1,                                //SPI mode 1
        .spics_io_num=-1,                       //CS pin
        .queue_size=2,                          //We want to be able to queue 2 transactions at a time
        .flags=SPI_DEVICE_3WIRE|SPI_DEVICE_HALFDUPLEX,
    };

    //Initialize the SPI bus, no DMA
    ret=spi_bus_initialize(VSPI_HOST, &buscfg, 0);
    assert(ret==ESP_OK);
    //Attach the sensor to the SPI bus
    ret=spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
}

void gpio_init()
{
    printf("%s: gpio_init !!!\n", TAG);
    //GPIO config for the data line.
    gpio_config_t io_conf={
        .intr_type=GPIO_INTR_NEGEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_down_en=1,
        .pin_bit_mask=(1<<PIN_NUM_DATA)
    };

    //Set up handshake line interrupt.
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    //gpio_set_intr_type(PIN_NUM_DATA, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(PIN_NUM_DATA, gpio_data_isr_handler, NULL);
}

void adc_loop()
{
    esp_err_t ret;
    spi_transaction_t trans[2];
    spi_transaction_t *rtrans;
    uint8_t oldChannel = 0; 
    while(1) {
        //Wait until data is ready
        xSemaphoreTake( rdySem, portMAX_DELAY );
        printf("%s: data is ready!\n", TAG);

        //Disable gpio and enable spi
        gpio_spi_switch(DATA_PIN_FUNC_SPI);

        memset(trans, 0, sizeof(trans));
        trans[0].rxlength=29;
        trans[0].flags=SPI_TRANS_USE_RXDATA;
        ret=spi_device_queue_trans(spi, &trans[0], portMAX_DELAY);
        assert(ret==ESP_OK);

        oldChannel = channelNum;
        channelNum = channelNum==1?0:1;
        trans[1].length=17;
        trans[1].tx_data[0]=0xCA;
        trans[1].tx_data[1]=channels[channelNum];
        trans[1].tx_data[2]=0x00;
        trans[1].flags=SPI_TRANS_USE_TXDATA;
        ret=spi_device_queue_trans(spi, &trans[1], portMAX_DELAY);
        assert(ret==ESP_OK);

        //Wait for all 2 transactions to be done and get back the results.
        for (int x=0; x<2; x++) {
            ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
            assert(ret==ESP_OK);
            if ( x==0 ) {
                uint32_t value = rtrans->rx_data[0]<<16|rtrans->rx_data[1]<<8|rtrans->rx_data[2];
                channel_values[oldChannel] = CONVERT_INPUT(value);
                printf("%d: 0x%x%x%x%x\n", oldChannel, rtrans->rx_data[0], rtrans->rx_data[1], rtrans->rx_data[2], rtrans->rx_data[3]);
            }
        }

        //Enable gpio again and wait for data
        gpio_spi_switch(DATA_PIN_FUNC_GPIO); 
    }
}

void adc_init()
{
    printf("%s: CS1238 start!!!\n", TAG);

    //Create the semaphore.
    rdySem=xSemaphoreCreateBinary();

    //SPI config
    spi_init();

    //GPIO config
    gpio_init();

    //Create task
    xTaskCreate(&adc_loop, "adc_task", 4096, NULL, 5, NULL);
}
