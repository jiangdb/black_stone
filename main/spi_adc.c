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
#include "queue_buffer.h"
#include "util.h"

/*
*/
#define TAG                   "ADC"

#define SINGLE_CHANNLE        0
#define PRECISION             6

#define PIN_NUM_DATA          23
#define PIN_NUM_CLK           18

#define DATA_PIN_FUNC_SPI     FUNC_GPIO23_VSPID
#define DATA_PIN_FUNC_GPIO    FUNC_GPIO23_GPIO23

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

#define BUFFER_SIZE           5
#define CALIBRATION_BUFFER_SIZE   10

static const uint8_t channel_configs[]={
    0x00|REFO_ON|SPEED_SEL_40HZ|PGA_SEL_64|CH_SEL_B,
    0x00|REFO_ON|SPEED_SEL_40HZ|PGA_SEL_64|CH_SEL_A,
};

//The semaphore indicating the data is ready.
static SemaphoreHandle_t rdySem = NULL;
static spi_device_handle_t spi;
static uint32_t lastDataReadyTime;
static bool calibration_enable = false;
static int32_t pre_value[2]={0,0};
static TaskHandle_t xHandle = NULL;
queue_buffer_t dataQueueBuffer[2];
int32_t dataBuffer[2][BUFFER_SIZE];

queue_buffer_t calibrationQueueBuffer[2];
int32_t calibrationBuffer[2][CALIBRATION_BUFFER_SIZE];

/*
This ISR is called when the data line goes low.
*/
static void IRAM_ATTR gpio_data_isr_handler(void* arg)
{
    //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
    //looking at the time between interrupts and refusing any interrupt too close to another one.
    uint32_t currtime=xthal_get_ccount();
    uint32_t diff=currtime-lastDataReadyTime;
    if (diff<1200000) return; //ignore everything <5ms after an earlier irq
    lastDataReadyTime=currtime;
    //Give the semaphore.
    BaseType_t mustYield=false;
    xSemaphoreGiveFromISR(rdySem, &mustYield);
    if (mustYield) portYIELD_FROM_ISR();
}

static void gpio_spi_switch(uint8_t mode)
{
    esp_err_t ret;
    if (mode == DATA_PIN_FUNC_GPIO) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_DATA], mode);
        ret=gpio_set_direction(PIN_NUM_DATA,GPIO_MODE_INPUT);
        assert(ret==ESP_OK);
        lastDataReadyTime=xthal_get_ccount();
        ret=gpio_intr_enable(PIN_NUM_DATA);
        assert(ret==ESP_OK);
    }else if (mode == DATA_PIN_FUNC_SPI) {
        ret=gpio_intr_disable(PIN_NUM_DATA);
        assert(ret==ESP_OK);
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[PIN_NUM_DATA], mode);
    }
}

static int32_t parse_adc(int channel, uint8_t data[4])
{
    int32_t value = 0;

    if (data[0] & 0x80) {
        value = 0xFF000000|data[0]<<16|data[1]<<8|data[2];
    }else{
        value = data[0]<<16|data[1]<<8|data[2];
    }

    /*
    if (channel == 0) {
        printf("adc value[%d]: (int)%d  ", channel, value >> PRECISION );
        print_bin(value, 3);
    }
    */

    /*
    //check if we need -1
    bool need_minus = false;
    int32_t cur_value = value >> (PRECISION-1);
    if (cur_value == (pre_value[channel]+1)) {
        int8_t last_two_digits = (int8_t)(cur_value & 0x3);
        if (last_two_digits == 0x2 || last_two_digits == 0) {
            need_minus = true;
            cur_value--;
        }
    }
    pre_value[channel] = cur_value;
    value = value >> PRECISION;
    if (need_minus) {
        value--;
    }
    if (channel == 1) {
        printf("value: %d\n ", value);
    }
    */
    value = value >> PRECISION;
    return value;
}

static int32_t config(int8_t config)
{
    esp_err_t ret;
    //Prepare spi receive buffer
    spi_transaction_t trans[2];
    spi_transaction_t *rtrans;

    memset(trans, 0, sizeof(trans));
    trans[0].rxlength=29;
    trans[0].flags=SPI_TRANS_USE_RXDATA;
    ret=spi_device_queue_trans(spi, &trans[0], portMAX_DELAY);
    assert(ret==ESP_OK);

    trans[1].length=17;
    trans[1].tx_data[0]=0xCA;
    trans[1].tx_data[1]=config;
    trans[1].tx_data[2]=0x00;
    trans[1].flags=SPI_TRANS_USE_TXDATA;
    ret=spi_device_queue_trans(spi, &trans[1], portMAX_DELAY);
    assert(ret==ESP_OK);

    int32_t value = 0;
    //Wait for all 2 transactions to be done and get back the results.
    for (int x=0; x<2; x++) {
        ret=spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret==ESP_OK);
        if ( x==0 ) {
            value = parse_adc(config&CH_SEL_B?0:1, rtrans->rx_data);
        }
    }
    return value;
}

#if SINGLE_CHANNLE    
static int32_t read_only()
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));          //Zero out the transaction
    t.rxlength=27;                     //Command is 8 bits
    t.flags=SPI_TRANS_USE_RXDATA;      //The data is the cmd itself
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);               //Should have had no issues.
    return parse_adc(1,t.rx_data);
}
#endif

static void push_to_buffer(int channel, int32_t value)
{
    queue_buffer_t *pBuffer;
    if (calibration_enable) {
        pBuffer = &calibrationQueueBuffer[channel];
    }else{
        pBuffer = &dataQueueBuffer[channel];
    }

    int32_t last = pre_value[channel];
    if (abs(last - value) >=2 ){
        queue_buffer_push(pBuffer, value);
        pre_value[channel] = value;
    }else{
        queue_buffer_push(pBuffer, last);
    }
}

static void spi_init()
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
        .clock_speed_hz=100000,                //Clock out at 100KHz
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

static void adc_gpio_init()
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
    // gpio_install_isr_service(0);
    //gpio_set_intr_type(PIN_NUM_DATA, GPIO_INTR_NEGEDGE);
    gpio_isr_handler_add(PIN_NUM_DATA, gpio_data_isr_handler, NULL);
}

static void adc_loop()
{
    uint8_t ch = 0;
    int32_t v = 0;
#if SINGLE_CHANNLE    
    bool configed = false;
    uint8_t conf = 0;
#endif
    while(1) {
        //Wait until data is ready
        xSemaphoreTake( rdySem, portMAX_DELAY );
        //Disable gpio and enable spi
        gpio_spi_switch(DATA_PIN_FUNC_SPI);

#if SINGLE_CHANNLE
        if (!configed) {
            conf = (0x00|REFO_ON|SPEED_SEL_40HZ|PGA_SEL_64|CH_SEL_B);
            v = config(conf);
            configed  = true;
        }else{
            v = read_only();
            push_to_buffer(1, v);
        }
#else
        v = config(channel_configs[ch]);
        push_to_buffer(ch, v);
        ch = (ch == 0 ? 1:0);
#endif
        vTaskDelay(10/portTICK_RATE_MS);

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
    adc_gpio_init();

    // Queue Buffer init
    memset(dataBuffer,0,sizeof(dataBuffer));
    queue_buffer_init(&dataQueueBuffer[0], dataBuffer[0], BUFFER_SIZE);
    queue_buffer_init(&dataQueueBuffer[1], dataBuffer[1], BUFFER_SIZE);

    memset(calibrationBuffer,0,sizeof(calibrationBuffer));
    queue_buffer_init(&calibrationQueueBuffer[0], calibrationBuffer[0], CALIBRATION_BUFFER_SIZE);
    queue_buffer_init(&calibrationQueueBuffer[1], calibrationBuffer[1], CALIBRATION_BUFFER_SIZE);

    //Create task
    xTaskCreate(&adc_loop, "adc_task", 4096, NULL, 5, &xHandle);
}

void adc_calibration(bool enable)
{
    calibration_enable = enable;
}

void adc_shutdown()
{
    if( xHandle != NULL )
    {
        vTaskDelete( xHandle );
    }

    //GPIO config for the clk pin.
    gpio_config_t io_conf={
        .intr_type=GPIO_PIN_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pull_down_en=0,
        .pull_up_en=0,
        .pin_bit_mask=(1<<PIN_NUM_CLK)
    };
    //Set up handshake line interrupt.
    gpio_config(&io_conf);
    gpio_set_level(PIN_NUM_CLK, 0);
    vTaskDelay(1/portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_CLK, 1);
    vTaskDelay(1/portTICK_RATE_MS);
}