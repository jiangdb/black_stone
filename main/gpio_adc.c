/*
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
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "queue_buffer.h"
#include "util.h"

/*
*/
#define TAG                   "GPIO_ADC"

#define PRECISION             6

#define PIN_NUM_DATA          4
#define PIN_NUM_CLK           0

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

#define USE_QUEUE_BUFFER          1
#define BUFFER_SIZE               4

#define INT_VALID_INTERVAL_10HZ      (240000 * 50)   //50ms for 10HZ
#define INT_VALID_INTERVAL_40HZ      (240000 * 15)   //15ms for 40HZ

static const uint8_t channel_config = (0x00|REFO_ON|SPEED_SEL_40HZ|PGA_SEL_64|CH_SEL_A);

//The semaphore indicating the data is ready.
static SemaphoreHandle_t dataReadySem = NULL;
static uint32_t lastDataReadyTime;
static uint32_t lastIsrTime=0;
static uint32_t isrInterval=0;
static TaskHandle_t xReaderTaskHandle = NULL;
static int32_t gpio_adc_value = 0;
#if USE_QUEUE_BUFFER
static queue_buffer_t qb_GpioAdcData;
static int32_t gpioDataBuffer[BUFFER_SIZE];
#endif

/*
This ISR is called when the data line goes low.
*/
static void data_isr_handler(void* arg)
{
    //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
    //looking at the time between interrupts and refusing any interrupt too close to another one.
    uint32_t currtime=xthal_get_ccount();
    uint32_t diff=currtime-lastDataReadyTime;
    isrInterval = (currtime - lastIsrTime)/240000;
    lastIsrTime = currtime;
    if (diff < INT_VALID_INTERVAL_40HZ) return; //ignore everything between valid interval
    lastDataReadyTime=currtime;
    //Give the semaphore.
    BaseType_t mustYield=false;
    xSemaphoreGiveFromISR(dataReadySem, &mustYield);
    if (mustYield) portYIELD_FROM_ISR();
}

static int32_t parse_adc(int32_t adcValue)
{
    int32_t value = adcValue;

    if (adcValue & 0x800000) {
        value = 0xFF000000|adcValue;
    }

    /*
    if (abs(gpio_adc_value - (value >> PRECISION)) >=2 ){
        printf("gpio adc value: (int)%d  ", value >> PRECISION );
        print_bin(value, 3);
    }
    */
    value = value >> PRECISION;
    return value;
}

static void send_clk()
{
    int clk_pin = PIN_NUM_CLK;
    gpio_set_level(clk_pin, 1);
    for (int i = 0; i < 10; ++i) {}
    gpio_set_level(clk_pin, 0);
    for (int i = 0; i < 10; ++i) {}
}

static void config(uint8_t config)
{
    int data_pin = PIN_NUM_DATA;
    //----------------------------------
    //1 ~ 3：clk1-clk27
    //----------------------------------
    int i; 
    for(i = 0; i < 27; i++)
    {
        send_clk();
    }

    //----------------------------------
    //4：clk28-clk29
    //----------------------------------
    gpio_set_direction(data_pin,GPIO_MODE_OUTPUT);
    send_clk();
    send_clk();

    //----------------------------------
    //5：clk30-clk36(发送写命令)
    //----------------------------------    
    uint8_t command = 0xCA;    
    for(i = 0; i < 7; i ++)
    {
        if(command & 0x80)              //MSB
        {
            gpio_set_level(data_pin, 1);
        }
        else
        {
            gpio_set_level(data_pin, 0);
        }
        command = command << 1;
        send_clk();
    }

    //----------------------------------
    //6：clk37
    //----------------------------------
    send_clk();
    
    //----------------------------------
    //7：clk38-clk45(写入寄存器)
    //----------------------------------
    for(i = 0; i < 8; i ++)
    {
        if(config & 0x80)              //MSB
        {
            gpio_set_level(data_pin, 1);
        }
        else
        {
            gpio_set_level(data_pin, 0);
        }
        config = config << 1;
        send_clk();
    }
        
    //----------------------------------
    //8：clk46
    //----------------------------------    
    gpio_set_direction(data_pin,GPIO_MODE_INPUT);
    send_clk();
}

static int32_t read_only()
{
    int data_pin = PIN_NUM_DATA;
    int32_t read = 0;
    for (int i = 0; i < 24; ++i)
    {
        read <<= 1;
        send_clk();
        if(gpio_get_level(data_pin)) {
            read |=1 ;
        }
    }
    send_clk();
    send_clk();
    send_clk();
    return parse_adc(read);
}

static void push_to_buffer(int32_t value)
{
#if USE_QUEUE_BUFFER     
    queue_buffer_push(&qb_GpioAdcData, value);
    value = queue_get_value(&qb_GpioAdcData, ALG_MEDIAN_VALUE);
#endif
    if (abs(gpio_adc_value - value) >=2 ) {
        gpio_adc_value = value;
        //printf("gpio_adc_value: %d\n", gpio_adc_value);
    }
}

static void gpio_adc_loop()
{
    int32_t v = 0;
    bool configed = false;
    while(1) {
        //Wait until data is ready
        xSemaphoreTake( dataReadySem, portMAX_DELAY );

        /*
        int data_level = gpio_get_level(PIN_NUM_DATA);
        ESP_LOGD(TAG,"gpio isr interval: %d data level: %d",isrInterval, data_level);
        if(gpio_get_level(PIN_NUM_DATA) == 1) {
            continue;
        }
        */
        //Disable data int
        gpio_intr_disable(PIN_NUM_DATA);

        if (!configed) {
            config(channel_config);
            configed  = true;
        }else{
            v = read_only();
            push_to_buffer(v);
        }

        vTaskDelay(10/portTICK_RATE_MS);

        //Enable data int
        gpio_intr_enable(PIN_NUM_DATA);
    }
}

int32_t gpio_adc_get_value()
{
    return gpio_adc_value;
}

void gpio_adc_shutdown()
{
    if( xReaderTaskHandle != NULL )
    {
        vTaskDelete( xReaderTaskHandle );
    }

    gpio_set_level(PIN_NUM_CLK, 0);
    vTaskDelay(1/portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_CLK, 1);
    vTaskDelay(1/portTICK_RATE_MS);
}

void gpio_adc_init()
{
    ESP_LOGD(TAG, "%s: CS1237 start!!!\n", __func__);

    //Create the semaphore.
    dataReadySem=xSemaphoreCreateBinary();

#if USE_QUEUE_BUFFER
    // Queue Buffer init
    memset(gpioDataBuffer,0,sizeof(gpioDataBuffer));
    queue_buffer_init(&qb_GpioAdcData, gpioDataBuffer, BUFFER_SIZE);
#endif

    //GPIO config
    gpio_config_t clk_conf={
        .intr_type=GPIO_PIN_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pull_down_en=0,
        .pull_up_en=0,
        .pin_bit_mask=(1<<PIN_NUM_CLK)
    };
    gpio_config(&clk_conf);
    gpio_set_level(PIN_NUM_CLK, 0);

    //GPIO config for the data line.
    gpio_config_t data_conf={
        .intr_type=GPIO_INTR_NEGEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=1,
        .pin_bit_mask=(1<<PIN_NUM_DATA)
    };

    //Set up handshake line interrupt.
    gpio_config(&data_conf);
    // gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_NUM_DATA, data_isr_handler, NULL);

    //Create task
    xTaskCreate(&gpio_adc_loop, "gpio_adc_task", 4096, NULL, 2, &xReaderTaskHandle);
}
