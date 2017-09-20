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
#include "delay_timer.h"

/*
*/
#define TAG                   "GPIO_ADC"

#define PRECISION             6

#define CH0_PIN_NUM_DATA      23
#define CH0_PIN_NUM_CLK       18
#define CH1_PIN_NUM_DATA      4
#define CH1_PIN_NUM_CLK       0

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

#define USE_QUEUE_BUFFER          0
#define BUFFER_SIZE               5
#define CALIBRATION_BUFFER_SIZE   10

static const uint8_t channel_config = (0x00|REFO_ON|SPEED_SEL_40HZ|PGA_SEL_64|CH_SEL_A);

static xQueueHandle data_ready_queue = NULL;
//The semaphore indicating the data is ready.
// static SemaphoreHandle_t dataReadySem = NULL;
static uint32_t lastDataReadyTime_ch0;
static uint32_t lastDataReadyTime_ch1;
static TaskHandle_t xReaderTaskHandle = NULL;
static bool calibration_enable = false;

#if USE_QUEUE_BUFFER
static int32_t pre_value = 0;
static queue_buffer_t qb_GpioAdcData;
static int32_t dataBuffer[BUFFER_SIZE];
static queue_buffer_t qb_GpioAdcCalibration;
static int32_t calibrationBuffer[CALIBRATION_BUFFER_SIZE];
#else
static int32_t gpio_adc_value[2] = {0,0};
#endif

/*
This ISR is called when the data line goes low.
*/
static void IRAM_ATTR ch0_data_isr_handler(void* arg)
{
    //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
    //looking at the time between interrupts and refusing any interrupt too close to another one.
    uint32_t currtime=xthal_get_ccount();
    uint32_t diff=currtime-lastDataReadyTime_ch0;
    if (diff<1200000) return; //ignore everything <5ms after an earlier irq
    lastDataReadyTime_ch0=currtime;
    //Give the semaphore.
    BaseType_t mustYield=false;
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(data_ready_queue, &gpio_num, &mustYield);
    if (mustYield) portYIELD_FROM_ISR();
}

static void IRAM_ATTR ch1_data_isr_handler(void* arg)
{
    //Sometimes due to interference or ringing or something, we get two irqs after eachother. This is solved by
    //looking at the time between interrupts and refusing any interrupt too close to another one.
    uint32_t currtime=xthal_get_ccount();
    uint32_t diff=currtime-lastDataReadyTime_ch1;
    if (diff<1200000) return; //ignore everything <5ms after an earlier irq
    lastDataReadyTime_ch1=currtime;
    //Give the semaphore.
    BaseType_t mustYield=false;
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(data_ready_queue, &gpio_num, &mustYield);
    if (mustYield) portYIELD_FROM_ISR();
}

static int32_t parse_adc(int ch, int32_t adcValue)
{
    int32_t value = adcValue;

    if (adcValue & 0x800000) {
        value = 0xFF000000|adcValue;
    }

    if (abs(gpio_adc_value[ch] - (value >> PRECISION)) >=2 ){
        printf("gpio adc[%d] value: (int)%d  ", ch, value >> PRECISION );
        print_bin(value, 3);
    }

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

static void send_clk(int ch)
{
    int clk_pin = ch == 0? CH0_PIN_NUM_CLK: CH1_PIN_NUM_CLK;
    gpio_set_level(clk_pin, 1);
    // delay_5us();
    for (int i = 0; i < 10; ++i) {}
    gpio_set_level(clk_pin, 0);
    // delay_5us();
    for (int i = 0; i < 10; ++i) {}
}

static void config(int ch, uint8_t config)
{
    int data_pin = ch == 0? CH0_PIN_NUM_DATA: CH1_PIN_NUM_DATA;
    //----------------------------------
    //1 ~ 3：clk1-clk27
    //----------------------------------
    int i; 
    for(i = 0; i < 27; i++)
    {
        send_clk(ch);
    }

    //----------------------------------
    //4：clk28-clk29
    //----------------------------------
    gpio_set_direction(data_pin,GPIO_MODE_OUTPUT);
    send_clk(ch);
    send_clk(ch);

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
        send_clk(ch);
    }

    //----------------------------------
    //6：clk37
    //----------------------------------
    send_clk(ch);
    
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
        send_clk(ch);
    }
        
    //----------------------------------
    //8：clk46
    //----------------------------------    
    gpio_set_direction(data_pin,GPIO_MODE_INPUT);
    send_clk(ch);
}

static int32_t read_only(int ch)
{
    int data_pin = ch == 0? CH0_PIN_NUM_DATA: CH1_PIN_NUM_DATA;
    int32_t read = 0;
    for (int i = 0; i < 24; ++i)
    {
        read <<= 1;
        send_clk(ch);
        if(gpio_get_level(data_pin)) {
            read |=1 ;
        }
    }
    send_clk(ch);
    send_clk(ch);
    send_clk(ch);
    return parse_adc(ch, read);
}

static void push_to_buffer(int ch, int32_t value)
{
#if USE_QUEUE_BUFFER     
    queue_buffer_t *pBuffer;
    if (calibration_enable) {
        pBuffer = &qb_GpioAdcCalibration;
    }else{
        pBuffer = &qb_GpioAdcData;
    }

    if (abs(pre_value - value) >=2 ){
        queue_buffer_push(pBuffer, value);
        pre_value = value;
    }else{
        queue_buffer_push(pBuffer, pre_value);
    }
#else
    if (abs(gpio_adc_value[ch] - value) >=2 ){
        gpio_adc_value[ch] = value;
    }
    // printf("gpio_adc_value[%d]: (int)%d\n", ch, gpio_adc_value[ch]);
#endif
}

static void gpio_adc_loop()
{
    int32_t v = 0;
    uint32_t io_num;
    int ch=0;
    // bool configed = false;
    while(1) {
        //Wait until data is ready
        // xSemaphoreTake( dataReadySem, portMAX_DELAY );
        if(xQueueReceive(data_ready_queue, &io_num, portMAX_DELAY)) {
            if (io_num == CH0_PIN_NUM_DATA) {
                ch = 0;
            }else{
                ch = 1;
            }
            // printf("%s: Got data on %d!!!\n", TAG, ch);
            if(gpio_get_level(io_num) == 1) {
                continue;
            }

            //Disable data int
            gpio_intr_disable(io_num);

            // vTaskDelay(1/portTICK_RATE_MS); //wait 1ms before start to read

            /*
            if (!configed) {
                config(channel_config);
                configed  = true;
            }else{
                v = read_only();
                push_to_buffer(v);
            }
            */
            v = read_only(ch);
            push_to_buffer(ch, v);

            // xSemaphoreGive( xMutexRead );
            // vTaskDelay(10/portTICK_RATE_MS);

            //Enable data int
            if (ch == 0) {
                lastDataReadyTime_ch0=xthal_get_ccount();
            }else{
                lastDataReadyTime_ch1=xthal_get_ccount();
            }

            gpio_intr_enable(io_num);
        }
    }
}

int32_t gpio_adc_get_value(int ch)
{
#if USE_QUEUE_BUFFER
    if (calibration_enable) {
        return queue_average(qb_GpioAdcCalibration);
    }else{
        return queue_average(qb_GpioAdcData);
    }
#else
    return gpio_adc_value[ch];
#endif
}

void gpio_adc_calibration(bool enable)
{
    calibration_enable = enable;
}

void gpio_adc_shutdown()
{
    if( xReaderTaskHandle != NULL )
    {
        vTaskDelete( xReaderTaskHandle );
    }

    gpio_set_level(CH0_PIN_NUM_CLK, 0);
    gpio_set_level(CH1_PIN_NUM_CLK, 0);
    vTaskDelay(1/portTICK_RATE_MS);
    gpio_set_level(CH0_PIN_NUM_CLK, 1);
    gpio_set_level(CH1_PIN_NUM_CLK, 1);
    vTaskDelay(1/portTICK_RATE_MS);
}

void gpio_adc_init()
{
    printf("%s: CS1238 start!!!\n", TAG);

    //Create the semaphore.
    // dataReadySem=xSemaphoreCreateBinary();

    //create a queue to handle gpio event from isr
    data_ready_queue = xQueueCreate(10, sizeof(uint32_t));

#if USE_QUEUE_BUFFER
    // Queue Buffer init
    memset(dataBuffer,0,sizeof(dataBuffer));
    queue_buffer_init(&qb_GpioAdcData, dataBuffer, BUFFER_SIZE);

    memset(calibrationBuffer,0,sizeof(calibrationBuffer));
    queue_buffer_init(&qb_GpioAdcCalibration, calibrationBuffer, CALIBRATION_BUFFER_SIZE);
#endif

    //GPIO config
    printf("%s: gpio_init !!!\n", TAG);

    gpio_config_t clk_conf={
        .intr_type=GPIO_PIN_INTR_DISABLE,
        .mode=GPIO_MODE_OUTPUT,
        .pull_down_en=0,
        .pull_up_en=0,
        .pin_bit_mask=((1<<CH0_PIN_NUM_CLK)|(1<<CH1_PIN_NUM_CLK))
    };
    gpio_config(&clk_conf);
    gpio_set_level(CH0_PIN_NUM_CLK, 0);
    gpio_set_level(CH1_PIN_NUM_CLK, 0);

    //GPIO config for the data line.
    gpio_config_t data_conf={
        .intr_type=GPIO_INTR_NEGEDGE,
        .mode=GPIO_MODE_INPUT,
        .pull_up_en=1,
        .pin_bit_mask=((1<<CH0_PIN_NUM_DATA)|(1<<CH1_PIN_NUM_DATA))
    };

    //Set up handshake line interrupt.
    gpio_config(&data_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(CH0_PIN_NUM_DATA, ch0_data_isr_handler, (void*)CH0_PIN_NUM_DATA);
    gpio_isr_handler_add(CH1_PIN_NUM_DATA, ch1_data_isr_handler, (void*)CH1_PIN_NUM_DATA);

    //Create task
    xTaskCreate(&gpio_adc_loop, "gpio_adc_task", 4096, NULL, 2, &xReaderTaskHandle);
}

