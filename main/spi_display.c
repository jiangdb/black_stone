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
#include "config.h"
#include "display.h"
#include "battery.h"
#include "key.h"
#include "calibration.h"
#include "queue_buffer.h"
#include "bs_timer.h"

/*
 * defines
 */

#define TAG  "DISPLAY"

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
#define NUMBER_OFF              0x00
#define OPT_DASH                0x40

#define CHAR_C                  0x39
#define CHAR_E                  0x79

#define DIGITAL_NUMBER          4
#define BATTERY_ADDRESS         13
#define WIRELESS_ADDRESS        14
#define ICON_ADDRESS            14

enum {
    ALARM_TIME,
    ALARM_NUMBER,
    ALARM_MAX,
};

static uint8_t display_data[]={
    COMMAND_ADDRESS_0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
};
static uint8_t* display_data_bak = NULL;

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

static uint8_t battery_levels[] = {
    0x0F,       //empty
    0x1F,       //1
    0x3F,       //2
    0x7F,       //3
};

static spi_device_handle_t spi;
static TaskHandle_t xHandle = NULL;
static int iChargingCount = 0;
static uint8_t sbAlarm = ALARM_MAX;
static uint32_t currentTime = 0;
static int32_t siWeights[2] = {0};

static void clear_display_data(bool leaveBattery);

/*******************************************************************************
**
** Function         setDisplayNumber
**
** Description      display value.
**
** Parameter        displayCh: display number.
**                  value: value should be displayed, in 100mg.
**
** Returns          weight in 0.1g.
**
*******************************************************************************/
void setDisplayNumber(uint8_t displayCh, int32_t value)
{
    int8_t data[DIGITAL_NUMBER];
    int8_t precision;

    //remember value
    siWeights[displayCh] = value;

    // ESP_LOGD(TAG,"setDisplayInteger(%d, %d)!!!\n", displayCh, value);
    if (value >= 99990) { value = 9999; precision=0;}
    else if (value > 9999) { value /= 10; precision=0;}
    else if (value > -1000) { precision=1;}
    else if (value > -9990) { value /= 10; precision=0;}
    else { value = -999; precision=0;}

    if (value == 0) {
        //show 0.0
        int start = 1+DIGITAL_NUMBER*displayCh;
        display_data[start] = NUMBER_OFF;
        display_data[start+1] = NUMBER_OFF;
        display_data[start+2] = NUMBER_0;
        display_data[start+2] |= 0x80;
        display_data[start+3] = NUMBER_0;
        return;
    }

    if (value < 10 && value > -10) {
        //show 0.*
        int start = 1+DIGITAL_NUMBER*displayCh;
        display_data[start] = NUMBER_OFF;
        display_data[start+1] = (value < 0) ? OPT_DASH:NUMBER_OFF;
        display_data[start+2] = NUMBER_0;
        display_data[start+2] |= 0x80;
        display_data[start+3] = numbers[abs(value)];
        return;
    }

    //convert to array, LSB mode
    memset(data, -2, sizeof(data));
    int32_t abs_value = value > 0 ? value: (0-value);
    int i;
    for(i=DIGITAL_NUMBER-1; i>=0; i--) {
        data[i] = abs_value % 10;
        abs_value/=10;
        if (abs_value==0) break;
    }

    if (value < 0) {
        data[i-1] = -1;
    }

    //set display data
    int start = 1+DIGITAL_NUMBER*displayCh;
    for (int j=0; j<DIGITAL_NUMBER; j++) {
        if (data[j] == -2){
            display_data[start+j] = NUMBER_OFF;
        }else if (data[j] == -1){
            display_data[start+j] = OPT_DASH;
        }else{
            display_data[start+j] = numbers[data[j]];
        }
        if (j == (DIGITAL_NUMBER-precision-1) && precision!=0) {
            display_data[start+j] |= 0x80;
        }
    }
}

void setDisplayTime(uint32_t seconds)
{
    //max to 1 hr
    if (seconds>3600) seconds = 3600;

    //remember time
    currentTime = seconds;

    uint8_t data[4];
    int8_t mins = currentTime/60;
    currentTime %= 60;
    data[3] = currentTime%10;
    data[2] = currentTime/10;
    data[1] = mins%10;
    data[0] = mins/10;

    int start = 1+DIGITAL_NUMBER*2;
    for (int i=0; i<4; i++) {
        display_data[start+i] = numbers[data[i]];
        if (i==2) {
            display_data[start+i] |= 0x80;
        }
    }

    if (config_get_alarm_enable() == 1) {
        uint16_t alarmSeconds = config_get_alarm_time(); 
        if (seconds >= alarmSeconds && seconds > 0) {
            sbAlarm = ALARM_TIME;
        }
    }

    //durating work, we do not let system timeout
    bs_timer_reset(TIMER_TIMEOUT);
}

static void setDisplayTimeOff()
{
    int start = 1+DIGITAL_NUMBER*2;
    for (int i=0; i<4; i++) {
        display_data[start+i] = 0;
    }
}

static void setDisplayNumberOff()
{
    int start = 1;
    for (int i=0; i<DIGITAL_NUMBER*2; i++) {
        display_data[start+i] = 0;
    }
}

void setBatteryLevel(int batteryLevel)
{
    if (iChargingCount == 0) {
        if (batteryLevel < BATTERY_LEVEL_EMPTY || batteryLevel > BATTERY_LEVEL_3) return;
        display_data[BATTERY_ADDRESS] = battery_levels[batteryLevel];
    }
}

//wifiSound 0 for wifi, 1 for Sound
void setWifiSound(int wifiSound, bool enable)
{
    uint8_t val = display_data[WIRELESS_ADDRESS];
    int bit = 0;

    if (wifiSound == 0) {
        bit = 0;
    }else{
        bit = 1;
    }

    if (enable) {
        val |= 1<<bit;
    }else{
        val &= (~(1<<bit));
    }
    display_data[WIRELESS_ADDRESS] = val;
}

void display_seticon(int icon, bool on)
{
    uint8_t val = display_data[ICON_ADDRESS];

    if (on) {
        val |= 1<<icon;
    }else{
        val &= (~(1<<icon));
    }
    display_data[ICON_ADDRESS] = val;
}

void display_setOperation(int operation, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    clear_display_data(true);
    int timerPos = 1+DIGITAL_NUMBER*2;
    int digitPos = 1+DIGITAL_NUMBER;
    switch(operation) {
        case OPERATION_CALIBRATION:
            // show C0 at timer
            timerPos+=2;        //start from the third digit
            display_data[timerPos++] = CHAR_C;
            display_data[timerPos] = NUMBER_0;
            break;
        case OPERATION_UPGRADE:
            // show C1 at timer
            timerPos+=2;        //start from the third digit
            display_data[timerPos++] = CHAR_C;
            display_data[timerPos] = NUMBER_1;
            break;
        default:
            return;
            break;
    }

    if (d0 < 10) {
        display_data[digitPos] = numbers[d0];
    }
    digitPos++;
    if (d1 < 10) {
        display_data[digitPos] = numbers[d1];
    }
    digitPos++;
    if (d2 < 10) {
        display_data[digitPos] = numbers[d2];
    }
    digitPos++;
    if (d3 < 10) {
        display_data[digitPos] = numbers[d3];
    }
}

void setError(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    clear_display_data(true);
    int timerPos = 1+DIGITAL_NUMBER*2+3;
    int digitPos = 1+DIGITAL_NUMBER;
    // show E at timer
    display_data[timerPos] = CHAR_E;

    if (d0 < 10) {
        display_data[digitPos] = numbers[d0];
    }
    digitPos++;
    if (d1 < 10) {
        display_data[digitPos] = numbers[d1];
    }
    digitPos++;
    if (d2 < 10) {
        display_data[digitPos] = numbers[d2];
    }
    digitPos++;
    if (d3 < 10) {
        display_data[digitPos] = numbers[d3];
    }
}

void alarmNumber()
{
    sbAlarm = ALARM_NUMBER;
}

//Send data to the TA6932. Uses spi_device_transmit, which waits until the transfer is complete.
/*
static void spi_trassfer_single_byte(const uint8_t data) 
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));          //Zero out the transaction
    t.length=8;                        //Command is 8 bits
    t.tx_buffer=&data;                 //command or data
    ret=spi_device_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);               //Should have had no issues.
}

static void spi_trassfer_2bytes(const uint8_t command, const uint8_t data) 
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
*/

static void spi_trassfer_display() 
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
    trans[1].length=15*8;                     //15bytes is 15*8 bits
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

static void clear_display_data(bool leaveBattery)
{
    for (int i = 1; i < sizeof(display_data); ++i)
    {
        if (leaveBattery && i == BATTERY_ADDRESS) continue;
        display_data[i] = 0;
    }
}

void display_indicate_charge_only()
{
    int count = 6;
    clear_display_data(false);

    while(count > 0) {
        if (count%2 == 0) {
            setBatteryLevel(BATTERY_LEVEL_EMPTY);
        }else{
            clear_display_data(false);
        }

        spi_trassfer_display();
        vTaskDelay(300/portTICK_RATE_MS);
        count--;
    }
}

static void increase_battery_level()
{
    static int level = BATTERY_LEVEL_EMPTY;
    display_data[BATTERY_ADDRESS] = battery_levels[level++];
    if (level > BATTERY_LEVEL_3) {
        level = BATTERY_LEVEL_EMPTY;
    }
}

static void display_loop()
{
    int alarmCount = 0;
    while(1) {
        if (iChargingCount > 0) {
            if (iChargingCount % 10 == 1) {
                increase_battery_level();
            }
            iChargingCount++;
        }

        if (sbAlarm == ALARM_TIME) {
            switch_beap_vibrate(true);
            setDisplayTimeOff();
            spi_trassfer_display();
            vTaskDelay(50/portTICK_RATE_MS);
            switch_beap_vibrate(false);
            setDisplayTime(currentTime);
            spi_trassfer_display();
            vTaskDelay(50/portTICK_RATE_MS);
            alarmCount++;
            if (alarmCount > 2) {
                sbAlarm = ALARM_MAX;
                alarmCount = 0;
            }
        }else if (sbAlarm == ALARM_NUMBER) {
            switch_beap_vibrate(true);
            setDisplayNumberOff();
            spi_trassfer_display();
            vTaskDelay(50/portTICK_RATE_MS);
            switch_beap_vibrate(false);
            setDisplayNumber(0, siWeights[0]);
            setDisplayNumber(1, siWeights[1]);
            spi_trassfer_display();
            vTaskDelay(50/portTICK_RATE_MS);
            alarmCount++;
            if (alarmCount > 2) {
                sbAlarm = ALARM_MAX;
                alarmCount = 0;
            }
        }else{
            spi_trassfer_display();
            vTaskDelay(100/portTICK_RATE_MS);
        }
    }
}


void display_indicate_charging_only()
{
    clear_display_data(false);
    increase_battery_level();
    spi_trassfer_display();
}

void display_indicate_charging()
{
    ESP_LOGD(TAG,"indicate charging !!!\n");
    iChargingCount = 1;
}

void display_disable_charging()
{
    ESP_LOGD(TAG,"disable indicate charging !!!\n");
    iChargingCount = 0;
    int level = get_battery_level();
    setBatteryLevel(level);
}

void display_backup()
{
    if (display_data_bak != NULL) return;

    display_data_bak = malloc(sizeof(display_data)); 
    for (int i = 1; i < sizeof(display_data); ++i)
    {
        display_data_bak[i] = display_data[i];
    }
}

void display_restore()
{
    if (display_data_bak == NULL) return;

    for (int i = 1; i < sizeof(display_data); ++i)
    {
        display_data[i] = display_data_bak[i];
    }
    free(display_data_bak);
    display_data_bak = NULL;
}

void display_start()
{
    //reset display time
    setDisplayTime(0);
    //setDisplayNumber(0,0);        //default single scale
    setDisplayNumber(1,0);

    if (config_get_alarm_enable()) {
        setWifiSound(1,1);
    }

    //Create task
    xTaskCreate(&display_loop, "display_task", 2048, NULL, 5, &xHandle);
}

void display_stop()
{
    if( xHandle != NULL )
    {
        vTaskDelete( xHandle );
    }

    clear_display_data(false);
    spi_trassfer_display();
}

void display_init()
{
    ESP_LOGD(TAG,"SPI Display!!!\n");

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
}
