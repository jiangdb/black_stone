#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

void setDisplayNumber(uint8_t displayNum, uint32_t value, int8_t precision);
void setDisplayTime(uint32_t seconds);
void setBatteryLevel(int8_t percentage);
void setWifiSound(bool wifi, bool sound);
void spi_trassfer_display();