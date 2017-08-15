#include <stdio.h>

void config_init();
int32_t config_read(char* name, int32_t default_value);
bool config_write(char* name, int32_t value);
