/* OTA example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "wifi_service.h"
#include "config.h"
#include "display.h"
#include "ota_service.h"

#define TAG "OTA"

#define CONTENT_LENGTH      "Content-Length"
#define BUFFSIZE            1024
#define TEXT_BUFFSIZE       1024

/*an ota data write buffer ready to write to the flash*/
static char ota_write_data[BUFFSIZE + 1] = { 0 };
/*an packet receive buffer*/
static char text[BUFFSIZE + 1] = { 0 };
/* an image total length*/
static int binary_file_length = 0;
static int binary_file_total_length = 0;
/*socket id*/
static int socket_id = -1;

/*read buffer by byte still delim ,return read bytes counts*/
static int read_until(char *buffer, char delim, int len)
{
//  /*TODO: delim check,buffer check,further: do an buffer length limited*/
    int i = 0;
    while (buffer[i] != delim && i < len) {
        ++i;
    }
    return i + 1;
}

static bool parse_header(char* header)
{
    if (strncmp(header, CONTENT_LENGTH, strlen(CONTENT_LENGTH)) == 0) {
        int i = strlen(CONTENT_LENGTH) + 2; //  2 for ": "
        char size[32] = {0};
        int j=0;
        while(header[i] != '\r') {
            size[j++] = header[i++];
        }
        binary_file_total_length = atoi(size);
        ESP_LOGI(TAG, "CONTENT_LENGTH %d", binary_file_total_length);
        return true;
    }
    return false;
}


/* resolve a packet from http socket
 * return true if packet including \r\n\r\n that means http packet header finished,start to receive packet body
 * otherwise return false
 * */
static bool read_past_http_header(char text[], int total_len, esp_ota_handle_t update_handle)
{
    /* i means current position */
    int i = 0, i_read_len = 0;
    bool parsed = false;
    while (text[i] != 0 && i < total_len) {
        i_read_len = read_until(&text[i], '\n', total_len);
        if (!parsed) {
            parsed = parse_header(&(text[i]));
        }

        // if we resolve \r\n line,we think packet header is finished
        if (i_read_len == 2) {
            int i_write_len = total_len - (i + 2);
            memset(ota_write_data, 0, BUFFSIZE);
            /*copy first http packet body to write buffer*/
            memcpy(ota_write_data, &(text[i + 2]), i_write_len);

            esp_err_t err = esp_ota_write( update_handle, (const void *)ota_write_data, i_write_len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
                return false;
            } else {
                ESP_LOGD(TAG, "esp_ota_write header OK");
                binary_file_length += i_write_len;
            }
            return true;
        }
        i += i_read_len;
    }
    return false;
}

static bool connect_to_http_server(firmware_t* firmware)
{
    int  http_connect_flag = -1;
    struct sockaddr_in sock_info;
    struct hostent *hp;
    struct ip4_addr *ip4_addr;

    ESP_LOGD(TAG, "create socket");
    socket_id = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_id == -1) {
        ESP_LOGE(TAG, "Create socket failed!");
        return false;
    }

    /* lookup the ip address */
    ESP_LOGD(TAG, "lookup the ip address for %s", firmware->host);
    hp = gethostbyname(firmware->host);
    if (hp == NULL) {
        ESP_LOGE(TAG, "No such host!");
        return false;
    }
    ip4_addr = (struct ip4_addr *)hp->h_addr;
    ESP_LOGD(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*ip4_addr));

    // set connect info
    memset(&sock_info, 0, sizeof(struct sockaddr_in));
    sock_info.sin_family = AF_INET;
    memcpy(&sock_info.sin_addr.s_addr,hp->h_addr,hp->h_length);
    sock_info.sin_port = htons(firmware->port);

    // connect to http server
    http_connect_flag = connect(socket_id, (struct sockaddr *)&sock_info, sizeof(sock_info));
    if (http_connect_flag == -1) {
        ESP_LOGE(TAG, "Connect to server failed! errno=%d", errno);
        close(socket_id);
        return false;
    } else {
        ESP_LOGD(TAG, "Connected to http server");
        return true;
    }
    return false;
}

static void task_fatal_error()
{
    ESP_LOGE(TAG, "Exiting task due to fatal error...");
    close(socket_id);
    setError(0,0,0,1);
}

bool ota_task(firmware_t* firmware)
{
    esp_err_t err;
    esp_ota_handle_t update_handle = 0 ;
    const esp_partition_t *update_partition = NULL;
    const esp_partition_t *configured = esp_ota_get_boot_partition();
    const esp_partition_t *running = esp_ota_get_running_partition();

    ESP_LOGI(TAG, "Starting OTA ...");
    assert(configured == running); /* fresh from reset, should be running from configured boot partition */
    ESP_LOGD(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             configured->type, configured->subtype, configured->address);

    if (firmware == NULL) {
        ESP_LOGE(TAG, "wrong firmware link");
        task_fatal_error();
        return false;
    }

    /*connect to http server*/
    if (!connect_to_http_server(firmware)) {
        task_fatal_error();
        return false;
    }

    int res = -1;
    char http_request[64] = {0};
    sprintf(http_request, "GET %s HTTP/1.1\r\nHost: %s \r\nAccept: */*  \r\n\r\n", firmware->path, firmware->host);
    /*send GET request to http server*/
    res = send(socket_id, http_request, strlen(http_request), 0);
    if (res == -1) {
        ESP_LOGE(TAG, "Send GET request to server failed");
        task_fatal_error();
        return false;
    } else {
        ESP_LOGD(TAG, "Send GET request to server succeeded");
    }

    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGD(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);

    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed, error=%d", err);
        task_fatal_error();
        return false;
    }
    ESP_LOGD(TAG, "esp_ota_begin succeeded");
    display_setOperation(OPERATION_UPGRADE, 255, 255, 255, 0);

    bool resp_body_start = false, flag = true;
    int percentage = 0;
    /*deal with all receive packet*/
    while (flag) {
        memset(text, 0, TEXT_BUFFSIZE);
        memset(ota_write_data, 0, BUFFSIZE);
        int buff_len = recv(socket_id, text, TEXT_BUFFSIZE, 0);
        if (buff_len < 0) { /*receive error*/
            ESP_LOGE(TAG, "Error: receive data error! errno=%d", errno);
            task_fatal_error();
            return false;
        } else if (buff_len > 0 && !resp_body_start) { /*deal with response header*/
            memcpy(ota_write_data, text, buff_len);
            resp_body_start = read_past_http_header(text, buff_len, update_handle);
        } else if (buff_len > 0 && resp_body_start) { /*deal with response body*/
            memcpy(ota_write_data, text, buff_len);
            err = esp_ota_write( update_handle, (const void *)ota_write_data, buff_len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error: esp_ota_write failed! err=0x%x", err);
                task_fatal_error();
                return false;
            }
            binary_file_length += buff_len;
            int per = ( binary_file_length*100 / binary_file_total_length );
            if ((per % 10 == 0) && (per > percentage)) {
                percentage = per;
                ESP_LOGD(TAG, "Have written image length %d%%", percentage);
                if (percentage != 100) {
                    display_setOperation(OPERATION_UPGRADE, 255, 255, percentage/10, 0);
                }else{
                    display_setOperation(OPERATION_UPGRADE, 255, 255, 9, 5);
                }
            }
        } else if (buff_len == 0) {  /*packet over*/
            flag = false;
            ESP_LOGD(TAG, "Connection closed, all packets received");
            close(socket_id);
        } else {
            ESP_LOGE(TAG, "Unexpected recv result");
        }
    }

    ESP_LOGI(TAG, "Total Write binary data length : %d", binary_file_length);
    display_setOperation(OPERATION_UPGRADE, 255, 1, 0, 0);

    if (esp_ota_end(update_handle) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed!");
        task_fatal_error();
        return false;
    }
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed! err=0x%x", err);
        task_fatal_error();
        return false;
    }
    return true;
}
