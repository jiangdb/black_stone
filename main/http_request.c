/*
 * =====================================================================================
 *
 *       Filename:  http_request.c
 *
 *    Description: Http request funtions, which will call bm backend 
 *
 *        Version:  1.0
 *        Created:  11/17/2017 11:21:08
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Dong Bin Jiang
 *   Organization:  
 *
 * =====================================================================================
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netdb.h>
#include "esp_log.h"
#include "http_request.h"
#include "config.h"

#define TAG "HTTP"

#define IP_HOST                 "api.ipify.org"             //"ip.chinaz.com"
#define IP_PATH                 "/"                         //"/getip.aspx"
#define IP_PORT                 80
#define HOST                    "bm.timemore.com"           //"bs.ziipoo.com.cn"
#define PORT                    80
#define API_DEVICE_ONLINE       "/api/v1/device/online"
#define BUFFSIZE                1024
#define HTTP_REQUEST_GET		\
	"GET %s HTTP/1.1\r\n"		\
	"Host:  %s\r\n"		        \
    "User-Agent: bm/1.0\r\n"    \
    "Accept: */*  \r\n\r\n"

#define HTTP_REQUEST_PUT		\
	"PUT %s HTTP/1.1\r\n"		\
	"Host:  %s\r\n"		        \
    "User-Agent: bm/1.0\r\n"    \
    "Accept: */*  \r\n"         \
    "Content-Length: %d\r\n"    \
    "Content-Type: application/x-www-form-urlencoded\r\n"    \
    "\r\n%s"

static int socket_id = -1;
static char recv_buf[BUFFSIZE];

static bool connect_to_http_server(char* host, uint8_t port)
{
    int  http_connect_flag = -1;
    struct sockaddr_in sock_info;
    struct hostent *hp;

    ESP_LOGD(TAG, "create socket");
    socket_id = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_id == -1) {
        ESP_LOGE(TAG, "Create socket failed!");
        return false;
    }

    /* lookup the ip address */
    ESP_LOGD(TAG, "lookup the ip address for %s", host);
    hp = gethostbyname(host);
    if (hp == NULL) {
        ESP_LOGE(TAG, "No such host!");
        return false;
    }

    // set connect info
    memset(&sock_info, 0, sizeof(struct sockaddr_in));
    sock_info.sin_family = AF_INET;
    memcpy(&sock_info.sin_addr.s_addr,hp->h_addr,hp->h_length);
    sock_info.sin_port = htons(port);

    // connect to http server
    http_connect_flag = connect(socket_id, (struct sockaddr *)&sock_info, sizeof(sock_info));
    if (http_connect_flag == -1) {
        ESP_LOGE(TAG, "Connect to server failed! errno=%d", errno);
        close(socket_id);
        return false;
    }

    ESP_LOGD(TAG, "Connected to http server");
    return true;
}

static bool send_request(char* request, uint16_t len)
{
    int res = -1;
    /*send GET request to http server*/
    res = send(socket_id, request, len, 0);
    if (res == -1) {
        ESP_LOGE(TAG, "Send GET request to server failed");
        return false;
    }

    ESP_LOGD(TAG, "Send GET request to server succeeded");
    return true;
}

static bool recv_response(char* buf, int* len)
{
    memset(recv_buf,0,sizeof(recv_buf));
    int buff_len = recv(socket_id, recv_buf, sizeof(recv_buf), 0);
    if (buff_len < 0) { /*receive error*/
        ESP_LOGE(TAG, "Receive data error! errno=%d", errno);
        return false;
    }

    char* ptr = NULL;
    ptr = strstr(recv_buf, "200 OK");
	if (!ptr) {
        ESP_LOGE(TAG, "Not response 200 OK");
        ESP_LOGE(TAG, "%s", recv_buf);
		return false;
    }

	ptr = strstr(ptr, "\r\n\r\n");
	if (!ptr) {
        ESP_LOGE(TAG, "Not find end of header");
		return false;
    }
	ptr += 4;
    *len = buff_len - (ptr-recv_buf);
    memcpy(buf, ptr, *len);
    return true;
}

static bool http_get_public_ip(char* ip, int* len) 
{
    /*connect to http server*/
    if (!connect_to_http_server(IP_HOST, IP_PORT)) {
        ESP_LOGE(TAG, "error connect to host");
        return false;
    }

    char http_request[256] = {0};
    sprintf(http_request, HTTP_REQUEST_GET, IP_PATH, IP_HOST);
    if (!send_request(http_request, strlen(http_request))) {
        ESP_LOGE(TAG, "error send request");
        close(socket_id);
        return false;
    }

    bool ret = false;
    if (recv_response(ip, len)) {
        ESP_LOGD(TAG, "get public ip: %s", ip);
        ret = true;
    }
    close(socket_id);
    return ret;
}

static bool http_put_device_online(char* ip) 
{
    /*connect to http server*/
    if (!connect_to_http_server(HOST, PORT)) {
        ESP_LOGE(TAG, "error connect to host");
        return false;
    }

    char http_request[256] = {0};
    char serial_number[13] = {0};
    config_get_serial_num(serial_number, 13);
    char http_body[100] =  {0};
    sprintf(http_body, "model_number=%s&serial_number=%s&fw_version=%s&ip_address=%s", MODEL_NUMBER, serial_number, FW_VERSION, ip);
    sprintf(http_request, HTTP_REQUEST_PUT, API_DEVICE_ONLINE, HOST, strlen(http_body), http_body);
    if (!send_request(http_request, strlen(http_request))) {
        ESP_LOGE(TAG, "error send request");
        close(socket_id);
        return false;
    }

    bool ret = false;
    char response[20] = {0};
    int len = 20;
    if (recv_response(response, &len)) {
        ESP_LOGD(TAG, "put device online");
        ret = true;
    }
    close(socket_id);
    return ret;
}

void device_online()
{
    ESP_LOGI(TAG, "device online");

    char ip[128] = {0};
    int len = 0;
    if (!http_get_public_ip(ip, &len)) {
        ESP_LOGE(TAG, "can't get public ip");
        return;
    }

    http_put_device_online(ip);
}
