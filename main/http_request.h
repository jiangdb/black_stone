/*
 * =====================================================================================
 *
 *       Filename:  http_request.h
 *
 *    Description:  Header file for http request
 *
 *        Version:  1.0
 *        Created:  11/17/2017 11:21:30
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Dong Bin Jiang 
 *   Organization:  
 *
 * =====================================================================================
 */
#ifndef _HTTP_REQUEST_H
#define _HTTP_REQUEST_H

#define HOST_PRODUCTION         "bm.timemore.com"
#define HOST_STAGING            "bs.ziipoo.com.cn"
#define PORT                    80
#define API_OTA_DOWNLOAD        "/api/v1/device/ota/%d/download"

void device_online();

#endif /* _HTTP_REQUEST_H */
