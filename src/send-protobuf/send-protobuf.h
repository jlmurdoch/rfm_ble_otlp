#ifndef __SEND_PROTOBUF_H_
#define __SEND_PROTOBUF_H_

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "../connection_details.h"

int sendProtobuf(char *host, int port, char *uri, char *apikey, uint8_t *buf, size_t bufsize);

#endif
