#include "send-protobuf.h"

int sendProtobuf(char *host, int port, char *uri, char *xsfkey, uint8_t *buf, size_t bufsize) { 
  /*
   * 0 = Good 
   * 1 = Socket failed
   * 2 = Bad HTTP response
   * 3 = Lost HTTP connection after send
   * 4 = Lost HTTP connection before send
   */

  // Check to see if the WiFi is still alive
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin();
    while (WiFi.status() != WL_CONNECTED) {
      delay(5000);
    }
  }

#if defined(OTEL_SSL)
  WiFiClientSecure client;
  client.setInsecure();
#else
  WiFiClient client;
#endif

  if (!client.connect(host, port)) {
    return 1;
  }

  // Transmit the readings
  if (client.connected()) {
    client.print(F("POST "));
    client.print(uri);
    client.print(F(" HTTP/1.1\r\nHost: "));
    client.print(host);
    // client.print(F(":"));
    // client.print(String(port));
    client.print(F("\r\nContent-Type: application/x-protobuf\r\nX-SF-Token: "));
    client.print(xsfkey);
    client.print(F("\r\nContent-Length: "));
    client.print(String(bufsize));
    client.print(F("\r\n\r\n"));
    client.write(buf, bufsize);

#ifdef ARDUINO_ARCH_ESP32
    client.setTimeout(5);
#endif

    // Validate if we get a response - if not, abort
    if (client.connected()) {
      char resp[16] = {0};
      int len = client.readBytesUntil('\n', resp, 15);
      resp[len] = '\0';
      if(strcmp(resp, "HTTP/1.1 200 OK")) {
        client.flush();
        client.stop();
        return 2; 
      }
    } else {
      return 3;
    }
  } else {
    return 4;
  }

  // Flush and clear the connection
  client.flush();
  client.stop();

  return 0;
}
