/*
 * Example of an Arduino running two decoding workloads into OTLP:
 * - BLE transmissions
 * - RFM transmissions
 * Intended to run on a dual-core ESP32. In this case a ESP32-PICO-D4
 */

// For RFM
#include <SPI.h>
#include "src/rfm69hcw/rfm69hcw.h"
#include "src/hardware_pins.h"

// For BLE
#include <NimBLEDevice.h>

// For outbound connectivity (OTLP)
#include "src/connection_details.h"
#include "src/hwclock/hwclock.h"
#include "src/otel-protobuf/otel-protobuf.h"
#include "src/send-protobuf/send-protobuf.h"

// To interrogate a specific ECODAN deployment 
#define UNIT_ID 0xF880

// Queue stuff:
#define PLATFORM_FIFO_SIZE 8 // determines the maximum size to be stored
QueueHandle_t RFMQueue;
QueueHandle_t BLEQueue;

// RFM Global Vars
extern SPIClass *vspi = NULL;

// Matches the RFM payload with a unstructured message within
struct rfmpayload { 
  union {
    uint8_t raw[32];
    struct {
      uint8_t len;      // Size of message, always 0x1C
      uint8_t addr;     // 0x8B: node, 0x0B broadcast
      // Custom header
      uint8_t netmsb;   // Unique Network?
      uint8_t netlsb;   // Unique Network?  
      uint8_t dst;      // Dest: 0=RX, 1-9=RC
      uint8_t src;      // Src: 0=RX, 1-9=RC
      uint16_t cmd;     // Unknown - Status? Pairing?
      // Message
      uint8_t msg[20];  // Encapsulated message
      uint8_t crc8;
      uint8_t crc16msb;    // Internal 8-bit CRC of msg
      uint8_t crc16lsb;    // Internal 8-bit CRC of msg
      uint8_t pad[1];   // Padding to go from 29 to 32 bytes
    }__attribute__((packed));
  };
};

// BLE Global Vars 
BLEScan *pBLEScan;
// 0xfcd2 == BTHome
static BLEUUID svc_data_uuid("0000fcd2-0000-1000-8000-00805f9b34fb");
std::map<std::string, std::string> seenMsg;

// Custom for BLE, take one payload and make it individual metrics
struct blepayload {
  union {
    // Keep to 32 bytes for RP2040 (8 x 32bit FIFO between cores)
    uint8_t raw[32];
    struct {
      // 6 octets of the MAC, in reverse AKA "Native"
      uint8_t macrev[6];
      // Some values, such as lux, are 12 bits
      uint64_t value;
      // 17 characters for a description, null terminated
      char desc[18];
    }__attribute__((packed));
  };
};

// This processes the broadcasts
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice *device)
  {
    // Do we have anything with Service Data?
    if (device->haveServiceData())
    {
      // Go through it all
      for (int x = 0; x < device->getServiceDataCount(); x++)
      {
        // Does it match our Service Data UUID we are looking for?
        if (device->getServiceDataUUID(x).equals(svc_data_uuid))
        {
          // Unpack the service data 
          std::string svc_data = device->getServiceData(x);

          // See https://bthome.io/format/ for the format...
          // 0100  = BTHome v2, 0100 = unencrypted, trigger-based
          if (svc_data[0] == 0x44)
          {
            // If this is a new, unseen message...
            if (seenMsg[device->getAddress().toString()] != svc_data)
            {
              // First things first - register it as "seen" - no point collecting duplicates
              seenMsg[device->getAddress().toString()] = svc_data;

              // Create some storage to unpack it
              struct blepayload blemsg;

              // Start with logging the MAC
              memcpy(blemsg.macrev, device->getAddress().getNative(), 6);

              // Now unpack the rest of the payload              
              for (int y = 1; y < svc_data.length(); y++)
              {
                switch (svc_data[y]) {
                  case 0x21: // IR Motion
                    blemsg.value = svc_data[++y];
                    strncpy(blemsg.desc, "ir-motion", sizeof(blemsg.desc) - 1);
                    break;
                  case 0x2D: // Door: Open/Close
                    blemsg.value = svc_data[++y];
                    strncpy(blemsg.desc, "door-open", sizeof(blemsg.desc) - 1);
                    break;
                  // Unused, but skip the data according to its expected size
                  case 0x00: // Packet ID (counter)
                  case 0x01: // Battery (percentage)
                  case 0x3A: // Button (press))
                    ++y;
                    break;
                  case 0x05: // Illuminance (lux)
                    ++y; // & 0xFF -- LSB
                    ++y; // << 8
                    ++y; // << 16 -- MSB
                    // divide by 100.0 -- convert to float
                    break;
                  case 0x3F: // Rotation (degrees)
                    ++y; // & 0xFF
                    ++y; // << 8
                    // Divide by 10.0 -- convert to float;
                    break;
                  default:
                    // Maybe do something here? 
                    break;
                }
              }

              // Pop it onto the queue for further processing
              uint32_t fifobuf[PLATFORM_FIFO_SIZE] = {0}; 
              memcpy(fifobuf, &blemsg.raw, sizeof(uint32_t) * PLATFORM_FIFO_SIZE);

              if (xQueueSendFromISR(BLEQueue, &fifobuf[0], NULL))
                for (int i = 1; i < PLATFORM_FIFO_SIZE; i++)
                  xQueueSendFromISR(BLEQueue, &fifobuf[i], NULL);
            }
          }
        }
      }
    }
  }
};

void IRAM_ATTR rfmPayloadReady() {
  // Interrupt basics: keep fast as possible, don't use blocking, such as Serial.print()
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (spi_single_read(vspi, 0x28) & 0x04) {
    int len = 0;
    uint8_t packet[32];
    uint32_t fifobuf[PLATFORM_FIFO_SIZE] = {0}; // conforms to RP2040 buffer size/limits if we ever want to use that

    // RegIrqFlags2: PayloadReady, as we need all the bytes
    while(spi_single_read(vspi, 0x28) & 0x40) // RegIrqFlags2: FifoNotEmpty
      packet[len++] = spi_single_read(vspi, 0x00); // RegFIFO to memory until empty

    // If the first 4 bytes don't check out, discard
    // 1) Must equal the Base ID or within the 7 below it
    // 2) Size must be 28 (0x1C)
    // 3) Broadcast must match the 7 bits: 0x0001011
    if (((packet[2] << 8 | packet[3]) >= UNIT_ID - 7) && ((packet[2] << 8 | packet[3]) <= UNIT_ID) 
        && packet[0] == 0x1C && (packet[1] & 0x7F) == 0x0B) {
      
      // Map the payload into a 32bit element array
      memcpy(fifobuf, &packet, sizeof(uint32_t) * PLATFORM_FIFO_SIZE);

      // Add it to the queue if not full
      if (xQueueSendFromISR(RFMQueue, &fifobuf[0], NULL))
        for (int i = 1; i < PLATFORM_FIFO_SIZE; i++)
          xQueueSendFromISR(RFMQueue, &fifobuf[i], NULL);
    }
  }

  if(xHigherPriorityTaskWoken){
    portYIELD_FROM_ISR();
  }
}

void procRFMMsg (uint32_t fifobuf[]) {
  struct rfmpayload rfmmsg;
  
  memcpy(rfmmsg.raw, fifobuf, sizeof(uint32_t) * PLATFORM_FIFO_SIZE);

  // Either: boiler (0x0b) sending with UNIT ID / rc (0x8b) sending with UNIT_ID - rc number
  // Use the faster CRC check
  if (rfmmsg.crc8 == crc8(rfmmsg.raw, 8, 28)) {
    uint8_t payloadData[MAX_PROTOBUF_BYTES] = { 0 };
    Resourceptr ptr = NULL;
    ptr = addOteldata();
    addResAttr(ptr, "service.name", "jlm-home");

    int valid_data = 0;

    switch (rfmmsg.msg[0]) {
      // RC calling out ambient temperature
      case 0x43:
      {
        double *celcius = (double *)malloc(sizeof(double));
        *celcius = (rfmmsg.msg[5] & 0x7F) * 0.5;
        addMetric(ptr, "ecodan_temperature", "Ecodan Ambient Temp", "Cel", METRIC_GAUGE, 0, 0);
        addDatapoint(ptr, AS_DOUBLE, celcius);
        addDpAttr(ptr,"sensor","DIY Sensor");
        valid_data = 1;
        break;
      }
      // Boiler calling out state
      case 0x63:
      {
        uint64_t *opstate = (uint64_t *)malloc(sizeof(uint64_t));
        *opstate = rfmmsg.msg[6];
        addMetric(ptr, "ecodan_state", "Ecodan Operational State", "1", METRIC_GAUGE, 0, 0);
        addDatapoint(ptr, AS_INT, opstate);
        addDpAttr(ptr,"sensor","DIY Sensor");
        valid_data = 1;

        break;
      }
      default:
        break;
    }

    if (valid_data) {
      // for debug to show whats in the payload
      printOteldata(ptr);
    
      size_t payloadSize = buildProtobuf(ptr, payloadData, MAX_PROTOBUF_BYTES);
      // Send the data if there's something there
      if(payloadSize > 0) {
        // Define OTEL_SSL if SSL is required
        sendProtobuf(OTEL_HOST, OTEL_PORT, OTEL_URI, OTEL_XSFKEY, payloadData, payloadSize);
      }
    }
    freeOteldata(ptr);
  }
}

void procBLEMsg (uint32_t fifobuf[]) {
  struct blepayload blemsg;

  memcpy(blemsg.raw, fifobuf, sizeof(uint32_t) * PLATFORM_FIFO_SIZE);

  uint8_t payloadData[MAX_PROTOBUF_BYTES] = { 0 };
  Resourceptr ptr = NULL;
  ptr = addOteldata();
  addResAttr(ptr, "service.name", "jlm-home");

  char addr[18] = {0};
  sprintf(addr, "%02x:%02x:%02x:%02x:%02x:%02x", blemsg.macrev[5], blemsg.macrev[4], blemsg.macrev[3], blemsg.macrev[2], blemsg.macrev[1], blemsg.macrev[0]);
  uint64_t *value = (uint64_t *)malloc(sizeof(uint64_t));
  *value = blemsg.value;
  addMetric(ptr, blemsg.desc, "N/A", "1", METRIC_GAUGE, 0, 0);
  addDatapoint(ptr, AS_INT, value);
  addDpAttr(ptr, "sensor", addr);

  printOteldata(ptr);
    
  size_t payloadSize = buildProtobuf(ptr, payloadData, MAX_PROTOBUF_BYTES);
  // Send the data if there's something there
  if(payloadSize > 0) {
    // Define OTEL_SSL if SSL is required
    sendProtobuf(OTEL_HOST, OTEL_PORT, OTEL_URI, OTEL_XSFKEY, payloadData, payloadSize);
  }
  freeOteldata(ptr);
}

void setup() {
  // Send output on serial console - need to enable CDC on boot to work
  Serial.begin(115200);

  // Takes about a second for the Serial to kick in on the M5 Atom Lite
  delay(1000);

  // Connect to WiFi before we attempt anything
  WiFi.begin(WIFI_SSID, WIFI_PSK);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println(F("...Waiting for WiFi..."));
    delay(5000);
  }
  Serial.println(F("WiFi Connected"));

  // NTP
  setHWClock(NTP_HOST);
  Serial.println(F("NTP Synced"));
 
  // RFM SPI setup
  vspi = new SPIClass(HSPI);
  vspi->begin(SOCKET_SPI_SCK, SOCKET_SPI_MISO, SOCKET_SPI_MOSI, SOCKET_SPI_SS);
  
  // RFM Pins
  pinMode(vspi->pinSS(), OUTPUT); // Device Select
  pinMode(SOCKET_I2C_SDA, OUTPUT); // Reset
  pinMode(SOCKET_I2C_SCL, INPUT);  // Data

  // Initialise the RFM, passing SPI and Reset pins
  rfminit(vspi, SOCKET_I2C_SDA, PKT_FSK_NONE, 868.299, 9600, FSK_12KHZ);

  // RFM Queuing and Kickoff
  RFMQueue = xQueueCreate(PLATFORM_FIFO_SIZE, sizeof(uint32_t));
  pinMode(digitalPinToInterrupt(SOCKET_I2C_SCL), INPUT);
  attachInterrupt(SOCKET_I2C_SCL, rfmPayloadReady, RISING);

  // Bluetooth setup()
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); // create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  pBLEScan->setActiveScan(false); // Passive

  // BLE Queuing and Kickoff
  BLEQueue = xQueueCreate(PLATFORM_FIFO_SIZE, sizeof(uint32_t));
  xTaskCreatePinnedToCore(taskLoop1, "Loop 1 on Core 0", 65536, NULL, 0, NULL, 0);
}

void taskLoop1(void *) {
  while (true)
    loop1();
}

void loop1()
{
  // Continuous passive scan and dump of BLE messages
  BLEScanResults foundDevices = pBLEScan->start(5, false);
  pBLEScan->clearResults();
}

void loop() {
  uint32_t fifobuf[PLATFORM_FIFO_SIZE];

  // RFM messages
  if (uxQueueMessagesWaiting(RFMQueue) >= PLATFORM_FIFO_SIZE) {
    while (uxQueueMessagesWaiting(RFMQueue) >= PLATFORM_FIFO_SIZE)
      for (int i = 0; i < PLATFORM_FIFO_SIZE; i++)
        xQueueReceive(RFMQueue, &fifobuf[i], (TickType_t) 10);
    procRFMMsg(fifobuf);
  }

  // BLE messages
  if (uxQueueMessagesWaiting(BLEQueue) >= PLATFORM_FIFO_SIZE) {
    while (uxQueueMessagesWaiting(BLEQueue) >= PLATFORM_FIFO_SIZE)
      for (int i = 0; i < PLATFORM_FIFO_SIZE; i++)
        xQueueReceive(BLEQueue, &fifobuf[i], (TickType_t) 10);
    procBLEMsg(fifobuf);
  }
}
