#include <Arduino.h>
#include <WiFi.h> // lets me do the wifi STA mode thing
#include <esp_now.h> // espnow api for radio comms

#include "freertos/FreeRTOS.h" // for multi tasking system 
#include "freertos/task.h" // lets me create tasks
#include "freertos/queue.h" // lets me create (mailbox)  queue

// -------------------- CONFIG --------------------
// config stuff basically adding the mac address of the UGV in UAV code to add to pair esp_now later
static uint8_t UGV_MAC[6] = {0xF8, 0xB3, 0xB7, 0x20, 0x69, 0xC0}; 

// -------------------- Protocol --------------------
/* i had to add protocol cause , when i was sending raw bytes over usb, sometimes a byte got lost or corrupted
 a protocol in this case is like a set of grammar rules. if the message starts with a specific "start" byte 
 and ends with a "checksum" we can prove that the data is 100% correct before using it.
*/
static const uint8_t SOF = 0xAA; // start of FRAME: marker (170)
static const uint8_t TYPE_TELEM = 1; // used for UAV -> UGV (telemetry data)
static const uint8_t TYPE_CMD   = 2; // used for UGV -> UAV (commands)

// -------------------- Data Structs --------------------

// Telemetry: what the UAV sends (18 bytes total)
// attribute (packed) is just making sure to not add the padding that c usually does
typedef struct __attribute__((packed)) {
  uint32_t seq;    // sequence number
  uint32_t t_ms;   // timestamp in milliseconds
  float vx;        // velocity x
  float vy;        // velocity y
  uint8_t marker;  // 1 if marker yes, 0 if not
  uint8_t estop;   // 1 if the UAV is in Emergency LAND mode
} TelemetryPayload;

// Command: what the UAV receives from the ground vehicle (6 bytes total)
typedef struct __attribute__((packed)) {
  uint32_t cmdSeq; // command sequence number
  uint8_t  cmd;    // 1:ARM, 2:DISARM, 3:TAKEOFF, 4:LAND, 5:MOVE
  uint8_t  estop;  // 1 to force a LAND, 0 otherwise
} CommandPayload;

// -------------------- Queues (Mailboxes) --------------------
static QueueHandle_t qTelemToNow  = nullptr; // Jetson (USB) -> send out over Radio
static QueueHandle_t qCmdToNow    = nullptr; // Jetson (USB) -> send out over Radio (to UGV)
static QueueHandle_t qCmdToSerial = nullptr; // Incoming Radio -> forward to Jetson (USB)

// -------------------- Helpers (Security Guard) --------------------

// basically this will calculate a XOR Checksum (fingerprint)
// it combines everything into one 8 bit number to prove data is clean
static uint8_t checksum_xor(uint8_t type, uint8_t len, const uint8_t* payload) {
  uint8_t c = type ^ len;
  for (uint8_t i = 0; i < len; i++) c ^= payload[i];
  return c;
}

// this function wraps data in a "Protocol Frame" for serial
static void serial_send_frame(uint8_t type, const uint8_t* payload, uint8_t len) {
  uint8_t chk = checksum_xor(type, len, payload);
  Serial.write(SOF);      // 1. send "Hello" byte
  Serial.write(type);     // 2. send type
  Serial.write(len);      // 3. send length
  if (len) Serial.write(payload, len); // 4. send data
  Serial.write(chk);      // 5. send signature
}

// -------------------- ESP-NOW Radio Callback --------------------

// this runs automatically whenever a radio packet hits the UAV's antenna
// currently used for Emergency LAND commands coming FROM the Ground
static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  // m checking if size matches Type + Command struct
  if (len == (1 + (int)sizeof(CommandPayload))) {
    if (data[0] == TYPE_CMD) {
      CommandPayload cmd;
      memcpy(&cmd, data + 1, sizeof(cmd));
      if (qCmdToSerial) xQueueSend(qCmdToSerial, &cmd, 0);
    }
  }
}

// -------------------- Serial Parser (The Gatekeeper) --------------------

enum ParseState { WAIT_SOF, WAIT_TYPE, WAIT_LEN, WAIT_PAYLOAD, WAIT_CHK };

typedef struct {
  ParseState st;
  uint8_t type;
  uint8_t len;
  uint8_t idx;
  uint8_t buf[64];
} SerialParser;

static void parser_init(SerialParser* p) {
  p->st = WAIT_SOF;
}

static bool parser_step(SerialParser* p, uint8_t b, uint8_t* outType, uint8_t* outLen, uint8_t* outPayload) {
  switch (p->st) {
    case WAIT_SOF:
      if (b == SOF) p->st = WAIT_TYPE;
      break;
    case WAIT_TYPE:
      p->type = b;
      p->st = WAIT_LEN;
      break;
    case WAIT_LEN:
      p->len = b;
      if (p->len > 64) { parser_init(p); break; }
      p->idx = 0;
      p->st = (p->len == 0) ? WAIT_CHK : WAIT_PAYLOAD;
      break;
    case WAIT_PAYLOAD:
      p->buf[p->idx++] = b;
      if (p->idx >= p->len) p->st = WAIT_CHK;
      break;
    case WAIT_CHK: {
      uint8_t expected = checksum_xor(p->type, p->len, p->buf);
      if (b == expected) {
        *outType = p->type;
        *outLen = p->len;
        memcpy(outPayload, p->buf, p->len);
        parser_init(p);
        return true;
      }
      parser_init(p);
      break;
    }
  }
  return false;
}

// -------------------- Tasks (The Workers) --------------------

// Worker 1: Listens to Jetson (USB) and parses data to send over Radio
void serialRxTask(void* pv) {
  (void)pv;
  SerialParser parser;
  parser_init(&parser);
  uint8_t fType, fLen, payload[64];

  for (;;) {
    while (Serial.available() > 0) {
      uint8_t b = (uint8_t)Serial.read();
      if (parser_step(&parser, b, &fType, &fLen, payload)) {
        // Did we get Telemetry?
        if (fType == TYPE_TELEM && fLen == sizeof(TelemetryPayload)) {
          TelemetryPayload t;
          memcpy(&t, payload, sizeof(t));
          if (qTelemToNow) xQueueSend(qTelemToNow, &t, 0);
        }
        // Did we get a Command (for the UGV)?
        else if (fType == TYPE_CMD && fLen == sizeof(CommandPayload)) {
          CommandPayload c;
          memcpy(&c, payload, sizeof(c));
          if (qCmdToNow) xQueueSend(qCmdToNow, &c, 0);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// Worker 2: Sends Telemetry over Radio to UGV
void radioTxTelemTask(void* pv) {
  (void)pv;
  TelemetryPayload t;
  uint8_t pkt[1 + sizeof(TelemetryPayload)];
  pkt[0] = TYPE_TELEM;

  for (;;) {
    if (xQueueReceive(qTelemToNow, &t, portMAX_DELAY) == pdTRUE) {
      memcpy(pkt + 1, &t, sizeof(t));
      esp_now_send(UGV_MAC, pkt, sizeof(pkt));
    }
  }
}

// Worker 3: Sends Commands over Radio to UGV
void radioTxCmdTask(void* pv) {
  (void)pv;
  CommandPayload c;
  uint8_t pkt[1 + sizeof(CommandPayload)];
  pkt[0] = TYPE_CMD;

  for (;;) {
    if (xQueueReceive(qCmdToNow, &c, portMAX_DELAY) == pdTRUE) {
      memcpy(pkt + 1, &c, sizeof(c));
      esp_now_send(UGV_MAC, pkt, sizeof(pkt));
    }
  }
}

// Worker 4: Forwards Radio-Commands to Jetson (USB)
void serialTxTask(void* pv) {
  (void)pv;
  CommandPayload c;
  for (;;) {
    if (xQueueReceive(qCmdToSerial, &c, portMAX_DELAY) == pdTRUE) {
      serial_send_frame(TYPE_CMD, (const uint8_t*)&c, (uint8_t)sizeof(c));
    }
  }
}

// -------------------- Setup & Loop --------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;

  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, UGV_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  qTelemToNow  = xQueueCreate(10, sizeof(TelemetryPayload));
  qCmdToNow    = xQueueCreate(10, sizeof(CommandPayload));
  qCmdToSerial = xQueueCreate(10, sizeof(CommandPayload));

  // Hiring the Workers
  xTaskCreate(serialRxTask,      "SerRx",   4096, nullptr, 2, nullptr);
  xTaskCreate(radioTxTelemTask,  "RadTelem",4096, nullptr, 2, nullptr);
  xTaskCreate(radioTxCmdTask,    "RadCmd",  4096, nullptr, 2, nullptr);
  xTaskCreate(serialTxTask,      "SerTx",   4096, nullptr, 1, nullptr);

  Serial.println("UAV Bridge Ready!");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
