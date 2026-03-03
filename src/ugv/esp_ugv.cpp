#include <Arduino.h>
#include <WiFi.h> // lets me do the wifi STA mode thing 
#include <esp_now.h> // espnow api for radio comms

#include "freertos/FreeRTOS.h" // for multi tasking system
#include "freertos/task.h" // lets me create tasks
#include "freertos/queue.h" // lets me create (mailbox) queue

// -------------------- CONFIG --------------------
// this is the mac address of the UAV. UGV sends commands here
static uint8_t UAV_MAC[6] = {0xf8, 0xb3, 0xb7, 0x20, 0x25, 0xa8}; 

// -------------------- Protocol --------------------
/* i had to add protocol cause , when i was sending raw bytes over usb, sometimes a byte got lost or corrupted
 a protocol in this case is like a set of grammar rules. if the message starts with a specific "start" byte 
 and ends with a "checksum" we can prove that the data is 100% correct before using it.
*/
static const uint8_t SOF = 0xAA; // start of FRAME marker (170 in decimal)
static const uint8_t TYPE_TELEM = 1; // from Air (UAV) 
static const uint8_t TYPE_CMD   = 2; // from Ground (Pi/PC)
static const uint8_t TYPE_MSG   = 3; // raw string messages (from Air)

// -------------------- Data Structs --------------------

// Telemetry: what the UAV sends (18 bytes total)
// __attribute__((packed)) makes sure compiler doesnt add junk padding
typedef struct __attribute__((packed)) {
  uint32_t seq;    // packet sequence number
  uint32_t t_ms;   // time in ms when sent
  float vx;        // velocity x
  float vy;        // velocity y
  uint8_t marker;  // 1 if target seen, 0 if not
  uint8_t estop;   // 1 if UAV is in emergency LAND
} TelemetryPayload;

// Command: what the ground sends to UAV (6 bytes total)
typedef struct __attribute__((packed)) {
  uint32_t cmdSeq; // command sequence number
  uint8_t  cmd;    // 1:ARM, 2:DISARM, 3:TAKEOFF, 4:LAND, 5:MOVE
  uint8_t  estop;  // 1 to force LAND, 0 otherwise
} CommandPayload;

// -------------------- QUEUES (Mailboxes) --------------------
static QueueHandle_t qTelemToSerial = nullptr; // Radio -> forward to Pi (USB)
static QueueHandle_t qCmdToSerial   = nullptr; // Radio (from Air) -> forward to Pi
static QueueHandle_t qMsgToSerial   = nullptr; // Radio (from Air) -> forward to Pi
static QueueHandle_t qTelemToNow    = nullptr; // Pi (USB) -> send out over Radio (to UAV)
static QueueHandle_t qCmdToNow      = nullptr; // Pi (USB) -> send to Radio
static QueueHandle_t qMsgToNow      = nullptr; // Pi (USB) -> send to Radio (Strings)

// -------------------- MUTEX (Safety Lock) --------------------
static SemaphoreHandle_t serialMutex = nullptr; // prevents scrambled USB data

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
  uint8_t chk = checksum_xor(type, len, payload); // get the signature
  
  // Use the lock so other tasks don't interrupt while we are typing a frame
  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    Serial.write(SOF);      // 1. send "Hello" byte
    Serial.write(type);     // 2. send type
    Serial.write(len);      // 3. send length
    if (len > 0) Serial.write(payload, len); // 4. send actual data
    Serial.write(chk);      // 5. send signature
    xSemaphoreGive(serialMutex);
  }
}

// -------------------- ESP-NOW Radio Callback --------------------

// this runs automatically when a radio packet arrives from the UAV
static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < 1) return;
  uint8_t fType = data[0];

  // 1. Radio Echo (Debug): Tell the RPi we got SOMETHING
  char dbg[64];
  snprintf(dbg, 64, "ESP: Got Radio PKT type %d len %d", fType, len);
  if (qMsgToSerial) {
    uint8_t dBuf[64];
    memset(dBuf, 0, 64);
    memcpy(dBuf, dbg, strlen(dbg));
    xQueueSend(qMsgToSerial, dBuf, 0);
  }

  // 2. Catching Telemetry
  if (fType == TYPE_TELEM && len >= (int)sizeof(TelemetryPayload)) {
    TelemetryPayload t;
    memcpy(&t, data + 1, sizeof(t));
    if (qTelemToSerial) xQueueSend(qTelemToSerial, &t, 0);
  }
  // 3. Catching Commands coming FROM THE AIR (Jetson mission)
  else if (fType == TYPE_CMD && len >= (int)sizeof(CommandPayload)) {
    CommandPayload cmd;
    memcpy(&cmd, data + 1, sizeof(cmd));
    if (qCmdToSerial) xQueueSend(qCmdToSerial, &cmd, 0);
  }
  // 4. Catching Messages from the UAV
  else if (fType == TYPE_MSG) {
      uint8_t payload[64];
      memset(payload, 0, 64);
      uint8_t msgLen = (len - 1 > 64) ? 64 : len - 1;
      memcpy(payload, data+1, msgLen);
      if (qMsgToSerial) xQueueSend(qMsgToSerial, payload, 0);
  }
}

// -------------------- Serial Parser (The Gatekeeper) --------------------

// stages for our state machine
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

// this function "steps" through bytes from Pi to find a valid frame
// it returns true only when a 100% clean message is found
static bool parser_step(SerialParser* p, uint8_t b, uint8_t* outType, uint8_t* outLen, uint8_t* outPayload) {
  switch (p->st) {
    case WAIT_SOF:
      if (b == SOF) p->st = WAIT_TYPE; // found 0xAA!
      break;
    case WAIT_TYPE:
      p->type = b;
      p->st = WAIT_LEN;
      break;
    case WAIT_LEN:
      p->len = b;
      if (p->len > 64) { parser_init(p); break; } // sanity check
      p->idx = 0;
      p->st = (p->len == 0) ? WAIT_CHK : WAIT_PAYLOAD;
      break;
    case WAIT_PAYLOAD:
      p->buf[p->idx++] = b;
      if (p->idx >= p->len) p->st = WAIT_CHK; // got all data
      break;
    case WAIT_CHK: {
      uint8_t expected = checksum_xor(p->type, p->len, p->buf);
      if (b == expected) { // signature matches! 
        *outType = p->type;
        *outLen = p->len;
        memcpy(outPayload, p->buf, p->len);
        parser_init(p);
        return true;
      }
      parser_init(p); // bad signature, reset
      break;
    }
  }
  return false;
}

// -------------------- Tasks (The Workers) --------------------

// Worker 1: Takes Telemetry from mailbox and sends to Pi (USB)
void serialTxTelemTask(void* pv) {
  (void)pv;
  TelemetryPayload t;
  for (;;) {
    if (xQueueReceive(qTelemToSerial, &t, portMAX_DELAY) == pdTRUE) {
      serial_send_frame(TYPE_TELEM, (const uint8_t*)&t, (uint8_t)sizeof(t));
    }
  }
}

// Worker 2: Takes Commands from Air and sends to Pi (USB)
void serialTxCmdTask(void* pv) {
  (void)pv;
  CommandPayload c;
  for (;;) {
    if (xQueueReceive(qCmdToSerial, &c, portMAX_DELAY) == pdTRUE) {
      serial_send_frame(TYPE_CMD, (const uint8_t*)&c, (uint8_t)sizeof(c));
    }
  }
}

// Worker 3: Sends Messages from UAV to Pi (USB)
void serialTxMsgTask(void* pv) {
  (void)pv;
  uint8_t payload[64];
  for (;;) {
    if (xQueueReceive(qMsgToSerial, payload, portMAX_DELAY) == pdTRUE) {
      // Find valid length
      uint8_t len = 0;
      while (len < 64 && payload[len] != 0) len++;
      if (len == 0) continue;
      
      serial_send_frame(TYPE_MSG, payload, len);
    }
  }
}

// Worker 4: Listens to Pi (USB) and parses commands
void serialRxTask(void* pv) {
  (void)pv;
  SerialParser parser;
  parser_init(&parser);
  uint8_t type, len, payload[64];

  for (;;) {
    while (Serial.available() > 0) {
      uint8_t b = (uint8_t)Serial.read();
      // feed byte into the gatekeeper
      if (parser_step(&parser, b, &type, &len, payload)) {
        if (type == TYPE_CMD && len == sizeof(CommandPayload)) {
          CommandPayload c;
          memcpy(&c, payload, sizeof(c));
          // drop command in radio mailbox
          if (qCmdToNow) xQueueSend(qCmdToNow, &c, 0);
        }
        else if (type == TYPE_TELEM && len == sizeof(TelemetryPayload)) {
          TelemetryPayload t;
          memcpy(&t, payload, sizeof(t));
          if (qTelemToNow) xQueueSend(qTelemToNow, &t, 0);
        }
        else if (type == TYPE_MSG) {
          uint8_t msgBuf[64];
          memset(msgBuf, 0, 64);
          memcpy(msgBuf, payload, len);
          if (qMsgToNow) xQueueSend(qMsgToNow, msgBuf, 0);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // rest a bit to be nice to CPU
  }
}

// Worker 5: Takes telemetry from mailbox and sends to UAV Radio
void radioTxTelemTask(void* pv) {
  (void)pv;
  TelemetryPayload t;
  uint8_t pkt[1 + sizeof(TelemetryPayload)];
  pkt[0] = TYPE_TELEM;

  for (;;) {
    if (xQueueReceive(qTelemToNow, &t, portMAX_DELAY) == pdTRUE) {
      memcpy(pkt + 1, &t, sizeof(t));
      esp_now_send(UAV_MAC, pkt, sizeof(pkt));
    }
  }
}

// Worker 6: Takes commands from mailbox and sends to Radio
void espNowTxTask(void* pv) {
  (void)pv;
  CommandPayload c;
  TelemetryPayload t;
  uint8_t m[64];
  
  uint8_t pktCmd[1 + sizeof(CommandPayload)];
  pktCmd[0] = TYPE_CMD;
  uint8_t pktTelem[1 + sizeof(TelemetryPayload)];
  pktTelem[0] = TYPE_TELEM;
  uint8_t pktMsg[1 + 64];
  pktMsg[0] = TYPE_MSG;

  for (;;) {
    // 1. Check for commands (to Air)
    if (xQueueReceive(qCmdToNow, &c, 0) == pdTRUE) {
      memcpy(pktCmd + 1, &c, sizeof(c));
      esp_now_send(UAV_MAC, pktCmd, sizeof(pktCmd));
    }
    // 2. Check for telemetry (to Air)
    if (xQueueReceive(qTelemToNow, &t, 0) == pdTRUE) {
      memcpy(pktTelem + 1, &t, sizeof(t));
      esp_now_send(UAV_MAC, pktTelem, sizeof(pktTelem));
    }
    // 3. Check for string messages (to Air)
    if (xQueueReceive(qMsgToNow, m, 0) == pdTRUE) {
      uint8_t mLen = 0;
      while (mLen < 64 && m[mLen] != 0) mLen++;
      if (mLen > 0) {
        memcpy(pktMsg + 1, m, mLen);
        esp_now_send(UAV_MAC, pktMsg, 1 + mLen);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// -------------------- SETUP AND LOOP --------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize WiFi for ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) return;

  // Register the Listener
  esp_now_register_recv_cb(onDataRecv);

  // Add the Partner (UAV)
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, UAV_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  // Creating the Lock
  serialMutex = xSemaphoreCreateMutex();

  // Physically build the Mailboxes
  qTelemToSerial = xQueueCreate(10, sizeof(TelemetryPayload));
  qCmdToSerial   = xQueueCreate(10, sizeof(CommandPayload));
  qMsgToSerial   = xQueueCreate(10, 64);
  qTelemToNow    = xQueueCreate(10, sizeof(TelemetryPayload));
  qCmdToNow      = xQueueCreate(10, sizeof(CommandPayload));
  qMsgToNow      = xQueueCreate(10, 64);

  // Hire the Workers (Tasks)
  xTaskCreate(serialTxTelemTask, "TxTelem", 4096, NULL, 2, NULL);
  xTaskCreate(serialTxCmdTask,   "TxCmd",   4096, NULL, 2, NULL);
  xTaskCreate(serialTxMsgTask,   "TxMsg",   4096, NULL, 2, NULL);
  xTaskCreate(serialRxTask,      "SerRx",   4096, NULL, 2, NULL);
  xTaskCreate(espNowTxTask,      "NowTx",   4096, NULL, 2, NULL);

  Serial.println("====================================");
  Serial.print("UGV Bridge Ready! MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println("====================================");
}

void loop() {
  // FreeRTOS tasks do everything, so main loop just sleeps
  vTaskDelay(portMAX_DELAY); 
}
