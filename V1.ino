/*
  DozeGuard — ESP32 car firmware (Bluetooth LE)
  CognoSpace × Sadhana Infinity International School

  The DozeGuard web page (DozeGuard.html, opened in Chrome or Edge)
  runs the computer-vision model and sends the car what to do over Bluetooth LE.
  This sketch drives the motors, LEDs, buzzer and OLED.

  Wiring
    Motor driver (L298N, same as the WiFi Rover):  IN1 13 · IN3 26 · ENA 14 · ENB 27
    Buzzer    GPIO 4
    Red LED   GPIO 18  (alert)
    Green LED GPIO 19  (DAS on, driver awake)
    OLED 0.96" SSD1306 I2C 128x64:  SDA 21 · SCL 22 · address 0x3C

  Libraries (Arduino Library Manager)
    Adafruit SSD1306, Adafruit GFX Library   (BLE is built into the ESP32 core)
  Board
    ESP32 Dev Module, ESP32 Arduino core 3.x
    Tools > Partition Scheme: "Huge APP (3MB No OTA)" if the sketch is too big.

  How to connect
    1. Upload this sketch. The OLED shows "BLE ready".
    2. Open DozeGuard.html in Chrome/Edge, go to Real-time, press "Connect car",
       pick "DozeGuard-Car".
    Safety: if the page stops sending for 1.5 s or Bluetooth disconnects, the motors stop.

  Driving from the page
    Accelerator (or the up arrow) speeds the car up; Brake (or the down arrow) slows it to a stop.
    The car keeps the speed you let go at. DAS safety rules can slow or stop it at any time.
    Watch the Serial Monitor (115200) to see each speed command and the motor PWM it becomes.

  BLE protocol (service 6e400001-d7a1-4a1e-9c2a-44726976ae01)
    CMD  ...0002  write, 7 bytes: pwm, buzzer(0 off/1 beep/2 fast), red(0 off/1 on/2 blink),
                  green(0/1), state(0 DAS off/1 awake/2 drowsy/3 asleep),
                  eyes-closed time in tenths of a second, alert count
    MSG  ...0003  write, up to 16 characters shown big on the OLED ("" = none)
    INFO ...0004  read, firmware name and version
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ── Motor pins (from the WiFi Rover) ─────────────────────────
#define IN1 13
#define IN3 26
#define ENA 14
#define ENB 27

// ── Outputs ──────────────────────────────────────────────────
#define BUZZER    4
#define LED_RED   18
#define LED_GREEN 19
// Most 3-pin buzzer modules are active (0). Use 1 for a passive buzzer that needs a tone.
#define BUZZER_IS_PASSIVE 0   // 1 = passive buzzer
#define BUZZER_TONE_HZ    2400

// ── OLED ─────────────────────────────────────────────────────
#define OLED_SDA  21
#define OLED_SCL  22
#define OLED_ADDR 0x3C
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;

// ── Bluetooth LE ─────────────────────────────────────────────
#define BLE_NAME "DozeGuard-Car"
#define SVC_UUID  "6e400001-d7a1-4a1e-9c2a-44726976ae01"
#define CMD_UUID  "6e400002-d7a1-4a1e-9c2a-44726976ae01"
#define MSG_UUID  "6e400003-d7a1-4a1e-9c2a-44726976ae01"
#define INFO_UUID "6e400004-d7a1-4a1e-9c2a-44726976ae01"
volatile bool bleConnected = false;
volatile bool restartAdvertising = false;

// ── State sent by the web page ───────────────────────────────
int    pwmCmd   = 0;
int    buzMode  = 0;
int    redMode  = 0;
int    greenOn  = 0;
int    driverSt = 0;          // 0 off · 1 awake · 2 drowsy · 3 asleep
float  closedS  = 0;
int    alerts   = 0;
char   msgBuf[17]  = "";      // OLED text from the page (written by the BLE task)
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

volatile unsigned long lastCmdMs = 0;
bool linkUp = false;
const unsigned long LINK_TIMEOUT_MS = 1500;

// ── Motors (same logic as moveForward/stopMotors in the rover) ─
// ── Motor tuning ─────────────────────────────────────────────
// Small DC motors don't turn below a certain power. Any speed above 0 from the page
// is mapped into MIN_MOVE_PWM..255 so the car moves as soon as you press Accelerator.
// If the car hums but doesn't move, raise MIN_MOVE_PWM (try 130-160).
// If it jumps forward too fast, lower it (try 90-100).
#define MIN_MOVE_PWM 115
// When starting from a stop, give a short full-power push to get the wheels turning.
#define KICK_MS      150

bool motorsMoving = false;
unsigned long kickUntil = 0;

void stopMotors() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
  motorsMoving = false;
}

void moveForward(int spd) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, LOW);
  ledcWrite(ENA, spd);
  ledcWrite(ENB, spd);
}

int motorPower(int cmd) {                 // page PWM (1-255) -> real motor PWM (MIN_MOVE_PWM-255)
  if (cmd <= 0) return 0;
  return map(constrain(cmd, 1, 255), 1, 255, MIN_MOVE_PWM, 255);
}

void applyMotors() {
  int cmd = linkUp ? pwmCmd : 0;
  static int lastCmd = -1;
  if (cmd != lastCmd) {                   // log speed changes in the Serial Monitor (115200 baud)
    lastCmd = cmd;
    Serial.printf("Speed command %d -> motor PWM %d\n", cmd, motorPower(cmd));
  }
  if (cmd <= 0) { if (motorsMoving) stopMotors(); return; }
  unsigned long now = millis();
  if (!motorsMoving) { motorsMoving = true; kickUntil = now + KICK_MS; }
  moveForward(now < kickUntil ? 255 : motorPower(cmd));
}

// ── Buzzer ───────────────────────────────────────────────────
bool buzzerIsOn = false;
void buzzer(bool on) {
  if (on == buzzerIsOn) return;
  buzzerIsOn = on;
#if BUZZER_IS_PASSIVE
  ledcWriteTone(BUZZER, on ? BUZZER_TONE_HZ : 0);
#else
  digitalWrite(BUZZER, on ? HIGH : LOW);
#endif
}

void updateOutputs(unsigned long now) {
  bool slow = (now / 250) % 2 == 0;
  bool fast = (now / 120) % 2 == 0;

  if (!linkUp) {
    // No page connected: green off, red slow blink only if the link was lost mid-drive.
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, lastCmdMs > 0 ? ((now / 600) % 2 == 0) : LOW);
    buzzer(false);
    return;
  }
  digitalWrite(LED_GREEN, greenOn ? HIGH : LOW);
  digitalWrite(LED_RED, redMode == 1 ? HIGH : (redMode == 2 && slow) ? HIGH : LOW);
  buzzer(buzMode == 1 ? slow : buzMode == 2 ? fast : false);
}

// ── OLED ─────────────────────────────────────────────────────
void drawCentered(const String& s, int y, int size) {
  int16_t x1, y1; uint16_t w, h;
  oled.setTextSize(size);
  oled.getTextBounds(s, 0, y, &x1, &y1, &w, &h);
  oled.setCursor((128 - w) / 2, y);
  oled.print(s);
}

void drawOled(unsigned long now) {
  if (!oledOk) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("DozeGuard");
  oled.setCursor(104, 0);
  oled.print(linkUp ? " BLE" : "----");
  oled.drawFastHLine(0, 10, 128, SSD1306_WHITE);

  if (!linkUp) {
    drawCentered(lastCmdMs ? "NO LINK" : "BLE ready", 15, 2);
    oled.setTextSize(1);
    drawCentered(bleConnected ? "Connected, waiting" : "Open the web page", 38, 1);
    drawCentered(bleConnected ? "for the page..." : "and Connect car", 50, 1);
    oled.display();
    return;
  }

  static const char* names[] = { "DAS OFF", "AWAKE", "DROWSY", "ASLEEP!" };
  char msg[17];
  portENTER_CRITICAL(&stateMux);
  memcpy(msg, msgBuf, sizeof(msg));
  portEXIT_CRITICAL(&stateMux);
  String big = msg[0] ? String(msg) : String(names[constrain(driverSt, 0, 3)]);
  bool blinkOff = (driverSt == 3) && ((now / 300) % 2);
  if (!blinkOff) {
    if (big.length() > 10) drawCentered(big, 18, 1);
    else drawCentered(big, 15, 2);
  }

  oled.setTextSize(1);
  oled.setCursor(0, 36); oled.print("SPD");
  oled.drawRect(22, 36, 80, 7, SSD1306_WHITE);
  oled.fillRect(24, 38, map(pwmCmd, 0, 255, 0, 76), 3, SSD1306_WHITE);
  oled.setCursor(106, 36); oled.print(map(pwmCmd, 0, 255, 0, 100)); oled.print("%");

  oled.setCursor(0, 52);
  oled.print("EYES "); oled.print(closedS, 1); oled.print("s");
  String a = "ALERT " + String(alerts);
  oled.setCursor(128 - a.length() * 6, 52); oled.print(a);
  oled.display();
}

// ── Bluetooth callbacks ──────────────────────────────────────
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    bleConnected = true;
    Serial.println("Bluetooth: page connected.");
  }
  void onDisconnect(BLEServer* s) override {
    bleConnected = false;
    restartAdvertising = true;           // handled in loop()
    Serial.println("Bluetooth: disconnected.");
  }
};

class CmdCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    auto v = c->getValue();               // String (core 3.x) or std::string (core 2.x)
    if (v.length() < 7) return;
    portENTER_CRITICAL(&stateMux);
    pwmCmd   = (uint8_t)v[0];
    buzMode  = constrain((int)(uint8_t)v[1], 0, 2);
    redMode  = constrain((int)(uint8_t)v[2], 0, 2);
    greenOn  = (uint8_t)v[3] ? 1 : 0;
    driverSt = constrain((int)(uint8_t)v[4], 0, 3);
    closedS  = (uint8_t)v[5] / 10.0;
    alerts   = (uint8_t)v[6];
    lastCmdMs = millis();
    portEXIT_CRITICAL(&stateMux);
  }
};

class MsgCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    auto v = c->getValue();
    char tmp[17]; size_t n = 0;
    for (; n < v.length() && n < 16; n++) tmp[n] = toupper((uint8_t)v[n]);
    tmp[n] = 0;
    portENTER_CRITICAL(&stateMux);
    memcpy(msgBuf, tmp, n + 1);
    portEXIT_CRITICAL(&stateMux);
  }
};

void startBle() {
  BLEDevice::init(BLE_NAME);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService* svc = server->createService(SVC_UUID);

  BLECharacteristic* cmd = svc->createCharacteristic(CMD_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  cmd->setCallbacks(new CmdCB());
  BLECharacteristic* msg = svc->createCharacteristic(MSG_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  msg->setCallbacks(new MsgCB());
  BLECharacteristic* info = svc->createCharacteristic(INFO_UUID, BLECharacteristic::PROPERTY_READ);
  info->setValue("DozeGuard 2.1");
  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("Bluetooth: advertising as " BLE_NAME);
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  ledcAttach(ENA, 1000, 8);
  ledcAttach(ENB, 1000, 8);
  stopMotors();

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
#if BUZZER_IS_PASSIVE
  ledcAttach(BUZZER, BUZZER_TONE_HZ, 8);
  ledcWriteTone(BUZZER, 0);
#else
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
#endif

  Wire.begin(OLED_SDA, OLED_SCL);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!oledOk) Serial.println("OLED not found at 0x3C (check wiring, or try 0x3D).");

  // Start-up check: both LEDs and a short chirp
  digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_RED, HIGH); buzzer(true);
  if (oledOk) {
    oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE);
    drawCentered("DozeGuard", 14, 2); drawCentered("CognoSpace x Sadhana", 40, 1);
    oled.display();
  }
  delay(150); buzzer(false); delay(850);
  digitalWrite(LED_GREEN, LOW); digitalWrite(LED_RED, LOW);

  startBle();
}

// ── Loop ─────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  if (restartAdvertising) {               // let a new page connect after a disconnect
    restartAdvertising = false;
    delay(200);
    BLEDevice::startAdvertising();
  }

  // A fresh command arrived: the link is up, apply it.
  portENTER_CRITICAL(&stateMux);
  unsigned long last = lastCmdMs;
  portEXIT_CRITICAL(&stateMux);
  if (bleConnected && last && now - last <= LINK_TIMEOUT_MS) {
    if (!linkUp) Serial.println("Link up: page is sending commands.");
    linkUp = true;
    applyMotors();
  }

  // Fail-safe: Bluetooth dropped or no message for 1.5 s -> stop the car.
  if (linkUp && (!bleConnected || now - last > LINK_TIMEOUT_MS)) {
    linkUp = false;
    pwmCmd = 0;
    stopMotors();
    Serial.println("Link lost: motors stopped.");
  }

  updateOutputs(now);

  static unsigned long lastOled = 0;
  if (now - lastOled > 150) { lastOled = now; drawOled(now); }
  delay(5);
}
