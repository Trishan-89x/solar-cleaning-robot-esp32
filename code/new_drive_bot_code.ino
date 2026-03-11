#include <WiFi.h>

/* ================= WiFi ================= */
const char *ssid = "ESP_BOT";
const char *password = "pass";

WiFiServer server(80);
WiFiClient client;

/* ================= Motor Pins ================= */
#define L_DIR  15
#define L_PWM  16
#define R_DIR  17
#define R_PWM  18

bool driveMoving = false;

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  brake();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
}

/* ================= LOOP ================= */
void loop() {
  handleWiFiClient();
}

/* ================= CLIENT HANDLER ================= */
void handleWiFiClient() {
  client = server.available();
  if (!client) return;

  String req = client.readStringUntil('\r');
  client.flush();

  Serial.println(req);

  int idx = req.indexOf("cmd=");
  if (idx == -1) {
    client.stop();
    return;
  }

  String data = req.substring(idx + 4);
  data = data.substring(0, data.indexOf(' '));

  char command = data.charAt(0);
  int value = data.substring(1).toInt();

  if (command == 'f') forward(value);
  else if (command == 'r') reverse(value);
  else if (command == 'b') brake();

  client.println("HTTP/1.1 200 OK");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Content-Type: text/plain");
  client.println();
  client.println("OK");

  client.stop();
}

/* ================= MOTOR FUNCTIONS ================= */

void forward(int pwm) {
  pwm = constrain(pwm, 0, 255);

  digitalWrite(L_DIR, HIGH);
  digitalWrite(R_DIR, HIGH);

  analogWrite(L_PWM, pwm);
  analogWrite(R_PWM, pwm);
}

void reverse(int pwm) {
  pwm = constrain(pwm, 0, 255);

  digitalWrite(L_DIR, LOW);
  digitalWrite(R_DIR, LOW);

  analogWrite(L_PWM, pwm);
  analogWrite(R_PWM, pwm);
}

void brake() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}#include <WiFi.h>

/* ================= WiFi ================= */
const char *ssid = "ESP_BOT";
const char *password = "pass";

WiFiServer server(80);
WiFiClient client;

/* ================= Motor Pins ================= */
#define L_DIR  15
#define L_PWM  16
#define R_DIR  17
#define R_PWM  18

bool driveMoving = false;

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  pinMode(L_DIR, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_PWM, OUTPUT);

  brake();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
}

/* ================= LOOP ================= */
void loop() {
  handleWiFiClient();
}

/* ================= CLIENT HANDLER ================= */
void handleWiFiClient() {
  client = server.available();
  if (!client) return;

  String req = client.readStringUntil('\r');
  client.flush();

  Serial.println(req);

  int idx = req.indexOf("cmd=");
  if (idx == -1) {
    client.stop();
    return;
  }

  String data = req.substring(idx + 4);
  data = data.substring(0, data.indexOf(' '));

  char command = data.charAt(0);
  int value = data.substring(1).toInt();

  if (command == 'f') forward(value);
  else if (command == 'r') reverse(value);
  else if (command == 'b') brake();

  client.println("HTTP/1.1 200 OK");
  client.println("Access-Control-Allow-Origin: *");
  client.println("Content-Type: text/plain");
  client.println();
  client.println("OK");

  client.stop();
}

/* ================= MOTOR FUNCTIONS ================= */

void forward(int pwm) {
  pwm = constrain(pwm, 0, 255);

  digitalWrite(L_DIR, HIGH);
  digitalWrite(R_DIR, HIGH);

  analogWrite(L_PWM, pwm);
  analogWrite(R_PWM, pwm);
}

void reverse(int pwm) {
  pwm = constrain(pwm, 0, 255);

  digitalWrite(L_DIR, LOW);
  digitalWrite(R_DIR, LOW);

  analogWrite(L_PWM, pwm);
  analogWrite(R_PWM, pwm);
}

void brake() {
  analogWrite(L_PWM, 0);
  analogWrite(R_PWM, 0);
}
