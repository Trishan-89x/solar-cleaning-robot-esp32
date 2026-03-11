/* 
 * Dual VL53L5CX Time-of-Flight Sensors + ICM-20948 IMU + LoRa Communication
 * Reads center 4 pixels from each ToF sensor and full IMU data
 * Includes Fall Detection using IMU with Pitch, Roll, and Yaw
 * Sends data via LoRa to receiver
 */

// ===================== LIBRARIES =====================
#include <Wire.h>                       // Library for I2C communication
#include <Adafruit_VL53L0X.h>  // Library for distance sensors
#include "ICM_20948.h"                  // Library for IMU sensor
#include <WiFi.h>                       // WiFi library for ESP32
#include <WiFiClient.h>                 // WiFi client library
#include <WiFiServer.h>                 // WiFi server library
#include <math.h>                       // Math functions library
#include <LoRa.h>                       // LoRa wireless library
#include <SPI.h>      // SPI communication library
#include "Kalman.h"  // Kalman filter library
#include <esp32-hal-ledc.h>




//---------------------------------------------------------------------------------------------------------------------------------------
// AP ssid and pass
const char *ssid = "MyESP32_AP";
const char *password = "12345678";


// Create a server on port 80
WiFiServer server(80);
// Client object for handling connections
WiFiClient client;
// String to store received data
String data = "";


//---------------------------------------------------------------------------------------------------------------------------------------
// LoRa module pin definitions
#define LORA_SS 5                       // LoRa chip select pin
#define LORA_RST 14                     // LoRa reset pin
#define LORA_DIO0 2                     // LoRa interrupt pin

// Motor and actuator control pins
#define pwm_drive 16                    // Drive motor speed pin
#define dir_drive 15                    // Drive motor direction pin
#define pwm_brush 18                    // Brush motor speed pin
#define dir_brush 17                    // Brush motor direction pin
#define lin_act_1_pwm 3                 // Actuator speed pin
#define lin_act_1_dir 6                 // Actuator direction pin

// ===================== TOF CONFIG =====================
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
#define XSHUT_1 47
#define XSHUT_2 48

// ====== Obstacle detection config ======
unsigned long lastToFCheck = 0;         // Last time distance was checked
const uint16_t OBSTACLE_THRESHOLD = 300; // Distance to detect obstacle (30cm)

// ====== FALL DETECTION CONFIG (IMU) ======
#define MAX_DELTA_PITCH_ROLL 20.0       // Max allowed tilt change
#define MAX_DELTA_YAW 45.0              // Max allowed yaw change

unsigned long lastIMUCheck = 0;         // Last time IMU was checked
const uint16_t IMU_CHECK_INTERVAL = 100; // Check every 100ms
unsigned long lastIMUUpdate = 0;        // Last time IMU was updated

// IMU angle variables
float initialPitch = 0.0;               // Starting pitch angle
float initialRoll = 0.0;                // Starting roll angle
float initialYaw = 0.0;                 // Starting yaw angle
float currentPitch = 0.0;               // Current pitch angle
float currentRoll = 0.0;                // Current roll angle
float currentYaw = 0.0;                 // Current yaw angle
// Kalman filters for each axis
Kalman kalmanRoll;
Kalman kalmanPitch;
Kalman kalmanYaw;

// Filtered IMU output (we'll copy into currentPitch/Roll/Yaw)
float imuPitch = 0.0;
float imuRoll = 0.0;
float imuYaw = 0.0;


bool isInitialAngleSet = false;         // Flag if angles are set

// Motor control variables
int rpm = 0;                            // Motor speed value
int pos = 0;                            // Current actuator position
int new_pos = 0;                        // Target actuator position
int dist = 0;                           // Distance to move actuator
int linear_actr_pwm = 255;              // Actuator speed value
long lastMPUCheck = 0;                  // Last time MPU was checked
long lastActionTime = 0;                // Last time action was performed
long lastCurrentCheck = 0;              // Last time current was checked

// ====== ROBOT STATUS FLAGS ======
bool driveMoving = false;               // Is drive motor moving?
bool brushMoving = false;               // Is brush motor moving?
bool actuatorMoving = false;            // Is actuator moving?
bool botDisoriented = false;            // Is robot fallen?
bool obstacleDetected = false;          // Is obstacle detected?
bool emergencyStop = false;             // Is emergency stop active?

// Sensor values for LoRa transmission
int tof1Distance = 0;                   // Distance from sensor 1
int tof2Distance = 0;                   // Distance from sensor 2


// IMU raw data values
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;  // Acceleration values
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;     // Rotation values
float magX = 0.0, magY = 0.0, magZ = 0.0;        // Magnetic field values
//float temperature = 0.0;                          // Temperature value

// LoRa transmission timing
unsigned long lastLoRaTransmission = 0; // Last time data was sent
const uint16_t LORA_INTERVAL = 2000;    // Send data every 2 seconds

// ===================== IMU CONFIG =====================
#define WIRE_PORT_IMU Wire1             // Use second I2C port for IMU
#define AD0_VAL 1                       // I2C address bit
#define SERIAL_PORT Serial              // Alias for Serial output

ICM_20948_I2C myICM;                    // IMU sensor object

// ===================== SETUP =====================
void setup() {
  // Initialize serial communication
  SERIAL_PORT.begin(115200);            // Start serial at 115200 baud
  delay(1000);                          // Wait 1 second
 SERIAL_PORT.println("Starting Dual ToF + IMU + LoRa Example");

  // --------- LoRa Initialization ---------
SERIAL_PORT.println("Initializing LoRa...");

SPI.begin(12, 13, 11, 5); 
LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

bool loraOk = false;
int attempts = 0;

while (!loraOk && attempts < 10) {
  if (LoRa.begin(433E6)) {
    loraOk = true;
  } else {
    SERIAL_PORT.println("LoRa failed...");
    delay(500);
    attempts++;
  }
}

if (loraOk) {
  LoRa.setSyncWord(0xA5);
  SERIAL_PORT.println("LoRa Initializing OK!");
} else {
  SERIAL_PORT.println("LoRa init failed permanently.");
}


  // --------- I2C Initialization ---------
  Wire.begin(8,9);                         // Start first I2C bus
  Wire.setClock(400000);           // Set fast speed for sensors

  WIRE_PORT_IMU.begin(35,36);           // Start second I2C for IMU
  WIRE_PORT_IMU.setClock(400000);       // Set medium speed for IMU

  // // --------- ToF Sensor Initialization ---------
  // // Reset sequence for sensor 1
// --------- ToF Initialization ---------
pinMode(XSHUT_1, OUTPUT);
pinMode(XSHUT_2, OUTPUT);

// Turn off both
digitalWrite(XSHUT_1, LOW);
digitalWrite(XSHUT_2, LOW);
delay(10);

// ---- Sensor 1 ----
digitalWrite(XSHUT_1, HIGH);
delay(100);

if (!lox1.begin(0x29)) {
  Serial.println("Sensor 1 not found");
  while (1);
}

// Change address of sensor 1
lox1.setAddress(0x30);
delay(10);


// ---- Sensor 2 ----
digitalWrite(XSHUT_2, HIGH);
delay(100);

if (!lox2.begin(0x29)) {
  Serial.println("Sensor 2 not found");
  while (1);
}

Serial.println("Both ToF sensors initialized.");
  // --------- IMU Initialization ---------
  bool imuInitialized = false;          // IMU start flag
  while (!imuInitialized) {             // Keep trying until IMU starts
    myICM.begin(WIRE_PORT_IMU, AD0_VAL);  // Try to start IMU
    SERIAL_PORT.print("IMU Init: ");    // Print status
    SERIAL_PORT.println(myICM.statusString());
    
    if (myICM.status != ICM_20948_Stat_Ok) {  // If failed
      SERIAL_PORT.println("Retrying IMU..."); // Print retry message
      delay(500);                       // Wait 500ms
    } else {
      imuInitialized = true;            // Set flag if success
    }
  }

  // --------- Set Initial Reference Angle for Fall Detection ---------
  SERIAL_PORT.println("Place robot on panel. Calculating initial tilt in 3 seconds...");
  delay(3000);                          // Wait 3 seconds

  // Take a reading to set the initial angle
  if (myICM.dataReady()) {              // If IMU data ready
    myICM.getAGMT();                    // Read sensor data
    calculateTilt(myICM.accX(), myICM.accY(), myICM.accZ(), &initialPitch, &initialRoll);  // Calculate tilt
   // ----- Compute averaged magnetometer heading -----
float sumHeading = 0;
const int samples = 10; // number of samples for averaging
for (int i = 0; i < samples; i++) {
    myICM.getAGMT();                    // Read fresh IMU data
    float h = atan2(myICM.magY(), myICM.magX()) * 180.0f / PI;  // raw heading
    if (h < 0) h += 360.0f;             // normalize 0..360
    sumHeading += h;
    delay(50);                           // small delay between readings
}
float initialMagHeading = sumHeading / samples; // averaged heading

// Set initial yaw
initialYaw = initialMagHeading;
currentYaw = initialMagHeading;

// Initialize Kalman filter
kalmanYaw.setAngle(initialYaw);
 
    isInitialAngleSet = true;           // Mark angles as set
    // initialize Kalman filters with reference angles
kalmanRoll.setAngle(initialRoll);
kalmanPitch.setAngle(initialPitch);


    SERIAL_PORT.print("Reference Angles Set - Pitch: ");  // Print angles
    SERIAL_PORT.print(initialPitch);
    SERIAL_PORT.print("°, Roll: ");
    SERIAL_PORT.print(initialRoll);
    SERIAL_PORT.print("°, Yaw: ");
    SERIAL_PORT.print(initialYaw);
    SERIAL_PORT.println("°");
  }

  // Pin Modes for motor control
  pinMode(dir_drive, OUTPUT);           // Set drive direction pin
  pinMode(pwm_drive, OUTPUT);           // Set drive speed pin
  pinMode(dir_brush, OUTPUT);           // Set brush direction pin
  pinMode(pwm_brush, OUTPUT);           // Set brush speed pin
  pinMode(lin_act_1_pwm, OUTPUT);       // Set actuator speed pin
  pinMode(lin_act_1_dir, OUTPUT);       // Set actuator direction pin

  // Reset all status flags
  resetStatusFlags();
  //----------------------------------------------------------------------------------------------------------------------------------------------
  // WiFi Setup
  Serial.println("Connecting to WIFI"); // Print WiFi message


  // Start AP
  WiFi.softAP(ssid, password);

  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());


ledcAttach(pwm_drive, 5000, 8);
ledcAttach(pwm_brush, 5000, 8);
ledcAttach(lin_act_1_pwm, 5000, 8);

 
  server.begin();                       // Start web server
  
}

// ===================== MAIN LOOP =====================
void loop() {
// -------- ToF Reading (VL53L0X) --------
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

lox1.rangingTest(&measure1, false);
if (measure1.RangeStatus != 4) {
  tof1Distance = measure1.RangeMilliMeter;
}

lox2.rangingTest(&measure2, false);
if (measure2.RangeStatus != 4) {
  tof2Distance = measure2.RangeMilliMeter;
}


checkToF();

  // -------- IMU Data Reading --------
  if (myICM.dataReady()) {            // If IMU data ready
    myICM.getAGMT();                   // Read sensor data
    
    // Store raw IMU data for transmission
    accelX = myICM.accX() / 1000.0;   // Convert to G
    accelY = myICM.accY() / 1000.0;
    accelZ = myICM.accZ() / 1000.0;
    gyroX = myICM.gyrX();             // Degrees per second
    gyroY = myICM.gyrY();
    gyroZ = myICM.gyrZ();
    magX = myICM.magX();              // Raw magnetometer
    magY = myICM.magY();
    magZ = myICM.magZ();
   // temperature = myICM.temp();       // Temperature
    
    printScaledAGMT(&myICM);           // Print data
    
    // ===== Kalman + magnetometer fusion =====
// compute dt (time since last IMU processing) — use your existing lastIMUUpdate variable
unsigned long now = millis();
float dt = 0.0;
if (lastIMUUpdate == 0) {
  // first iteration: fall back to IMU_CHECK_INTERVAL
  dt = IMU_CHECK_INTERVAL / 1000.0f;
} else {
  dt = (now - lastIMUUpdate) / 1000.0f;
}
lastIMUUpdate = now;

// 1) accelerometer-based angles (degrees)
float accRoll  = atan2(accelY, accelZ) * 180.0f / PI;
float accPitch = atan2(-accelX, sqrt(accelY*accelY + accelZ*accelZ)) * 180.0f / PI;

// 2) fuse accel + gyro for roll & pitch via Kalman
float fusedRoll  = kalmanRoll.getAngle(accRoll, gyroX, dt);
float fusedPitch = kalmanPitch.getAngle(accPitch, gyroY, dt);

// 3) tilt-compensate magnetometer and compute heading
// convert filtered roll/pitch to radians for compensation
float rollRad  = fusedRoll  * (PI / 180.0f);
float pitchRad = fusedPitch * (PI / 180.0f);

// Use raw magnetometer readings (you may need to apply calibration offsets here)
float mx = magX;
float my = magY;
float mz = magZ;

// Tilt compensation (Xh, Yh)
float Xh = mx * cos(pitchRad) + mz * sin(pitchRad);
float Yh = mx * sin(rollRad) * sin(pitchRad) + my * cos(rollRad) - mz * sin(rollRad) * cos(pitchRad);

// heading from tilt-compensated mag (0..360)
float magHeading = atan2(Yh, Xh) * 180.0f / PI;
if (magHeading < 0.0f) magHeading += 360.0f;

static float prevMagHeading = initialYaw;
magHeading = 0.9 * prevMagHeading + 0.1 * magHeading;  // low-pass filter
prevMagHeading = magHeading;


// 4) fuse magnetometer heading + gyroZ with Kalman for yaw
float fusedYaw = kalmanYaw.getAngle(magHeading, gyroZ, dt);
 


// normalize yaw to -180..180
if (fusedYaw > 180.0f) fusedYaw -= 360.0f;
if (fusedYaw < -180.0f) fusedYaw += 360.0f;

// 5) write fused results into the variables your code already uses
currentRoll  = fusedRoll;
currentPitch = fusedPitch;
currentYaw   = fusedYaw;

  }

  // -------- Fall Detection Check --------
  checkIMU();                         // Check for falls

  // -------- Handle Actuator Movement --------
  handleActuator();                   // Handle actuator

  // -------- Handle WiFi Commands --------
  handleWiFiClient();                 // Handle WiFi

  // -------- Send Data via LoRa --------
   if (millis() - lastLoRaTransmission >= LORA_INTERVAL) {  // If time to send
     sendLoRaData();                    // Send data
     lastLoRaTransmission = millis();   // Update time
   }

  delay(100);                          // Short delay
}

// ===================== HELPER FUNCTIONS =====================

/**
 * Prints average distance of center 4 pixels in centimeters and returns value
 * @param data VL53L5CX measurement data structure
 * @return Average distance in mm
 */
// int printToFData(VL53L5CX_ResultsData &data) {
//   const int centerIndices[] = {27, 28, 35, 36};  // Center pixel positions
//   float sum = 0;                    // Initialize sum
  
//   // Sum distances from center pixels
//   for (int i = 0; i < 4; i++) {    // Loop through 4 pixels
//     sum += data.distance_mm[centerIndices[i]];  // Add distance
//   }
  
//   int avgDistance = sum / 4.0f;     // Calculate average
  
//   // Convert to cm and print average
//  /*SERIAL_PORT.print("Center avg: ");  // Print label
//   SERIAL_PORT.print(avgDistance / 10.0f, 1);  // mm→cm
//   SERIAL_PORT.println(" cm");  */      // Print unit

//   return avgDistance;               // Return average
// }

void checkToF() {

  if (millis() - lastToFCheck < 100) return;

  lastToFCheck = millis();

  bool newObstacleDetected = false;

  if ((tof1Distance > 0 && tof1Distance < OBSTACLE_THRESHOLD) ||
      (tof2Distance > 0 && tof2Distance < OBSTACLE_THRESHOLD)) {

    newObstacleDetected = true;
  }

  if (newObstacleDetected && !obstacleDetected) {

    obstacleDetected = true;

    Serial.println("Obstacle detected! Stopping motors...");
    stop_brush();
    brake();
  }

  else if (!newObstacleDetected && obstacleDetected) {

    obstacleDetected = false;
    Serial.println("Obstacle cleared.");
  }
}


// === obstacle check function  ===
// void checkToF() {
//   if (millis() - lastToFCheck >= 100) {  // If 100ms passed
//     lastToFCheck = millis();            // Update time

//     int dist1 = -1;                     // Sensor 1 distance
//     int dist2 = -1;                     // Sensor 2 distance
//     bool newObstacleDetected = false;   // Temp obstacle flag

//     // --- Sensor 1 ---
//     if (myImager1.isDataReady() && myImager1.getRangingData(&measurementData1)) {  // If data ready
//       const int centerIndices[] = {27, 28, 35, 36};  // Center positions
//       float sum = 0;                  // Initialize sum
//       for (int i = 0; i < 4; i++) sum += measurementData1.distance_mm[centerIndices[i]];  // Add distances
//       dist1 = sum / 4.0f;             // Calculate average
//     }

//     // --- Sensor 2 ---
//     if (myImager2.isDataReady() && myImager2.getRangingData(&measurementData2)) {  // If data ready
//       const int centerIndices[] = {27, 28, 35, 36};  // Center positions
//       float sum = 0;                  // Initialize sum
//       for (int i = 0; i < 4; i++) sum += measurementData2.distance_mm[centerIndices[i]];  // Add distances
//       dist2 = sum / 4.0f;             // Calculate average
//     }

//     // --- Obstacle detection ---
//     if ((dist1 > 0 && dist1 < OBSTACLE_THRESHOLD) ||  // If obstacle near sensor 1
//         (dist2 > 0 && dist2 < OBSTACLE_THRESHOLD)) {  // Or obstacle near sensor 2
//       newObstacleDetected = true;     // Set flag
//     }

//     // Only print message if status changed
//     if (newObstacleDetected && !obstacleDetected) {
//       obstacleDetected = true;        // Set obstacle flag
//       Serial.println("Obstacle detected by ToF! Stopping motors...");
//       stop_brush();                   // Stop brush
//       brake();                        // Stop drive
//     } 
//     else if (!newObstacleDetected && obstacleDetected) {
//       obstacleDetected = false;       // Clear flag
//       Serial.println("Obstacle cleared.");
//     }
//   }
// }

/**
 * Updates yaw angle by integrating gyroscope Z-axis data
 */
void updateYawAngle() {
  static unsigned long lastUpdate = 0;  // Last update time
  unsigned long now = millis();         // Current time
  
  if (lastUpdate == 0) {               // If first time
    lastUpdate = now;                  // Set time
    return;                            // Exit
  }
  
  float dt = (now - lastUpdate) / 1000.0; // Time difference in seconds
  lastUpdate = now;                    // Update time
  
  // Integrate gyroscope data
  float gyroZ_dps = myICM.gyrZ();      // Get rotation speed
  currentYaw += gyroZ_dps * dt;        // Calculate new yaw
  
  // Keep yaw between -180 and 180
  if (currentYaw > 180.0) currentYaw -= 360.0;  // Normalize
  if (currentYaw < -180.0) currentYaw += 360.0; // Normalize
}

// === Fall Detection Function ===
void checkIMU() {
  if (millis() - lastIMUCheck >= IMU_CHECK_INTERVAL && isInitialAngleSet) {  // If time to check
    lastIMUCheck = millis();           // Update time

    if (myICM.dataReady()) {           // If data ready
      myICM.getAGMT();                  // Read data
      
      // Update yaw angle
      updateYawAngle();                // Update yaw

      float currentPitch, currentRoll; // Current angles
      // Calculate current tilt
      calculateTilt(myICM.accX(), myICM.accY(), myICM.accZ(), &currentPitch, &currentRoll);

      // Calculate tilt change
      float deltaPitch = abs(currentPitch - initialPitch);  // Pitch change
      float deltaRoll = abs(currentRoll - initialRoll);     // Roll change
      
      // Handle yaw wrap-around
      float deltaYaw = abs(currentYaw - initialYaw);       // Yaw change
      if (deltaYaw > 180.0) {          // If over 180
        deltaYaw = 360.0 - deltaYaw;   // Adjust
      }

      // Print for monitoring
     /* Serial.print("IMU Tilt Δ - Pitch: ");  // Print changes
      Serial.print(deltaPitch);
      Serial.print("°, Roll: ");
      Serial.print(deltaRoll);
      Serial.print("°, Yaw: ");
      Serial.print(deltaYaw);
      Serial.println("°");*/

      // Check if change too large
      if (deltaPitch > MAX_DELTA_PITCH_ROLL ||  // If pitch too much
          deltaRoll > MAX_DELTA_PITCH_ROLL ||   // Or roll too much
          deltaYaw > MAX_DELTA_YAW) {           // Or yaw too much
            
        triggerFallProtocol(deltaPitch, deltaRoll, deltaYaw);  // Trigger fall
      }
    }
  }
}

/**
 * Calculates Pitch and Roll angles in degrees from raw accelerometer data
 * @param accX Raw accelerometer X value
 * @param accY Raw accelerometer Y value
 * @param accZ Raw accelerometer Z value
 * @param pitch Calculated pitch angle (output)
 * @param roll Calculated roll angle (output)
 */
void calculateTilt(int16_t accX, int16_t accY, int16_t accZ, float *pitch, float *roll) {
  // Convert to G-Force
  float accX_g = accX / 1000.0;       // X acceleration in G
  float accY_g = accY / 1000.0;       // Y acceleration in G
  float accZ_g = accZ / 1000.0;       // Z acceleration in G

  // Calculate Pitch
  *pitch = atan2(-accX_g, sqrt(accY_g * accY_g + accZ_g * accZ_g)) * 180.0 / PI;

  // Calculate Roll
  *roll = atan2(accY_g, accZ_g) * 180.0 / PI;
}

// === Emergency Fall Protocol ===
void triggerFallProtocol(float dPitch, float dRoll, float dYaw) {
  botDisoriented = true;               // Set fallen flag
  emergencyStop = true;                // Set emergency flag
  Serial.println("!!! FALL DETECTED !!!");  // Print message
  Serial.print("Orientation Change - Pitch: ");  // Print changes
  Serial.print(dPitch);
  Serial.print("°, Roll: ");
  Serial.print(dRoll);
  Serial.print("°, Yaw: ");
  Serial.print(dYaw);
  Serial.println("°");
  Serial.println("Engaging Emergency Stop...");  // Print stop message

  // Stop all motors
  brake();                            // Stop drive
  stop_brush();                       // Stop brush
  lin_stop();                         // Stop actuator

  // Send emergency status
   sendLoRaData();                     // Send data

  // Stay in safe state
  while (true) {                      // Infinite loop
    delay(1000);                      // Wait 1 second
  }
}

/**
 * Prints formatted floating-point numbers with leading zeros
 * @param val Value to print
 * @param leading Total number of digits
 * @param decimals Number of decimal places
 */
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);              // Absolute value
  if (val < 0) SERIAL_PORT.print("-");  // Print minus if negative
  else SERIAL_PORT.print(" ");        // Print space if positive

  // Print leading zeros
  for (uint8_t i = 0; i < leading; i++) {  // Loop through digits
    uint32_t tenpow = pow(10, leading - 1 - i);  // Calculate power
    if (aval < tenpow) SERIAL_PORT.print("0");  // Print zero
    else break;                     // Exit loop
  }
  SERIAL_PORT.print(aval, decimals);  // Print value
}

/**
 * Prints formatted IMU data
 * @param sensor Pointer to IMU object
 */
void printScaledAGMT(ICM_20948_I2C *sensor) {
 /* SERIAL_PORT.print("IMU: Acc (mg) [ ");  // Print accelerometer
  printFormattedFloat(sensor->accX(), 5, 2); SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2); SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2); SERIAL_PORT.print(" ] ");

  SERIAL_PORT.print("Gyr (DPS) [ ");  // Print gyroscope
  printFormattedFloat(sensor->gyrX(), 5, 2); SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2); SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2); SERIAL_PORT.print(" ]");

  SERIAL_PORT.println();*/              // New line
}

// ===================== LoRa COMMUNICATION =====================

/**
 * Sends ALL sensor data and status flags via LoRa
 */
void sendLoRaData() {

  char buffer[256];

  snprintf(buffer, sizeof(buffer),
  "TOF1:%d,TOF2:%d,PITCH:%.1f,ROLL:%.1f,YAW:%.1f,"
  "ACCX:%.2f,ACCY:%.2f,ACCZ:%.2f,"
  "GYRX:%.2f,GYRY:%.2f,GYRZ:%.2f,"
  "MAGX:%.2f,MAGY:%.2f,MAGZ:%.2f,"
  "DRIVE:%d,BRUSH:%d,ACT:%d,OBS:%d,DIS:%d,EMR:%d",
  tof1Distance, tof2Distance,
  currentPitch, currentRoll, currentYaw,
  accelX, accelY, accelZ,
  gyroX, gyroY, gyroZ,
  magX, magY, magZ,
  driveMoving, brushMoving, actuatorMoving,
  obstacleDetected, botDisoriented, emergencyStop
  );

  LoRa.beginPacket();
  LoRa.print(buffer);
  LoRa.endPacket();
}


// ===================== WiFi AND MOTOR FUNCTIONS =====================

void handleWiFiClient() {
  client = server.available();        // Check for client
  
  if (!client) return;                // Exit if no client

  data = checkClient();               // Get data

  if (data.length() == 0) return;     // Exit if no data

String prefix;
int value = 0;

if (data.length() >= 2) {
  prefix = data.substring(0, 2);
  value = data.substring(2).toInt();
} else {
  prefix = data;   // for b, s, p, x
}


Serial.print("Prefix: ");
Serial.println(prefix);
Serial.print("Value: ");
Serial.println(value);
  // Get value

  Serial.print("Received command: "); // Print command
  Serial.println(data);
 // Execute command
if (prefix == "fm") forward(value);
else if (prefix == "rm") reverse(value);

else if (prefix == "fb") run_brush(value);
else if (prefix == "rb") run_brush_rev(value);

else if (prefix == "fl") lin_inc();
else if (prefix == "rl") lin_dec();

else if (prefix == "b") brake();
else if (prefix == "s") stop_brush();
else if (prefix == "p") lin_stop();
else if (prefix == "x") resetStatusFlags();

  client.stop();                      // Close connection
}

String checkClient() {
  String request = client.readStringUntil('\r');

  int cmdIndex = request.indexOf("cmd=");
  if (cmdIndex == -1) return "";

  String cmd = request.substring(cmdIndex + 4);
  int spaceIndex = cmd.indexOf(' ');
  if (spaceIndex != -1) {
    cmd = cmd.substring(0, spaceIndex);
  }

  return cmd;
}


void move_actuator(int target_pos) {
  if (actuatorMoving) return;         // Exit if moving
  
  new_pos = target_pos;               // Set target
  dist = abs(pos - new_pos);          // Calculate distance
  
  if (dist > 0) {                     // If need to move
    actuatorMoving = true;            // Set flag
    lastActionTime = millis();        // Record time
    if (pos < new_pos) lin_inc();     // Move up
    else lin_dec();                   // Move down
  }
}

void handleActuator() {
  if (actuatorMoving && millis() - lastActionTime >= calc_time(dist)) {  // If time elapsed
    lin_stop();                         // Stop
    actuatorMoving = false;             // Clear flag
    pos = new_pos;                      // Update position
  }
}

// Motor Functions
void forward(int pwm) {
  driveMoving = true;                  // Set flag
  emergencyStop = false;               // Clear emergency
  digitalWrite(dir_drive, HIGH);       // Set forward
  ledcWrite(pwm_drive, pwm);

}

void reverse(int pwm) {
  driveMoving = true;                  // Set flag
  emergencyStop = false;               // Clear emergency
  digitalWrite(dir_drive, LOW);        // Set reverse
  ledcWrite(pwm_drive, pwm);      // Set speed
}

void brake() {
  driveMoving = false;                 // Clear flag
  digitalWrite(dir_drive, LOW);        // Set direction
  ledcWrite(pwm_drive, 0);         // Stop
}

void run_brush(int pwm) {
  brushMoving = true;                  // Set flag
  emergencyStop = false;               // Clear emergency
  digitalWrite(dir_brush, HIGH);       // Set forward
  ledcWrite(pwm_brush, pwm);
       // Set speed
}

void run_brush_rev(int pwm) {
  brushMoving = true;                  // Set flag
  emergencyStop = false;               // Clear emergency
  digitalWrite(dir_brush, LOW);        // Set reverse
  ledcWrite(pwm_brush, pwm);
      // Set speed
}

void stop_brush() {
  brushMoving = false;                 // Clear flag
  ledcWrite(pwm_brush, 0);
         // Stop
}

// Actuator Functions
void lin_inc() {
  actuatorMoving = true;               // Set flag
  emergencyStop = false;               // Clear emergency
  digitalWrite(lin_act_1_dir, LOW);    // Set up
  ledcWrite(lin_act_1_pwm, linear_actr_pwm);
 // Set speed
}

void lin_dec() {
  actuatorMoving = true;               // Set flag
  emergencyStop = false;               // Clear emergency
  digitalWrite(lin_act_1_dir, HIGH);   // Set down
  ledcWrite(lin_act_1_pwm, linear_actr_pwm);
 // Set speed
}

void lin_stop() {
  actuatorMoving = false;              // Clear flag
  ledcWrite(lin_act_1_pwm, 0);
     // Stop
}

// === Reset Status Flags ===
void resetStatusFlags() {
  driveMoving = false;                 // Clear drive flag
  brushMoving = false;                 // Clear brush flag
  actuatorMoving = false;              // Clear actuator flag
  obstacleDetected = false;            // Clear obstacle flag
  // Don't reset fallen and emergency flags
  Serial.println("Status flags reset"); // Print message
}

int calc_time(int dist) {
  return (int)(1000 * dist / 6.893);   // Calculate time needed
}

int calc_pwm(float volt) {
  return (int)((11.0 / volt) * 255);   // Calculate PWM from voltage
}


//new Acess point code
