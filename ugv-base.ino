#include <Wire.h>
#include <VL53L0X.h>
#include "Gpsneo.h"
#include "MqttClient.h"
#include "FastIMU.h"

#define LESS_THAN(x, y) x < y

#define SAFE_DISTANCE_MM 40

/* motor controls pins */
#define MOTOR_AB1 11
#define MOTOR_AB2 10
#define MOTOR_CD1 9
#define MOTOR_CD2 6

/* lidar xshut pins */
#define XSHUT_1 A0
#define XSHUT_2 A1
#define XSHUT_3 A2
#define XSHUT_4 A3

/* gsm uart pins */
#define GSM_TX 2
#define GSM_RX 3

/* gps uart pins */
#define GPS_TX 12
#define GPS_RX 13

/* tof address setting */
#define TOF_ADDR_1 0x62
#define TOF_ADDR_2 0x61
#define TOF_ADDR_3 0x63
#define TOF_ADDR_4 0x60

/* mqtt broker settings */
#define BROKER_URL "test.mosquitto.org"
#define BROKER_PORT 1883

/* gsm apn settings */
#define AIRTEL_APN "airtelgprs.com"
#define AIRTEL_USER ""
#define AIRTEL_PASS ""

#define MPU_ADDR 0x68  // i2c address for mpu6500

/* motor speed settings */
#define SPEED1 240
#define SPEED2 180
#define SPEED3 120
#define SPEED4 90
#define SPEED5 0

/* direction settings */
enum Dir { 
  NWD = 1,
  ND,
  NED,
  WD,
  STOP,
  ED,
  SWD,
  SD,
  SED,
  ROTC,
  ROTCC,
};

Gpsneo gps(GPS_TX, GPS_RX);
SoftwareSerial GSM(GSM_TX, GSM_RX);
MqttClient mqtt(GSM);
MPU6500 mpu;

void setup_gsm() {
  mqtt.setup_modem();
  while (!mqtt.connect_gprs(AIRTEL_APN, AIRTEL_USER, AIRTEL_PASS)) {
    Serial.println(F("Failed initiating GPRS network"));
    Serial.println(F("Retrying GPRS establishment after 1s..."));
    delay(1000);
  }
  Serial.print(F("Connected to GPRS: "));
  Serial.println(AIRTEL_APN);
  Serial.println(F("Connecting to MQTT Broker..."));
  mqtt.set_broker(BROKER_URL, BROKER_PORT);
  mqtt.connect_broker();
  while (!mqtt.connect_broker()) {
    Serial.println(F("Failed in connecting to MQTT Broker"));
    Serial.println(F("Retrying connection after 1s..."));
    delay(1000);
  }
  Serial.println(F("Connected to MQTT Broker."));
}

void setup_mpu6500() {
  calData calib = { 0 };
  int err = mpu.init(calib, MPU_ADDR);
  if (err != 0) {
    Serial.print(F("Error initializing MPU6500, error code: "));
    Serial.println(err);
    while (1);
  }
}

void getGpsData() {
  char glat[50], glong[50];
  gps.getDataGPRMC(glat, glong);
  float glati = gps.convertLatitude(glat);
  float glongi = gps.convertLongitude(glong);
  mqtt.send_gps_data(glati, glongi);
  Serial.print(F("Latitude: "));
  Serial.print(glati);
  Serial.print(F(", Longitude: "));
  Serial.println(glongi);
}

void applyMotorDrive(char fwl, char fwr, char rev = 0) {
  if ((rev & 2) >> 1) {
    digitalWrite(MOTOR_AB1, LOW);
    analogWrite(MOTOR_AB2, fwl);
  } else {
    analogWrite(MOTOR_AB1, fwl);
    digitalWrite(MOTOR_AB2, LOW);
  }
  if (rev & 1) {
    digitalWrite(MOTOR_CD1, LOW);
    analogWrite(MOTOR_CD2, fwr);
  } else {
    analogWrite(MOTOR_CD1, fwr);
    digitalWrite(MOTOR_CD2, LOW);
  }
}

void moveVehicle() {
  byte mode = 0;
  byte left = SPEED1;
  byte right = SPEED2;
  switch (MqttClient::control) {
    case Dir::ND:
      break;
    case Dir::ED:
      right = SPEED3;
      break;
    case Dir::WD:
      left = SPEED3;
      break;
    case Dir::NED:
      right = SPEED2;
      break;
    case Dir::NWD:
      left = SPEED2;
      break;
    case Dir::SD:
      mode = 3;
      break;
    case Dir::SWD:
      mode = 3;
      left = SPEED2;
      break;
    case Dir::SED:
      mode = 3;
      right = SPEED2;
      break;
    case Dir::ROTC:
      mode = 1;
      break;
    case Dir::ROTCC:
      mode = 2;
      break;
    case Dir::STOP:
    default:
      left = SPEED5;
      right = SPEED5;
      break;
  }
  applyMotorDrive(left, right, mode);
  mqtt.send_motor_data(left, right);

  Serial.println(F("Motor Data:"));
  Serial.print(F("Left Speed: ")); 
  Serial.println(left);
  Serial.print(F("Right Speed: ")); 
  Serial.println(right);
}

void get_mpudata() {
  GyroData gyroData;
  AccelData accelData;
  mpu.update();
  mpu.getAccel(&accelData);
  mpu.getGyro(&gyroData);
  mqtt.send_gyro_data(gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ);
  mqtt.send_accel_data(accelData.accelX, accelData.accelY, accelData.accelZ);

  Serial.println(F("Gyroscope Data:"));
  Serial.print(F("X axis: "));
  Serial.println(gyroData.gyroX);
  Serial.print(F("Y axis: "));
  Serial.println(gyroData.gyroY);
  Serial.print(F("Z axis: "));
  Serial.println(gyroData.gyroZ);
  Serial.println(F("Accelerometer Data:"));
  Serial.print(F("X axis: "));
  Serial.println(accelData.accelX);
  Serial.print(F("Y axis: "));
  Serial.println(accelData.accelY);
  Serial.print(F("Z axis: "));
  Serial.println(accelData.accelZ);
}

bool initTof(uint8_t addr) {
  VL53L0X lox;
  lox.setAddress(addr);
  if (!lox.init()) {
    Serial.println(F("LTOF cannot be found"));
    return false;
  }
  lox.startContinuous();
  return true;
}

bool setTofAddress() {
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_3, LOW);
  if (!initTof(TOF_ADDR_4)) {
    Serial.println(F("LTOF cannot be found"));
    return true;
  }

  digitalWrite(XSHUT_2, HIGH);
  if (!initTof(TOF_ADDR_2)) {
    Serial.println(F("RTOF cannot be found"));
    return true;
  }

  digitalWrite(XSHUT_1, HIGH);
  if (!initTof(TOF_ADDR_1)) {
    Serial.println(F("FTOF cannot be found"));
    return true;
  }
  
  digitalWrite(XSHUT_3, HIGH);
  if (!initTof(TOF_ADDR_3)) {
    Serial.println(F("BTOF cannot be found"));
    return true;
  }

  Serial.println(F("All TOFs ready for launch"));
  return false;
}

inline void resetTof() {
  digitalWrite(XSHUT_4, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_3, LOW);
  delay(10);
  digitalWrite(XSHUT_4, HIGH);
  digitalWrite(XSHUT_2, HIGH);
  digitalWrite(XSHUT_1, HIGH);
  digitalWrite(XSHUT_3, HIGH);
  delay(10);
}

uint16_t getTofData(uint8_t addr) {
  VL53L0X lox;
  lox.setAddress(addr);
  return lox.readRangeContinuousMillimeters();
}

bool getRangingResults() {
  VL53L0X lox;
  uint16_t left = getTofData(TOF_ADDR_4);
  uint16_t right = getTofData(TOF_ADDR_2);
  uint16_t front = getTofData(TOF_ADDR_1);
  uint16_t back = getTofData(TOF_ADDR_3);

  mqtt.send_lidar_data(front, right, back, left);

  Serial.println(F("TOF Ranging Results:"));
  Serial.print(F("TOF 1: "));
  Serial.println(front);
  Serial.print(F("TOF 2: ")); 
  Serial.println(right);
  Serial.print(F("TOF 3: ")); 
  Serial.println(back);
  Serial.print(F("TOF 4: ")); 
  Serial.println(left);

  return LESS_THAN(front, SAFE_DISTANCE_MM) ||
    LESS_THAN(back, SAFE_DISTANCE_MM) ||
    LESS_THAN(left, SAFE_DISTANCE_MM) ||
    LESS_THAN(right, SAFE_DISTANCE_MM);
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Initializing the pins and peripherals..."));

  Serial.println(F("Setting pin modes..."));
  pinMode(MOTOR_AB1, OUTPUT);
  pinMode(MOTOR_AB2, OUTPUT);
  pinMode(MOTOR_CD1, OUTPUT);
  pinMode(MOTOR_CD2, OUTPUT);
  pinMode(XSHUT_4, OUTPUT);
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  Serial.println(F("Pins modes has been set!"));

  Serial.println(F("Setting up GSM Module..."));
  GSM.begin(9600);
  setup_gsm();
  Serial.println(F("GSM Module setup completed!"));

  Serial.println(F("Setting up MPU6500 Sensor..."));
  setup_mpu6500();
  Serial.println(F("MPU6500 sensor setup completed!"));

  Serial.println(F("Setting up ToF sensors..."));
  do {
   resetTof();
  } while (setTofAddress());
  Serial.println(F("ToF sensors setup completed!"));

  Serial.println(F("Initialization successfully completed!!!"));
}

void loop() {
  getGpsData();
  get_mpudata();
  if (getRangingResults()) {
    moveVehicle();
  } else {
    Serial.println(F("There is a block"));
  }
  mqtt.loop();
}
