#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "Gpsneo.h"
#include "MqttClient.h"
#include "FastIMU.h"
#include "UgvDataTypes.h"

#define CLOSEOBJ(x) (x > 50)

/* motor controls pins */
#define MOTOR_AB1 PB12
#define MOTOR_AB2 PB13
#define MOTOR_CD1 PB15
#define MOTOR_CD2 PB14

/* lidar xshut pins */
#define XSHUT_1 PA5
#define XSHUT_2 PA6
#define XSHUT_3 PA4
#define XSHUT_4 PA7

/* gsm uart pins */
#define GSM_TX PA3
#define GSM_RX PA2

/* gps uart pins */
#define GPS_TX PA10
#define GPS_RX PA9

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

LidarData lidardata;
GpsData gpsdata;
MotorData motordata;
AccelData accelData;
GyroData gyroData;

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
Adafruit_VL53L0X lox;
HardwareSerial GSM(GSM_TX, GSM_RX);
MqttClient mqtt(GSM);
MPU6500 mpu;

void getGpsData() {
  char glat[50], glong[50];
  gps.getDataGPRMC(glat, glong);
  gpsdata.latitude = gps.convertLatitude(glat);
  gpsdata.longitude = gps.convertLongitude(glong);
  mqtt.send_gps_data(gpsdata);  
  Serial.printf("Latitude: %f, Longitude: %f\n", gpsdata.latitude, gpsdata.longitude);
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
  int mode = 0;
  motordata.left = SPEED1;
  motordata.right = SPEED2;
  switch (MqttClient::control) {
    case Dir::ND:
      break;
    case Dir::ED:
      motordata.right = SPEED3;
      break;
    case Dir::WD:
      motordata.left = SPEED3;
      break;
    case Dir::NED:
      motordata.right = SPEED2;
      break;
    case Dir::NWD:
      motordata.left = SPEED2;
      break;
    case Dir::SD:
      mode = 3;
      break;
    case Dir::SWD:
      mode = 3;
      motordata.left = SPEED2;
      break;
    case Dir::SED:
      mode = 3;
      motordata.right = SPEED2;
      break;
    case Dir::ROTC:
      mode = 1;
      break;
    case Dir::ROTCC:
      mode = 2;
      break;
    case Dir::STOP:
    default:
      motordata.left = SPEED5;
      motordata.right = SPEED5;
      break;
  }
  applyMotorDrive(motordata.left, motordata.right, mode);
  mqtt.send_motor_data(motordata);  
}

void setup_mpu6500() {
  calData calib = { 0 };
  int err = mpu.init(calib, MPU_ADDR);
  if (err != 0) {
    Serial.print("Error initializing MPU6500");
    Serial.println(err);
    while (1);
  }
}

void get_mpudata() {
  mpu.update();
  mpu.getAccel(&accelData);
  mpu.getGyro(&gyroData);
  mqtt.send_gyro_data(gyroData);
  mqtt.send_accel_data(accelData);  
}

void setup_gsm() {
  mqtt.setup_modem();
  if (!mqtt.connect_gprs(AIRTEL_APN, AIRTEL_USER, AIRTEL_PASS)) {
    Serial.println("Failed in connecting to GPRS.");
    while (1);
  }
  Serial.println("Connected to GPRS: " + String(AIRTEL_APN));
  mqtt.set_broker(BROKER_URL, BROKER_PORT);
  mqtt.connect_broker();
  while (!mqtt.connect_broker()) Serial.printf("Connecting to MQTT Broker: %s\n", BROKER_URL);
  Serial.println("Connected to MQTT Broker.");
}

bool setTofAddress() {
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_3, LOW);
  if (!lox.begin(TOF_ADDR_4, false)) {
    Serial.println(F("LTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
  }

  digitalWrite(XSHUT_2, HIGH);
  if (!lox.begin(TOF_ADDR_2, false)) {
    Serial.println(F("RTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
  }

  digitalWrite(XSHUT_1, HIGH);
  if (!lox.begin(TOF_ADDR_1, false)) {
    Serial.println(F("FTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
  }
  
  digitalWrite(XSHUT_3, HIGH);
  if (!lox.begin(TOF_ADDR_3, false)) {
    Serial.println(F("BTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
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

void probeObstacles() {
  lox.begin(TOF_ADDR_4, false);
  lidardata.left = lox.readRange();

  lox.begin(TOF_ADDR_2, false);
  lidardata.right = lox.readRange();

  lox.begin(TOF_ADDR_1, false);
  lidardata.front = lox.readRange();

  lox.begin(TOF_ADDR_3, false);
  lidardata.back = lox.readRange();

  mqtt.send_lidar_data(lidardata);

  Serial.printf("LTOF: %d\n", lidardata.left);
  Serial.printf("RTOF: %d\n", lidardata.right);
  Serial.printf("FTOF: %d\n", lidardata.front);
  Serial.printf("BTOF: %d\n", lidardata.back);
}

bool isAllClear() {
  probeObstacles();
  return CLOSEOBJ(lidardata.front) && CLOSEOBJ(lidardata.back) && CLOSEOBJ(lidardata.left) && CLOSEOBJ(lidardata.right);
}

void setup() {
  // motor driver pins
  pinMode(MOTOR_AB1, OUTPUT);
  pinMode(MOTOR_AB2, OUTPUT);
  pinMode(MOTOR_CD1, OUTPUT);
  pinMode(MOTOR_CD2, OUTPUT);

  // lidar setup
  pinMode(XSHUT_4, OUTPUT);
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);

  // Lidar setup
  Serial.begin(115200);
  GSM.begin(9600);

  // GSM setup
  setup_gsm();

  // MPU6500 setup
  setup_mpu6500();

  // reset tof
  do {
   resetTof();
  } while (setTofAddress());

  // initial direction
  Serial.println("End of setup()");
}

void loop() {
  getGpsData();
  get_mpudata();
  if (isAllClear()) {
    moveVehicle();
  } else {
    Serial.println(F("There is a block"));
  }
  mqtt.loop();
}
