#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "Gpsneo.h"
#include "MqttClient.h"
#include "mpu6500x.h"
#include "UgvDataTypes.h"

#define CLOSEOBJ(x) (x > 50)

/* pin locations */
#define LMOTA PB4     // direction control motor left
#define LMOTB PB3     // direction control motor left
#define LMOTS PB5     // speed control motor left
#define RMOTA PB15    // direction control motor right
#define RMOTB PB14    // direction control motor right
#define RMOTS PB13    // speed control motor right
#define LTOFPIN PA7   // left tof xshut pin
#define RTOFPIN PA6   // right tof xshut pin
#define FTOFPIN PA5   // front tof xshut pin
#define BTOFPIN PA4   // back tof xshut pin
#define GPS_TX PA0    // tx pin for gps
#define GPS_RX PA1    // rx pin for gps
#define GSM_TX PA2    // tx pin for gsm
#define GSM_RX PA3    // rx pin for gsm
#define SDA_PIN PB9   // sda pin
#define SCL_PIN PB8   // scl pin
TwoWire WIRE1(SDA_PIN, SCL_PIN);

#define BROKER_URL "test.mosquitto.org"
#define BROKER_PORT 1883

#define AIRTEL_APN "airtelgprs.com"
#define AIRTEL_USER ""
#define AIRTEL_PASS ""

/* tof address */
#define LTOFADDR 0x60  // new address for left tof
#define RTOFADDR 0x61  // new address for right tof
#define FTOFADDR 0x62  // new address for front tof
#define BTOFADDR 0x63  // new address for back tof
#define MPU_ADDR 0x68  // i2c address for mpu6500

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

// direction control
enum Dir { NWD = 1,
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
HardwareSerial GSM(GSM_RX, GSM_TX);
MqttClient mqtt(GSM);
MPU6500X mpu;

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
    digitalWrite(LMOTA, LOW);
    digitalWrite(LMOTB, HIGH);
  } else {
    digitalWrite(LMOTA, HIGH);
    digitalWrite(LMOTB, LOW);
  }
  if (rev & 1) {
    digitalWrite(RMOTA, LOW);
    digitalWrite(RMOTB, HIGH);
  } else {
    digitalWrite(RMOTA, HIGH);
    digitalWrite(RMOTB, LOW);
  }
  analogWrite(LMOTS, fwl);
  analogWrite(RMOTS, fwr);
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

void setup() {
  // motor driver pins
  pinMode(LMOTA, OUTPUT);
  pinMode(LMOTB, OUTPUT);
  pinMode(LMOTS, OUTPUT);
  pinMode(RMOTA, OUTPUT);
  pinMode(RMOTB, OUTPUT);
  pinMode(RMOTS, OUTPUT);

  // lidar setup
  pinMode(LTOFPIN, OUTPUT);
  pinMode(FTOFPIN, OUTPUT);
  pinMode(BTOFPIN, OUTPUT);
  pinMode(RTOFPIN, OUTPUT);

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

void setup_mpu6500() {
  calData calib = { 0 };
  int err = mpu.init(calib, MPU_ADDR, SDA_PIN, SCL_PIN);
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
  digitalWrite(RTOFPIN, LOW);
  digitalWrite(FTOFPIN, LOW);
  digitalWrite(BTOFPIN, LOW);
  if (!lox.begin(LTOFADDR, false, &WIRE1)) {
    Serial.println(F("LTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
  }

  digitalWrite(RTOFPIN, HIGH);
  if (!lox.begin(RTOFADDR, false, &WIRE1)) {
    Serial.println(F("RTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
  }

  digitalWrite(FTOFPIN, HIGH);
  if (!lox.begin(FTOFADDR, false, &WIRE1)) {
    Serial.println(F("FTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
  }
  
  digitalWrite(BTOFPIN, HIGH);
  if (!lox.begin(BTOFADDR, false, &WIRE1)) {
    Serial.println(F("BTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
  }

  Serial.println(F("All TOFs ready for launch"));
  return false;
}

inline void resetTof() {
  digitalWrite(LTOFPIN, LOW);
  digitalWrite(RTOFPIN, LOW);
  digitalWrite(FTOFPIN, LOW);
  digitalWrite(BTOFPIN, LOW);
  delay(10);
  digitalWrite(LTOFPIN, HIGH);
  digitalWrite(RTOFPIN, HIGH);
  digitalWrite(FTOFPIN, HIGH);
  digitalWrite(BTOFPIN, HIGH);
  delay(10);
}

void probeObstacles() {
  lox.begin(LTOFADDR, false, &WIRE1);
  lidardata.left = lox.readRange();

  lox.begin(RTOFADDR, false, &WIRE1);
  lidardata.right = lox.readRange();

  lox.begin(FTOFADDR, false, &WIRE1);
  lidardata.front = lox.readRange();

  lox.begin(BTOFADDR, false, &WIRE1);
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
