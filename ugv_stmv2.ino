#include <IRremote.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "Gpsneo.h"
#include "HardwareSerial.h"

#define CLOSEOBJ(x) (x > 50)

/* pin locations */
#define IRRECPIN PB9  // IR receiver pin
#define LMOTA PB4     // direction control motor left
#define LMOTB PB3     // direction control motor left
#define LMOTS PB5     // speed control motor left
#define RMOTA PB15    // direction control motor right
#define RMOTB PB14    // direction control motor right
#define RMOTS PB13    // speed control motor right
#define LTOFPIN PA3   // left tof xshut pin
#define RTOFPIN PA2   // right tof xshut pin
#define FTOFPIN PA1   // front tof xshut pin
#define BTOFPIN PA0   // back tof xshut pin
#define GPS_TX PB6    // tx pin for gps
#define GPS_RX PB7    // rx pin for gps
#define MY_SDA PB9    // two wire sda pin
#define MY_SCL PB8    // two wire scl pin
#define GSM_TX 0      // tx pin for gsm
#define GSM_RX 0      // rx pin for gsm

/* tof address */
#define LTOFADDR 0x60  // new address for left tof
#define RTOFADDR 0x61  // new address for right tof
#define FTOFADDR 0x62  // new address for front tof
#define BTOFADDR 0x63  // new address for back tof

#define SPEED1 240
#define SPEED2 180
#define SPEED3 120
#define SPEED4 90
#define SPEED5 0

struct Dist {
  uint16_t front;
  uint16_t back;
  uint16_t left;
  uint16_t right;
} dist;

struct GpsData {
  float latitude;
  float longitude;
} GpsData;

// direction control
enum Dir { ND,
           SD,
           WD,
           ED,
           NED,
           SED,
           NWD,
           SWD,
           ROTC,
           ROTCC,
           STOP } comm;

TwoWire WIRE1(MY_SDA, MY_SCL);
Gpsneo gps(GPS_RX, GPS_TX);
Adafruit_VL53L0X lox;
HardwareSerial SerialGsm(GSM_RX, GSM_TX);

void getGpsData() {
  char glat[50], glong[50];
  gps.getDataGPRMC(glat, glong);
  GpsData.latitude = gps.convertLatitude(glat);
  GpsData.longitude = gps.convertLongitude(glong);
  Serial.printf("Latitude: %f, Longitude: %f\n", GpsData.latitude, GpsData.longitude);
}

void getDirection() {
  if (IrReceiver.decode()) {
    Serial.println(IrReceiver.decodedIRData.decodedRawData);
    switch (IrReceiver.decodedIRData.decodedRawData) {
      case 3877175040:
        comm = Dir::ND;
        Serial.println(F("ND"));
        break;
      case 4077715200:
        comm = Dir::NWD;
        Serial.println(F("NWD"));
        break;
      case 2707357440:
        comm = Dir::NED;
        Serial.println(F("NED"));
        break;
      case 4144561920:
        comm = Dir::WD;
        Serial.println(F("WD"));
        break;
      case 2774204160:
        comm = Dir::ED;
        Serial.println(F("ED"));
        break;
      case 3175284480:
        comm = Dir::SWD;
        Serial.println(F("SWD"));
        break;
      case 2907897600:
        comm = Dir::SD;
        Serial.println(F("SD"));
        break;
      case 3041591040:
        comm = Dir::SED;
        Serial.println(F("SED"));
        break;
      case 3810328320:
        comm = Dir::ROTC;
        Serial.println(F("CW Rotate"));
        break;
      case 3910598400:
        comm = Dir::ROTCC;
        Serial.println(F("CCW Rotate"));
        break;
      case 3158572800:
        comm = Dir::STOP;
        Serial.println(F("Brakes"));
        break;
      case 1271936906: break;  // special case for infrared emitted by VL53L0X
      default:
        Serial.println(F("Control not programmed... Ask the programmer"));
    }
  }
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
  switch (comm) {
    case Dir::ND:
      applyMotorDrive(SPEED1, SPEED1, 0);
      break;
    case Dir::ED:
      applyMotorDrive(SPEED1, SPEED3, 0);
      break;
    case Dir::WD:
      applyMotorDrive(SPEED3, SPEED1, 0);
      break;
    case Dir::NED:
      applyMotorDrive(SPEED1, SPEED2, 0);
      break;
    case Dir::NWD:
      applyMotorDrive(SPEED2, SPEED1, 0);
      break;
    case Dir::SD:
      applyMotorDrive(SPEED1, SPEED1, 3);
      break;
    case Dir::SWD:
      applyMotorDrive(SPEED2, SPEED1, 3);
      break;
    case Dir::SED:
      applyMotorDrive(SPEED1, SPEED2, 3);
      break;
    case Dir::ROTC:
      applyMotorDrive(SPEED1, SPEED1, 1);
      break;
    case Dir::ROTCC:
      applyMotorDrive(SPEED1, SPEED1, 2);
      break;
    case Dir::STOP:
      applyMotorDrive(SPEED5, SPEED5, 0);
      break;
    default: break;
  }
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
  IrReceiver.begin(IRRECPIN, DISABLE_LED_FEEDBACK);

  // reset tof
  do {
   resetTof();
  } while (setTofAddress());

  // initial direction
  comm = Dir::STOP;
  Serial.println("End of setup()");
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
  lox.begin(LTOFADDR);
  dist.left = lox.readRange();

  lox.begin(RTOFADDR);
  dist.right = lox.readRange();

  lox.begin(FTOFADDR);
  dist.front = lox.readRange();

  lox.begin(BTOFADDR);
  dist.back = lox.readRange();

  Serial.print(F("LTOF: "));
  Serial.println(dist.left);
  Serial.print(F("RTOF: "));
  Serial.println(dist.right);
  Serial.print(F("FTOF: "));
  Serial.println(dist.front);
  Serial.print(F("BTOF: "));
  Serial.println(dist.back);
}

bool allIsClear() {
  return CLOSEOBJ(dist.front) && CLOSEOBJ(dist.back) && CLOSEOBJ(dist.left) && CLOSEOBJ(dist.right);
}

void loop() {
  probeObstacles();
  getGpsData();
  getDirection();
  if (!allIsClear()) {
    comm = Dir::STOP;
    Serial.println(F("There is a block"));
  }
  moveVehicle();
  IrReceiver.resume();
}