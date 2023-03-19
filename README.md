# UGV Codebase
This repo contains all the arduino and related c++ code written for the final year IoT project. The code is written with the help of stm32duino core as opposed to normal Arduino core so as to use the existing knowledge of Arduino coding with boards from the STM32 series.

GPS library has been fetched from [GPS-neo-6m](https://github.com/cristiansteib/GPS-neo-6m) repo and minor changes have been made to adapt the library for the use case that is needed.

This project is mainly built upon the Ardiuno Core port for STM32 series microcontrollers called [STM32duino](https://github.com/stm32duino/Arduino_Core_STM32)

### Libraries Used
- [FastIMU](https://github.com/LiquidCGS/FastIMU)
- [ArduinoJson](https://arduinojson.org/)
- [Adafruit_VL53L0X](https://github.com/adafruit/Adafruit_VL53L0X)
- [TinyGSM](https://github.com/vshymanskyy/TinyGSM)
- [PubSubClient](https://pubsubclient.knolleary.net/)
