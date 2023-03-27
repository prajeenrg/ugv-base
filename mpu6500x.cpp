#include "mpu6500x.h"

int MPU6500X::init(calData cal, uint8_t address, uint32_t sda, uint32_t scl) {
	Wire.setSDA(sda);
	Wire.setSCL(scl);
	MPU6500::init(cal, address);
}
