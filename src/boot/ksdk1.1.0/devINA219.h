// default I2C address


#define INA219_ADDRESS (0x40)

//read
#define INA219_READ (0x01)

// define the registers

#define INA219_REG_CONFIG (0x00)
#define INA219_REG_SHUNTVOLTAGE (0x01)
#define INA219_REG_BUSVOLTAGE (0x02)
#define INA219_REG_POWER (0x03)
#define INA219_REG_CURRENT (0x04)
#define INA219_REG_CALIBRATION (0x05)

void            initdevINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus      writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload);
WarpStatus      readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
void            printSensorDataINA219(bool hexModeFlag);
uint8_t         appendSensorDataINA219(uint8_t* buf);

const uint8_t bytesPerMeasurementINA219            = 4;
const uint8_t bytesPerReadingINA219                = 2;
const uint8_t numberOfReadingsPerMeasurementINA219 = 2;


