void		initINA219(const uint8_t i2cAddress);
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payloadBtye);
WarpStatus 	configureSensorINA219(uint16_t payloadF_SETUP, uint16_t payloadCTRL_REG1);
void		printSensorDataINA219(bool hexModeFlag);


const uint8_t bytesPerMeasurementINA219            = 8;
const uint8_t bytesPerReadingINA219                = 2;
const uint8_t numberOfReadingsPerMeasurementINA219 = 4;