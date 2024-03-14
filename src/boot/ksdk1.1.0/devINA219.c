#include <stdint.h>
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"

#include "devINA219.h"

extern volatile WarpI2CDeviceState      deviceINA219State;
extern volatile uint32_t                gWarpI2cBaudRateKbps;
extern volatile uint32_t                gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t                gWarpSupplySettlingDelayMilliseconds;

void
initdevINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
        deviceINA219State.i2cAddress                  = i2cAddress;
        deviceINA219State.operatingVoltageMillivolts  = operatingVoltageMillivolts;
	calibrateINA219();
        return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload)
{
        uint8_t         payloadByte[2], commandByte[1];
        i2c_status_t    returnValue;

        switch (deviceRegister)
        {
		case 0x00: case 0x05: /* this is the configuration register */
                {
                        /* OK */
                        break;
                }

                default:
                {
                        return kWarpStatusBadDeviceCommand;
                }
        }

        i2c_device_t slave =
                {
                        .address       = deviceINA219State.i2cAddress,
                        .baudRate_kbps = gWarpI2cBaudRateKbps};

        warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

        commandByte[0] = deviceRegister;
        payloadByte[0] = (payload >> 8) & 0xFF; /* MSB first */
        payloadByte[1] = payload & 0xFF;        /* LSB */
	warpEnableI2Cpins();
        returnValue    = I2C_DRV_MasterSendDataBlocking(
                0 /* I2C instance */,
                &slave,
                commandByte,
                1,
                payloadByte,
                2,
                gWarpI2cTimeoutMilliseconds);
        if (returnValue != kStatus_I2C_Success)
        {
                return kWarpStatusDeviceCommunicationFailed;
        }

        return kWarpStatusOK;
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes){
	uint8_t cmdBuf[1] = {0xFF};
        i2c_status_t status;

	USED(numberOfBytes);
        switch (deviceRegister)
        {
		case 0x00: case 0x01: case 0x02: case 0x03: case 0x04: case 0x05:
                {
                        /* OK */
                        break;
                }

                default:
                {
                        return kWarpStatusBadDeviceCommand;
                }
        }


        i2c_device_t slave =
                {
                        .address       = deviceINA219State.i2cAddress,
                        .baudRate_kbps = gWarpI2cBaudRateKbps};

        USED(numberOfBytes);

        warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
        warpEnableI2Cpins();
	// now need to implement read
	// first trigger the read 
	cmdBuf[0] = deviceRegister;
	
	//now need to add a time delay to wait for the conversion to complete
	//OSA_TimeDelay(0.6);
	/* this information was found from the table on page 3 of the datasheet */
	//now read the register value
	status = I2C_DRV_MasterReceiveDataBlocking(
                        0 /* I2C peripheral instance */,
                        &slave,
                        cmdBuf,
                        1,
                        (uint8_t*)deviceINA219State.i2cBuffer,
                        numberOfBytes,
                        gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
                {
                        return kWarpStatusDeviceCommunicationFailed;
                }
	return kWarpStatusOK;

}
void
printSensorDataINA219(bool hexModeFlag)
{
        uint16_t        readSensorRegisterValueLSB;
        uint16_t        readSensorRegisterValueMSB;
        int16_t         readSensorRegisterValueCombined;
        WarpStatus      i2cReadStatus;
	

        warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
        i2cReadStatus = readSensorRegisterINA219(0x01, 2 /* numberOfBytes */); //read shunt voltage
        readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);
        if (i2cReadStatus != kWarpStatusOK)
        {
                warpPrint(" ----,");
        }
        else
        {
                if (hexModeFlag)
                {
                        warpPrint(" 0x%02x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
                }
                else
                {
                        /*
                         *      Convert to an actual voltage in microvolts.
                         */
			int16_t shuntVoltage = readSensorRegisterValueCombined*10;
                        warpPrint(" %d,", shuntVoltage);
                }
        }
	i2cReadStatus = readSensorRegisterINA219(0x02, 2 /* numberOfBytes */); // read bus voltage
        readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	// want to combine the bytes to get rid of unuseful bits 
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 5) | ((readSensorRegisterValueLSB & 0xFF)>>3);
	if (i2cReadStatus != kWarpStatusOK)
        {
                warpPrint(" ----,");
        }
        else
        {
                if (hexModeFlag)
                {
                        warpPrint(" 0x%02x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
                }
                else
                {
                        /*
                         *     Converts binary to milli volts 
                         */
                       int16_t busVoltage = readSensorRegisterValueCombined * 4;
                        warpPrint(" %d,", busVoltage);
                }
        }
	i2cReadStatus = readSensorRegisterINA219(0x03, 2 /* numberOfBytes */); // read power
        readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);
        if (i2cReadStatus != kWarpStatusOK)
        {
                warpPrint(" ----,");
        }
        else
        {
                if (hexModeFlag)
                {
                        warpPrint(" 0x%02x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
                }
                else
                {
                        /*
                         *     Since the conversion is 1.0mW I don't need to do anything for the conversion
                         */
			//int16_t power = readSensorRegisterValueCombined * 400;
                        warpPrint(" %d,", readSensorRegisterValueCombined);
                }
        }
	i2cReadStatus = readSensorRegisterINA219(0x04, 2 /* numberOfBytes */); // read current
        readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);
        if (i2cReadStatus != kWarpStatusOK)
        {
                warpPrint(" ----,");
        }
        else
        {
                if (hexModeFlag)
                {
                        warpPrint(" 0x%02x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
                }
                else
                {
                        /*
                         *     Converts binary to microamps - changed
                         */
                       	int32_t current = readSensorRegisterValueCombined * 50;
                        warpPrint(" %d,", current);
                }
        }
	i2cReadStatus = readSensorRegisterINA219(0x05, 2 /* numberOfBytes */); // read current
        readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);
        if (i2cReadStatus != kWarpStatusOK)
        {
                warpPrint(" ----,");
        }
        else
        {
                if (hexModeFlag)
                {
                        warpPrint(" 0x%02x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
                }
                else
                {
                        /*
                         *     Converts binary
                         */
                        warpPrint(" %d,", readSensorRegisterValueCombined);
                }
        }


}



uint8_t
appendSensorDataINA219(uint8_t* buf)
{
        uint8_t index = 0;

        uint16_t readSensorRegisterValueLSB;
        uint16_t readSensorRegisterValueMSB;
        int16_t readSensorRegisterValueCombined;
        WarpStatus i2cReadStatus;

        warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
        i2cReadStatus                   = readSensorRegisterINA219(0x01, 2 /* numberOfBytes */);
        readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

        /*
         *      NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
         */

        if (i2cReadStatus != kWarpStatusOK)
        {
                buf[index] = 0;
                index += 1;

                buf[index] = 0;
                index += 1;
        }
        else
        {      

                readSensorRegisterValueCombined = readSensorRegisterValueCombined / 100;
                /*
                 * MSB first
                 */
                buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
                index += 1;

                buf[index] = (uint8_t)(readSensorRegisterValueCombined);
                index += 1;
	}
	i2cReadStatus                   = readSensorRegisterINA219(0x02, 2 /* numberOfBytes */);
        readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 5) | ((readSensorRegisterValueLSB & 0xFF) >> 3);

        /*
         *      NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
         */

        if (i2cReadStatus != kWarpStatusOK)
        {
                buf[index] = 0;
                index += 1;

                buf[index] = 0;
                index += 1;
        }
        else
        {
                
                readSensorRegisterValueCombined = readSensorRegisterValueCombined*4;
                /*
                 * MSB first
                 */
                buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
                index += 1;

                buf[index] = (uint8_t)(readSensorRegisterValueCombined);
                index += 1;
        }
	i2cReadStatus                   = readSensorRegisterINA219(0x03, 2 /* numberOfBytes */);
        readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

        /*
         *      NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
         */

        if (i2cReadStatus != kWarpStatusOK)
        {
                buf[index] = 0;
                index += 1;

                buf[index] = 0;
                index += 1;
        }
        else
        {
                /* 
                
                 * MSB first
                 */
                buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
                index += 1;

                buf[index] = (uint8_t)(readSensorRegisterValueCombined);
                index += 1;
        }
	i2cReadStatus                   = readSensorRegisterINA219(0x04, 2 /* numberOfBytes */);
        readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
        readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
        readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

        /*
         *      NOTE: Here, we don't need to manually sign extend since we are packing directly into an int16_t
         */

        if (i2cReadStatus != kWarpStatusOK)
        {
                buf[index] = 0;
                index += 1;

                buf[index] = 0;
                index += 1;
        }
        else
        {
                
                readSensorRegisterValueCombined = readSensorRegisterValueCombined*50;
                /*
                 * MSB first
                 */
                buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
                index += 1;

                buf[index] = (uint8_t)(readSensorRegisterValueCombined);
                index += 1;
        }

return index;

}

uint16_t ina219_calValue = 8192; /* this is from the adafruit arduino library for 16V and 400mA */

void
calibrateINA219()
{
	writeSensorRegisterINA219(0x05, ina219_calValue);
	writeSensorRegisterINA219(0x00, (0b0000000110011111)); // calibration is based of adafruit arduino library
}

