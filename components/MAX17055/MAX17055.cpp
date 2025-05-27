/**********************************************************************
 *
 * MIT License
 *
 * Copyright (c) 2018 Awot Ghirmai
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 * Awot Ghirmai; ghirmai.awot@gmail.com
 * Ole Dreessen; ole.dreessen@maximintegrated.com
 *
 **********************************************************************/

#include <MAX17055.h>
#include <Wire.h>

/**********************************************************************
 * @brief MAX17055 - The MAX17055 is a low 7μA operating current fuel gauge that implements
 * Maxim ModelGauge™ m5 EZ algorithm. ModelGauge m5 EZ makes fuel gauge implementation
 * easy by eliminating battery characterization requirements and simplifying host software interaction.
 * The ModelGauge m5 EZ robust algorithm provides tolerance against battery diversity for most
 * lithium batteries and applications. ModelGauge m5 EZ algorithm combines the short-term
 * accuracy and linearity of a coulomb counter with the longterm stability of a voltage-based fuel
 * gauge, along with temperature compensation to provide industry-leading fuel gauge accuracy.
 * The MAX17055 automatically compensates for cell aging, temperature, and discharge rate, and
 * provides accurate state of charge (SOC in %) and remaining capacity in milliampere-hours (mAh).
 * As the battery approaches the critical region near empty, the ModelGauge m5 algorithm invokes
 * a special compensation that eliminates any error. It also provides three methods for reporting
 * the age of the battery: reduction in capacity, increase in battery resistance, and cycle odometer.
 * The MAX17055 provides precision measurements of current, voltage, and temperature. Temperature
 * of the battery pack is measured using an internal temperature measurement or external thermistor.
 * A 2-wire I2C interface provides access to data and control registers. The MAX17055 is available
 * in a tiny, lead-free 0.4mm pitch 1.4mm x 1.5mm, 9-pin WLP package, and a 2mm x 2.5mm, 10-pin
 * TDFN package.
 *
 * Resources can be found at
 * https://www.maximintegrated.com/en/products/power/battery-management/MAX17055.html
 * https://www.maximintegrated.com/en/app-notes/index.mvp/id/6365
 * https://www.maximintegrated.com/en/app-notes/index.mvp/id/6358
 **********************************************************************/
// battery capacity designed to be 3500mAh
// #define DesignCap  3500u
// charger quits at 50mA
#define IchgTerm 3u
// for now default Vempty (3.3V/3.88V) 0xA561
#define VEmpty 0xA561

// Constructors
MAX17055::MAX17055(void)

{
	// doesn't need anything here
}

void prit_hex(uint8_t data)
{
	Serial.print(data < 16 ? "0" : "");
	Serial.print(data, HEX);
}

esp_err_t MAX17055::init(uint16_t batteryCapacity, float resistorValue, int force)
{
	uint16_t status = 0;
	uint16_t HibCFG = 65535;
	int timeout = 0;
	i2c_error = ESP_OK;
	// step one - Check for POR
	HibCFG = readReg16Bit(STATUS);
	if ((i2c_error != ESP_OK)||(HibCFG == 65535))
	{
		if (HibCFG == 65535){
			return ESP_FAIL;
		}
	}
	if ((batteryCapacity != getCapacity()) || (resistorValue != getResistSensor()) || (HibCFG & 0x0002) || (force != 0))
	{
		Serial.print("BAT params not correct:");
		Serial.print(getCapacity());
		Serial.print(",");
		Serial.print(getResistSensor());
		Serial.print(",");
		prit_hex(HibCFG >> 8);
		prit_hex(HibCFG);
		Serial.println(".");
		// if (HibCFG & 0x0002){
		// if (true)
		//  initialise
		//   step 2
		timeout = 10000;
		do
		{
			status = readReg16Bit(F_STAT);
			delay(10); // some delay
		} while ((status & 1) && (timeout > 0)); // do not continue until FSTAT.DNR == 0
		if (timeout <= 0)
		{
			return ESP_ERR_TIMEOUT;
		}
		// step 3
		HibCFG = readReg16Bit(HIB_CFG);	  // Store original HibCFG value
		writeReg16Bit(SOFT_WAKEUP, 0x90); // Exit Hibernate Mode step 1
		writeReg16Bit(HIB_CFG, 0x00);	  // Exit Hibernate Mode step 2
		writeReg16Bit(SOFT_WAKEUP, 0x00); // Exit Hibernate Mode step 3

		// 3.1 OPTION 1 EZ Config:
		writeReg16Bit(DESIGN_CAP, batteryCapacity * 2);			 // Write DesignCap
		writeReg16Bit(DQ_ACC, (uint16_t)(batteryCapacity / 32)); // Write dQAcc
		writeReg16Bit(I_CHG_TERM, IchgTerm);					 // Write IchgTerm
		writeReg16Bit(V_EMPTY, VEmpty);							 // Write VEmpty

		int dQAcc = (int)(batteryCapacity / 32);
		writeReg16Bit(DP_ACC, (uint16_t)(dQAcc * 44138 / (int)batteryCapacity)); // Write dPAcc

		writeReg16Bit(MODEL_CFG, 0x8000); // Refresh model
		timeout = 10000;
		do
		{
			timeout--;
			status = readReg16Bit(MODEL_CFG);
			delay(10); // 10ms delay
		} while ((status & 0x8000) && (timeout > 0)); // Do not continue until ModelCFG.Refresh == 0.
		if (timeout <= 0)
		{
			return ESP_ERR_TIMEOUT;
		}
		writeReg16Bit(HIB_CFG, HibCFG); // Restore original HibCFG value
		// store Rsense
		Serial.print(",Rstore=");
		Serial.print((uint16_t)(resistorValue * 1000));
		writeReg16Bit(R_SENSE_USER_MEM3, (uint16_t)(resistorValue * 1000));
		// step4
		status = readReg16Bit(STATUS);			// Read Status
		writeReg16Bit(STATUS, status & 0xFFFD); // Write and Verify Status with POR bit cleared
	}
	// 4.3 Read the reported capacity and state of charge (SOC)
	// 4.4 Read the remaining time to empty (TTE)
	// MAX17055_status();
	return ESP_OK;
}

MAX17055::MAX17055(uint16_t batteryCapacity)
{
	// calcuation based on AN6358 page 13 figure 1.3 for Capacity, but reversed to get the register value
	writeReg16Bit(DESIGN_CAP, batteryCapacity * 2);
}
// Public Methods
void MAX17055::setCapacity(uint16_t batteryCapacity)
{
	// calcuation based on AN6358 page 13 figure 1.3 for Capacity, but reversed to get the register value
	writeReg16Bit(DESIGN_CAP, batteryCapacity * 2);
}

float MAX17055::getRepCapacity()
{
	uint16_t capacity_raw = readReg16Bit(REP_CAP);
	return (capacity_raw * capacity_multiplier_mAH);
}

float MAX17055::getCapacity()
{
	uint16_t capacity_raw = readReg16Bit(DESIGN_CAP);
	return (capacity_raw * capacity_multiplier_mAH);
}
void MAX17055::setResistSensor(float resistorValue)
{
	// resistSensor = resistorValue;
	writeReg16Bit(R_SENSE_USER_MEM3, (uint16_t)(resistorValue * 1000));
}

float MAX17055::getResistSensor()
{
	float resistorValue;
	resistorValue = (float)(readReg16Bit(R_SENSE_USER_MEM3)) / 1000;
	return resistorValue;
}

float MAX17055::getInstantaneousCurrent()
{
	float out;
	int16_t current_raw = readReg16Bit(CURRENT);
	out = current_raw * current_multiplier_mV;
	return out;
}

float MAX17055::getInstantaneousVoltage()
{
	uint16_t voltage_raw = readReg16Bit(V_CELL);
	return voltage_raw * voltage_multiplier_V;
}

float MAX17055::getSOC()
{
	uint16_t SOC_raw = readReg16Bit(REP_SOC);
	return SOC_raw * percentage_multiplier;
}

float MAX17055::getTimeToEmpty()
{
	uint16_t TTE_raw = readReg16Bit(TTE);
	return TTE_raw * time_multiplier_Hours;
}

// Private Methods

void MAX17055::writeReg16Bit(uint8_t reg, uint16_t value)
{
	// Write order is LSB first, and then MSB. Refer to AN635 pg 35 figure 1.12.2.5
	Wire.beginTransmission(I2CAddress);
	Wire.write(reg);
	Wire.write(value & 0xFF);		 // value low byte
	Wire.write((value >> 8) & 0xFF); // value high byte
	// uint8_t last_status = Wire.endTransmission();
	i2c_error = Wire.endTransmission();
}

uint16_t MAX17055::readReg16Bit(uint8_t reg)
{
	uint16_t value = 0;
	Wire.beginTransmission(I2CAddress);
	Wire.write(reg);
	// uint8_t last_status = Wire.endTransmission(false);
	i2c_error = Wire.endTransmission(false);

	i2c_error |= Wire.requestFrom(I2CAddress, (uint8_t)2);
	value = Wire.read();
	value |= (uint16_t)Wire.read() << 8; // value low byte
	return value;
}