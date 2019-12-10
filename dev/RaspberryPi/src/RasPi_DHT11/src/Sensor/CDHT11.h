/*
 * CDHT11.h
 *
 *  Created on: 2019/12/06
 *      Author: CoutrySideEngineer
 */

#ifndef SENSOR_CDHT11_H_
#define SENSOR_CDHT11_H_
#include <cstdio>
#include "pigpio.h"

class CDHT11 {
public:
	CDHT11();
	CDHT11(uint8_t pin);
	virtual ~CDHT11();

	uint8_t getPin() const { return m_pin; }
	int32_t getHumidity() const { return m_humidity; }
	int32_t getTemperature();

	int setupGpioPin(const uint8_t pin, const int mode = 0);
	virtual int read();

protected:
	virtual void readSequence();
	virtual uint32_t waitForPulse(
			const unsigned int level,
			const unsigned int time);
	virtual bool isScanIntervalPassed();
	virtual bool validateCheckSum();
	virtual void InitDataBuff();


protected:
	static const uint32_t	WAIT_SIGNAL_TIMEOUT;
	static const uint32_t DATA_BUFF_SIZE;

	enum {
		CDH11_DATA_PART_SIZE = 4,
	};
	enum {
		CDH11_DATA_BUFF_INDEX_RH_HIGH = 0,
		CDH11_DATA_BUFF_INDEX_RH_LOW,
		CDH11_DATA_BUFF_INDEX_T_HIGH,
		CDH11_DATA_BUFF_INDEX_T_LOW,
		CDH11_DATA_BUFF_INDEX_CHECK_SUM,
		CDH11_DATA_BUFF_SIZE
	};

	uint8_t		m_dataBuff[CDH11_DATA_BUFF_SIZE];
	uint8_t		m_pin;
	int32_t		m_temperature;
	int32_t		m_humidity;
	uint32_t	m_currentTime;
};

/**
 * @brief	Initialize buffer used to store received data via
 */
inline void CDHT11::InitDataBuff() {
	for (int index = 0; index < CDH11_DATA_BUFF_SIZE; index++) {
		this->m_dataBuff[index] = 0;
	}
}

#endif /* SENSOR_CDHT11_H_ */
