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
	enum {
		CDHT11_ERROR_OK = 0,
		CDHT11_ERROR_PULLED_UP_TIMEOUT,
		CDHT11_ERROR_SENSOR_PULL_LOW_TIMEOUT,
		CDHT11_ERROR_SENSOR_PULL_HIGH_TIMEOUT,
		CDHT11_ERROR_SENSOR_READ_DATA_TIMEOUT,
		CDHT11_ERROR_SENSOR_READ_DATA_INVALID,
		CDHT11_ERROR_SENSOR_NO_UPDATA,
		CDHT11_ERROR_MAX,
	};

public:
	CDHT11();
	CDHT11(uint8_t pin);
	virtual ~CDHT11();

	uint8_t getPin() const { return m_pin; }
	int32_t getHumidity();
	int32_t getTemperature();

	int setupGpioPin(const uint8_t pin, const int mode = 0);
	virtual int read();

protected:
	virtual int readSequence();
	virtual uint32_t waitForPulse(
			const unsigned int level,
			const unsigned int time);
	virtual bool isScanIntervalPassed();
	virtual void updateInterval();
	virtual bool validateCheckSum();
	virtual void InitDataBuff();
	virtual void ShowBuff();

protected:
	static const uint32_t	WAIT_SIGNAL_TIMEOUT;
	static const uint32_t DATA_BUFF_SIZE;

	enum {
		CDH11_DATA_PART_SIZE = 4,
	};
	enum {
		CDHT11_DATA_BUFF_INDEX_RH_HIGH = 0,
		CDHT11_DATA_BUFF_INDEX_RH_LOW,
		CDHT11_DATA_BUFF_INDEX_T_HIGH,
		CDHT11_DATA_BUFF_INDEX_T_LOW,
		CDHT11_DATA_BUFF_INDEX_CHECK_SUM,
	};
	enum {
		CDHT11_DATA_BUFF_SIZE = 5,
	};

	uint8_t		m_dataBuff[CDHT11_DATA_BUFF_SIZE];
	uint8_t		m_pin;
	int32_t		m_temperature;
	int32_t		m_humidity;
	uint32_t	m_currentTime;
};

/**
 * @brief	Initialize buffer used to store received data via
 */
inline void CDHT11::InitDataBuff() {
	for (int buffIndex = 0; buffIndex < CDHT11_DATA_BUFF_SIZE; buffIndex++) {
		this->m_dataBuff[buffIndex] = 0;
	}
}

inline void CDHT11::ShowBuff() {
	for (int buffIndex = 0; buffIndex < CDHT11_DATA_BUFF_SIZE; buffIndex++) {
		if ((0 != buffIndex) && (0 == (buffIndex % 16))) {
			printf("\n");
		}
		printf("0x%02X, ", this->m_dataBuff[buffIndex]);
	}
	printf("\n");
}

#endif /* SENSOR_CDHT11_H_ */
