/*
 * CDHT11.cpp
 *
 *  Created on: 2019/12/06
 *      Author: CoutrySideEngineer
 */

#include <time.h>
#include "CDHT11.h"
#include "CGpio/CGpio.h"
#include "Log/CLog.h"

const uint32_t CDHT11::WAIT_SIGNAL_TIMEOUT = (uint32_t)(-1);
const uint32_t CDHT11::DATA_BUFF_SIZE = 10;

/**
 * @brief	Default constructor.
 */
CDHT11::CDHT11()
: m_pin(0)
, m_temperature(0)
, m_humidity(0)
, m_currentTime(0)
{}

/**
 * @brief	Constructor with GPIO pin number.
 */
CDHT11::CDHT11(uint8_t pin)
: m_pin(pin)
, m_temperature(0)
, m_humidity(0)
, m_currentTime(0)
{}

/**
 * Destructor
 */
CDHT11::~CDHT11() {}


/**
 * @brief	Setup GPIO pin and its mode(access direction), input or output.
 * @param	pin		GPIO pin No.
 * @param	mode	GPIO pin mode, other words, access direction, input, or output.
 * 					This parameter is 0 in default, it is "INPUT".
 * @return	Result of setup, returns 0 if the setup finished successfully,
 * 			otherwise return none 0 value.
 */
int CDHT11::setupGpioPin(const uint8_t pin, const int mode) {

	this->m_pin = pin;

	CGpio* instance = CGpio::getInstance();
	int setupResult = instance->SetMode(this->m_pin, mode);
	if (0 != setupResult) {
		CLog::Warn("CDH11 : Setup GPIO pin failed.");
	} else {
		instance->SetPullUpDownMode(this->m_pin, CGpio::CGPIO_PULL_UP_DOWN_UP);
	}
	return setupResult;
}

/**
 * @brief	Read sensor value.
 * @return	Returns 0 if reading sensor value finished succeeded, otherwise
 * 			returns none 0 value.
 */
int CDHT11::read() {

	if (false == this->isScanIntervalPassed()) {
		return CDHT11_ERROR_SENSOR_NO_UPDATA;
	}

	int sequenceResult = this->readSequence();
	if (CDHT11_ERROR_OK != sequenceResult) {
		CLog::Error("Reading sensor failed.");
	} else {
		printf("T = %d.%d\n",
				this->getTemperature() / 10,
				this->getTemperature() % 10);
	}
	this->updateInterval();

	return sequenceResult;
}


/*
 * A wait time to set data signal level keeping low to send start signal.
 * From the data sheet, the time should be larger than 18 msec, no more than
 * 30 msec.
 */
//#define	DHT11_START_SIGNAL_LOW_TIME_MILLISEC				(18)
//A little bit longer than the value in data sheet...
#define	DHT11_START_SIGNAL_LOW_TIME_MILLISEC				(18)
#define	DHT11_START_SIGNAL_HIGH_TIME_MILLISEC				(20)
#define	MILLI2MICRO_SEC(milli_sec)							((milli_sec) * 1000)
#define	DHT11_SENSOR_READY_TO_OUTPUT_SIGNAL_TIME_MILLSEC	(80)

#define	DHT11_DATA_START_LOW_BIT_WAIT_TIME					(50)
#define	DHT11_DATA_FOLLOW_HIGH_BIT_WAIT_TIME				(70)

/**
 * @brief	Run sequence to read the data from sensor, DHT11.
 * 			A sequence run in this method is on the data sheet.
 * @return	Returns true if the sequence finished succesfully, otherwise
 * 			retunrs none 0 value.
 */
int CDHT11::readSequence() {

	CGpio* gpio = CGpio::getInstance();

	//Sending start signal.
	//Step1:Seding start signals.
	gpio->SetPullUpDownMode(this->m_pin, CGpio::CGPIO_PULL_UP_DOWN_UP);
	gpio->SetMode(this->m_pin, CGpio::CGPIO_PIN_IN_OUT_OUTPUT);
	gpio->Write(this->m_pin, CGpio::CGPIO_PIN_LEVEL_LOW);
	gpio->Delay(MILLI2MICRO_SEC(DHT11_START_SIGNAL_LOW_TIME_MILLISEC));

	//Step2:Waiting for response signal by setting GPIO pin pulled up.
	gpio->SetMode(this->m_pin, CGpio::CGPIO_PIN_IN_OUT_INPUT);
	gpio->SetPullUpDownMode(this->m_pin, CGpio::CGPIO_PULL_UP_DOWN_UP);
	if (WAIT_SIGNAL_TIMEOUT == this->waitForPulse(
			CGpio::CGPIO_PIN_LEVEL_HIGH,
			DHT11_START_SIGNAL_HIGH_TIME_MILLISEC))
	{
		CLog::Warn("Response to pulled up signal can not be detected.");
		return CDHT11_ERROR_PULLED_UP_TIMEOUT;
	}

	/*
	 * Step3:Waiting for response signal sensor send to notify host,
	 * raspberry-pi, to ready to output data.
	 */
	if (WAIT_SIGNAL_TIMEOUT == this->waitForPulse(
			CGpio::CGPIO_PIN_LEVEL_LOW,
			DHT11_SENSOR_READY_TO_OUTPUT_SIGNAL_TIME_MILLSEC))
	{
		CLog::Warn("Response to start signal of low can not be detected.");
		return CDHT11_ERROR_SENSOR_PULL_LOW_TIMEOUT;
	}
	if (WAIT_SIGNAL_TIMEOUT == this->waitForPulse(
			CGpio::CGPIO_PIN_LEVEL_HIGH,
			DHT11_SENSOR_READY_TO_OUTPUT_SIGNAL_TIME_MILLSEC))
	{
		CLog::Warn("Response to start signal of high can not be detected.");
		return CDHT11_ERROR_SENSOR_PULL_HIGH_TIMEOUT;
	}

	//Step4:Read signal sensor sends.
	uint32_t cycleBuff[80] = { 0 };
	for (int index = 0; index < 80; index += 2) {
		cycleBuff[index] = this->waitForPulse(
				(const unsigned int)CGpio::CGPIO_PIN_LEVEL_LOW,
				(const unsigned int)DHT11_DATA_START_LOW_BIT_WAIT_TIME);
		cycleBuff[index + 1] = this->waitForPulse(
				(const unsigned int)CGpio::CGPIO_PIN_LEVEL_HIGH,
				(const unsigned int)DHT11_DATA_FOLLOW_HIGH_BIT_WAIT_TIME);
	}

	//Step
	this->InitDataBuff();
	for (int index = 0; index < 40; index++) {
		uint32_t startBit = cycleBuff[index * 2];
		uint32_t followBit = cycleBuff[index * 2 + 1];
		if ((CDHT11::WAIT_SIGNAL_TIMEOUT == startBit) ||
			(CDHT11::WAIT_SIGNAL_TIMEOUT == followBit))
		{
			CLog::Warn("Reading data failed.");
			return CDHT11_ERROR_SENSOR_READ_DATA_TIMEOUT;;
		}

		this->m_dataBuff[index / 8] <<= 1;	//
		if (startBit < followBit) {
			this->m_dataBuff[index / 8] |= 1;
		}
	}

	if (false == this->validateCheckSum()) {
		CLog::Error("Receive data invalid.");
		return CDHT11_ERROR_SENSOR_READ_DATA_INVALID;
	} else {
		CLog::Debug("Receive data valid.");
		return CDHT11_ERROR_OK;
	}
}

/**
 * Wait while the pin level is kept.
 *
 * @param	level	The pin level to wait while.
 * @param	time	Max time to wait for, specified by micro seconds.
 * @return	Returns wait time. If timeout occurred, returns 0xFFFFFFFF, meaning
 * 			-1.
 */
uint32_t CDHT11::waitForPulse(
		const unsigned int level,
		const unsigned int time)
{
	CGpio* instance = CGpio::getInstance();

	unsigned int readLevel = 0;
	instance->Read(this->m_pin, &readLevel);

	uint32_t passedTime = 0;
	while (level == readLevel) {
		if ((time * 1000) < passedTime) {
			//Time out!
			return CDHT11::WAIT_SIGNAL_TIMEOUT;
		}
		passedTime += instance->Delay(1);
		instance->Read(this->m_pin, &readLevel);
		passedTime++;
	}
	CLog::Debug("Wait for sensor start signal : OK");
	return passedTime;
}

#define	DHT11_SCAN_INTERVAL		(2000000)
/**
 * Returns the time to interval is passed.
 *
 * @return	Returns true if the time to be needed to wait to get correct data
 * 			from has passed, otherwise false.
 */
bool CDHT11::isScanIntervalPassed() {

	bool isPassed = false;
	CGpio* instance = CGpio::getInstance();
	uint32_t currentTime = instance->GetCurrentTime();
	if (DHT11_SCAN_INTERVAL < (currentTime - this->m_currentTime)) {
		isPassed = true;
	} else {
		isPassed = false;
	}

	return isPassed;
}

/**
 * Update currnet time used to check update time interval.
 */
void CDHT11::updateInterval() {
	CGpio* instance = CGpio::getInstance();
	this->m_currentTime = instance->GetCurrentTime();
}

/**
 * Calc and compare check sum.
 *
 * @return	Retuns true if the checksum in data equals to that calculated from
 * 			the received data.
 */
bool CDHT11::validateCheckSum() {
	uint16_t checkSum = 0;
	for (int index = 0; index < CDH11_DATA_PART_SIZE; index++) {
		checkSum += this->m_dataBuff[index];
	}

	if ((uint8_t)checkSum == this->m_dataBuff[CDHT11_DATA_BUFF_INDEX_CHECK_SUM]) {
		return true;
	} else {
		return false;
	}
}

/**
 * Returns temperature read from sensors.
 *
 * @return	Temperature.
 */
int32_t CDHT11::getTemperature() {
	int32_t temperature = (int32_t)
			(((this->m_dataBuff[CDHT11_DATA_BUFF_INDEX_T_HIGH] & 0x7F) << 8) +
			  (this->m_dataBuff[CDHT11_DATA_BUFF_INDEX_T_LOW]));
	if (0 != (this->m_dataBuff[CDHT11_DATA_BUFF_INDEX_T_HIGH] & 0x80)) {
		temperature *= (-1);
	}
	return temperature;

}
