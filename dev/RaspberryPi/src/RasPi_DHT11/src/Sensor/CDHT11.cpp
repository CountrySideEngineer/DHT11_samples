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
		return 0;
	}

	this->readSequence();

	CGpio* instance = CGpio::getInstance();
	this->m_currentTime = instance->GetCurrentTime();

	printf("T = %d.%d\n",
			this->getTemperature() / 10,
			this->getTemperature() % 10);

	return 0;
}


/*
 * A wait time to set data signal level keeping low to send start signal.
 * From the data sheet, the time should be larger than 18 msec, no more than
 * 30 msec.
 */
//#define	DHT11_START_SIGNAL_LOW_TIME_MILLISEC		(18)
//A little bit longer than the value in data sheet...
#define	DHT11_START_SIGNAL_LOW_TIME_MILLISEC		(18)
#define	DHT11_START_SIGNAL_HIGH_TIME_MILLISEC		(20)
#define	MILLI2MICRO_SEC(milli_sec)	((milli_sec) * 1000)
#define	DHT11_DATA_START_LOW_BIT_WAIT_TIME			(50)
#define	DHT11_DATA_FOLLOW_HIGH_BIT_WAIT_TIME		(70)

/**
 * @brief	Run sequence to read the data from sensor, DHT11.
 * 			A sequence run in this method is on the data sheet.
 */
void CDHT11::readSequence() {

	timespec startTime = { 0 };
	timespec curTime = { 0 };
	CGpio* gpio = CGpio::getInstance();

	//Sending start signal.
	//Step1:Seding start signals.
	gpio->SetPullUpDownMode(this->m_pin, CGpio::CGPIO_PULL_UP_DOWN_UP);
	gpio->SetMode(this->m_pin, CGpio::CGPIO_PIN_IN_OUT_OUTPUT);
	gpio->Write(this->m_pin, CGpio::CGPIO_PIN_LEVEL_LOW);
	//clock_gettime(CLOCK_MONOTONIC_RAW, &startTime);
	int waitCount = 0;
	unsigned int readDateTemp = 0;
	do {
		gpio->Read(this->m_pin, &readDateTemp);
		//printf("readDateTemp = %s\n", (0 == readDateTemp ? "LOW" : "HIGH"));
		clock_gettime(CLOCK_MONOTONIC_RAW, &curTime);
		waitCount++;
	} while ((curTime.tv_nsec - startTime.tv_nsec) < (18 * 1000));
	//printf("passedTime = %lu - waitCount = %d\n", (curTime.tv_nsec - startTime.tv_nsec), waitCount);

	//gpio->Delay(MILLI2MICRO_SEC(DHT11_START_SIGNAL_LOW_TIME_MILLISEC));
#if 0
	for (int index = 0; index < 100; index++) {
		{
			unsigned int tempRead = 0;
			gpio->Read(this->m_pin, &tempRead);
			printf("tempRead = %s\n", (0 == tempRead) ? "LOW" : "HIGH");
		}
		gpio->Delay(MILLI2MICRO_SEC(DHT11_START_SIGNAL_LOW_TIME_MILLISEC));
	}
#endif

	//Step2:Waiting for response signal by setting GPIO pin pulled up.
	gpio->SetMode(this->m_pin, CGpio::CGPIO_PIN_IN_OUT_INPUT);
	gpio->SetPullUpDownMode(this->m_pin, CGpio::CGPIO_PULL_UP_DOWN_UP);//先にモードを変更してからでないと、PULL-UPに設定されない。

	waitCount = 0;
	//clock_gettime(CLOCK_MONOTONIC_RAW, &startTime);
	do {
		gpio->Read(this->m_pin, &readDateTemp);
		//printf("readDateTemp = %s\n", (0 == readDateTemp ? "LOW" : "HIGH"));
		waitCount++;
	} while (1 == readDateTemp);
	//clock_gettime(CLOCK_MONOTONIC_RAW, &curTime);
	//printf("passedTime = %lu - waitCount = %d\n", (curTime.tv_nsec - startTime.tv_nsec), waitCount);

#if 0
	for (int index = 0; index < 100; index++) {
		{
			unsigned int tempRead = 0;
			gpio->Read(this->m_pin, &tempRead);
			printf("tempRead = %s\n", (0 == tempRead) ? "LOW" : "HIGH");
		}
		gpio->Delay(MILLI2MICRO_SEC(DHT11_START_SIGNAL_LOW_TIME_MILLISEC));
	}
#endif

#if 0
	clock_gettime(CLOCK_MONOTONIC_RAW, &startTime);
	if (WAIT_SIGNAL_TIMEOUT == this->waitForPulse(CGpio::CGPIO_PIN_LEVEL_HIGH, 40)) {
		CLog::Warn("Response to pulled up signal can not be detected.");
		return;
	}
	clock_gettime(CLOCK_MONOTONIC_RAW, &curTime);
	printf("CDHT11::readSequence() : passedTime_nsec(sec) = %ld\n", (curTime.tv_nsec - startTime.tv_nsec));
#endif
	if (WAIT_SIGNAL_TIMEOUT == this->waitForPulse(CGpio::CGPIO_PIN_LEVEL_LOW, 80)) {
		CLog::Warn("Response to start signal of low can not be detected.");
		return;
	}
	if (WAIT_SIGNAL_TIMEOUT == this->waitForPulse(CGpio::CGPIO_PIN_LEVEL_HIGH, 80)) {
		CLog::Warn("Response to start signal of high can not be detected.");
		return;
	}


	uint32_t cycleBuff[80] = { 0 };

	for (int index = 0; index < 80; index += 2) {
		cycleBuff[index] = this->waitForPulse(
				(const unsigned int)CGpio::CGPIO_PIN_LEVEL_LOW,
				(const unsigned int)DHT11_DATA_START_LOW_BIT_WAIT_TIME);
		cycleBuff[index + 1] = this->waitForPulse(
				(const unsigned int)CGpio::CGPIO_PIN_LEVEL_HIGH,
				(const unsigned int)DHT11_DATA_FOLLOW_HIGH_BIT_WAIT_TIME);
	}

	this->InitDataBuff();
	for (int index = 0; index < 40; index++) {
		uint32_t startBit = cycleBuff[index * 2];
		uint32_t followBit = cycleBuff[index * 2 + 1];
		if ((CDHT11::WAIT_SIGNAL_TIMEOUT == startBit) ||
			(CDHT11::WAIT_SIGNAL_TIMEOUT == followBit))
		{
			CLog::Warn("Reading data failed.");
			return;
		}

		this->m_dataBuff[index / 8] <<= 1;	//
		if (startBit < followBit) {
			this->m_dataBuff[index / 8] |= 1;
		}
	}

#if 1
	//printf("CDHT11::readSequence() : passedTime_nsec(sec) = %ld\n", (curTime.tv_nsec - startTime.tv_nsec));
	for (int index = 0; index < 5; index++) {
		printf("0x%02X ", this->m_dataBuff[index]);
	}
	printf("\n");
#endif

	if (false == this->validateCheckSum()) {
		CLog::Error("Receive data invalid.");
	} else {
		CLog::Debug("Receive data valid.");
	}

	return;
#if 0
	CGpio* instance = CGpio::getInstance();

	timespec startTime = { 0 };
	timespec curTime = { 0 };

	//Send start signal
	//step 1 : Data signal pulls LOW.
	instance->SetPullUpDownMode(this->m_pin, CGpio::CGPIO_PULL_UP_DOWN_UP);
	instance->SetMode(this->m_pin, CGpio::CGPIO_PIN_IN_OUT_OUTPUT);
	instance->Write(this->m_pin, CGpio::CGPIO_PIN_LEVEL_LOW);
#if 1
	{
		unsigned int tempRead = 0;
		instance->Read(this->m_pin, &tempRead);
		printf("tempRead = %s\n", (0 == tempRead) ? "LOW" : "HIGH");
	}
#endif
	clock_gettime(CLOCK_MONOTONIC_RAW, &startTime);
	instance->Delay(MILLI2MICRO_SEC(DHT11_START_SIGNAL_LOW_TIME_MILLISEC));
	clock_gettime(CLOCK_MONOTONIC_RAW, &curTime);
	printf("CDHT11::readSequence() : passedTime_nsec(sec) = %ld\n", (curTime.tv_nsec - startTime.tv_nsec));


#if 0
	{
		unsigned int tempRead = 0;
		instance->Read(this->m_pin, &tempRead);
		printf("tempRead = %s\n", (0 == tempRead) ? "LOW" : "HIGH");
	}
#endif

	clock_gettime(CLOCK_MONOTONIC_RAW, &startTime);
	//step 2 : Data signal pulls HIGH for about 20 to 40 usec.
	//instance->Write(this->m_pin, CGpio::CGPIO_PIN_LEVEL_HIGH);
	//instance->Delay(DHT11_START_SIGNAL_HIGH_TIME_MILLISEC);
	instance->SetMode(this->m_pin, CGpio::CGPIO_PIN_IN_OUT_INPUT);
	instance->SetPullUpDownMode(this->m_pin, CGpio::CGPIO_PULL_UP_DOWN_UP);
	instance->Delay(55);

#if 1
	{
		unsigned int tempRead = 0;
		instance->Read(this->m_pin, &tempRead);
		printf("tempRead = %s\n", (0 == tempRead) ? "LOW" : "HIGH");
	}
#endif

	//Check response to start signal.
	//Wait for an external signal end of low level,
	//Change GPIO pin mode into INPUT, PULL-UP
	if (WAIT_SIGNAL_TIMEOUT == this->waitForPulse(CGpio::CGPIO_PIN_LEVEL_LOW, 80)) {
		CLog::Warn("Response to start signal of low can not be detected.");
		return;
	}
	clock_gettime(CLOCK_MONOTONIC_RAW, &curTime);

	printf("CDHT11::readSequence() : passedTime_nsec(sec) = %ld\n", (curTime.tv_nsec - startTime.tv_nsec));

	//printf("CGpio::Delay() : microsec = %lu, passedTime = %lu\n", microSec, passedTime);

	//Wait for a signal level high to notice peripherals is ready to receive teh
	if (WAIT_SIGNAL_TIMEOUT == this->waitForPulse(CGpio::CGPIO_PIN_LEVEL_HIGH, 80)) {
		CLog::Warn("Response to start signal of high can not be detected.");
		return;
	}

	uint32_t cycleBuff[80] = { 0 };

	for (int index = 0; index < 80; index += 2) {
		//clock_gettime(CLOCK_MONOTONIC, &startTime);
		cycleBuff[index] = this->waitForPulse(
				(const unsigned int)CGpio::CGPIO_PIN_LEVEL_LOW,
				(const unsigned int)DHT11_DATA_START_LOW_BIT_WAIT_TIME);
		//clock_gettime(CLOCK_MONOTONIC, &curTime);
		cycleBuff[index + 1] = this->waitForPulse(
				(const unsigned int)CGpio::CGPIO_PIN_LEVEL_HIGH,
				(const unsigned int)DHT11_DATA_FOLLOW_HIGH_BIT_WAIT_TIME);
		//clock_gettime(CLOCK_MONOTONIC, &curTime);
		//long int passedTime = curTime.tv_nsec - startTime.tv_nsec;
		//printf("CDHT11::readSequence(%d) : passedTime_nsec = %ld\n", index, passedTime);
	}

#if 0
	for (int index = 0; index < 80; index++) {
		if ((0 != index) && (0 == (index % 16))) {
			printf("\n");
		}
		printf("%3u - 0x%08X ", index, cycleBuff[index]);
	}
	printf("\n");
#endif

	this->InitDataBuff();
	for (int index = 0; index < 40; index++) {
		uint32_t startBit = cycleBuff[index * 2];
		uint32_t followBit = cycleBuff[index * 2 + 1];
		if ((CDHT11::WAIT_SIGNAL_TIMEOUT == startBit) ||
			(CDHT11::WAIT_SIGNAL_TIMEOUT == followBit))
		{
			CLog::Warn("Reading data failed.");
			return;
		}

		this->m_dataBuff[index / 8] <<= 1;	//
		if (startBit < followBit) {
			this->m_dataBuff[index / 8] |= 1;
		}
	}

#if 0
	for (int index = 0; index < 5; index++) {
		printf("0x%02X ", this->m_dataBuff[index]);
	}
	printf("\n");
#endif

	if (false == this->validateCheckSum()) {
		CLog::Error("Receive data invalid.");
	} else {
		CLog::Debug("Receive data valid.");
	}
#endif
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
#if 0
	{
		printf("CDHT11::waitForPulse(start) : readLevel = %s\n", (0 == readLevel) ? "LOW" : "HIGH");
	}
#endif

#if 1
	uint32_t passedTime = 0;
	while (level == readLevel) {
		if ((time * 1000) < passedTime) {
			//Time out!
			return CDHT11::WAIT_SIGNAL_TIMEOUT;
		}
		passedTime += instance->Delay(1);
		instance->Read(this->m_pin, &readLevel);
#if 0
		{
			printf("CDHT11::waitForPulse(read) : readLevel = %s\n", (0 == readLevel) ? "LOW" : "HIGH");
		}
#endif
		passedTime++;
	}
#endif
	//CLog::Debug("Wait for sensor start signal : OK");
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

	if ((uint8_t)checkSum == this->m_dataBuff[CDH11_DATA_BUFF_INDEX_CHECK_SUM]) {
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
			(((this->m_dataBuff[CDH11_DATA_BUFF_INDEX_T_HIGH] & 0x7F) << 8) +
			  (this->m_dataBuff[CDH11_DATA_BUFF_INDEX_T_LOW]));
	if (0 != (this->m_dataBuff[CDH11_DATA_BUFF_INDEX_T_HIGH] & 0x80)) {
		temperature *= (-1);
	}
	return temperature;

}
