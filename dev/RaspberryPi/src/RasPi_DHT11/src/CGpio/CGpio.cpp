/*
 * CGpio.cpp
 *
 *  Created on: 2019/12/06
 *      Author: CountrySideEngineer
 */

#include "CGpio.h"
#include <time.h>
#include "pigpio.h"
#include "Log/CLog.h"

/**
 * @brief	Constructor
 */
CGpio::CGpio()
{
	if (gpioInitialise() < 0) {
		CLog::Warn("GPIO Initialize failed!");
	}
}

/**
 * @brief	Destructor
 */
CGpio::~CGpio() { gpioTerminate(); }

/**
 * Get pointer to instance of CGpio object.
 */
CGpio*	CGpio::getInstance() {
	static CGpio instance;

	return &instance;
}

/**
 * @brief	Set mode of Pin, INPUT or OUTPUT.
 * @param	pin   Pin number of GPIO pin.
 * @param	mode  Pin mode, INPUT or OUTPUT.
 */
int CGpio::SetMode(
		const unsigned int pin,
		const unsigned int mode)
{
	int result = gpioSetMode(pin, mode);
	if (PI_BAD_GPIO == result) {
		CLog::Warn("GPIO pin invalid.");
	} else  if (PI_BAD_MODE == result) {
		CLog::Warn("GPIO mode invalid.");
	} else {
		CLog::Debug("GPIO set mode OK.");
	}
	return result;
}

/**
 * @brief	Set mode of register pull up, down, or clear.
 * 			The mode is "clear" in default.
 * @param	pin		Pin number of GPIO pin.
 * @param	mode	Pin mode, pull up, pull down, or clear
 */
int CGpio::SetPullUpDownMode(
		const unsigned int pin,
		const unsigned int mode)
{
	int result = gpioSetMode(pin, mode);
	if (PI_BAD_GPIO == result) {
		CLog::Warn("GPIO pin invalid.");
	} else  if (PI_BAD_PUD == result) {
		CLog::Warn("GPIO mode invalid.");
	} else {
		CLog::Debug("GPIO set pull up or down mode OK.");
	}
	return result;
}

/**
 * @brief	Set the GPIO level, on or off.
 * @param	pin		The gpio number to change the level.
 * @param	level	Pin level, on or off.
 * @return  Returns the result of level change.
 */
int CGpio::Write(const unsigned int pin, const unsigned int level) {
	int levelToWrite = 0;
	if (CGpio::CGPIO_PIN_LEVEL_LOW != level) {
		levelToWrite = CGpio::CGPIO_PIN_LEVEL_HIGH;
	} else {
		levelToWrite = CGpio::CGPIO_PIN_LEVEL_LOW;
	}

	int result = gpioWrite(pin, levelToWrite);
	if (PI_BAD_GPIO == result) {
		CLog::Warn("GPIO pin invalid.");
	} else  if (PI_BAD_LEVEL == result) {
		CLog::Warn("GPIO level invalid.");
	} else {
		CLog::Debug("GPIO write level OK.");
	}
	return result;

}

/**
 * @brief	Get the GPIO level. on or off.
 * @param	pin		The gpio pin number to read the level.
 * @param	level	Pointer to set the read level.
 * @return	Result of read, the value is PI_BAD_GPIO if reading failed,
 * 			otherwise returns the read level.
 */
int CGpio::Read(const unsigned int pin, unsigned int* level) {

	assert(NULL != level);

	timespec startTime = { 0 };
	timespec curTime = { 0 };

	clock_gettime(CLOCK_MONOTONIC_RAW, &startTime);
	int readLevel = gpioRead(pin);
	clock_gettime(CLOCK_MONOTONIC_RAW, &curTime);
	printf("passedTime = %lu\n", (curTime.tv_nsec - startTime.tv_nsec));
	if (PI_BAD_GPIO == readLevel) {
		CLog::Warn("GPIO pin invalid.");
	} else {
		CLog::Debug("GPIO read level OK.");

		*level = readLevel;
	}
	return readLevel;
}

/**
 * @brief	Wait for the time, sec and microSec, passes.
 * @param	sec			Seconds of sleep.
 * @param	microSec	Micro seconds of sleep.
 * @return	Retunrs 0 if the wait time succeeded, PI_BAD_SECONDS if the time
 * 			to wait is invalid.
 * @note	If the time to wait is shorted than 50 micro seconds, use "Delay"
 * 			method.
 * @ref		See more detail "http://abyz.me.uk/rpi/pigpio/cif.html#gpioSleep"
 */
int CGpio::Sleep(const int sec, const int microSec) {
	int result = gpioSleep(PI_TIME_RELATIVE, sec, microSec);
	if (PI_BAD_TIMETYPE == result) {
		CLog::Warn("GPIO Sleep time mode invalid.");	//Unreachable
	} else if (PI_BAD_SECONDS == result) {
		CLog::Warn("GPIO Sleep time invalid.");
	} else {
		CLog::Debug("GPIO sleep OK.");
	}
	return result;
}

/**
 * @brief	Wait for the time specified by argument microSec.
 * @param	microSec	Time to wait in micro second.
 * @return	Returns actually wait time.
 */
uint32_t CGpio::Delay(const uint32_t microSec) {
	timespec startTime = { 0 };
	clock_gettime(CLOCK_REALTIME, &startTime);

	uint32_t passedTime = 0;
	uint32_t timeToWait = microSec * 1000;
	while (passedTime < timeToWait) {
		timespec curTime = { 0 };
		clock_gettime(CLOCK_REALTIME, &curTime);

		if (curTime.tv_nsec < startTime.tv_nsec) {
			passedTime = curTime.tv_nsec + ((1000 * 1000 * 1000) - startTime.tv_nsec);
		} else {
			passedTime = curTime.tv_nsec - startTime.tv_nsec;
		}
	}
	return passedTime;
}

uint32_t CGpio::GetCurrentTime() {
	return gpioTick();
}
