/*
 * CGpio.h
 *
 *  Created on: 2019/12/06
 *      Author: CountrySideEngineer
 */

#ifndef CGPIO_CGPIO_H_
#define CGPIO_CGPIO_H_

#include "pigpio.h"

class CGpio {

public:

	/*
	 * Enum to define the min and max of pin number.
	 */
	enum {
		CGPIO_PIN_NUM_MIN = 0,
		CGPIO_PIN_NUM_MAX = 53,
	};

	/*
	 * Enum of GPIO pin direction, input, or output.
	 */
	enum {
		CGPIO_PIN_IN_OUT_INPUT = 0,
		CGPIO_PIN_IN_OUT_OUTPUT,
		CGPIO_PIN_IN_OUT_MAX
	};

	/**
	 * Enum of register, pull up , down, or clear.
	 */
	enum {
		CGPIO_PULL_UP_DOWN_OFF = 0,
		CGPIO_PULL_UP_DOWN_DOWN,
		CGPIO_PULL_UP_DOWN_UP,
		CGPIO_PULL_UP_DOWN_MAX,
	};

	/*
	 * Enum of gpio pin level, LOW and HIGH.
	 */
	enum {
		CGPIO_PIN_LEVEL_LOW = 0,
		CGPIO_PIN_LEVEL_HIGH,
	};

public:
	virtual ~CGpio();

	static CGpio* getInstance();

	int	SetMode(const unsigned int pin, const unsigned int mode);
	int	SetPullUpDownMode(const unsigned int pin, const unsigned int mode);
	int	Write(const unsigned int pin, const unsigned int level);
	int	Read(const unsigned int pin, unsigned int* level);

	int	Sleep(const int sec, const int microSec);
	uint32_t	Delay(const uint32_t microSec);

	uint32_t	GetCurrentTime();

protected:
	CGpio();	//To singleton.
};

#endif /* CGPIO_CGPIO_H_ */
