//============================================================================
// Name        : RasPi_DHT11.cpp
// Author      : CountrySideEngineer
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "Sensor/CDHT11.h"
#include "CGpio/CGpio.h"

using namespace std;

#define	DHT11_DATA		(21)

int main() {
	CDHT11 dht11;
	dht11.setupGpioPin(DHT11_DATA);
	while (1) {
		dht11.read();
	}
	return 0;
}
