#ifndef OneWireTemp_h
#define OneWireTemp_h

#include <OneWire.h>

class OneWireTemp{
public:
	OneWireTemp(int pin);
	float getTemp();

private:
	OneWire* sensor_;
	int pin_;
};

#endif