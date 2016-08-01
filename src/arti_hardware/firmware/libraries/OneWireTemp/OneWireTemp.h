#ifndef OneWireTemp_h
#define OneWireTemp_h

#include <OneWire.h>

class OneWireTemp{
public:
	OneWireTemp();
	OneWireTemp(int pin);
	~OneWireTemp();
	void initialize(int pin);
	float getTemp();

private:
	OneWire* sensor_;
	int pin_;
};

#endif