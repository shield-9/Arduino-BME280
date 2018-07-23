#include <BME280.hpp>

template<class T> inline Print &operator <<(Print &stream, T arg) { stream.print(arg); return stream; }

void setup() {
	Serial.begin(9600);
	Serial << F("This is a demo for BME280 Library by Extend Wings (www.extendwings.com).\n\n");

	bme.begin(0x76);
}

void loop() {
	float t, p, h;
	bme.getData(&t, &p, &h);

	Serial << "Temperature: " << t << " C\n";
	Serial << "Pressure: " << p << " hPa\n";
	Serial << "Humidity: " << h << " %\n";
	Serial << "---------------\n";

	delay(5000);
}
