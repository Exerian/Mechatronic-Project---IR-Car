#include <Arduino.h>
#include "util/led.h"

//------ Defines:



//------ Constants:

const uint8_t US_TRIGGER_PIN = 4; // Ultrasonic sensor.
const uint8_t US_ECHO_PIN = 2;
const uint8_t IR_SIGNAL_RIGHT_PIN = 18; // Infrared sensor.
const uint8_t IR_SIGNAL_LEFT_PIN = 19;
const uint8_t MOTOR_RIGHT_FORWARD_PIN = 12; // DC motor.
const uint8_t MOTOR_RIGHT_REVERSE_PIN = 13;
const uint8_t MOTOR_RIGHT_POWER_PIN = 11;
const uint8_t MOTOR_LEFT_FORWARD_PIN = 14;
const uint8_t MOTOR_LEFT_REVERSE_PIN = 15;
const uint8_t MOTOR_LEFT_POWER_PIN = 10;
const uint8_t LED_RED_PIN = 6; // RGB LED.
const uint8_t LED_GREEN_PIN = 5;
const uint8_t LED_BLUE_PIN = 3;

//------ Variables:

uint32_t currentMillis = 0UL;
uint32_t currentMicros = 0UL;
uint32_t ultrasonicCycleStart = 0UL;
uint32_t prevBlink = 0UL;

//------ Enums:

enum class MotorSpeed:uint8_t {
	Stop = 0,
	Slow = 64,
	Fast = 255,
	Reverse = 32,
};

//------ Forward declaration:

void directionCorrection(volatile uint8_t &port);

//------ Classes:


void setup() {
	// Set pins in DDRx to input (0) and output (1):
	DDRB |= 0x38;
	DDRC |= 0x0F;
	DDRD |= 0x00 | (1 << US_TRIGGER_PIN);
	// Set default state of pins in PORTx:
	PORTB |= 0x00;
	PORTC |= 0x00;
	PORTD |= 0x00;

	// Starting Serial for troubleshooting.
	Serial.begin(9600);
	Serial.println(F("Initialised!")); // Store string in Flash memory.
	LED::setLedColour(LED::Olive);
}

void loop() {
	currentMicros = micros();
	currentMillis = millis();

	if (~PIND & 0x10) // If US trigger pin is low.
	{
		PORTD ^= 0x10; // Invert pin.
		ultrasonicCycleStart = currentMicros;
	}
	else if (currentMicros - ultrasonicCycleStart >= 20UL) // If US trigger pin is high and been that for x µs.
	{
		noInterrupts(); // Turn off interrupts for read timings.
		PORTD ^= 0x10; // Alt: (1 << US_TRIGGER_PIN)
		uint32_t distance = pulseIn(US_ECHO_PIN, HIGH);
		interrupts(); // Re-enable interrupts.
		distance = ((distance * 343) >> 1)/1000; // Calculate the distance the sound travelled to mm.

		if (100 >= distance)
		{
			Serial.print(distance);
			Serial.println(F("mm"));
		}
		
	}

	if (~PINC & 0x20)
	{
		Serial.println(F("Blå"));
	}
	if (~PINC & 0x10)
	{
		Serial.println(F("Lilla"));
	}
	
	

	if (~PINC & 0x30) // If IR-sensor lose signal.
	{
		Serial.println(F("Vehicle going off line ..."));
		if (PINC & 0x30 == 0x30) // Both sensors lose signal.
		{
			Serial.println(F("Well ... this is awkward."));
		}
		else if (PINC & 0x10) // Right sensor lose signal.
		{
			Serial.println(F("Turning right ->"));
		}
		else // Otherwise the left sensor lose signal.
		{
			Serial.println(F("Turning left <-"));
		}
		
	}
	
}

void directionCorrection() 
{

}

void LED::setLedColour(const uint32_t hexColour)
{
	setLedColour(hexColour >> 16, hexColour >> 8, hexColour);
}
void LED::setLedColour(const uint8_t red, const uint8_t green, const uint8_t blue)
{
	analogWrite(LED_RED_PIN, red);
	analogWrite(LED_GREEN_PIN, green);
	analogWrite(LED_BLUE_PIN, blue);
}