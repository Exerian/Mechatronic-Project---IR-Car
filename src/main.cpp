#include <Arduino.h>
#include "util/led.h"
#include "motor.h"

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
const uint32_t SPEED_OF_SOUND = 343; // m/s

//------ Variables:
uint32_t ultrasonicCycleStart = 0UL; //

//------ Classes:
G16::led_t led{.redPin = 6, .greenPin = 3, .bluePin = 5}; // RGB Led.
G16::Motor motors{MOTOR_LEFT_FORWARD_PIN, MOTOR_LEFT_REVERSE_PIN, MOTOR_RIGHT_FORWARD_PIN, MOTOR_RIGHT_REVERSE_PIN};


void setup() {
	// Set output pins in Data Direction Register (DDRx) (input (0) and output (1)):
	DDRB |= 0x00; // | (1 << MOTOR_RIGHT_FORWARD_PIN - 8) | (1 << MOTOR_RIGHT_REVERSE_PIN - 8) | (1 << MOTOR_RIGHT_POWER_PIN - 8) | (1 << MOTOR_LEFT_POWER_PIN - 8);
	DDRC |= 0x00; // | (1 << MOTOR_LEFT_FORWARD_PIN - 14) | (1 << MOTOR_LEFT_REVERSE_PIN - 14);
	DDRD |= 0x00 | (1 << US_TRIGGER_PIN) | (1 << led.redPin) | (1 << led.greenPin) | (1 << led.bluePin);
	// Set default state of pins in PORTx:
	PORTB = 0x00;
	PORTC = 0x00 | (1 << IR_SIGNAL_RIGHT_PIN - 14);
	PORTD = 0x00;

	// Starting Serial for troubleshooting.
	Serial.begin(9600);
	Serial.println(F("Initialised!")); // Store string in Flash memory.
	G16::setLedColour(led, G16::Olive);
}

void loop() {
	// Store the timing of current loop:
	uint32_t currentMicros = micros();
	uint32_t currentMillis = millis();

	if (~PIND & 0x10) // If US trigger pin is low.
	{
		PORTD ^= 0x10; // Invert pin.
		ultrasonicCycleStart = currentMicros;
	}
	else if (currentMicros - ultrasonicCycleStart >= 20UL) // If US trigger pin is high and been that for x µs.
	{
		noInterrupts(); // Turn off interrupts for reading of timings.
		PORTD ^= 0x10;
		uint32_t distance = pulseIn(US_ECHO_PIN, HIGH);
		interrupts(); // Re-enable interrupts.
		distance = ((distance * 343UL) >> 1)/1000; // Calculate the distance the sound travelled to mm.

		if (100 >= distance && currentMillis - led.previousChange >= 500UL)
		{
			Serial.print(distance);
			Serial.println(F("mm"));
			G16::setLedColour(led, G16::Red);
			led.previousChange = currentMillis;
			motors.forward();
			motors.write(100,100);
		}
		else if (currentMillis - led.previousChange >= 500UL)
		{
			G16::setLedColour(led, G16::Green);
			motors.stop();
		}
		
	}

	if (~PINC & 0x30) // If IR-sensor lose signal.
	{
		Serial.println(F("Vehicle going off line ..."));
		if (~PINC & 0x30 == 0x30) // Both sensors lose signal.
		{
			Serial.println(F("Well ... this is awkward."));
		}
		else if (~PINC & 0x10) // Right sensor lose signal.
		{
			Serial.println(F("Turning right ->"));
			//motors.turnRight();
		}
		else if (~PINC & 0x20)// Otherwise the left sensor lose signal.
		{
			Serial.println(F("Turning left <-"));
			//motors.turnLeft();
		}
	}	
}