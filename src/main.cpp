#include <Arduino.h>
#include "util/led.h"
#include "controllers/motor.h"
#include "controllers/pid.h"

//------ Defines:
#if 0 // Set to 1 for debug mode, 0 otherwise.
	#define DEBUG
#endif
// Motor flags: xxxx xxxx
#define CLOSE 	0x01
#define MEDIUM 	0x02
#define FAR 	0x03
#define RETURN 	0x40
#define STOP 	0x80


//------ Constants: // For masks: (1 << pin - minPinOnRegister), minPinOnRegister = 0 for D, 8 for B, and 14 for C.
const uint8_t US_TRIGGER = (1 << 4); // Ultrasonic trigger mask.
const uint8_t US_ECHO_PIN = 2; // Ultrasonic Echo pin.
const uint8_t IR_SIGNAL_RIGHT = (1 << (18 - 14)); // Right infrared sensor mask.
const uint8_t IR_SIGNAL_LEFT = (1 << (19 - 14)); // Left infrared sensor mask.
const uint8_t RESUME_BUTTON = (1 << 7);
const uint8_t MOTOR_RIGHT_FORWARD_PIN = 12; // DC motor.
const uint8_t MOTOR_RIGHT_REVERSE_PIN = 13;
const uint8_t MOTOR_RIGHT_POWER_PIN = 11;
const uint8_t MOTOR_LEFT_FORWARD_PIN = 14;
const uint8_t MOTOR_LEFT_REVERSE_PIN = 15;
const uint8_t MOTOR_LEFT_POWER_PIN = 10;

//------ Variables:
uint8_t motorUpdateFlag = 0x00; // 0x01 = Dist; 0x02 = Dist; 0x04 = N/A; 0x08 = N/A; 0x10 = N/A; 0x20 = N/A; 0x40 = Return; 0x80 = Stop;

//------ Classes:
G16::led_t led{.redPin = 6, .greenPin = 3, .bluePin = 5}; // RGB Led.
G16::Motor motors{MOTOR_LEFT_FORWARD_PIN, MOTOR_LEFT_REVERSE_PIN, MOTOR_RIGHT_FORWARD_PIN, MOTOR_RIGHT_REVERSE_PIN};
G16::PID motorPID{5, 3, 3}; // Not tuned.

//------ Timers:
uint32_t ultrasonicCycleStart = 0UL; // Time in µs, when the US sensor triggered.
uint32_t motorUpdateTime = 0UL;

//------ Forward declare functions:

void turn180deg();


void setup() {
	// Set output pins in Data Direction Register (DDRx) (input (0) and output (1)):
	DDRB |= 0x00; // DDRB is pin 8 to 13.
	DDRC |= 0x00; // DDRC is pin 14 to 19.
	DDRD |= 0x00 | US_TRIGGER | (1 << led.redPin) | (1 << led.greenPin) | (1 << led.bluePin); // DDRD is pin 0 to 7.
	// Set default state of pins in PORTx:
	PORTB |= 0x00;
	PORTC |= 0x00;
	PORTD |= 0x00;

	motors.initialise();

	#ifdef DEBUG
	// Starting Serial for troubleshooting.
	Serial.begin(9600);
	Serial.println(F("Initialised!")); // Store string in Flash memory.
	G16::setLedColour(led, G16::Olive);
	#endif
}

void loop() {
	// Store the timing of current loop:
	uint32_t currentMicros = micros();
	uint32_t currentMillis = millis();

	if (~PIND & US_TRIGGER) // If US trigger pin is low.
	{
		PORTD ^= US_TRIGGER; // Invert pin.
		ultrasonicCycleStart = currentMicros;
	}
	else if (currentMicros - ultrasonicCycleStart >= 20UL) // If US trigger pin is high and been that for x µs.
	{
		PORTD ^= US_TRIGGER;
		noInterrupts(); // Turn off interrupts for reading of timings, > 8-bit.
		uint32_t distance = pulseIn(US_ECHO_PIN, HIGH);
		interrupts(); // Re-enable interrupts.
		distance = ((distance * 343UL) >> 1)/1000; // Calculate the distance the sound travelled to mm.

		if (80 >= distance && !(motorUpdateFlag & RETURN))
		{
			G16::setLedColour(led, G16::Red);
			motors.stop();
			turn180deg();
			motorUpdateFlag = RETURN;
		}
		else if (160 >= distance && !((motorUpdateFlag & FAR) == CLOSE))
		{
			G16::setLedColour(led, G16::Orange);
			motors.forward();
			motors.writeDutyCycle(G16::MotorDutyCycle::SLOW);
			motorUpdateFlag = (motorUpdateFlag & ~FAR) | CLOSE;
		}
		else if (300 >= distance && !((motorUpdateFlag & FAR) == MEDIUM))
		{
			G16::setLedColour(led, G16::Yellow);
			motors.forward();
			motors.writeDutyCycle(G16::MotorDutyCycle::MEDFAS);
			motorUpdateFlag = (motorUpdateFlag & ~FAR) | MEDIUM;
		}
		else if (!((motorUpdateFlag & FAR) == FAR))
		{
			G16::setLedColour(led, G16::Green);
			motors.forward();
			motors.writeDutyCycle(G16::MotorDutyCycle::TOP);
			motorUpdateFlag |= FAR;
		}
		
	}

	switch (PINC & (IR_SIGNAL_LEFT | IR_SIGNAL_RIGHT))
	{
	case IR_SIGNAL_LEFT:
		#ifdef DEBUG
		Serial.println(F("Correct left <-"));
		#endif

		motorPID.update(1);
		break;
	case IR_SIGNAL_RIGHT:
		#ifdef DEBUG
		Serial.println(F("Correct right ->"));
		#endif

		motorPID.update(-1);
		break;
	case (IR_SIGNAL_LEFT | IR_SIGNAL_RIGHT):
		#ifdef DEBUG
		Serial.println(F("Well ... this is awkward."));
		#endif

		motors.stop();
		G16::setLedColour(led, G16::Colours::Teal);
		while (~PIND & RESUME_BUTTON); // Pause until resume button is pressed.
		break;
	default:
		motorPID.update(0);
		break;
	}

	if (currentMillis - motorUpdateTime >= 50UL) // Update motor every 50ms.
	{
		uint8_t errorCorrection = motorPID.getPID();
		uint16_t motorPWM = motors.getMeanPWM();
		motors.write(motorPWM - errorCorrection, motorPWM + errorCorrection);
		motorUpdateTime = millis();
	}
}

void turn180deg()
{
	motors.stop(); // Stop motorer.
	motors.turnLeft();
	uint32_t startTime = millis();
	motors.writeDutyCycle(G16::MotorDutyCycle::MEDSLO); // Kjør motorer.
	while (~PINC & IR_SIGNAL_RIGHT); // Vent til høyre IR sensor bryter linjen.
	uint32_t timeToBreakLine = millis() - startTime;
	while (~PINC & IR_SIGNAL_LEFT); // Vent til venstre IR sensor bryter linjen.
	startTime = millis();
	while (millis() - startTime < timeToBreakLine); // Vent til venstre IR sensor er like langt over linjen som høyre brukte til å krysse linjen.
	motors.stop();
}