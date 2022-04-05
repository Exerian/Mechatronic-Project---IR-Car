#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Servo.h>

//------ Defines:



//------ Constants:

const uint8_t US_TRIGGER_PIN = 4; // Ultrasonic sensor.
const uint8_t US_ECHO_PIN = 3;
const uint8_t IR_SIGNAL_WHITE_PIN = 18; // Infrared sensor.
const uint8_t IR_SIGNAL_BLACK_PIN = 19;
const uint8_t MOTOR_FORWARD_PIN = 12; // DC motor.
const uint8_t MOTOR_REVERSE_PIN = 13;
const uint8_t MOTOR_POWER_PIN = 11;
const uint8_t SERVO_POWER_PIN = 5; // Servo motor.
const uint8_t LCD_RS_PIN = 7;
const uint8_t LCD_EN_PIN = 2;
const uint8_t LCD_DB4_PIN = 14;
const uint8_t LCD_DB5_PIN = 15;
const uint8_t LCD_DB6_PIN = 16;
const uint8_t LCD_DB7_PIN = 17;
const uint8_t LED_RED_PIN = 6;
const uint8_t LED_GREEN_PIN = 9;
const uint8_t LED_BLUE_PIN = 10;
const uint16_t SPEED_OF_SOUND = 343; // µm/µs

//------ Variables:

uint8_t driveFlags = 0x00;
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
namespace Colours {
enum :uint32_t {
	Blank = 0x000000,
	White = 0xFFFFFF,
	Red = 0xFF0000,
	Green = 0x00FF00,
	Blue = 0x0000FF,
	Orange = 0xFF8000,
	Yellow = 0xFFFF00,
};
}

//------ Functions:

void setLedColour(uint32_t hexColour);
void setLedColour(uint8_t red, uint8_t green, uint8_t blue);

//------ Classes:

Servo servo;
LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_DB4_PIN, LCD_DB5_PIN, LCD_DB6_PIN, LCD_DB7_PIN);


void setup() {
	// Set pins to input and output in data/port register (DDRx):
	DDRB = 0x38;
	DDRC = 0x0F;
	DDRD = 0x00 | (1 << US_TRIGGER_PIN); // 0 = Input, 1 = Output
	// Set default state of pins/ports (PORTx):
	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = 0x00;

	servo.attach(SERVO_POWER_PIN);
	lcd.begin(16,2);

	// Starting Serial for troubleshooting.
	Serial.begin(9600);
	Serial.println(F("Initialised!")); // Store string in Flash memory.
	lcd.print(F("Initialised!"));
}

void loop() {
	currentMicros = micros();
	currentMillis = millis();

	if (!(PORTD & 0x10)) // If US trigger pin is low.
	{
		PORTD ^= 0x10; // Invert pin.
		ultrasonicCycleStart = currentMicros;
	}
	else if (currentMicros - ultrasonicCycleStart >= 10UL) // If US trigger pin is high and been that for x µs.
	{
		noInterrupts(); // Turn off interrupts for read timings.
		PORTD ^= 0x10;
		uint32_t distance = pulseIn(US_ECHO_PIN, HIGH);
		interrupts(); // Re-enable interrupts.
		distance = (distance * SPEED_OF_SOUND) >> 1; // Calculate the distance the sound travelled.
		Serial.println(distance);
	}
	if ((PORTC & 0x30) ^ 0x10) // If white IR sensor reads black, or black IR sensor read white.
	{
		Serial.println(F("TEST"));
	}
	
}

void directionCorrection() {

}

// Sets the colour of a RGB led given a hexadecimal 24-bit value (HTML colour).
void setLedColour(const uint32_t hexColour) {
	setLedColour(hexColour >> 16, hexColour >> 8, hexColour);
}
// Sets the colour of a RGB led given an 8-bit value for each RGB colour.
void setLedColour(const uint8_t red, const uint8_t green, const uint8_t blue) {
	analogWrite(LED_RED_PIN, red);
	analogWrite(LED_GREEN_PIN, green);
	analogWrite(LED_BLUE_PIN, blue);
}