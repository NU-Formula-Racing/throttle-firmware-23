#include <Arduino.h>
#include "virtualTimer.h"
#include "throttle.h"

#define SERIAL_DEBUG

#if defined(ARDUINO_TEENSY40) || defined(ARDUINO_TEENSY41)
#include "teensy_can.h"
// The bus number is a template argument for Teensy: TeensyCAN<bus_num>
TeensyCAN<1> can_bus{};
#endif

#ifdef ARDUINO_ARCH_ESP32
#include "esp_can.h"
// The tx and rx pins are constructor arguments to ESPCan, which default to TX = 5, RX = 4
ESPCAN can_bus{};
#endif

// Initialize board
Throttle throttle;

// Structure for handling timers
VirtualTimerGroup read_timer;

// TX CAN Signal
CANSignal<uint8_t, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> cur_throttle_signal{};

// TX CAN Message (Might need to edit transmit period (currently 100))
CANTXMessage<1> tx_message{can_bus, 0x300, 1, 100, read_timer, cur_throttle_signal};

void ReadAcceleratorPress() {
	cur_throttle_signal = throttle.ReadAcceleratorPress();
};

void setup() {
	#ifdef SERIAL_DEBUG
	// Initialize serial output 
	Serial.begin(115200);
	#endif
	
	/*
	// This only works on ESP32, will crash on compile for Teensy
	// This makes us trigger reading wheel speed in an interrupt
	// JUST FOR REFERENCE (currently based off daqWheel code)
	attachInterrupt(wheel_board.wheelSpeedSensorPin, WheelSpeedISR, RISING);
	*/

	// Initialize can bus
	can_bus.Initialize(ICAN::BaudRate::kBaud1M);

	/*
	//Initialize our timer(s)
	read_timer.addTimer(100, ReadAccelerometerPress);
	read_timer.addTimer(100, updateThrottleLimit); // Global variable that constantly needs to be changed
	*/
}

void loop() {
	read_timer.Tick(millis());
}
