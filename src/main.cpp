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

// TX CAN Signal for motor temp
CANSignal<float, 0, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> motor_temp_signal{};
// CANSignal<float, 16, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> coolant_temp_signal{};
// CANSignal<float, 32, 16, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(-40), false> ambient_temp_signal{};

// TX CAN Signal for battery amperage and voltage
CANSignal<float, 0, 8, CANTemplateConvertFloat(0.04), CANTemplateConvertFloat(0), false> battery_amperage_signal{};
CANSignal<float, 8, 16, CANTemplateConvertFloat(0.04), CANTemplateConvertFloat(0), false> battery_voltage_signal{};

// CANRXMessage
CANRXMessage<1> motor_message{can_bus, 0x420, motor_temp_signal};
CANRXMessage<2> amp_message{can_bus, 0x233, battery_amperage_signal, battery_voltage_signal};

void ReadAcceleratorPress() {
	cur_throttle_signal = throttle.ReadAcceleratorPress(motor_temp_signal, battery_amperage_signal, battery_voltage_signal);
	Serial.print("cur_throttle_signal: ");
	Serial.println(cur_throttle_signal);
	Serial.println("\n");
};

void printReceiveSignals() {
	can_bus.Tick();
	Serial.print("Motor Temp: ");
	Serial.println((float)motor_temp_signal);
	Serial.print("Battery Amperage: ");
	Serial.println((float)battery_amperage_signal);
	Serial.print("Battery Voltage: ");
	Serial.println((float)battery_voltage_signal);
	/*
	Serial.print("coolant_temp_signal");
	Serial.println((float)coolant_temp_signal);
	Serial.print("ambient_temp_signal");
	Serial.println((float)ambient_temp_signal);
	*/
	Serial.println("\n");
};

void setup() {
	#ifdef SERIAL_DEBUG
	// Initialize serial output 
	Serial.begin(9600); // Baud rate (Can transfer max of 9600 bits/second)
	#endif

	// Initialize can bus
	can_bus.Initialize(ICAN::BaudRate::kBaud1M);

	//Initialize our timer(s)
	read_timer.AddTimer(100, ReadAcceleratorPress);
	read_timer.AddTimer(100, printReceiveSignals);
}

void loop() {
	delay(0);
	read_timer.Tick(millis());
}
