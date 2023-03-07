#include <Arduino.h>

#include "inverter_driver.h"
#include "throttle.h"
#include "virtualTimer.h"

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

// Structure for handling timers
VirtualTimerGroup read_timer;

// Initialize board
Throttle throttle{};    

// Instantiate inverter
Inverter inverter(can_bus);

// TX CAN Signal for battery amperage and voltage
CANSignal<float, 48, 16, CANTemplateConvertFloat(0.01), CANTemplateConvertFloat(0), false> battery_amperage_signal{};
CANSignal<float, 24, 16, CANTemplateConvertFloat(0.01), CANTemplateConvertFloat(0), false> battery_voltage_signal{};

// CANRXMessage
CANRXMessage<2> amp_message{can_bus, 0x240, battery_amperage_signal, battery_voltage_signal};

void ReadAcceleratorPress()
{
    uint16_t throttle_percent = throttle.GetAcceleratorPress(
        inverter.GetMotorTemperature(), battery_amperage_signal, battery_voltage_signal, inverter.GetRPM());
    inverter.RequestTorque(throttle_percent);

    bool debug_mode = true;
    if (debug_mode)
    {
        /*
        Serial.print("cur_throttle_signal: ");
        Serial.println(throttle_percent);
        Serial.println("\n");
        */
    }
};

void printReceiveSignals()
{
    can_bus.Tick();
    Serial.print("Motor Temp: ");
    Serial.println(inverter.GetMotorTemperature());
    Serial.print("Battery Amperage: ");
    Serial.println((float)battery_amperage_signal);
    Serial.print("Battery Voltage: ");
    Serial.println((float)battery_voltage_signal);
    Serial.println("\n");
};


void setup()
{
#ifdef SERIAL_DEBUG
    // Initialize serial output
    Serial.begin(9600);  // Baud rate (Can transfer max of 9600 bits/second)
#endif

    // Initialize can bus
    can_bus.Initialize(ICAN::BaudRate::kBaud1M);

    // Initialize our timer(s)
    read_timer.AddTimer(10, ReadAcceleratorPress);
    read_timer.AddTimer(1, []() {throttle.CalculateMovingAverage();});
    read_timer.AddTimer(1000, []() {Serial.println(analogRead(25));});

    bool debug_mode = false;
    if (debug_mode)
    {
        read_timer.AddTimer(100, printReceiveSignals);
    }

    // Request values from inverter
    inverter.RequestMotorTemperature(100);
    inverter.RequestRPM(100);
}

void loop()
{
    delay(0);
    read_timer.Tick(millis());
}