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

enum class BMSState
{
    kShutdown = 0,
    kPrecharge = 1,
    kActive = 2,
    kCharging = 3,
    kFault = 4
};

enum class BMSCommand
{
    NoAction = 0,
    PrechargeAndCloseContactors = 1,
    Shutdown = 2
};

enum state
{
    OFF,
    N,
    DRIVE,
    FDRIVE
};

// CAN Signal for BMS
CANSignal<BMSState, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BMS_State{};

CANRXMessage<1> BMS_message{can_bus, 0x241, BMS_State};

CANSignal<BMSCommand, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> BMS_Command{};

CANTXMessage<1> BMS_command_message{can_bus, 0x242, 8, 100, read_timer, BMS_Command};

CANSignal<state, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> throttleStatus{};

CANTXMessage<1> throttleStatus_message{can_bus, 0x301, 8, 100, read_timer, throttleStatus};

CANSignal<bool, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> on_switch{};

CANRXMessage<1> on_message{can_bus, 0x100, on_switch};

bool onButton = false;

state currentState = OFF;

void RequestTorque()
{
    uint16_t throttle_percent = throttle.GetAcceleratorPress(
        inverter.GetMotorTemperature(), battery_amperage_signal, battery_voltage_signal, inverter.GetRPM());
    float rpm = inverter.GetRPM();
    // 332104 comes from power = 2*pi*rm*torque/60 and throttle_percent = torque/max_torque
    // equations where max_torque = 230 N*m
    uint16_t maxthrottlepercent = (332104/rpm);
    throttle_percent = min(throttle_percent, maxthrottlepercent);
    inverter.RequestTorque(throttle_percent);
    bool debug_mode = false;
    if (debug_mode)
    {
        Serial.print("cur_throttle_signal: ");
        Serial.println(throttle_percent);
        Serial.println("\n");
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


void changeState()
{
    float threshold = 100;
    float speed = inverter.GetRPM();
    switch (currentState) {
        case OFF:
            // if brake and button pressed, switch to N
            if (onButton) {
                currentState = N;
                throttleStatus = state::N;
            }
            break;
        case N:
            // listen to BMS status
            // if precharge is done, switch to drive
            if (BMS_State == BMSState::kActive && throttle.PotentiometersAgree()) {
                BMS_Command = BMSCommand::NoAction;
                currentState = DRIVE;
                throttleStatus = state::DRIVE;
            }
            // else
            else {
                // if BMS fault, switch to off
                if (BMS_State == BMSState::kFault) {
                    currentState = OFF;
                    throttleStatus = state::OFF;
                    onButton = false;
                }
                // else stay in N
            }
            break;
        case DRIVE:
            // if pedals off by more than 10%, switch to N
            if (throttle.PotentiometersAgree() == false) {
                currentState = N;
                throttleStatus = state::N;
            }
            // listen to BMS status
            // if BMS fault, switch to off
            if (BMS_State == BMSState::kFault) {
                currentState = OFF;
                throttleStatus = state::OFF;
                onButton = false;
            }
            // if switch is off and speed > threshold, switch to fault drive
            if (onButton == false) {
                if (speed >= threshold) {
                    currentState = FDRIVE;
                    throttleStatus = state::FDRIVE;
                } else {
                    currentState = OFF;
                    throttleStatus = state::OFF;
                    onButton = false;
                }
            }
            break;
        case FDRIVE:
            // if switch on, switch to drive
            if (onButton) {
                currentState = DRIVE;
                throttleStatus = state::DRIVE;
            // if switch off and speed < threshold, switch to off
            } else if (speed < threshold) {
                currentState = OFF;
                throttleStatus = state::OFF;
            }
            // listen to BMS
            // if BMS fault, switch to off
            if (BMS_State == BMSState::kFault) {
                currentState = OFF;
                throttleStatus = state::OFF;
                onButton = false;
            }
            break;
    }
}

void processState()
{
    switch (currentState) {
        case OFF:
            // do nothing
            break;
        case N:
            // send message to BMS (BMS command message)
            // PrechargeAndCloseContactors
            BMS_Command = BMSCommand::PrechargeAndCloseContactors;
            break;
        case DRIVE:
            // request torque based on pedal values
            RequestTorque;
            break;
        case FDRIVE:
            // request 0 torques
            inverter.RequestTorque(0);
            break;
    }
}

void test()
{
    // print current state
    switch (currentState) {
        case OFF:
            Serial.print("State: OFF\n");
            Serial.print("On: ");
            Serial.print(on_switch);
            Serial.print("\n");
            break;
        case N:
            Serial.print("State: N\n");
            Serial.print("On: ");
            Serial.print(on_switch);
            Serial.print("\n");
            break;
        case DRIVE:
            Serial.print("State: DRIVE\n");
            break;
        case FDRIVE:
            Serial.print("State: FDRIVE\n");
            break;
    }
    // print throttle percent
    uint16_t throttle_percent = throttle.GetAcceleratorPress(
        inverter.GetMotorTemperature(), battery_amperage_signal, battery_voltage_signal, inverter.GetRPM());
    Serial.print("Throttle percent: ");
    Serial.println(throttle_percent);
    Serial.print("\n");
    // print speed
    float speed = inverter.GetRPM();
    Serial.print("Speed: ");
    Serial.println(speed);
    Serial.print("\n");
}

void turnOn() {
    if (onButton) {
        onButton = false;
    } else {
        onButton = true;
    }
}

void setup()
{
#ifdef SERIAL_DEBUG
    // Initialize serial output
    Serial.begin(9600);  // Baud rate (Can transfer max of 9600 bits/second)
#endif

    // Initialize can bus
    can_bus.Initialize(ICAN::BaudRate::kBaud1M);

    // Initialize our timer(s)
    // read_timer.AddTimer(10, RequestTorque);
    read_timer.AddTimer(10, changeState);
    read_timer.AddTimer(10, processState);
    read_timer.AddTimer(1, []() {throttle.CalculateMovingAverage();});
    // read_timer.AddTimer(1000, []() {Serial.println(analogRead(25));});
    read_timer.AddTimer(500, test);


    bool debug_mode = false;
    if (debug_mode)
    {
        read_timer.AddTimer(100, printReceiveSignals);
    }

    // Request values from inverter
    inverter.RequestMotorTemperature(100);
    inverter.RequestRPM(100);

    // Set up interrupt
    attachInterrupt(40, turnOn, FALLING);
}

void loop()
{
    delay(0);
    read_timer.Tick(millis());
    can_bus.Tick();
}