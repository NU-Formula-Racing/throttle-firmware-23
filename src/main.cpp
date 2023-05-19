#include <Arduino.h>

#include "inverter_driver.h"
#include "throttle.h"
#include "virtualTimer.h"

#define SERIAL_DEBUG

#include "esp_can.h"
// The tx and rx pins are constructor arguments to ESPCan, which default to TX = 5, RX = 4
ESPCAN can_bus{10, gpio_num_t::GPIO_NUM_22, gpio_num_t::GPIO_NUM_21};

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

CANSignal<uint8_t, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> accel_perc{};

CANSignal<uint8_t, 8, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> brake_perc{};

CANSignal<uint8_t, 16, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> maxavailabletorque{};

CANTXMessage<3> accel_brake_torque_message{can_bus, 0x300, 3, 10, read_timer, accel_perc, brake_perc, maxavailabletorque};

bool driveButton = false;

state currentState = OFF;

void RequestTorque()
{
    float rpm = abs(inverter.GetRPM());
    uint16_t throttle_percent = throttle.GetAcceleratorPress(
        inverter.GetMotorTemperature(), battery_amperage_signal, battery_voltage_signal, rpm);
    // 332149 comes from power = 2*pi*rpm*torque/60 and throttle_percent = torque/max_torque
    // equations where max_torque = 230 N*m, max_power = 80 kW
    uint16_t maxthrottlepercent = (332149/max(0.01f, rpm));
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
    float threshold = 150;
    float speed = inverter.GetRPM();
    switch (currentState) {
        case OFF:
            // If BMS is active, switch to N
            if (BMS_State == BMSState::kActive) {
                currentState = N;
                throttleStatus = state::N;
            }
            break;
        case N:
            // if drive button and brake pressed and potentiometers agree, switch to DRIVE
            if (throttle.brakePressed() && driveButton && throttle.PotentiometersAgree()) {
                BMS_Command = BMSCommand::NoAction;
                currentState = DRIVE;
                throttleStatus = state::DRIVE;
            }
            // if there is a fault, switch to fault
            if (BMS_State == BMSState::kFault || BMS_State == BMSState::kShutdown) {
                currentState = OFF;
                throttleStatus = state::OFF;
                driveButton = false;
            }
            break;
        case DRIVE:
            // if pedals off by more than 10%, switch to N
            if (!throttle.PotentiometersAgree() || (throttle.GetBrakePercentage() > 5 && throttle.GetAccPos() > 0) || throttle.to3V3orGND()) {
                currentState = N;
                throttleStatus = state::N;
            }
            // listen to BMS status
            // if BMS fault, switch to off
            if (BMS_State == BMSState::kFault) {
                currentState = OFF;
                throttleStatus = state::OFF;
                driveButton = false;
            }
            // if drive button is off and speed > threshold, switch to fault drive
            if (driveButton == false) {
                if (speed >= threshold) {
                    currentState = FDRIVE;
                    throttleStatus = state::FDRIVE;
                } else {
                    currentState = N;
                    throttleStatus = state::N;
                }
            }
            break;
        case FDRIVE:
            // if switch on, switch to drive
            if (driveButton) {
                currentState = DRIVE;
                throttleStatus = state::DRIVE;
            // if switch off and speed < threshold, switch to N
            } else if (speed <= threshold) {
                currentState = N;
                throttleStatus = state::N;
            }
            // listen to BMS
            // if BMS fault, switch to off
            if (BMS_State == BMSState::kFault) {
                currentState = OFF;
                throttleStatus = state::OFF;
                driveButton = false;
            }
            break;
    }
}

void processState()
{
    throttle.updateValues();
    accel_perc = throttle.GetAccPos();
    brake_perc = throttle.GetBrakePercentage();
    maxavailabletorque = throttle.GetMaxAvailableTorque();
    switch (currentState) {
        case OFF:
            // do nothing
            inverter.RequestTorque(0);
            break;
        case N:
            // send message to BMS (BMS command message)
            // PrechargeAndCloseContactors
            BMS_Command = BMSCommand::PrechargeAndCloseContactors;
            inverter.RequestTorque(0);
            break;
        case DRIVE:
            // request torque based on pedal values
            RequestTorque();
            break;
        case FDRIVE:
            // request 0 torque
            inverter.RequestTorque(0);
            break;
    }
}

void test()
{
    // print current state
    // Serial.println(currentState);
    Serial.print(analogRead(36));
    Serial.print(",");
    Serial.print(analogRead(35));
    Serial.print("\n");
    // Serial.println(analogRead(39));
    // Serial.println(throttle.GetLeftAccPos());
    // Serial.println(throttle.GetRightAccPos());
    // Serial.println(throttle.GetBrakePercentage());
    // Serial.println(throttle.PotentiometersAgree());
    // // Serial.println(throttle.GetAcceleratorPress(
    //     inverter.GetMotorTemperature(), battery_amperage_signal, battery_voltage_signal, inverter.GetRPM()));
    // // Serial.println(battery_amperage_signal);
    // Serial.println(battery_voltage_signal);
    // Serial.println(inverter.GetRPM());
    // float torque_perc = min(throttle.convertBattAmp(battery_amperage_signal, battery_voltage_signal, abs(inverter.GetRPM())), (float)1);
    // Serial.println(torque_perc);
    // Serial.println(driveButton);
    // Serial.println(throttle.PotentiometersAgree());
    // Serial.println(brake_perc);
    // Serial.println(accel_perc);
}

void turnOn() {
    if (driveButton) {
        driveButton = false;
    } else {
        driveButton = true;
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
    attachInterrupt(23, turnOn, FALLING);
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
}

void loop()
{
    delay(0);
    read_timer.Tick(millis());
    can_bus.Tick();
}