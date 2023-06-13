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
CANSignal<float, 0, 12, CANTemplateConvertFloat(0.1), CANTemplateConvertFloat(0), false> battery_amperage_signal{};
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
CANSignal<bool, 8, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> releaseAccelFault{};
CANSignal<bool, 9, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> potentiometerFault{};
CANSignal<bool, 10, 1, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> gnd3v3Fault{};

CANTXMessage<4> throttleStatus_message{
    can_bus, 0x301, 2, 100, read_timer, throttleStatus, releaseAccelFault, potentiometerFault, gnd3v3Fault};

CANSignal<bool, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> on_switch{};

CANRXMessage<1> on_message{can_bus, 0x100, on_switch};

CANSignal<uint8_t, 0, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> accel_perc{};

CANSignal<uint8_t, 8, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> brake_perc{};

CANSignal<uint8_t, 16, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> maxavailabletorqueperc{};

CANSignal<uint8_t, 24, 8, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0), false> requestedtorqueperc{};

CANTXMessage<4> accel_brake_torque_message{
    can_bus, 0x300, 4, 10, read_timer, accel_perc, brake_perc, maxavailabletorqueperc, requestedtorqueperc};

CANSignal<uint16_t, 0, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0)> left_ADC{};
CANSignal<uint16_t, 16, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0)> right_ADC{};
CANSignal<uint16_t, 32, 16, CANTemplateConvertFloat(1), CANTemplateConvertFloat(0)> brake_ADC{};
CANTXMessage<3> adc_value_message{can_bus, 0x302, 6, 10, read_timer, left_ADC, right_ADC, brake_ADC};

bool driveButton = false;

state currentState = OFF;

uint16_t RequestTorque()
{
    bool debug_mode = false;
    if (debug_mode)
    {
        Serial.print("batt amp signal: ");
        Serial.println(battery_amperage_signal.value_ref());
        Serial.println("\n");
    }
    float rpm = abs(inverter.GetRPM());
    uint16_t throttle_percent = throttle.GetThrottlePercent(
        inverter.GetMotorTemperature(), battery_amperage_signal, battery_voltage_signal, rpm);
    // 332149 comes from power = 2*pi*rpm*torque/60 and throttle_percent = torque/max_torque
    // equations where max_torque = 230 N*m, max_power = 80 kW
    uint16_t maxthrottlepercent = (332149 / max(0.01f, rpm));
    throttle_percent = min(throttle_percent, maxthrottlepercent);
    inverter.RequestTorque(throttle_percent);
    return throttle_percent;
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
    float threshold = 200;
    float speed = inverter.GetRPM();
    switch (currentState)
    {
        case OFF:
            // If BMS is active, switch to N
            if (BMS_State == BMSState::kActive)
            {
                currentState = N;
                throttleStatus = state::N;
            }
            break;
        case N:
            // if drive button and brake pressed and potentiometers agree, switch to DRIVE
            if (throttle.brakePressed() && driveButton && throttle.PotentiometersAgree())
            {
                BMS_Command = BMSCommand::NoAction;
                currentState = DRIVE;
                throttleStatus = state::DRIVE;
            }
            // if there is a fault, switch to fault
            if (BMS_State == BMSState::kFault || BMS_State == BMSState::kShutdown)
            {
                currentState = OFF;
                throttleStatus = state::OFF;
                driveButton = false;
            }
            break;
        case DRIVE:
            // listen to BMS status
            // if BMS fault, switch to off
            if (BMS_State != BMSState::kActive)
            {
                currentState = OFF;
                throttleStatus = state::OFF;
                driveButton = false;
            }
            // if drive button is off and speed > threshold, switch to fault drive
            if (driveButton == false)
            {
                if (speed >= threshold)
                {
                    currentState = FDRIVE;
                    throttleStatus = state::FDRIVE;
                }
                else
                {
                    currentState = N;
                    throttleStatus = state::N;
                }
            }
            break;
        case FDRIVE:
            // if switch on, switch to drive
            if (driveButton)
            {
                currentState = DRIVE;
                throttleStatus = state::DRIVE;
                // if switch off and speed < threshold, switch to N
            }
            else if (speed <= threshold)
            {
                currentState = N;
                throttleStatus = state::N;
            }
            // listen to BMS
            // if BMS fault, switch to off
            if (BMS_State == BMSState::kFault)
            {
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
    maxavailabletorqueperc = 0;
    requestedtorqueperc = 0;

    // handle faults
    throttle.UpdateFaults();
    // releaseAccelFault = throttle.release_accel_fault;
    potentiometerFault = throttle.potentiometer_fault;
    gnd3v3Fault = throttle.gnd_3v3_fault;
    switch (currentState)
    {
        case OFF:
            // do nothing
            BMS_Command = BMSCommand::PrechargeAndCloseContactors;
            inverter.RequestTorque(0);
            break;
        case N:
            // send message to BMS (BMS command message)
            // PrechargeAndCloseContactors
            BMS_Command = BMSCommand::NoAction;
            inverter.RequestTorque(0);
            break;
        case DRIVE:
            // request torque based on pedal values
            requestedtorqueperc = RequestTorque();
            maxavailabletorqueperc = throttle.GetMaxAvailableTorquePercent();
            break;
        case FDRIVE:
            // request 0 torque
            inverter.RequestTorque(0);
            break;
    }
}

void test()
{
    // Serial.printf("currentState:%d \n", currentState);
    // Serial.printf("PotentiometersAgree:%d \n", throttle.PotentiometersAgree());
    // Serial.printf("to3V3orGND:%d \n", throttle.to3V3orGND());
    // Serial.printf("driveButton:%d \n", driveButton);
    // Serial.printf("brake_perc:%d \n", static_cast<uint8_t>(brake_perc));
    // Serial.printf("accel_perc:%d \n", static_cast<uint8_t>(accel_perc));
    // Serial.printf(
    //     "torque request:%d \n",
    //     throttle.GetThrottlePercent(
    //         inverter.GetMotorTemperature(), battery_amperage_signal, battery_voltage_signal, inverter.GetRPM()));
    // Serial.printf("left sensor: %i \n", analogRead(34));
    // Serial.printf("right sensor: %i \n", analogRead(35));
    // Serial.printf("brake sensor: %i \n", throttle.brakeaverage);
    left_ADC = analogRead(34);
    right_ADC = analogRead(35);
    brake_ADC = analogRead(39);
}

void turnOn()
{
    if (driveButton)
    {
        driveButton = false;
    }
    else
    {
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
    read_timer.AddTimer(1, []() { throttle.CalculateMovingAverage(); });

    bool debug_mode = true;
    if (debug_mode)
    {
        // read_timer.AddTimer(100, printReceiveSignals);
        read_timer.AddTimer(1000, test);
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