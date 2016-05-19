/* File motor.cpp */
#include "motor.h"
#include "mbed.h"

//#include "BufferedSerial.h"
//extern BufferedSerial pc;
const int PwmPeriod = 10000; // 100Hz - do promotora

// Servo PI Controller - this constants are only for motor, they're not transmitted via bluetooth
static const float ServoP = 5.0;
static const float ServoI = 1.0;
static const float ServoTp = 0.1;
// constants local for this file!

/*********************************************
 * Motor Class Constructor
 *********************************************/
Motor::Motor(PinName EncoderA, PinName EncoderB, PinName BridgePwm, PinName Logic1, PinName Logic2)
: mEncoder(EncoderA, EncoderB, NC, 1920, QEI::X4_ENCODING),
  mBridgePwm(BridgePwm),
  mLogic1(Logic1, 1),
  mLogic2(Logic2, 1)
  //Controller(ServoP, ServoI, 0, ServoTp)
{
    // initialize PWM
    mBridgePwm.period_us(PwmPeriod);
    
    // Controller initialization
    //Controller.setInputLimits(-1120, 1120); // przemysl to jeszcze raz - koła mogą kręcić się w obie strony
    //Controller.setOutputLimits(-0.7, 0.7);
    //If there's a bias.
    // Controller.setBias(0.3);
    ControlTimer.start();
}

/* Destructor */
Motor::~Motor()
{ }

/*********************************************
 * void Control(float SetPoint)
 *
 * Calculate control value and drive motors
 *********************************************/
void Motor::Control(float TargetSpeed) //PID &Controller, QEI &Encoder)
{
    /*
    Controller.setSetPoint(TargetSpeed);
    static int WheelPosition = mEncoder.getPulses();
    // Calculate rotational speed
    static float Speed = (WheelPosition - previousWheelPosition); // / ControlTimer.read_us();
    //pc.printf("Speed = %2.6f\n", Speed);
    // Update PV
    Controller.setProcessValue(Speed);
    // Keep old value
    previousWheelPosition = WheelPosition;
    // Reset timer - HERE?
    ControlTimer.reset();
    // Get CO
    BridgeSetSpeed(Controller.compute()); // member function to write
    */
    BridgeSetSpeed(TargetSpeed);
}

/*********************************************
 * void BridgeSetSpeed(float Speed)
 *
 * Drive motors by setting PWM and H-bridge 'Enables'
 *********************************************/
void Motor::BridgeSetSpeed(float Speed)
{
    mLogic1 = (Speed > 0);
    mLogic2 = (Speed < 0);
    mBridgePwm = abs(Speed); // math?
    
    // jaki speed ustawiam?
    
}