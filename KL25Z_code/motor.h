#ifndef MOTOR_H
#define MOTOR_H

#include "QEI.h"
//#include "PID.h"
#include "mbed.h"

class Motor
{
    public:
        Motor(PinName EncoderA, PinName EncoderB, PinName BridgePwm, PinName Logic1, PinName Logic2);
        ~Motor();
        void Control(float SetPoint); // wewnatrz zmienic HBridgePwm
        void BridgeSetSpeed(float Speed); // mostek h - kierunki i pwm
    //private:
        // Motor + Encoder + H bridge
        QEI mEncoder; //(PinName channelA, PinName channelB, PinName index, int pulsesPerRev, QEI::Encoding encoding); // przeliczyć 1920 z promotorem
        PwmOut mBridgePwm; //(PinName pin);
        DigitalOut mLogic1; //(PinName pin, int value);
        DigitalOut mLogic2; //(PinName pin, int value);

        //PID Controller; //(float ServoP, float ServoI, float ServoD, float ServoTp);
        int previousWheelPosition;
        Timer ControlTimer;
    

}; // end class Motor

// potem mozna nadbudowac klase ktora bedzie skrecac uzywajac interfejsow motorów

/* Stare deklaracje:
// Right Motor + Encoder
        QEI LeftEncoder(PTD1, PTD3, NC, 1920, QEI::X4_ENCODING);
        PwmOut LeftMotor(PTC9);
        DigitalOut LeftLogic1(PTE22, 1);
        DigitalOut LeftLogic2(PTE23, 1);
        // Left Motor + Encoder
        QEI RightEncoder (PTA16, PTA17, NC, 1920, QEI::X4_ENCODING);
        PwmOut RightMotor(PTC8);
        DigitalOut RightLogic1(PTB2, 1);
        DigitalOut RightLogic2(PTB3, 1);
// */

#endif
