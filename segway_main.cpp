/*TO DO:
-sprawdzic w jakich zakresach bedzie nam wychodzic sterowanie - na tej podstawie zrobic przygotowanie do funkcji map, bo narazie to bardziej constrain niz map
-sprawdzić czy jeździ do przodu / tylu na 1/0
-sprawdzic PINOUT
*/

#include "mbed.h"              
 
Serial pc(USBTX, USBRX);
/* To use Serial:
pc.printf();
pc.getc()
pc.puts();
*/
Timer t; 
/*
t.start();
t.stop();
t.reset();
t.read();
t.read_us();
*/


//for Proportional Part
#define K 1.0

//for Integral Part
int errors[100];
int curr_err_index;
#define Tp 0.1
#define Ti 10.0

//for Deriviative Part
#define en_2 errors[1]
#define Td 2.0

//for motors
PwmOut LeftMotor(PTE20);
DigitalOut LeftLogic1(PTB0);
DigitalOut LeftLogic2(PTB1);

PwmOut RightMotor(PTE21);
DigitalOut RightLogic1(PTB2);
DigitalOut RightLogic2(PTB3);

//general timer wait time
#define WAIT_MS_TIME 5


double map(double control, double min, double max)
{
    if(control < min)
        return  min;
    else if(control >max)
        return  max;
    else
        return control;
}


void Motors_Set_Speed(double speed)
{
    //extract the direction from the value of the speed
    if(speed < 0) {
        LeftLogic1  = 1;
        LeftLogic2  = 0;
        RightLogic1 = 1;
        RightLogic2 = 0;
        
        speed *= -1;       
    }
    else { 
        LeftLogic1  = 1;
        LeftLogic2  = 0;
        RightLogic1 = 1;
        RightLogic2 = 0;      
    }
    
    
    //PWMOuts uses percentage values
    LeftMotor  = (speed/255.0);
    RightMotor = (speed/255.0);
}

void Control_Motors(double angle)
{
    /*  We get angle - the last error - from MPU. Then we add it to the errors array and calculate PID.
    Controlling the motors is based on PID output. */
    
    
    //adding new angle to error array
    errors[curr_err_index] = angle;
    //incrementing current index in array
    curr_err_index++;
    if(curr_err_index > 100)
        curr_err_index = 0;
        
    //calculating integral part (error part)    
    double er = 0;
    for(int i = 0 ; i<100; i++) {
        er+=errors[i];       
    }
    
    
    double control, speed;
    control = K*(angle + (Tp/Ti)*er + Td*(angle - en_2 )/Tp);    
    speed = map(control, -255, 255);
    Motors_Set_Speed(speed);
}


int main() {
   
   //scheme
   //1. Get angle value from MPU
   //double angle = MPU_Get_Angle();
   double angle = 1;
   //2. Send the value to PID
   Control_Motors(angle);
   wait_ms(WAIT_MS_TIME);
}


/* NUMERICAL INTEGRAL
double control;
    double er = 0;
    for(int i = 0 ; i<100; i++) {
        er+=errors[i];       
    }*/