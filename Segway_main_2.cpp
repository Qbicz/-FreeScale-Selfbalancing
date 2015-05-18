/*TO DO:
-sprawdzic w jakich zakresach bedzie nam wychodzic sterowanie - na tej podstawie zrobic przygotowanie do funkcji map, bo narazie to bardziej constrain niz map
-sprawdzić czy jeździ do przodu / tylu na 1/0
-sprawdzic PINOUT

Do zrobienia: 
1) sprawdzic czy na PWM na pewno nie podajemy wiecej niz 1.0 (jeden if zalatwi sprawe)
2) Odnosnie PID z biblioteki:
Poczekac ten minimalny czas
*/

#include "PID.h"
#include "mbed.h"
#include "MPU6050.h"
#include "kalman.h"

// programowe zabezpieczenie przed niskim napieciem zasilajacym - jedno wejscie ADC pomiar napiecia

#define P       1.0
#define I       0.0
#define D       0.0
#define RATE    0.001
#define Rad2Dree       57.295779513082320876798154814105

MPU6050 mpu6050;
Timer t;
Serial pc(USBTX, USBRX); // tx, rx

// for Kalman
Timer ProgramTimer;
kalman filter;
float R;
double angle;
double abs_angle;
unsigned long timer;
long loopStartTime;

// for motors logic
float speed;
bool direction;

PID controller(P, I, D, RATE);

//for motors
PwmOut LeftMotor(PTC9); // lewy czerwony kabel, prawy bialy
DigitalOut LeftLogic1(PTE22, 1);
DigitalOut LeftLogic2(PTE23, 1);
// podpiecie do PTD0 i PTD1 zmienia kolor wbudowanej diody!


DigitalOut RightLogic1(PTB2, 1);
DigitalOut RightLogic2(PTB3, 1);
PwmOut RightMotor(PTE21);
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

void Motors_Set_Speed(double speed, bool direction)
{
    // speed is always positive, use direction
    if(direction) {
        // forward
        LeftLogic1  = 1;
        RightLogic1 = 1;
        LeftLogic2  = 0;
        RightLogic2 = 0;      
    }
    else { 
        // backward
        LeftLogic1  = 0;
        RightLogic1 = 0;
        LeftLogic2  = 1;    
        RightLogic2 = 1;      
    }
   
    //PWMOuts uses percentage values
    double tmp = speed;
    LeftMotor  = (tmp); // /255.0
    RightMotor = (tmp);
}

/*
void Control_Motors(double angle)
{
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
    pc.printf("Control = %f\n\rSpeed = %f\n\r\n", control, speed);
    Motors_Set_Speed(speed);
}*/

/* 
 Hardware setup:
 MPU6050 Breakout --------- FRDM KL25Z
 VCC ----------------------- P3V3
 SDA ----------------------- PTB1
 SCL ----------------------- PTB0
 GND ----------------------- GND

  Note: The MPU6050 is an I2C sensor and uses the Arduino Wire library.
 Because the sensor is not 5V tolerant, we are using a 3.3 V 8 MHz Pro Mini or a 3.3 V Teensy 3.1.
 We have disabled the internal pull-ups used by the Wire library in the Wire.h/twi.c utility file.
 We are also using the 400 kHz fast I2C mode by setting the TWI_FREQ  to 400000L /twi.h utility file.
 */

int main()
{
    pc.baud(9600);

    //Angle input from -15 to 15
    controller.setInputLimits(0, 30); // + info about direction
    //Pwm output from 0.0 to 1.0
    controller.setOutputLimits(0.0, 1.0);
    controller.setMode(AUTO_MODE);
    controller.setSetPoint(0.0);

    //Set up I2C
    i2c.frequency(400000);  // use fast (400 kHz) I2C
    t.start();

    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    pc.printf("I AM 0x%x\n\r", whoami);
    pc.printf("I SHOULD BE 0x68\n\r");

    if (whoami == 0x68) { // WHO_AM_I should always be 0x68
        pc.printf("MPU6050 is online...");
        wait(1);
        pc.printf("MPU6050 OK");

        mpu6050.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
        pc.printf("x-axis self test: acceleration trim within : ");
        pc.printf("%f", SelfTest[0]);
        pc.printf("% of factory value \n\r");
        pc.printf("y-axis self test: acceleration trim within : ");
        pc.printf("%f", SelfTest[1]);
        pc.printf("% of factory value \n\r");
        pc.printf("z-axis self test: acceleration trim within : ");
        pc.printf("%f", SelfTest[2]);
        pc.printf("% of factory value \n\r");
        pc.printf("x-axis self test: gyration trim within : ");
        pc.printf("%f", SelfTest[3]);
        pc.printf("% of factory value \n\r");
        pc.printf("y-axis self test: gyration trim within : ");
        pc.printf("%f", SelfTest[4]);
        pc.printf("% of factory value \n\r");
        pc.printf("z-axis self test: gyration trim within : ");
        pc.printf("%f", SelfTest[5]);
        pc.printf("% of factory value \n\r");
        wait(1);

        if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
            mpu6050.resetMPU6050(); // Reset registers to default in preparation for device calibration
            mpu6050.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
            mpu6050.initMPU6050();
            pc.printf("MPU6050 initialized for active data mode....\n\r"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
            wait(2);
        } else pc.printf("Device did not the pass self-test!\n\r");
    } else {
        pc.printf("Could not connect to MPU6050: \n\r");
        pc.printf("%#x \n",  whoami);

        while(1) ; // Loop forever if communication doesn't happen
    }

    // Parameters ( R_angle, Q_gyro, Q_angle ) 
    kalman_init(&filter, R_matrix, Q_Gyro_matrix, Q_Accel_matrix); 
    
    ProgramTimer.start();
    loopStartTime = ProgramTimer.read_us();
    timer = loopStartTime;

    while(1) {

        // ODCZYT WARTOSCI
        // If data ready bit set, all data registers have new data
        if(mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {
            // check if data ready interrupt
            mpu6050.readAccelData(accelCount);  // Read the x/y/z adc values
            mpu6050.getAres();

            // Now we'll calculate the accleration value into actual g's
            ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
            ay = (float)accelCount[1]*aRes - accelBias[1];
            az = (float)accelCount[2]*aRes - accelBias[2];

            mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values
            mpu6050.getGres();

            // Calculate the gyro value into actual degrees per second
            gx = (float)gyroCount[0]*gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
            gy = (float)gyroCount[1]*gRes; // - gyroBias[1];
            gz = (float)gyroCount[2]*gRes; // - gyroBias[2];

            tempCount = mpu6050.readTempData();  // Read the x/y/z adc values
            temperature = (tempCount) / 340. + 36.53; // Temperature in degrees Centigrade (from MPU docs)
        }
 
        // Calcuate the resulting vector R from the 3 acc axes - POTRZEBNE DO ACOS()
        // R = std::sqrt(static_cast<float>(std::pow(ax,2) + std::pow(ay,2) + std::pow(az,2)));
        
        kalman_predict(&filter, gy,  (ProgramTimer.read_us() - timer)); 
        kalman_update(&filter, atan(ax/az));
        
        angle = kalman_get_angle(&filter);
        timer = ProgramTimer.read_us(); 

        /* Sterowanie przez kat z kalmana */
        //pc.printf("Input angle = %f", angle);
        
        // PID & Motors
        angle *= 180.0f/PI;
        if(angle>0) direction = 1;
        else        direction = 0;
        abs_angle  = (angle >= 0) ? angle : -1*angle;
        pc.printf("abs_angle = %f\n\r", abs_angle);
        //controller.setProcessValue(abs_angle);
        //speed = controller.compute();
        speed = (abs_angle / 15);
        pc.printf("speed = %f\n\r", speed);
        Motors_Set_Speed(speed, direction);

        // Serial print and/or display at 0.5 s rate independent of data rates
        delt_t = t.read_ms() - count;
        if (delt_t > 500) {

            pc.printf(" ax = %f", 1000*ax);
            pc.printf(" ay = %f", 1000*ay);
            pc.printf(" az = %f  mg\n\r", 1000*az);

            pc.printf(" gx = %f", gx);
            pc.printf(" gy = %f", gy);
            pc.printf(" gz = %f  deg/s\n\r", gz);

            pc.printf("Kalman Angle = %f\n\n", angle);
            
            pc.printf("LeftLogic:  %d %d \n\r", int(LeftLogic1), int(LeftLogic2));
            pc.printf("RightLogic: %d %d \n\r", int(RightLogic1), int(RightLogic2));
            pc.printf("Direction:  %d\n\r", direction);
            pc.printf("Speed:      %.6f\n\r\n\r", speed);
            
            //myled= !myled;
            count = t.read_ms();
        }
    }

}

