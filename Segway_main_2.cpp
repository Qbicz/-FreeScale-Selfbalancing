/*TO DO:
-sprawdzic w jakich zakresach bedzie nam wychodzic sterowanie - na tej podstawie zrobic przygotowanie do funkcji map, bo narazie to bardziej constrain niz map


Do zrobienia: 
1) sprawdzic czy na PWM na pewno nie podajemy wiecej niz 1.0 (jeden if zalatwi sprawe)
2) umiescic zmienne globalne w pliku Segway_main.h
*/

#include "mbed.h"
#include "MPU6050.h"
#include "kalman.h"

#define Rad2Dree       57.295779513082320876798154814105

//for Proportional Part
#define K 10.0 // 1.0
#define ERROR_COUNT 10
//for Integral Part
float errors[ERROR_COUNT];
int curr_err_index;
#define Tp 0.0033
#define Ti 10000.0

//for Derivative Part
#define Td 5.0
int Freq = 3000; // w

// Piłat:
// zarejestrowac przebiegi z kąta
// do inzynierki dokumentowac kazdy przebieg
// czestotliwosc petli sterowania
// rysunek architektury i sekwencji czasowej - zeby mogl nam cos pomoc

// dostrojenie filtru Kalmana - jak zaszumiony pomiar
// jakie jest przesuniecie w (fazie) czasie przez Kalmana - surowe ax, ay, ... , gz i te same dane z kalmana
// Olek: trzeba rejetrowac wartosci podczas dzialania, potem wylaczyc sterowanie i przeslac dane

// jaki jest czas trwania petli glownej
// tylko PD dzis
// bardzo male D na poczatku

// Tutaj: programowe zabezpieczenie przed niskim napieciem zasilajacym - jedno wejscie ADC pomiar napiecia


MPU6050 mpu6050;
Timer t;
Serial pc(USBTX, USBRX); // tx, rx

// for Kalman
Timer ProgramTimer;
kalman filter;
//float R;
double angle;
double abs_angle;
unsigned long timer;
long loopStartTime;

long count1 = 0;
//long delt_t;

//for remain power measurement
//AnalogIn Baterry(PTB0);
//somewhere in the code put:
//if (Baterry.read() < LOW_VOLTAGE) {
//    STOSOWNY KOMUNIKAT
//    }


// for frequency measurement
Timer loopTimer;
//long startTime, stopTime;

// for motors logic
float speed;
bool direction;

//for motors
PwmOut LeftMotor(PTC9); // lewy czerwony kabel, prawy bialy
DigitalOut LeftLogic1(PTE22, 1);
DigitalOut LeftLogic2(PTE23, 1);
// podpiecie do PTD0 i PTD1 zmienia kolor wbudowanej diody!

PwmOut RightMotor(PTC8);
DigitalOut RightLogic1(PTB2, 1);
DigitalOut RightLogic2(PTB3, 1);

float map(float control, float minIn, float maxIn, float minOut, float maxOut)
{
    if(control < minIn)
        return  0;
    else if(control > maxIn)
        return  maxOut;
    else
        return control; // poki co
}

void Motors_Set_Speed(float speed, bool direction)
{
    // speed is always positive, use direction
    if(!direction) {
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
    float tmp;
    /* if(speed >= 1)
        tmp = 1;
    else if (speed < 0)
        tmp = 0;
    else
    tmp = speed;
    */
    if(speed >= 3)
    {
        tmp = 1;
    }
    else if (speed <= 0.01)
        tmp = 0;
    else
        tmp = 3.3*speed*0.1;
    

    //pc.printf("Right before sending to 1 motor: %f", tmp);
    LeftMotor  = (tmp);
    //pc.printf("Right before sending to 2 motor: %f", tmp);
    RightMotor = (tmp);
}

void Control_Motors(float angle, bool direction)
{
    static float control;
    //bool direction;
    //adding new angle to error array
    //errors[curr_err_index] = angle;
    //incrementing current index in array
    int temp_index;
    //pc.printf("Current index is: %d\r\n", curr_err_index);
    if(curr_err_index > ERROR_COUNT) {
        curr_err_index = 0;
        temp_index = ERROR_COUNT - 1;
        }
    else {
        temp_index = curr_err_index;
        curr_err_index++;   
        }
    errors[curr_err_index] = angle; 
    //calculating integral part (error part)    
    float er = 0;
    for(int i = 0 ; i<ERROR_COUNT; i++) {
        er+=errors[i];       
    }
    //pc.printf("Sum of the errors: %f\r\n", er);
    
    // WYLACZONA CZESC CALKUJACA
    control = K*(angle + 0*(Tp/Ti)*er + Td*(angle - errors[temp_index] )/Tp);  
    //control = map(control, 1, 8, 0, 1);
    //control = control*0.1;
    /*
    if(control>0) direction = 1;
    else        direction = 0;
    float abs_control  = (control >= 0) ? control : -1*control;
    */
    //pc.printf("Control = %f\n\r", control);
    
    Motors_Set_Speed(control, direction); // ta funkcja obcina z duzego zakresu do 0-1
}


int main()
{
    for(int i = 0 ; i<ERROR_COUNT; i++) {
    errors[i] = 0;       
    }
    curr_err_index = 0;
    pc.baud(9600);

    //Angle input from -15 to 15
    //controller.setInputLimits(0, 30); // + info about direction
    //Pwm output from 0.0 to 1.0
    //controller.setOutputLimits(0.0, 1.0);
    //controller.setMode(AUTO_MODE);
    //controller.setSetPoint(0.0);

    //Set up I2C
    i2c.frequency(400000);  // use fast (400 kHz) I2C

    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    pc.printf("I AM 0x%x\n\r", whoami);
    pc.printf("I SHOULD BE 0x68\n\r");

    if (whoami == 0x68) { // WHO_AM_I should always be 0x68
        pc.printf("MPU6050 is online...");
        wait(1);
        pc.printf("MPU6050 OK");

        mpu6050.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
        pc.printf("x-axis self test: acceleration trim within : %f% of factory value \n\r", SelfTest[0]);
        pc.printf("y-axis self test: acceleration trim within : %f% of factory value \n\r", SelfTest[1]);
        pc.printf("z-axis self test: acceleration trim within : %f% of factory value \n\r", SelfTest[2]);
        pc.printf("x-axis self test: gyration trim within : %f% of factory value \n\r", SelfTest[3]);
        pc.printf("y-axis self test: gyration trim within : %f% of factory value \n\r", SelfTest[4]);
        pc.printf("z-axis self test: gyration trim within : %f% of factory value \n\r", SelfTest[5]);

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

    loopTimer.start(); // czestotliwosc
    t.start();
    
    while(1)
    {   
        //startTime = loopTimer.read_us(); // START - 950us trwa petla (953-956us zazwyczaj) - mozemy zrobic co 1ms - czestotliwosc dzialania 1kHz
        /* Czesc synchroniczna */
        if(loopTimer.read_us() % Freq == 0)
        {
        
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
            }
     
            // Calcuate the resulting vector R from the 3 acc axes - POTRZEBNE DO ACOS(ax/R)
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
            //pc.printf("abs_angle = %f\n\r", abs_angle);
            
            //abs_angle = (abs_angle / 10);
            Control_Motors(abs_angle, direction);
    
            //pc.printf("speed = %f\n\r", speed);
            //Motors_Set_Speed(speed, direction);
            //wait(RATE);
                    
            // Serial print and/or display at 0.5 s rate independent of data rates
            delt_t = t.read_ms() - count1;
            //pc.printf("delt_t = %ld\n", delt_t);
            if (delt_t > 500) {
    
                pc.printf(" ax = %f", 1000*ax);
                pc.printf(" ay = %f", 1000*ay);
                pc.printf(" az = %f  mg\n\r", 1000*az);
    
                //pc.printf(" gx = %f", gx);
                //pc.printf(" gy = %f", gy);
                //pc.printf(" gz = %f  deg/s\n\r", gz);
                pc.printf("Kalman Angle = %f\n\n", angle);
    
                //pc.printf("LeftLogic:  %d %d \n\r", int(LeftLogic1), int(LeftLogic2));
                //pc.printf("RightLogic: %d %d \n\r", int(RightLogic1), int(RightLogic2));
                //pc.printf("Direction:  %d\n\r", direction);
                
                pc.printf("Control:    %.2f\n\r\n\r", speed);
                
                //
                //myled= !myled;
                count1 = t.read_ms();
            }
            
            //stopTime = loopTimer.read_us(); // STOP
            //pc.printf("Loop Time: %ld\n", stopTime-startTime);
            }
            /* Koniec czesci synchronicznej */
    }
}
