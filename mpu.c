#include "mbed.h"
#include "MPU6050.h"
/* ------ Test of MPU6050 ------ */

// jesli sie uprzec, to mozna budowac wszystko ze zrodla, ktore jest juz open source
// http://developer.mbed.org/users/mbed_official/code/mbed-src/

DigitalOut myled(LED1);
Serial pc(USBTX, USBRX);
MPU6050 mpu;
Timer timer;                // for timing test

int16_t ax, ay, az;
int16_t gx, gy, gz;         // both accelerometer and gyro has 16-bit ADCs

int main()
{
    pc.printf("MPU6050 test\n\n");
    pc.printf("MPU6050 initialize \n");

    mpu.initialize(); // go out of sleep mode
    pc.printf("MPU6050 testConnection \n");

    bool mpu6050TestResult = mpu.testConnection();
    if(mpu6050TestResult) {
        pc.printf("MPU6050 test passed \n");
    } else {
        pc.printf("MPU6050 test failed \n");
    }

    while(1) {
        wait(1);
        // measure time of reading all 6 axes
        timer.reset();
        timer.start();

        // read sensors
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        mpuRead_time=timer.read_us();
        timer.stop();

        // writing current accelerometer and gyro position
        pc.printf("Accelerometer:       %d; %d; %d\n",ax,ay,az);
        pc.printf("Gyroscope:           %d; %d; %d\n",gx,gy,gz);
        pc.printf("MPU reading took:    %dus\n", mpuRead_time);
    }
}
