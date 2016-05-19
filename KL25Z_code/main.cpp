// 1 [OK] przygotowac funkcje do kontroli PI silnikow (PID, QEI)
// 2 [OK] przygotowac funkcje do nawiazania polaczenia i odczytu MPU
// 2.1 [OK] filtr komplementarny
// 3 przygotowac funkcje do stabilizacji wokol punktu stacjonarnego
// 4 [OK] umiescic wywolania w cyklicznym przerwaniu (Ticker)
// 5 [OK] funkcja do mierzenia napiecia
// 6 [OK] ocenic jakosc regulacji
// 7 [OK] kontrola nastaw regulatora przez BT

#include "mbed.h"
#include "motor.h"
#include "BufferedSerial.h"
//#include "MODSERIAL.h"
#include "MPU6050.h"
#include "DigitalPID.h"
//#include "TSISensor.h"

#define TALKATIVE 0
#define HUMAN_READABLE 0
#define OFFSETS 0

/* * * * * * *
 * Functions *
 * * * * * * */
float ComplementaryFilter(float ax, float az, float gy);
float StabilisationPD(float angle, float setPoint, float Kp, float Kd);
float StabilisationPID(float angle, float setPoint, float Kp, float Ti, float Td);
void BluetoothSetTunings();
// Interrupt Subroutines
void PlotRobotAngle_ISR(void);
void Stabilise_ISR(void);

/* * * * * * *
 *  Globals  *
 * * * * * * */
// Wired communication with PC
// BufferedSerial pc(USBTX, USBRX); // pta1, pta2 used

// Wired
BufferedSerial blue(PTE0, PTE1);          // TX, RX
// BufferedSerial blue(USBTX, USBRX);

//MODSERIAL blue(PTE0, PTE1);
PwmOut blueLed(LED_BLUE);

DigitalPID PID;

static MPU6050 mpu;
static float accAngle;
static float RobotAngle;
Timer ComplemTimer;
const float aComplem = 0.96;

// Time
Ticker StabilTicker;
//Ticker PrintTicker;
Timer t;
Timer jitterTimer;

Motor LeftMotor(PTD1, PTD3, PTC9, PTE22, PTE23); // EncoderA, B, PwmOut, LeftLogic1, LeftLogic2
Motor RightMotor(PTA16, PTA17, PTC8, PTB3, PTB2); // Odwrocona logika - zeby miec ten sam kierunek na obu silnikach (PTB2,3)
//static PID Stabilisator(StabilP, 0.0, StabilD, StabilTp);

AnalogIn BatteryInput(PTC0); // ADC0_SE14
TSISensor tsi;

// Odpowiedz skokowa
//Timer JumpTimer;

int main(void)
{
    mpu.Initialize_MPU();
    PID.setSetPoint(0.0);
    
    // Overwrite accel biases
    const int32_t new_accel_bias[3] = {63, 0, 0}; // with 90 X_BIAS it leans to the L298 side, with 40 it leans to the FRDM side
    mpu.mpu_push_accel_bias(new_accel_bias);
    
    blue.baud(38400);
    blue.printf("Hello, I'm Self Balancing JACK.\n");
    blueLed = 1;

    // Set ticker interrupts
    t.start();
    jitterTimer.start();
    ComplemTimer.start();
    StabilTicker.attach_us(&Stabilise_ISR, StabilRate);
    
    while(true)
    {
        
    }
}

void BluetoothSetTunings(char Tune)
{
    // ANDROID - http://appinventor.mit.edu/explore/
    blue.printf("SetTunings()");
    
    //if(!isdigit(Tune[i])) { blue.printf("3 last chars are not a number!") return; }
    //if(!isalpha(Tune[0])) { blue.printf("First char is not alpha!"); return; }
    
    if (Tune == 'P')
        StabilP += 0.01;
    else if (Tune == 'p')
        StabilP -= 0.01;
    else if(Tune == 'D')
        StabilD += 0.01;
    else if (Tune == 'd')
        StabilD -= 0.01;
    else blue.printf("Tuning char must be P, p, D or d");
    blue.printf("PID Tunings:\n  P = %f\n  D = %f\n", StabilP, StabilD);
}

void Stabilise_ISR(void)
{
    
    // DisableInterrupts()
    //__disable_irq();
    
    // Bluetooth section
    if(blue.readable())
    {
        blueLed = !blueLed;
        BluetoothSetTunings(blue.getc());
    }
    
    // Read IMU
    mpu.Fetch_Values_MPU();
    RobotAngle = ComplementaryFilter(ax, az, gy);
    static float ControlValue;
    
    
    
    if (RobotAngle < 0.05 && RobotAngle > -0.05)
        ControlValue = 0.0;
    else if (RobotAngle > 45 || RobotAngle < -45)
    {
        ControlValue = 0.0;
        //blue.printf("The angle is too big. STOP.\nPress any key to continue program...");
    }
    else
        // ControlValue = StabilisationPD(RobotAngle, 0, StabilP, StabilD); // + 0.2*tsi.readPercentage());
        ControlValue = PID.CalculatePID(-RobotAngle);
    
    blue.printf("%2.6f\n", jitterTimer.read());
    
    LeftMotor.Control(ControlValue); // + wynik z PID pozycji?
    RightMotor.Control(ControlValue);
    
    jitterTimer.reset();
    
    #if TALKATIVE
        #if HUMAN_READABLE
            blue.printf("Time = %3.2f, Angle = %2.5f, Control = %1.6f\n", t.read(), RobotAngle, ControlValue);
        #else
            blue.printf("%3.2f %2.5f %1.6f\n", t.read(), RobotAngle, ControlValue);
        #endif
    #endif
    
    // EnableInterrupts()
    //__enable_irq();
    
}


float ComplementaryFilter(float ax, float az, float gy)
{
    // dt = ComplemTimer.read()
    accAngle = (-180)*atan(ax/az)/PI;
    
    float filteredAngle = aComplem*(RobotAngle + gy * ComplemTimer.read()) + (1 - aComplem) * accAngle;
    ComplemTimer.reset();
    return filteredAngle;
}

/* Plot Time | Filtered Angle | Accelerometer Angle */
void PlotRobotAngle_ISR()
{
    mpu.Fetch_Values_MPU();
    
    RobotAngle = ComplementaryFilter(ax, az, gy); // gy in degrees per second

    blue.printf("%3.2f %2.5f %2.5f\n", t.read(), RobotAngle, -180*atan(ax/az)/PI);
    
}


void BatteryRead_ISR(void)
{
    float voltage = BatteryInput.read()*((9.92+2.17)/2.17)*3.3; // dzielnik napięcia
    pc.printf("Voltage is %f\n", voltage);
    if(voltage < 10.5)
        ;// Migaj dioda
    if(voltage < 9)
        ;// Shutdown
}


