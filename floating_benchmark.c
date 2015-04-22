#include "mbed.h"
/* 	This program determines the time it takes to perform floating point
    and integer operations.
    To determine the time it takes, a Timer is used to measure the time
    it takes to complete a large amount of iterations. The time for a single
    operation can then be determined.

    To increase accuracy of the results, an empty for loop is timed to determine
    the loop overhead and the overhead is subtracted from the time it takes to
    complete the operation loop.

	PID with 100 element sum, double precision -> 257 [us]
	PID with 100 element sum, uint32_t -> 45 [us]
	
	http://en.wikipedia.org/wiki/Single-precision_floating-point_format 6e-8 precision?
	
    */
#define ITERATIONS 100000    // Number of calculations.
#define CLOCK 48              // Clock freqency in MHz (only for benchmarking!)
Timer timer;                       // Timer..
DigitalOut myled(LED_GREEN);

// ---------- PID Defines ----------
#define ERR_NUM 10

//for Proportional Part
#define K 1.2

//for Integral Part
volatile double errors[ERR_NUM];
#define Tp 1.3
#define Ti 10.4

//for Derivative Part
#define en_2 errors[1]
#define Td 2.1

//Add last error
double PID_step(double en_1)
{
    double control;
    double er = 0;
    for(int i = 0 ; i<ERR_NUM; i++) {
        er+=errors[i];       
    }
    //control = en_1/5.1f;
    control = K*(en_1 + (Tp/Ti)*er + Td*(en_1 - en_2 )/Tp);
    return control;
}
// ----------- end PID ------------

Serial pc(USBTX, USBRX);
float number_of_cycles, single_operation_time;
//volatile float a, b,c;            // Float operands and result. Must be volatile!
//volatile int a, b,c;              // Int operands and result. Must be volatile!
//volatile uint32_t a,b,c;
volatile double a,b,c;

int main() {

    //generate errors vector
    for (int i=0 ; i<ERR_NUM; i++)
    {
         errors[i] = (2*i*i - i )% 10 + 0.14f*i + 0.085f;
         //errors[i] = 0.2f+0.001f*i*i;   
         pc.printf("errors[%d] = %f\n", i, errors[i]);     
    }
    
    unsigned int i, for_time, total_time, operation_time;
    a=2.3;
    b=5.33;
    
    while(true)
    {
        myled != myled;
        timer.reset();      // Reset timer
        timer.start();      // Start timer
        pc.printf("Operations in progress.. May take some time.\n\n");
        /* Determine loop overhead */
        for (i=0; i<ITERATIONS; i++);
        for_time=timer.read_us();
        timer.stop();
    
        /* Determine the total loop time */
        timer.reset();
        timer.start();
        
        /* The operation takes place in the body of
        this for loop. */
        
        for (i=0; i<ITERATIONS; i++)
        {
            //c = a+b;
            c = PID_step(errors[0]);
            //c=sin(a);
            //c=sqrt(a);
    
        }
        total_time=timer.read_us();
    
        operation_time = total_time-for_time;   // Calculate the time it took for the number of operations
    
        single_operation_time=float(operation_time)/float(ITERATIONS);
        number_of_cycles = single_operation_time*CLOCK;
    
        pc.printf("for overhead: \t\t%dus \n", for_time);
        pc.printf("total time: \t\t%dus \n\n", total_time);
        pc.printf("%d calculations took:\t%dus \n", ITERATIONS, operation_time);
        pc.printf("single operation took: \t\t%fus\n", single_operation_time);
        pc.printf("single operation took: \t\t%.3f cycles\n", number_of_cycles); 
    }
}
