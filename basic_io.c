/*
Test predkosci mikroklocka:

obliczenie wyrazenia:

answer =  ((t.read_us() % 1111 + 2788) * 4 ) + 7*3*4*893;
answer = answer / 17 * 3;

Daje odpowiedz:

Time elapsed for calculation: 2[us].


Regulator PID pozycyjny (od Oprzêdkiewicza z wyk³adu AA):

Un = k*En-1 + (k*tp/ti)*(sum(Er)) + k*td*(En-1 - En-2)/tp

Implementacja: ponizej

Odpowiedz:

Time elapsed for calculation: 314[us].

*/

#include "mbed.h"              
 
Serial pc(USBTX, USBRX);
Timer t; 


//for Proportional Part
#define K 1.0

//for Integral Part
int errors[100];
#define Tp 1.0
#define Ti 10.0

//for Deriviative Part
#define en_2 errors[1]
#define Td 2.0


//Add last error
double PID_step(int en_1)
{
    double control;
    double er = 0;
    for(int i = 0 ; i<100; i++) {
        er+=errors[i];       
    }  
    control = K*(en_1 + (Tp/Ti)*er + Td*(en_1 - en_2 )/Tp);    
    return control;    
}


int main() {
    pc.printf("Type 't' to get the time needed for PID calculation\r\n");
    pc.printf("Proportional Part: %f Integral Part: %f Derivative Part: %f\r\n", K, K*Tp/Td, K*Td/Tp );
    char from_serial;
    double answer = 0;
    
    //generate errors vector
    for (int i =0 ; i<100; i++) {
     errors[i] = (2*i*i - i )% 10;        
    }
    
    while(1) {
        from_serial = pc.getc();
        switch(from_serial) {
         case 't': 
            t.start();
            answer = PID_step(errors[0]);
            t.stop();
            pc.printf("Time elapsed for calculation: %d[us]\r\n", t.read_us());
            pc.printf("Control value is: %f\r\n", answer);
            t.reset();
         break;
         default:
         pc.puts("Type 't' to get the time needed for calculation\n"); 
        }
        //pc.putc(pc.getc());
    }
}