/*Test komunikacji via Bluetooth*/
//TIP: Nazwy pinow bierzemy z plytki
#include "mbed.h"              
 
Serial pc(PTA2, PTA1);


int main() {
    pc.printf("Type digit to  get its square !\r\n");
    char number;
        
    while(1) {
        number = pc.getc();
        pc.printf("Square of your number is: %c. Transmission is done via Bluetooth !", ( number-'0') * ( number-'0') );
        }
}