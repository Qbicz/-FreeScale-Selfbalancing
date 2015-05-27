#include <SoftwareSerial.h>
const int ledpin = 13;
SoftwareSerial mySerial(10, 11); // RX, TX

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Serial.println("Goodnight moon!");
  pinMode(ledpin, OUTPUT);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial.println("Hello, world?");
}

void loop() // run over and over
{
  if (mySerial.available()) {
    char trash;
    int a  = mySerial.parseInt();
    trash =  mySerial.read();
    int b  = mySerial.parseInt();
    trash =  mySerial.read();
    int c  = mySerial.parseInt();
      //int a = mySerial.read();
      if(a == 0)
        digitalWrite(ledpin, HIGH);
      else
        digitalWrite(ledpin, LOW);
     mySerial.print("Entered a: ");
     mySerial.print(a);
     mySerial.print(", b: ");
     mySerial.print(b); 
     mySerial.print(", c: ");
     mySerial.println(c); 
  }
}

