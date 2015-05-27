const int ledpin = 13;

void setup() {
  pinMode(ledpin, OUTPUT);
  Serial.begin(9600);
  Serial.print("I'm connected via Bluetooth");
}

void loop() {
int data = Serial.read();
Serial.println(data);
delay(500);
}

