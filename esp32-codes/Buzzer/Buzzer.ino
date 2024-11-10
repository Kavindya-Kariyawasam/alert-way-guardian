int buzzer=2;

void setup() {

   pinMode(buzzer, OUTPUT);

}

 
void loop() {

     //turn on the buzzer at 1535 frequency for 500 milliseconds

     tone(buzzer,1535,500);

     //add another 500 milliseconds of silence

     delay(1000);

}