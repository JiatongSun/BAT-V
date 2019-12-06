/*
Lab 4.1.2
Author Yupeng Li
*/
const int External_LED = 0;//Set P0 as the external LED output
int Input_Pin = 25;//set P25 as the input pin
int val = 0;//set a value used to store the analog read

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(External_LED, OUTPUT);//set the  external led port as output
  Serial.begin(115200);//begin the serial communication at 115200 bauds

}

// the loop function runs over and over again forever
void loop() {
  val = map(analogRead(Input_Pin),0,4095,100,10000);//map the voltage read value to 100-10000ms
  Serial.println(val);//print out the value read from the analog pin

  digitalWrite(External_LED, HIGH);   // turn the LED on (HIGH is the voltage level)                     // wait for a second
  delay(val/2);//wait for half of period
  digitalWrite(External_LED, LOW);    // turn the LED off by making the voltage LOW                       // wait for a second
  delay(val/2);//wait for half of period

}
