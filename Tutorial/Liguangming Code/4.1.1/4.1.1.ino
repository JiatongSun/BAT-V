//Code Auther Yupeng Li
//Code is for Lab 4.1.1
const int External_LED = 0; //set P0 as the external LED pin
int Input_Pin = 25;//Set P25 as the input pin
int val = 0;//Integer used to store the read value

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(External_LED, OUTPUT);//Set the external LED as the output pin
  pinMode(Input_Pin, INPUT);//Set the Input_Pin as input
  Serial.begin(115200);//Begin serial communication at baud 115200

}

// the loop function runs over and over again forever
void loop() {
  val = digitalRead(Input_Pin);//Read the value from the button and store in val
  if (val ==1)//When the button is pressed
  {
  digitalWrite(External_LED, HIGH);   // turn the LED on (HIGH is the voltage level)                     // wait for a second

  }
  if (val == 0)//When the button is released
  {
  digitalWrite(External_LED, LOW);    // turn the LED off by making the voltage LOW                       // wait for a second
  }

}
