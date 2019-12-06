#define pot 32
#define pwm 22
#define dir 21
#define ndir 4
//#define e1 33
//#define e2 27


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(pot, INPUT);
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
  pinMode(ndir,OUTPUT);
//  pinMode(e1, INPUT);
//  pinMode(e2, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int potRead = map(analogRead(pot), 0, 4095, -10, 10);
  Serial.println(potRead);
  if(potRead >= 0){
    digitalWrite(dir, HIGH);
    digitalWrite(ndir, LOW);
  }
  else{
    digitalWrite(dir, LOW);
    digitalWrite(ndir, HIGH);
  }
  int reading = abs(potRead);
//  Serial.println(reading);
  digitalWrite(pwm, HIGH);
  delay(reading);
  digitalWrite(pwm, LOW);
  delay(10-reading);
}
