#define    TRIGGER_PIN             23
#define    ECHO_PIN                22

long distance = 0;
long duration = 0;

void setup(){
    Serial.begin(115200); 
    pinMode(TRIGGER_PIN,OUTPUT);
    pinMode(ECHO_PIN,INPUT);
}  

void loop(){
    ultraDectect();
    show();
}

void ultraDectect(){
    digitalWrite(TRIGGER_PIN,LOW);
    delayMicroseconds(2);
    
    digitalWrite(TRIGGER_PIN,HIGH);
    delayMicroseconds(10);

    digitalWrite(TRIGGER_PIN,LOW);
    duration = pulseIn(ECHO_PIN,HIGH);
    distance = (duration/2)*0.0343;
}

void show(){
    if(millis() % 500 == 0){
        Serial.print("Distance: ");
        if(distance>=400 || distance<=2){
            Serial.println("Out of range!");
        } else {
            Serial.print(distance);
            Serial.println(" cm");
        }
    }
}
