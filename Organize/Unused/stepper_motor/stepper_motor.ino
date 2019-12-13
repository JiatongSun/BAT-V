#define    CLOCKWISE              1  
#define    COUNTERCLOCKWISE       -1 

#define    STEPPER_PIN_1          19
#define    STEPPER_PIN_2          18
#define    STEPPER_PIN_3          5
#define    STEPPER_PIN_4          17 

uint8_t step_number = 0;

//========================================================================
// step motor signal
int step_signal[8][4]={
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};
//========================================================================

void setup(){
    Serial.begin(115200); 
    pinSetup();
}

void loop(){
    stepMotor(CLOCKWISE);
}

//========================================================================
//set up pin
void pinSetup(){
    pinMode(STEPPER_PIN_1, OUTPUT);
    pinMode(STEPPER_PIN_2, OUTPUT);
    pinMode(STEPPER_PIN_3, OUTPUT);
    pinMode(STEPPER_PIN_4, OUTPUT);
}
//========================================================================


//========================================================================
// step motor control
// dir = ±1 represents clockwise/counterclockwise
// the resolution is 4096
// in four phase mode, the resolution is 2048
void stepMotor(int dir){         
    for(int a=0;a<2048;++a){     // rotate clockwise for one round  
        oneStep(step_number);
        step_number = (step_number + 2 * dir);
        step_number %= 8;
        delay(2);
    }
    for(int a=0;a<2048;++a){    // rotate counterclockwise for one round
        oneStep(step_number);
        step_number = (step_number - 2 * dir);
        step_number %= 8;
        delay(2);
    }
}
//========================================================================

//========================================================================
// one step in the step motor
void oneStep(int step_number){
    digitalWrite(STEPPER_PIN_1, step_signal[step_number][0]);
    digitalWrite(STEPPER_PIN_2, step_signal[step_number][1]);
    digitalWrite(STEPPER_PIN_3, step_signal[step_number][2]);
    digitalWrite(STEPPER_PIN_4, step_signal[step_number][3]);
}
//========================================================================
