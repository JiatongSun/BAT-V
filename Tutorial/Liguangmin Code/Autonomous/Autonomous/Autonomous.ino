#define diode 33 // output from the circuit
#define diode2 35 //
//#define pulse 22
int prevT = 0; // the previous time for calculating both the pulse width and the time for x and y distance
int currT = 0; // find the current time using micros()
int flagx = 1; // flag for the x pulse. Returns 0 if we looked at the x pulse and sets y flag = 1
int flagy = 0; // flag for the y pulse. Returns 0 if we looked at the y pulse. Does not reset the x flag
int start = 0; // remembers the rising edge of the synce pulse to calculate the x/y time distance
int counter = 0; // counts the sync pulses. If counter = 3, then we know the next pulse is an x pulse.
int timediff = 0; // calculates the time difference between two times, whether for pulse width or x/y distance

int prevT2 = 0; // the previous time for calculating both the pulse width and the time for x and y distance
int currT2 = 0; // find the current time using micros()
int flagx2 = 1; // flag for the x pulse. Returns 0 if we looked at the x pulse and sets y flag = 1
int flagy2 = 0; // flag for the y pulse. Returns 0 if we looked at the y pulse. Does not reset the x flag
int start2 = 0; // remembers the rising edge of the synce pulse to calculate the x/y time distance
int counter2 = 0; // counts the sync pulses. If counter = 3, then we know the next pulse is an x pulse.
int timediff2 = 0; // calculates the time difference between two times, whether for pulse width or x/y distance

int xt = -1; // This is to store the values of the time for the x pulse. Set to -1 as to show we havent changed it yet.
int xt2 = -1;
int yt = -1;
int yt2 = -1;

int xdiff = -10000;
int ydiff = -10000;

int vr = 1; // Vive region. Values can go from 1-4
int v2r = 1;

void IRAM_ATTR calcT(){ // interrupt function
  if(digitalRead(diode) == LOW){ // This checks for pulse width
    currT = micros(); // finds time in microseconds
    timediff = currT - prevT; // calculate time of the pulse width
    if(timediff < 60){ // if the width is less than 60, which is less than the pulse width of the sync pulse, but larger than the x/y pulses
      if(flagx){ // if the x flag is on
//        Serial.print("x-pulse: "); // print out the pulse value
//        Serial.println(timediff);
//        delayMicroseconds(1);
        flagx = 0; // set the flag to 0 so that we dont read the next pulse as x
        flagy = 1; // turn this flag on to read the next pulse as y
      }
      else if(flagy){ // if x flag isnt on but y flag is on
//        Serial.print("  y-pulse: "); // print out the pulse value
//        Serial.println(timediff);
//        delayMicroseconds(1);
//        flagx = 1;
        flagy = 0; // reset the flag back to 0. At this point, both the x and y flags are off.
      }
      counter = 0; // reset the counter. This is mainly for the else statement where we look at the counter value and the flags on line 49 (if nothing changes)
    }
    else{
//      Serial.println("im counting up");
      counter++; // increments the counter
//      Serial.println(counter);
      if(counter == 3){ // if I have counted 3 sync pulses, then I know the next pulse has to be an x pulse.
        flagx = 1; // because we know the next one is an x pulse, we can safely set the x flag on and prepare for the next pulse.
        counter = 1; // reset the counter to 1. This time we did not set it to 0 because we need to calculate the x time distance, which requires the counter to be 1 (as seen in the else statement below)
//        Serial.println(counter);
      }
    }
    start = prevT; // sets the start time to the rising edge of the pulse. This helps find the x/y time distance
    prevT = currT; // resets the previous time to the current time to be used in finding the pulse width
  }
  else{ // Here we are looking at the rising edge.
//    Serial.println("im in else");
    currT = micros(); // finds the time in microseconds
    timediff = currT - start; //calculate the time difference
    if(counter == 1 && (flagx || flagy)){ // checks whether the counter is 1 and if either flag is on. The counter is always on before the x and y pulses, as well as after the y pulse; however, we prevent the issue of counting the sync pulse after the y pulse by checking the flags, which are both 0 after the y pulse.
      if(flagx){ // checks the x flag. If x flag is on here, then we are on the x pulse. Else, on the y pulse.
//        Serial.print("    x time:"); // print the x time distance
//        Serial.println(timediff);
//        delayMicroseconds(1);
        xt = timediff;
        if(xt2 != -1){
          xdiff = xt2 - xt;
          xt=-1;
          xt2 = -1;
        }
      }
      else{ // on y pulse
//        Serial.print("      y time:"); // print the y time distance
//        Serial.println(timediff);
//        delayMicroseconds(1);
        yt = timediff;
        if(yt2 != -1){
          ydiff = yt2 - yt;
          yt = -1;
          yt2 = -1;
        }
      }
    }
    prevT = currT; // set the previous time to current time (on rising edge) for pulse width.
  }
}
void IRAM_ATTR calcT2(){ // interrupt function
  if(digitalRead(diode2) == LOW){ // This checks for pulse width
    currT2 = micros(); // finds time in microseconds
    timediff2 = currT2 - prevT2; // calculate time of the pulse width
    if(timediff2 < 60){ // if the width is less than 60, which is less than the pulse width of the sync pulse, but larger than the x/y pulses
      if(flagx2){ // if the x flag is on
//        Serial.print("x2-pulse: "); // print out the pulse value
//        Serial.println(timediff2);
//        delayMicroseconds(1);
        flagx2= 0; // set the flag to 0 so that we dont read the next pulse as x
        flagy2 = 1; // turn this flag on to read the next pulse as y
      }
      else if(flagy2){ // if x flag isnt on but y flag is on
//        Serial.print("  y2-pulse: "); // print out the pulse value
//        Serial.println(timediff2);
//        delayMicroseconds(1);
//        flagx2 = 1;
        flagy2 = 0; // reset the flag back to 0. At this point, both the x and y flags are off.
      }
      counter2 = 0; // reset the counter. This is mainly for the else statement where we look at the counter value and the flags on line 49 (if nothing changes)
    }
    else{
//      Serial.println("im counting up");
      counter2++; // increments the counter
//      Serial.println(counter2);
      if(counter2 == 3){ // if I have counted 3 sync pulses, then I know the next pulse has to be an x pulse.
        flagx2 = 1; // because we know the next one is an x pulse, we can safely set the x flag on and prepare for the next pulse.
        counter2 = 1; // reset the counter to 1. This time we did not set it to 0 because we need to calculate the x time distance, which requires the counter to be 1 (as seen in the else statement below)
//        Serial.println(counter);
      }
    }
    start2 = prevT2; // sets the start time to the rising edge of the pulse. This helps find the x/y time distance
    prevT2 = currT2; // resets the previous time to the current time to be used in finding the pulse width
  }
  else{ // Here we are looking at the rising edge.
//    Serial.println("im in else");
    currT2 = micros(); // finds the time in microseconds
    timediff2 = currT2 - start2; //calculate the time difference
    if(counter2 == 1 && (flagx2 || flagy2)){ // checks whether the counter is 1 and if either flag is on. The counter is always on before the x and y pulses, as well as after the y pulse; however, we prevent the issue of counting the sync pulse after the y pulse by checking the flags, which are both 0 after the y pulse.
      if(flagx2){ // checks the x flag. If x flag is on here, then we are on the x pulse. Else, on the y pulse.
//        Serial.print("    x2 time:"); // print the x time distance
//        Serial.println(timediff2);
//        delayMicroseconds(1);
        xt2 = timediff2;
        if(xt != -1){
          xdiff = xt2 - xt;
          xt=-1;
          xt2 = -1;
        }
      }
      else{ // on y pulse
//        Serial.print("      y2 time:"); // print the y time distance
//        Serial.println(timediff2);
//        delayMicroseconds(1);
        yt2 = timediff2;
        if(yt != -1){
          ydiff = yt2 - yt;
          yt = -1;
          yt2 = -1;
        }
      }
    }
    prevT2 = currT2; // set the previous time to current time (on rising edge) for pulse width.
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // initialize serial
  pinMode(diode,INPUT);
  pinMode(diode2,INPUT);
//  pinMode(pulse, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(diode), calcT, CHANGE); // make the interrupt
  attachInterrupt(digitalPinToInterrupt(diode2),calcT2, CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(xdiff != -10000 && ydiff != -10000){ // maybe turn motors on to 100%. If we did if statements instead, assuming the time calc takes a while, it may already do a pwm for us. Need to test
    if(vr == 1 && v2r == 1){  // here we assume that v2r is in front
      while(abs(xdiff) >= 5){ // possible to change this to an if statement if we need to break into smaller chunk
        if(xdiff >= 0){
          //set Servo to the left
        }
        else{
          //set Servo to the right
        }
        // turn on motors
        // maybe take care not to hit stuff like SAD
      }
    }
    if(vr == 1 && v2r == 4){ // could not care about this
      while(abs(xdiff) >= 5){
        //turn servo left and turn on motors
      }
    }
    if(vr == 4 && v2r == 4){
      //plan A: dont care about this and just keep turning
      //plan B: turn to <-1,0> or <1,0> depending if I am left or right of objective (which is ramp rn)
      while(abs(ydiff) >= 5){
        if(leftOfObjective){
          //turn servo left and motors on
        }
        else{
          //turn servo right and motor on
        }
      }
    }
    if(vr == 4 && v2r == 1){
      while(abs(xdiff) >= 5){
        //turn servo right and motor on
      }
    }
    if(vr == 1 && v2r == 3){
      while(abs(ydiff) >= 5){
        //turn servo right
      }
    }
    if(vr == 3 && v2r == 3){
      //keep moving until you reach ~4 inch from button. 
      if(leftOfObjective){
        //Turn left and hit it. Hopefully
      }
      else{
        //Turn right and hit it. Hopefully T^T.
      }
    }
    if(vr == 2 && v2r == 2){
      while(abs(xdiff) >= 5){ // possible to change this to an if statement if we need to break into smaller chunk
        if(xdiff >= 0){
          //set Servo to the left
        }
        else{
          //set Servo to the right
        }
        // turn on motors
        // maybe take care not to hit stuff like SAD
      }
    }
    if(vr == 2 && v2r == 4){ // could also not care about this
      while(abs(xdiff) >= 5){
        //turn servo right and turn on motors
      }
    }
    if(vr == 4 && v2r == 2){
      while(abs(xdiff) >= 5){
        //turn servo left and motor on
      }
    }
    if(vr == 2 && v2r == 3){
      while(abs(ydiff) >= 5){
        //turn servo left
      }
    }
  }

}
