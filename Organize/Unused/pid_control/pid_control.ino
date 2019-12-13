double back_left_speed = 0, back_right_speed = 0;                 // PWM for two back motors
double left_rps = 0, right_rps = 0;                               // speed of two wheels
double diff_rps = 0;                                              // speed error of two wheels
double last_diff_rps = 0;                                         // speed error last loop
double all_diff_rps = 0;                                          // cumulative speed error


void setup() {
    Serial.begin(115200);
}

void loop() {
    pidControl();
}

void pidControl(){
    double P = 2, D = 0.4, I = 0.1;                                // pid parameters
    diff_rps = left_rps - right_rps;                               // calculate speed error   
    all_diff_rps += diff_rps;                                      // calculate cumulative speed error
    double d_error = D * (diff_rps - last_diff_rps);               // calculate d_error       
    double p_error = P * diff_rps;                                 // calculate p_error 
    double i_error = I * all_diff_rps;                             // calculate i_error
    back_right_speed += (d_error + p_error + i_error);             // calculate PWM for the right motor
    back_right_speed = max(min(back_right_speed,255.0),0.0);       // modify the PWM into [0,255]
    last_diff_rps = diff_rps;                                      // save current speed error
}
