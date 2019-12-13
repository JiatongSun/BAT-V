// -----------------------------------------------------------------------------
// tool function
// date: October 10, 2019
// author: Jiatong Sun
// -----------------------------------------------------------------------------
#include "tool.h"  // includes tool functions
#include "t_usb.h"

//convert decimal numbers to binary numbers
int* dec2bin(int num){   
    // m_usb_tx_uint(num); ENTER;
    static int bin_array[4] = {0};
    if(num/8) {
        bin_array[3] = 1;
        num -= 8;
    } else bin_array[3] = 0;
    if(num/4){
        bin_array[2] = 1;
        num -= 4;
    } else bin_array[2] = 0;
    if(num/2){
        bin_array[1] = 1;
        num -= 2;
    } else bin_array[1] = 0;
    if(num) bin_array[0] = 1;
    else bin_array[0] = 0;
    
    return bin_array;
}

//software filter
int avgFilter(int data1, int data2) {  //average software filter
    return (data1 + data2)/2;
}

int movingAvgFilter(int new_value, int last_value, int weight) {  //moving average filter
    int update = (weight * last_value + new_value)/(weight + 1);
    return update;
}

