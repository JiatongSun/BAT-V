// -----------------------------------------------------------------------------
// tool functions
// date: October 10, 2019
// author: Jiatong Sun
// -----------------------------------------------------------------------------
#ifndef TOOL_
#define TOOL_
// useful tool functions

//print tool
#define ENTER   m_usb_tx_string("\n")   
#define SPACE   m_usb_tx_string(" ")
#define TAB     m_usb_tx_string("\t")

//convert decimal number to binary number
int* dec2bin(int num);                  

#endif