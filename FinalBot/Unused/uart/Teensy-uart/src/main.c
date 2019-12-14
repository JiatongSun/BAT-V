#include "teensy_general.h"
#include <avr/pgmspace.h>
#include "t_usb.h"
#include "uart.h"
#include "math.h"

int main(void) {
  uint8_t data = 0;						// use a uint8_t variable for receiving and sending data 

  m_usb_init();							// usb initialized
  while(!m_usb_isconnected());			// wait for usb connection to form	

  teensy_clockdivide(0);  				// set clock speed to 16Mhz required to get baud rate correct
  uart_init(115200);					// set baud rate

  while (1) {
    if (uart_available()) {
      data = uart_getchar();           // read one byte
      uart_putchar(data);              // write one byte

      m_usb_tx_int(data);			   // print data	
      m_usb_tx_string("\n");
      
    } 
  }
}

