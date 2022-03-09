#ifdef TEST

CONNECT TO ROS WITH C++
#include <ros.h>
// http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup#:~:text=The%20Arduino%20and%20Arduino%20IDE,works%20over%20your%20Arduino's%20UART.

CONNECT TO ROS WITH PYTHON
// https://realpython.com/arduino-python/

MEASURE INPUT VOLTAGE

long readVcc() {
 
long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

DYNAMICS 
stages:
    orient self 
    search for "food"
        if located food, call others
    search for path/ robot that found food 
    once arrive, encircle food (make equidistant as more arrive)    

#endif