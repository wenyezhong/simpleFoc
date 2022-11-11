#ifndef HW_SERIAL_H
#define HW_SERIAL_H

#include "Print.h"
class HardwareSerial : public Print
{
  public:   
   
    // size_t write(uint8_t);
    virtual size_t write(uint8_t);
    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }
    using Print::write; // pull in write(str) and write(buf, size) from Print
    // operator bool() { return true; }
};

#endif
