#pragma once 

#include "Arduino.h"
#include "SPI.h"

#ifndef ADS1256_h
#define ADS1256_h

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
#define pinDRDY 9
#define pinRST  8
#define pinCS   10
#define spiobject SPI
// SCK : 13;
// MISO : 12;
// MOSI : 11;
// SS : 10;

#elif   defined(ARDUINO_ARCH_ESP32)
#define pinDRDY 16
#define pinRST  4
#define pinCS   15
#include <esp32-hal-spi.h>
SPIClass spiobject(HSPI);
// Choose HSPI:
// SCK : 14;
// MISO : 12;
// MOSI : 13;
// SS : 15;

#else 
	#warning  "Oops! Pins for your board are not defined: pinDRDY, pinRST, pinCS"
#endif


// ADS1256 Register address
#define ADS1256_RADD_STATUS 0x00
#define ADS1256_RADD_MUX 0x01
#define ADS1256_RADD_ADCON 0x02
#define ADS1256_RADD_DRATE 0x03
#define ADS1256_RADD_IO 0x04
#define ADS1256_RADD_OFC0 0x05
#define ADS1256_RADD_OFC1 0x06
#define ADS1256_RADD_OFC2 0x07
#define ADS1256_RADD_FSC0 0x08
#define ADS1256_RADD_FSC1 0x09
#define ADS1256_RADD_FSC2 0x0A

// ADS1256 Command
#define ADS1256_CMD_WAKEUP 0x00
#define ADS1256_CMD_RDATA 0x01
#define ADS1256_CMD_RDATAC 0x03
#define ADS1256_CMD_SDATAC 0x0f
#define ADS1256_CMD_RREG 0x10
#define ADS1256_CMD_WREG 0x50
#define ADS1256_CMD_SELFCAL 0xF0
#define ADS1256_CMD_SELFOCAL 0xF1
#define ADS1256_CMD_SELFGCAL 0xF2
#define ADS1256_CMD_SYSOCAL 0xF3
#define ADS1256_CMD_SYSGCAL 0xF4
#define ADS1256_CMD_SYNC 0xFC
#define ADS1256_CMD_STANDBY 0xFD
#define ADS1256_CMD_RESET 0xFE

// define multiplexer codes
#define ADS1256_MUXP_AIN0 0x00
#define ADS1256_MUXP_AIN1 0x10
#define ADS1256_MUXP_AIN2 0x20
#define ADS1256_MUXP_AIN3 0x30
#define ADS1256_MUXP_AIN4 0x40
#define ADS1256_MUXP_AIN5 0x50
#define ADS1256_MUXP_AIN6 0x60
#define ADS1256_MUXP_AIN7 0x70
#define ADS1256_MUXP_AINCOM 0x80

#define ADS1256_MUXN_AIN0 0x00
#define ADS1256_MUXN_AIN1 0x01
#define ADS1256_MUXN_AIN2 0x02
#define ADS1256_MUXN_AIN3 0x03
#define ADS1256_MUXN_AIN4 0x04
#define ADS1256_MUXN_AIN5 0x05
#define ADS1256_MUXN_AIN6 0x06
#define ADS1256_MUXN_AIN7 0x07
#define ADS1256_MUXN_AINCOM 0x08

// define gain codes
#define ADS1256_GAIN_1 0x00
#define ADS1256_GAIN_2 0x01
#define ADS1256_GAIN_4 0x02
#define ADS1256_GAIN_8 0x03
#define ADS1256_GAIN_16 0x04
#define ADS1256_GAIN_32 0x05
#define ADS1256_GAIN_64 0x06

// define drate codes
/*
        NOTE : 	Data Rate vary depending on crystal frequency. Data rates
   listed below assumes the crystal frequency is 7.68Mhz
                for other frequency consult the datasheet.
*/

#define ADS1256_DRATE_30000SPS 0xF0
#define ADS1256_DRATE_15000SPS 0xE0
#define ADS1256_DRATE_7500SPS 0xD0
#define ADS1256_DRATE_3750SPS 0xC0
#define ADS1256_DRATE_2000SPS 0xB0
#define ADS1256_DRATE_1000SPS 0xA1
#define ADS1256_DRATE_500SPS 0x92
#define ADS1256_DRATE_100SPS 0x82
#define ADS1256_DRATE_60SPS 0x72
#define ADS1256_DRATE_50SPS 0x63
#define ADS1256_DRATE_30SPS 0x53
#define ADS1256_DRATE_25SPS 0x43
#define ADS1256_DRATE_15SPS 0x33
#define ADS1256_DRATE_10SPS 0x23
#define ADS1256_DRATE_5SPS 0x13
#define ADS1256_DRATE_2_5SPS 0x03

//other
#define BUFFER_SIZE 3

enum mode_t {ONESHOT, CONTINUOUS};

class ADS1256 {
 public:
  ADS1256(float clockspdMhz, float vref, bool useresetpin);
  void writeRegister(unsigned char reg, unsigned char wdata);
  unsigned char readRegister(unsigned char reg);
  void sendCommand(unsigned char cmd);
  void setChannel(byte channel);
  void setChannel(byte AIP, byte AIN);
  void begin(unsigned char drate, unsigned char gain, bool bufferenable);
  uint8_t getStatus();  
  void waitDRDY();
  void waitNotDRDY();
  boolean isDRDY();
  void ADS1256::selfcal();
  void setGain(uint8_t gain);
  float readCurrentChannelC();
  long readCurrentChannelCRaw();
  float convertADStoVoltage(long ads);
  void digitalWriteADS(uint8_t gpio_pin, uint8_t state);
  void pinModeADS(uint8_t gpio_pin, uint8_t mode);
  uint8_t digitalReadADS(uint8_t gpio_pin);

 private:
  void CSON();
  void CSOFF();
  unsigned long read_uint24();
  long read_int32();
  float read_float32();
  uint8_t readGPIO();
  byte _pga;
  float _VREF;
  uint8_t _buffer[BUFFER_SIZE];
  uint8_t _reg_IO;

  SPIClass spiobject;
};

#endif
