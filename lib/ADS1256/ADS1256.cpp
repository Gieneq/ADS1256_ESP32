/*
        ADS1256.h - Arduino Library for communication with Texas Instrument ADS1256 ADC
        Written by Adien Akhmad, August 2015
		    Modfified  Jan 2019 by Axel Sepulveda for ATMEGA328
        Reworked in Mar 2022 by Gieneq/Pyrograf to support ESP32
*/

#include "ADS1256.h"
#include "Arduino.h"
#include "SPI.h"

#if   defined(ARDUINO_ARCH_ESP32)
SPIClass spiobject(HSPI);
#endif

ADS1256::ADS1256(float clockspdMhz, float vref, bool useResetPin) {
  // Set DRDY as input
  pinMode(pinDRDY, INPUT);      
  // Set CS as output
  pinMode(pinCS, OUTPUT);
  
  if (useResetPin) {
    // set RESETPIN as output
    pinMode(pinRST, OUTPUT );
    // pull RESETPIN high
    pinMode(pinRST, HIGH);
  }

  // Voltage Reference
  _VREF = vref;

  // Copy reg IO, default all inputs, IO0 - CLKOUT dont care
  _reg_IO = B11110000;

  // Start SPI on a quarter of ADC clock speed
  spiobject.begin();
  spiobject.beginTransaction(SPISettings(clockspdMhz * 1000000 / 4, MSBFIRST, SPI_MODE1));
}

void ADS1256::writeRegister(unsigned char reg, unsigned char wdata) {
  CSON();
  spiobject.transfer(ADS1256_CMD_WREG | reg); // opcode1 Write registers starting from reg
  spiobject.transfer(0);  // opcode2 Write (1 register) - 1 = 0
  spiobject.transfer(wdata);  // write wdata
  delayMicroseconds(1);              
  CSOFF();
}

unsigned char ADS1256::readRegister(unsigned char reg) {
  unsigned char readValue;
  CSON();
  spiobject.transfer(ADS1256_CMD_RREG | reg); // opcode1 read registers starting from reg
  spiobject.transfer(0);                  // opcode2 Write (1 register) - 1 = 0
  delayMicroseconds(7);              //  t6 delay (4*tCLKIN 50*0.13 = 6.5 us)    
  readValue = spiobject.transfer(0);          // read registers
  delayMicroseconds(1);              //  t11 delay (4*tCLKIN 4*0.13 = 0.52 us)    
  CSOFF();
  return readValue;
}

void ADS1256::sendCommand(unsigned char reg) {
  CSON();
  waitDRDY();
  spiobject.transfer(reg);
  delayMicroseconds(1);              //  t11 delay (4*tCLKIN 4*0.13 = 0.52 us)    
  CSOFF();
}


float ADS1256::convertADStoVoltage(long ads){
  return ((float)(ads) / 0x7FFFFF) * ((2 * _VREF) / (float)_pga);
}

float ADS1256::readCurrentChannelC() {
  long adsCode = readCurrentChannelCRaw();       
  return (((float)(adsCode) / 0x7FFFFF) * ((2 * _VREF) / (float)_pga));
}

long ADS1256::readCurrentChannelCRaw() {
  CSON();
  waitDRDY();
  long adsCode = read_int32();
  CSOFF();
  return adsCode;
}

// Call this ONLY after ADS1256_CMD_RDATA command
unsigned long ADS1256::read_uint24() {
  _buffer[0] = 0;
  _buffer[1] = 0;
  _buffer[2] = 0;
  spiobject.transfer(_buffer, 3);

  // Combine all 3-bytes to 24-bit data using byte shifting.
  // return uint8_t(*buffer) >> 8;
  return ((long)_buffer[0] << 16) + ((long)_buffer[1] << 8) + ((long)_buffer[2]);
}

// Call this ONLY after ADS1256_CMD_RDATA command
// Convert the signed 24bit stored in an unsigned 32bit to a signed 32bit
long ADS1256::read_int32() {
  long value = read_uint24();

  if (value & 0x00800000) { // if the 24 bit value is negative reflect it to 32bit
    value |= 0xff000000;
  }

  return value;
}

// Call this ONLY after ADS1256_CMD_RDATA command
// Cast as a float
float ADS1256::read_float32() {
  long value = read_int32();
  return (float)value;
}

// Channel switching for single ended mode. Negative input channel are
// automatically set to AINCOM
void ADS1256::setChannel(byte channel) { setChannel(channel, -1); }

// Channel Switching for differential mode. Use -1 to set input channel to
// AINCOM
void ADS1256::setChannel(byte AIN_P, byte AIN_N) {
  unsigned char MUX_CHANNEL;
  unsigned char MUXP;
  unsigned char MUXN;

  switch (AIN_P) {
    case 0:
      MUXP = ADS1256_MUXP_AIN0;
      break;
    case 1:
      MUXP = ADS1256_MUXP_AIN1;
      break;
    case 2:
      MUXP = ADS1256_MUXP_AIN2;
      break;
    case 3:
      MUXP = ADS1256_MUXP_AIN3;
      break;
    case 4:
      MUXP = ADS1256_MUXP_AIN4;
      break;
    case 5:
      MUXP = ADS1256_MUXP_AIN5;
      break;
    case 6:
      MUXP = ADS1256_MUXP_AIN6;
      break;
    case 7:
      MUXP = ADS1256_MUXP_AIN7;
      break;
    default:
      MUXP = ADS1256_MUXP_AINCOM;
  }

  switch (AIN_N) {
    case 0:
      MUXN = ADS1256_MUXN_AIN0;
      break;
    case 1:
      MUXN = ADS1256_MUXN_AIN1;
      break;
    case 2:
      MUXN = ADS1256_MUXN_AIN2;
      break;
    case 3:
      MUXN = ADS1256_MUXN_AIN3;
      break;
    case 4:
      MUXN = ADS1256_MUXN_AIN4;
      break;
    case 5:
      MUXN = ADS1256_MUXN_AIN5;
      break;
    case 6:
      MUXN = ADS1256_MUXN_AIN6;
      break;
    case 7:
      MUXN = ADS1256_MUXN_AIN7;
      break;
    default:
      MUXN = ADS1256_MUXN_AINCOM;
  }

  MUX_CHANNEL = MUXP | MUXN;

  CSON();
  writeRegister(ADS1256_RADD_MUX, MUX_CHANNEL);
  sendCommand(ADS1256_CMD_SYNC);
  sendCommand(ADS1256_CMD_WAKEUP);
  CSOFF();
}

/*
Init chip with set datarate and gain and perform self calibration
*/ 
void ADS1256::begin(unsigned char drate, unsigned char gain, bool buffenable) {
  _pga = 1 << gain;
  sendCommand(ADS1256_CMD_RESET); 
  sendCommand(ADS1256_CMD_SDATAC);  // send out ADS1256_CMD_SDATAC command to stop continous reading mode.
  writeRegister(ADS1256_RADD_DRATE, drate);  // write data rate register   
  uint8_t bytemask = B00000111;
  uint8_t adcon = readRegister(ADS1256_RADD_ADCON);
  uint8_t byte2send = (adcon & ~bytemask) | gain;
  writeRegister(ADS1256_RADD_ADCON, byte2send);
  if (buffenable) {  
    uint8_t status = readRegister(ADS1256_RADD_STATUS);   
    bitSet(status, 1); 
    writeRegister(ADS1256_RADD_STATUS, status);
  }
}

/*
Reads and returns STATUS register
*/ 
uint8_t ADS1256::getStatus() {
  sendCommand(ADS1256_CMD_SDATAC);  // send out ADS1256_CMD_SDATAC command to stop continous reading mode.
  return readRegister(ADS1256_RADD_STATUS); 
}

void ADS1256::selfcal() {
  sendCommand(ADS1256_CMD_SELFCAL);  // perform self calibration
  waitDRDY(); // wait ADS1256 to settle after self calibration
}


void ADS1256::CSON() {
  //PORT_CS &= ~(1 << PINDEX_CS);
  digitalWrite(pinCS, LOW);
}  // digitalWrite(_CS, LOW); }

void ADS1256::CSOFF() {
  digitalWrite(pinCS, HIGH);
  //PORT_CS |= (1 << PINDEX_CS);
}  // digitalWrite(_CS, HIGH); }

void ADS1256::waitDRDY() {
  //while (PIN_DRDY & (1 << PINDEX_DRDY));
  while (digitalRead(pinDRDY));
}
void ADS1256::waitNotDRDY() {
  //while (PIN_DRDY & (1 << PINDEX_DRDY));
  while (!digitalRead(pinDRDY));
}

boolean ADS1256::isDRDY() {
  return !digitalRead(pinDRDY);
}	


void ADS1256::digitalWriteADS(uint8_t gpio_pin, uint8_t state) {
  if(state == HIGH)
    _reg_IO |= (1<<gpio_pin);
  else
    _reg_IO &= ~(1<<gpio_pin);
  writeRegister(ADS1256_RADD_IO, _reg_IO);
}

// no PULLUPs
void ADS1256::pinModeADS(uint8_t gpio_pin, uint8_t mode) {
  // 0 - output, 1 - input, state bits shifted + 4
  if(mode == OUTPUT)
    _reg_IO &= ~(1<<(gpio_pin+4));
  else
    _reg_IO |= (1<<(gpio_pin+4));
  writeRegister(ADS1256_RADD_IO, _reg_IO);
}

uint8_t ADS1256::digitalReadADS(uint8_t gpio_pin) {
  uint8_t ioreg = readRegister(ADS1256_RADD_IO); 

  return (ioreg & (1<<gpio_pin)) > 0 ? HIGH : LOW;
}
