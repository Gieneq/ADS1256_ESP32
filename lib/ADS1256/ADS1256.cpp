/*
        ADS1256.h - Arduino Library for communication with Texas Instrument ADS1256 ADC
        Written by Adien Akhmad, August 2015
		    Modfified  Jan 2019 by Axel Sepulveda for ATMEGA328
        Reworked in Mar 2022 by Gieneq/Pyrograf to support ESP32
*/

#include "ADS1256.h"
#include "Arduino.h"
#include <esp32-hal-spi.h>
#include "SPI.h"

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

  // Default conversion factor
  _conversionFactor = 1.0;

  // Choose HSPI:
  // SCK : 14;
  // MISO : 12;
  // MOSI : 13;
  // SS : 15;
  spiobject = SPIClass(HSPI);

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

void ADS1256::setContinuousMode(bool useContinuous){
  if(useContinuous)
    sendCommand(ADS1256_CMD_RDATAC);
  else
    sendCommand(ADS1256_CMD_SDATAC);
}

float ADS1256::convertADStoVoltage(long ads){
  return ((float)(ads) / 0x7FFFFF) * ((2 * _VREF) / (float)_pga);
}


float ADS1256::readCurrentChannel() {
  long adsCode = readCurrentChannelRaw();       
  return convertADStoVoltage(adsCode);
}

// Reads raw ADC data, as 32bit int
long ADS1256::readCurrentChannelRaw() {
  CSON();
  waitDRDY();
  spiobject.transfer(ADS1256_CMD_RDATA);
  delayMicroseconds(7);              //  t6 delay (4*tCLKIN 50*0.13 = 6.5 us)       
  long adsCode = read_int32();
  CSOFF();
  return adsCode;
}

float ADS1256::readCurrentChannelC() {
  long adsCode = readCurrentChannelCRaw();       
  return (((float)(adsCode) / 0x7FFFFF) * ((2 * _VREF) / (float)_pga)) * _conversionFactor;
}

long ADS1256::readCurrentChannelCRaw() {
  waitDRDY(); // should be already ready
  CSON();
  long adsCode = read_int32();
  // long adsCode = 0;
  // spiobject.transfer(buffer, 3);
  CSOFF();
  return adsCode;
}

// Call this ONLY after ADS1256_CMD_RDATA command
unsigned long ADS1256::read_uint24() {
  buffer[0] = 0;
  buffer[1] = 0;
  buffer[2] = 0;
  spiobject.transfer(buffer, 3);

  // Combine all 3-bytes to 24-bit data using byte shifting.
  // return uint8_t(*buffer) >> 8;
  return ((long)buffer[2] << 16) + ((long)buffer[1] << 8) + ((long)buffer[0]);
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
  sendCommand(ADS1256_CMD_SELFCAL);  // perform self calibration
  
  waitDRDY(); // wait ADS1256 to settle after self calibration
}

/*
Init chip with default datarate and gain and perform self calibration
*/ 
void ADS1256::begin() {
  sendCommand(ADS1256_CMD_SDATAC);  // send out ADS1256_CMD_SDATAC command to stop continous reading mode.
  uint8_t status = readRegister(ADS1256_RADD_STATUS);      
  sendCommand(ADS1256_CMD_SELFCAL);  // perform self calibration  
  waitDRDY();   // wait ADS1256 to settle after self calibration
}

/*
Reads and returns STATUS register
*/ 
uint8_t ADS1256::getStatus() {
  sendCommand(ADS1256_CMD_SDATAC);  // send out ADS1256_CMD_SDATAC command to stop continous reading mode.
  return readRegister(ADS1256_RADD_STATUS); 
}

// float ADS1256::pollContinuousChannel() {
//   CSON(); 
//   long adsCode = read_int32();
//   CSOFF();
// }


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

boolean ADS1256::isDRDY() {
  return !digitalRead(pinDRDY);
}	


// 0 - output, 1 - input
void ADS1256::setGPIOState(uint8_t gpios) {
  uint8_t reg_value = (readGPIO() & 0xF0) + ((gpios << 4) & 0x0F);
  writeRegister(ADS1256_RADD_IO, reg_value);

}

void ADS1256::setGPIOMode(uint8_t modes) {
  uint8_t reg_value = (modes & 0xF0) + (readGPIO() & 0x0F);
  writeRegister(ADS1256_RADD_IO, reg_value);
}

uint8_t ADS1256::readGPIO() {
  return readRegister(ADS1256_RADD_IO); 
}
