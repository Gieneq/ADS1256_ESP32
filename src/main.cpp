// Arudino Sample Code to use ADS1256 library
// http://www.ti.com/lit/ds/symlink/ads1256.pdf

#include <Arduino.h>
#include "ADS1256.h"
#include <SPI.h>

float clockMHZ = 7.68; // crystal frequency used on ADS1256
float vRef = 2.47; // voltage reference

// Construct and init ADS1256 object
ADS1256 adc(clockMHZ,vRef,false); // RESETPIN is permanently tied to 3.3v

long sensor1;
// SPIClass* spiobject;

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting ADS1256");

  // start the ADS1256 with data rate of 15 SPS and gain x1
  adc.begin(ADS1256_DRATE_15000SPS,ADS1256_GAIN_1,false); 
 
  // Set MUX Register to AINO so it start doing the ADC conversion
  Serial.println("Channel set to Single end ch0");
  adc.setChannel(7);
  adc.pinModeADS(1, OUTPUT);
  adc.digitalWriteADS(1, HIGH);
  adc.setMode(CONTINUOUS);

  delayMicroseconds(7);  
}

void loop()
{
  // adc.getStatus();
  // adc.writeRegister(ADS1256_RADD_IO, 0x0F);
    // sensor1 = adc.readCurrentChannelCRaw();
    // Serial.println(sensor1);

    // }
      // adc.pollCurrentChannel();
    // digitalWrite(pinCS, LOW);  //CS LOW
    // while (digitalRead(pinDRDY)); //w8 til DRDY goes LOW

    // uint8_t buff[] = {0,0,0}; 
    // unsigned long value;

    // spiobject->transfer(buff, 3);
    // value = ((long)buff[2] << 16) + ((long)buff[1] << 8) + ((long)buff[0]);

    // float voltage = 2*2.47 * value / (1*0x7FFFFF);
    // // Serial.print("V "); Serial.print(voltage, DEC);
    // // Serial.print(", "); Serial.print(value, DEC);
    // // Serial.println(".");

    // digitalWrite(pinCS, HIGH);



    float sens = adc.readCurrentChannelC();
    // // delay(1);
    delay(5);
  // adc.digitalWriteADS(1, LOW);
    Serial.println(sens);
    // delay(5);
  // adc.digitalWriteADS(1, HIGH);
}