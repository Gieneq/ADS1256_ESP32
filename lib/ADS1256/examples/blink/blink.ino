// Arudino Sample Code to use ADS1256 library
// http://www.ti.com/lit/ds/symlink/ads1256.pdf

#include <Arduino.h>
#include "ADS1256.h"
#include <SPI.h>

float clockMHZ = 7.68; // crystal frequency used on ADS1256
float vRef = 2.47; // voltage reference

// Construct and init ADS1256 object
ADS1256 adc(clockMHZ,vRef,false); // RESETPIN is permanently tied to 3.3v

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting ADS1256");

  // start the ADS1256 with data rate of 15 SPS and gain x1
  adc.begin(ADS1256_DRATE_15000SPS,ADS1256_GAIN_1,false); 
 
  // Set MUX Register to AINO so it start doing the ADC conversion
  Serial.println("Channel set to Single end ch7");
  adc.setChannel(7);
  adc.selfcal();
}

void loop()
{
  adc.digitalWriteADS(1, HIGH);
  delay(250);
  adc.digitalWriteADS(1, LOW);
  delay(250);
}