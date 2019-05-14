
//Arduino_nano

#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <EEPROM.h>
#include <stdio.h>
#include <string.h>


void gainADC(int16_t &a, float &b);
Adafruit_ADS1115 ads(0x48);
int outputPin = 13;
float Resistor;
int16_t adc0;  
float miliVolts=0.1875;
//int maxvalue=32767;
char powerHarv_ID[10];

    

struct toEEPROM{
  char IDPH[10];
  float resistorValue;
  char date[10];
};


void setup(void)
{
  int eeAddress = 0; //Move address to the next byte after float 'f'.
  Serial.begin(9600);
  pinMode(outputPin, OUTPUT);
//  ads.setGain(GAIN_SIXTEEN); //Lowest Gain as default 0.0078125 milivolts
  ads.setGain(GAIN_TWOTHIRDS); //2/3x gain +/- 6.144V  1 bit = 0.1875mV (default)
  ads.begin();
  digitalWrite(outputPin, HIGH);
  toEEPROM Object;
  EEPROM.get(eeAddress,Object);
  Resistor= Object.resistorValue;
  strcpy(powerHarv_ID,Object.IDPH);
  
}
  
void loop()
{ 

  float AveragePower = 0.0;
  float Voltage=0.0;
  float Power = 0.0;
for (int counter=0;counter<20;counter++)
{
     adc0 = ads.readADC_SingleEnded(0);
     gainADC(adc0,miliVolts);
    
     Voltage = (adc0 * miliVolts) / 1000;
     Power = ((Voltage * Voltage) / Resistor) * 1000;
     AveragePower+=Power;
//     
  //    Serial.print("AIN2_NANO: "); 
//      Serial.print(adc2);  
   //  Serial.print("\tVoltage: ");
   //
   
  //Serial.print(Voltage, 15);
    //Serial.print("\n");

//      Serial.print("\tPower: ");
//      Serial.println(Power,15);  
}

//  Serial.print("Average Power: ");
  //
  Serial.print(powerHarv_ID);
  Serial.print(":00"); 
  Serial.print(" ");
  Serial.print(AveragePower/20,15);

  Serial.print("\n");

  delay(1);   
}


void gainADC(int16_t &a, float &b){
  
     
     if(a<21844 && b==0.1875)  {ads.setGain(GAIN_ONE);     b=0.125;}
     if(a<16384  && b==0.125)   {ads.setGain(GAIN_TWO);     b=0.0625; return;}
     if(a<16384   && b==0.0625)  {ads.setGain(GAIN_FOUR);    b=0.03125;return;}
     if(a<16384   && b==0.03125) {ads.setGain(GAIN_EIGHT);   b=0.015625;return;}
     if(a<16384  && b==0.015625){ads.setGain(GAIN_SIXTEEN); b=0.0078125;return;}


     if(a==32767 && b==0.0078125){ads.setGain(GAIN_EIGHT); b=0.015625;return;}
     if(a==32767 && b==0.015625){ads.setGain(GAIN_FOUR); b=0.03125;return;}
     if(a==32767 && b==0.03125){ads.setGain(GAIN_TWO); b=0.0625;return;}
     if(a==32767 && b==0.0625){ads.setGain(GAIN_ONE); b=0.125;return;}
     if(a==32767 && b==0.125){ads.setGain(GAIN_TWOTHIRDS); b=0.1875;return;}

     
}

