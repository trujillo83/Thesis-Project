
#include <EEPROM.h>

struct MyObject{
  char field1[10];
  float field2;
  char name[11];
};

void setup(){

  int eeAddress = 0; //EEPROM address to start reading from

  Serial.begin( 9600 );
  
  //Get the float data from the EEPROM at position 'eeAddress'
  
  // get() can be used with custom structures too. 
  MyObject customVar; //Variable to store custom object read from EEPROM.
  EEPROM.get( eeAddress, customVar );

  Serial.println( "Read custom object from EEPROM: " );
  Serial.println( customVar.field1 );
  Serial.println( customVar.field2 );
  Serial.println( customVar.name );
}

void loop(){ /* Empty loop */ }

