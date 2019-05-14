
#include <EEPROM.h> // EEPROM libraries
#include <string.h>   

//structure to be uploaded in the EEPROM
struct ardEEPROM {
  char ID[10];
  float resistor;
  char date[11];
};

void setup() {
  Serial.begin(9600);
  while (!Serial) {
  }
  int eeAddress = 0;   //location in the EEPROM memory
  //Data to store.
    ardEEPROM toArd = {
    "P2110B004",
    286.1f,
    "17.08.2018",
  };
  EEPROM.put(eeAddress, toArd);
}

void loop() {
  /* Empty loop */
}



