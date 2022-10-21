#include<SPIMemory.h>

uint32_t memAddr;
double data[8];

SPIFlash flash(SS, &SPI);       //Use this constructor if using an SPI bus other than the default SPI. Only works with chips with more than one hardware SPI bus
// SPIFlash flash;

void setup() {
  Serial.begin(115200);

  flash.begin();

  Serial.print(F("Total capacity: "));
  Serial.println(flash.getCapacity());

  memAddr = 0;
  data[0] = 12.1;
  data[1] = 0.2;
  data[2] = 1.0;
  data[3] = 30.2;
  data[4] = 1.1;
  data[5] = 4.2;
  data[6] = 1.5;
  data[7] = 3.4;

  Serial.println("\nWrite\n");
  for(int i=0; i<10; i++) {
    long start = micros();
    flash.writeByteArray(memAddr, (uint8_t *)data, sizeof(data));
    long end = micros();
    data[0] += 1.2;
    memAddr += 256;
    Serial.print("Time: ");
    Serial.println(end - start);
  }

  memAddr = 0;
  data[0] = 0;
  data[1] = 0;
  data[2] = 0;
  data[3] = 0;
  data[4] = 0;
  data[5] = 0;
  data[6] = 0;
  data[7] = 0;

  Serial.println("\nRead\n");
  for(int i=0; i<10; i++) {
    long start = micros();
    flash.readByteArray(memAddr, (uint8_t *)data, sizeof(data));
    long end = micros();
    Serial.println(data[0]);
    memAddr += 256;
    Serial.print("Time: ");
    Serial.println(end - start);
  }  

}

void loop() {
}

















