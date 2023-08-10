#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;
const int chipSelect = 5;

void setup()
{
  Serial.begin(9600);
  //Inicializar Módulo SD
  Serial.print("Initializing SD card...");

  pinMode(SS, OUTPUT);
  
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    return;
  }
  Serial.println("Card initialized.");

  //Inicializar BMP280
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop()
{
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (bmp.takeForcedMeasurement()) {
    // can now print out the new measurements
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    
    //Escreve no SD
    dataFile.print("Temperature = ");
    dataFile.print(bmp.readTemperature());
    dataFile.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    
    //Escreve no SD
    dataFile.print("Pressure = ");
    dataFile.print(bmp.readPressure());
    dataFile.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    Serial.println(" m");
    
    //Escreve no SD
    dataFile.print(F("Approx altitude = "));
    dataFile.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
    dataFile.println(" m");

  } else {
    Serial.println("Forced measurement failed!");
  }
  dataFile.close();
  delay(300);
}