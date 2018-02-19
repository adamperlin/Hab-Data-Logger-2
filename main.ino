#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_Si7021.h> 
#include <SD.h>

 /** Main Data logger sketch for HAB crew #1 
  *  
  * Sensors in this version - 
  * 
  * Adafruit Altimeter Combo MPL3115A2
  * Adafruit humidity sensor Si7021
  * SainSmart MQ131 ozone sensor
  * 
  * Adafruit microSD card shield
  * 
  */

/*
 * CONSTANTS
 */

#define SD_PIN 10 //can change
#define OZONE_PIN (5)
#define IMU_SENSOR_ID 55
#define SD_DATA_FILE "data.txt"
#define SEATTLE_BASELINE_ATM 102710.0

#define POLL_DELAY 15000

#define LOG_PERIOD 15000  //Logging period in milliseconds, recommended value 15000-60000.
#define MAX_PERIOD 60000
#define TIME_PERIOD_MULTIPLIER (MAX_PERIOD / LOG_PERIOD)

// conversion factor between geiger counter counts / min
// and microsieverts
#define CPM_CONVERSION_FACTOR (1.0/151.0)
// pin to read switch input; switch enables/disables
// writing to SD card
#define SWITCH_PIN 4

// analog pin for reading analog datapoints from 
// MQ131 ozone sensor
#define OZONE_ANALOG_PIN 0


/*
 * GLOBAL sensor instances
 */

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
Adafruit_Si7021 hum = Adafruit_Si7021();

/**
 * main SD data file 
 */
File dataFile;

// time accounting (mainly for geiger counter)
unsigned long currentTime = 0L;
unsigned long previousTime = 0L;

void setup() {

  Serial.begin(9600);
  Serial.println(F("Beginning initialization..."));

  // initialize all devices
  devicesInit();
  //enable pin 2 as input for SD write switch 
  pinMode(SWITCH_PIN, INPUT);
  
  Serial.println(F("Initialization Finished"));


  // testing code; runs three data collection periods.
  
  for (int i = 0; i < 3; ++i) {
      previousTime = currentTime;
      currentTime = millis();
      pollSensors();
      delay(POLL_DELAY);
  }

  Serial.println(F("Test data read: "));
  testDataRead();

  currentTime = 0L;
  previousTime = 0L;
  
  delay(1000);
}

// check if SD switch is on or not.
bool checkSwitch() {
  return digitalRead(SWITCH_PIN) == HIGH;
}

// read data from the SD Card. Only used during testing.
void testDataRead() {
  dataFile = SD.open(SD_DATA_FILE, FILE_READ);
  while (dataFile.available()) {
    Serial.write(dataFile.read());
  }
  dataFile.close();
}

// we want to use a new data file for each run
void removeDataFileIfExists() {
  if (SD.exists(SD_DATA_FILE)) {
    SD.remove(SD_DATA_FILE);
  }
}


/**
 * Sensor initialization functions
 */
void initHumiditySensor() {
  if (!hum.begin()) { 
    Serial.println(F("No Humidity Sensor Detected"));
    while(1);
  }
}
void initAltimeter(){
  if (!baro.begin()) {
      Serial.println(F("No Altimeter Detected... idling"));
      while(1);
  }
  baro.setSeaPressure(SEATTLE_BASELINE_ATM);
}

void initOzoneSensor() {
  pinMode(OZONE_ANALOG_PIN, INPUT);
}


// Header to write to CSV data file
const char CSV_HEADER[] = {"time,humidity,ozone_analog,alt,temp,pressure,rad"};
void initSD() {
  pinMode(SD_PIN, OUTPUT);
  if (!SD.begin(SD_PIN)) {
    Serial.println(F("SD Card not present!"));
    while(1);
  }

  removeDataFileIfExists();

  dataFile = SD.open(SD_DATA_FILE, FILE_WRITE);
  if (!dataFile) {
    Serial.println(F("Error opening SD data file!"));
    while(1);
  }

  //write initial header to datafile
  dataFile.println(CSV_HEADER);
  dataFile.close();
}

int geigerCounts = 0;


void tube_impulse_handler() {
  geigerCounts++;
} 

// Geiger counter is interrupt-based, need to register
// interrupt handler
void initGeiger() {
  attachInterrupt(0, tube_impulse_handler, FALLING);
}

// root device initialization function
void devicesInit() {
  initAltimeter();
  initHumiditySensor();
  initSD();
  initGeiger();
  initOzoneSensor();
}


void loop() {
  previousTime = currentTime;
  currentTime = millis();
  pollSensors();
  checkAltimeter();
  delay(POLL_DELAY);
}


#define PREVIOUS_POINTS 3 

/**
 * Data variables
 */

double altimeterValues[PREVIOUS_POINTS];
double alt;
unsigned int alt_index = 0;
double microSieverts = 0.0;
double temp = 0.0;
double pressure = 0.0;
int ozoneVoltage = 0.0;
double humidity = 0.0;

const char SEP[] = {", "};

/**
 * Main Polling routine
 */
void pollSensors() {
  // if the switch is on, return; we can't use the SD Card
  if (checkSwitch()) return;
  dataFile = SD.open(SD_DATA_FILE, FILE_WRITE);
  
	
  temp = baro.getTemperature();
  pressure = baro.getPressure();

  alt = baro.getAltitude();
  for (int i = 1; i < PREVIOUS_POINTS; i++) {
    altimeterValues[i] = altimeterValues[i - 1];
  }
  altimeterValues[0] = alt;

  ozoneVoltage = analogRead(OZONE_ANALOG_PIN);
  humidity = hum.readHumidity();

  
  writeCSVPoint(currentTime);
  writeCSVPoint(humidity);
  writeCSVPoint(ozoneVoltage);


  writeCSVPoint(alt);
  writeCSVPoint(temp);
  writeCSVPoint(pressure);  

  // check if within geiger counter logging period...
  if(currentTime - previousTime > LOG_PERIOD) {
    Serial.println(F("logging"));
    Serial.println(F("uSV: "));
    Serial.println(microSieverts, DEC);
    Serial.println(currentTime - previousTime, DEC);
    microSieverts = geigerCounts * TIME_PERIOD_MULTIPLIER * CPM_CONVERSION_FACTOR;
    // reset geigerCounts

    geigerCounts = 0;
    writeCSVLastPoint(microSieverts);
  } 
  Serial.println(F("write finished"));
}

// simple wrapper to write a point to csv file + comma seperator
void writeCSVPoint(double data) {
  int n_bytes;
  dataFile.print(data, DEC);
  n_bytes = dataFile.print(SEP);
  if (n_bytes == 0) {
    Serial.println(F("Error: possible SD disconnect"));
  }
}

// writes last data point + eol to datafile and closes it
void writeCSVLastPoint(double data) {
  int n_bytes = dataFile.println(data, DEC);
  if (n_bytes == 0) {
    Serial.println(F("Error: possible SD disconnect"));
  }
  dataFile.close();
}

// This may do something in the future with the altimeter
// values.
void checkAltimeter() {
  return;
}
