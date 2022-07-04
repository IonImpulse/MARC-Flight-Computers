#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_GPS.h>
#include <string.h>
#include <bits/stdc++.h> 

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_DPS310.h>

using namespace std;

#define GRAVITY 10

#define ICM_CS 10

#define DEPLOY_PIN 12

/* for feather m0 */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Accelerometer/magnotometer
Adafruit_ICM20649 icm;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Barometer
Adafruit_DPS310 dps;

// GPS Setup
// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true

uint32_t timer = millis();

void setup()
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  pinMode(DEPLOY_PIN, OUTPUT);
  digitalWrite(DEPLOY_PIN, LOW);

  Serial.begin(115200);

  delay(5000);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init())
  {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1)
      ;
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }
  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  // Barometer start:
  if (!dps.begin_I2C())
  { // Can pass in I2C address here
    //if (! dps.begin_SPI(DPS310_CS)) {  // If you want to use SPI
    Serial.println("Failed to find DPS");
    while (1)
      yield();
  }
  Serial.println("DPS OK!");

  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

  // GPS Start:

  Serial.println("Adafruit GPS library basic parsing test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  // Accel/Mag section
  // Try to initialize!
  if (!icm.begin_I2C())
  {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20649 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("ICM20649 Found!");

  icm.setAccelRange(ICM20649_ACCEL_RANGE_30_G);

  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange())
  {
  case ICM20649_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20649_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20649_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  case ICM20649_ACCEL_RANGE_30_G:
    Serial.println("+-30G");
    break;
  }

  icm.setGyroRange(ICM20649_GYRO_RANGE_4000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange())
  {
  case ICM20649_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);
  Serial.println();
}

long id = 6969;

float x_data_avg[100] = {};
float y_data_avg[100] = {};
float z_data_avg[100] = {};
float alt_data_avg[100] = {};

int avg_index = 0;

float x_avg = 0;
float y_avg = 0;
float z_avg = 0;
float last_alt_avg = -1;
float alt_avg = 0;

bool can_deploy = false;
bool should_deploy = false;
bool has_deployed = false;

float median_filter(float *data, int size)
{
  sort(data, data + size);

  if(size % 2 == 0) {
      return (data[size/2 - 1] + data[size/2])/2; 
  } else {
      return data[size/2];
  }
}

void loop()
{
  // Read accelerometer and gyro data
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t temp_event, pressure_event;

  icm.getEvent(&accel, &gyro, &temp);
  dps.getEvents(&temp_event, &pressure_event);

  x_data_avg[avg_index] = accel.acceleration.x;
  y_data_avg[avg_index] = accel.acceleration.y;
  z_data_avg[avg_index] = accel.acceleration.z;

  float altitude = dps.readAltitude();

  float pressure = pressure_event.pressure;
  float temperature = temp_event.temperature;

  float altitude_equ = ((temperature + 273) / (-6.5/1000)) * (pow((pressure * 100) / 101325, -(6.5/1000)*8.31432) - 1); // in meters

  alt_data_avg[avg_index] = altitude;

  if (avg_index == 99)
  {
    avg_index = 0;

    x_avg = median_filter(x_data_avg, 100);
    y_avg = median_filter(y_data_avg, 100);
    z_avg = median_filter(z_data_avg, 100);

    last_alt_avg = alt_avg;
    alt_avg = median_filter(alt_data_avg, 100);

    float mag = sqrt(pow(accel.acceleration.x, 2) + pow(accel.acceleration.y, 2) + pow(accel.acceleration.z, 2));

    // CAN_DEPLOY CASE 1: experiences G-force of 5 or more
    if (mag > 10 * GRAVITY)
    {
      can_deploy = true;
    }

    // CAN_DEPLOY CASE 2: it's reached 800 meters
    if (alt_avg > 5000)
    {
      can_deploy = true;
    }

    if (last_alt_avg > alt_avg && can_deploy )
    {
      should_deploy = true;
    }

    if (mag < 1.2 * GRAVITY && can_deploy)
    {
      should_deploy = true;
    }


  }
  else
  {
    avg_index++;
  }

  // read data from the GPS in the 'main loop'
  char c = GPS.read();

  if (GPS.newNMEAreceived())
  {
    Serial.println("Transmitting..."); // Send a message to rf95_server

    String data = "";
    data += id;
    data += ",";
    data += GPS.lastNMEA();
    int index = data.length();
    data.remove(index - 1, 1);

    String br_data = "";
    br_data += ",$ALT,";
    br_data += altitude;
    br_data += ",";
    br_data += altitude_equ;

    String acc_data = "";
    acc_data += id;
    acc_data += ",$ACCEL,";
    acc_data += x_avg;
    acc_data += ",";
    acc_data += y_avg;
    acc_data += ",";
    acc_data += z_avg;

    acc_data.concat(br_data);

    bool result1 = rf95.send((uint8_t *)acc_data.c_str(), strlen(acc_data.c_str()));
    rf95.waitPacketSent();
    bool result2 = rf95.send((uint8_t *)data.c_str(), strlen(data.c_str()));
    rf95.waitPacketSent();

    Serial.println(acc_data);
    Serial.println(data);

    if (result1 && result2)
    {
      Serial.println("Transmission successful!");
    }
    else
    {
      Serial.println("Transmission failed!");
    }

    Serial.print("Waiting for packet to complete...");
    rf95.waitPacketSent();
  }

  if (can_deploy && should_deploy && !has_deployed)
  {
    // Send a message to rf95_server
    String data = "";
    data += id;
    data += ",";
    data += "DEPLOY";

    rf95.send((uint8_t *)data.c_str(), strlen(data.c_str()));

    Serial.print(data);

    digitalWrite(DEPLOY_PIN, HIGH);
    delay(5000);
    digitalWrite(DEPLOY_PIN, LOW);

    should_deploy = false;
    has_deployed = true;
  }
}