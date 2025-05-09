#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <TinyGPSPlus.h>
#include <utility/imumaths.h>
#include <HardwareSerial.h>

#define GPS_I2C_ADDRESS 0x10
#define BNO_I2C_ADDRESS 0x28
#define BME_I2C_ADDRESS 0x76

#define iterationnumber 30
#define BAUDRATE 115200

#define M0_PIN 33
#define M1_PIN 26
#define AUX_PIN 27

#define SD_MISO 19
#define SD_MOSI 23
#define SD_CLK  18
#define SD_CS 5

#define SCL_PIN 21
#define SDA_PIN 22

HardwareSerial LoraSerial(1);

TwoWire myWire(0);

Adafruit_BNO055 bno = Adafruit_BNO055(-1, BNO_I2C_ADDRESS, &myWire);

Adafruit_BME280 bme;

File CANSAT_File;

TinyGPSPlus gps;

imu::Vector<3> Acc_Vector, Gyro_Vector, Eul_Vector, Mag_Vector;

float Acc_X_Offset = 0;
float Acc_Y_Offset = 0;
float Acc_Z_Offset = 0;
float Eul_X_Offset = 0;
float Eul_Y_Offset = 0;
float Eul_Z_Offset = 0;


int convertToCESTHour(int utcHour) {
  int cestHour = utcHour + 2;
  if (cestHour >= 24) cestHour -= 24; // wrap around
  return cestHour;
}


void setup(){

  uint8_t Cal_sys = 0; 
  uint8_t Cal_gyr = 0;
  uint8_t Cal_acc = 0;
  uint8_t Cal_mag = 0;
  uint16_t i = 0;

  //---------------------- INITIALIZING LoraSerial AND SENSORS ----------------------//  

  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(AUX_PIN, INPUT);
  delay(100);
  digitalWrite(M0_PIN, LOW);
  digitalWrite(M1_PIN, LOW);
  delay(100);

  LoraSerial.begin(BAUDRATE, SERIAL_8N1, 16, 17);

  LoraSerial.print("Serial conn. success, Baud :"); LoraSerial.println(BAUDRATE);    // LoraSerial CONNECTION

  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  myWire.begin(SDA_PIN, SCL_PIN);
  delay(1000);

  SD.begin(SD_CS);
  if(!SD.begin(SD_CS)){
    LoraSerial.println("SD conn. fail");
  }
  else{
    LoraSerial.println("SD conn. success");
  }


  bno.begin();
  if(!bno.begin()){
    LoraSerial.println("BNO conn. fail");
  }
  else {
    LoraSerial.println("BNO conn. success");
  }

  if (!bme.begin(BME_I2C_ADDRESS, &myWire)){
    LoraSerial.println("BME conn. fail");
  }
  else{
    LoraSerial.println("BME conn. success");
  }
  
  bno.setExtCrystalUse(true);                                               // EXTERNAL CRYSTAL

  //---------------------- SENSOR CALIBRATION ----------------------//

  LoraSerial.println("Calibration Status");
  while((Cal_sys + Cal_gyr + Cal_acc + Cal_mag) != 12){                     
    bno.getCalibration(&Cal_sys, &Cal_gyr, &Cal_acc, &Cal_mag);
    LoraSerial.print("sys - "); LoraSerial.println(Cal_sys);
    LoraSerial.print("acc - "); LoraSerial.println(Cal_acc);
    LoraSerial.print("gyr - "); LoraSerial.println(Cal_gyr);
    LoraSerial.print("mag - "); LoraSerial.println(Cal_mag);
    LoraSerial.print("--------//");LoraSerial.print(i);LoraSerial.println("//--------");
    i++;
    delay(3000);
  }
  delay(10000);
  bno.getCalibration(&Cal_sys, &Cal_gyr, &Cal_acc, &Cal_mag);
  LoraSerial.print("sys - "); LoraSerial.println(Cal_sys);
  LoraSerial.print("acc - "); LoraSerial.println(Cal_acc);
  LoraSerial.print("gyr - "); LoraSerial.println(Cal_gyr);
  LoraSerial.print("mag - "); LoraSerial.println(Cal_mag);
  LoraSerial.println("-----------");
  LoraSerial.println("Sensors are good");    
  LoraSerial.println("0");                                     

  //---------------------- OFFSET ----------------------//

  delay(1000);


  CANSAT_File = SD.open("/offset.csv", FILE_WRITE);
  LoraSerial.println("Starting Calibration, keep sensor still");
  LoraSerial.println("1");

  for(int j = 0; j <= iterationnumber; j++){
    Acc_Vector = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    Gyro_Vector = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    Eul_Vector = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    Acc_X_Offset += Acc_Vector.x();
    Acc_Y_Offset += Acc_Vector.y();
    Acc_Z_Offset += Acc_Vector.z() - 9.81;

    Eul_X_Offset += Eul_Vector.x();
    Eul_Y_Offset += Eul_Vector.y();
    Eul_Z_Offset += Eul_Vector.z();

    delay(5);
    // somma tutti i 600 valori di accelerazione e velocità angolare
  }

  Acc_X_Offset /= iterationnumber;
  Acc_Y_Offset /= iterationnumber;
  Acc_Z_Offset /= iterationnumber;

  Eul_X_Offset /= iterationnumber;
  Eul_Y_Offset /= iterationnumber;
  Eul_Z_Offset /= iterationnumber;

  // divide per 600 per calcolare la media dell'offset
  LoraSerial.print("Acc_X OFFSET");                       LoraSerial.print(",");
  LoraSerial.println(Acc_X_Offset);                        
  
  LoraSerial.print("Acc_Y OFFSET");                       LoraSerial.print(",");
  LoraSerial.println(Acc_Y_Offset);                       
  
  LoraSerial.print("Acc_Z OFFSET");                       LoraSerial.print(",");  
  LoraSerial.println(Acc_Z_Offset);                      
  
  LoraSerial.print("Eul_X OFFSET");                       LoraSerial.print(",");
  LoraSerial.println(Eul_X_Offset);                        
  
  LoraSerial.print("Eul_Y OFFSET");                       LoraSerial.print(",");
  LoraSerial.println(Eul_Y_Offset);                       
  
  LoraSerial.print("Eul_Z OFFSET");                       LoraSerial.print(",");
  LoraSerial.println(Eul_Z_Offset);                       


  CANSAT_File.print("Acc_X OFFSET");                       CANSAT_File.print(",");
  CANSAT_File.println(Acc_X_Offset);                        

  CANSAT_File.print("Acc_Y OFFSET");                       CANSAT_File.print(",");
  CANSAT_File.println(Acc_Y_Offset);                       

  CANSAT_File.print("Acc_Z OFFSET");                       CANSAT_File.print(",");  
  CANSAT_File.println(Acc_Z_Offset);                      

  CANSAT_File.print("Eul_X OFFSET");                       CANSAT_File.print(",");
  CANSAT_File.println(Eul_X_Offset);                        

  CANSAT_File.print("Eul_Y OFFSET");                       CANSAT_File.print(",");
  CANSAT_File.println(Eul_Y_Offset);                       

  CANSAT_File.print("Eul_Z OFFSET");                       CANSAT_File.print(",");
  CANSAT_File.println(Eul_Z_Offset);                       

  LoraSerial.println("Calibration Completed");
  LoraSerial.println("2");

  //---------------------- CSV ----------------------//

  CANSAT_File.close();
  delay(4000);
  CANSAT_File = SD.open("/CANSAT.csv", FILE_WRITE);

  CANSAT_File.print("ACC_X_b");                           CANSAT_File.print(",");
  CANSAT_File.print("ACC_Y_b");                           CANSAT_File.print(",");
  CANSAT_File.print("ACC_Z_b");                           CANSAT_File.print(",");
  CANSAT_File.print("GYRO_X");                            CANSAT_File.print(",");
  CANSAT_File.print("GYRO_Y");                            CANSAT_File.print(",");
  CANSAT_File.print("GYRO_Z");                            CANSAT_File.print(",");
  CANSAT_File.print("EUL_X");                             CANSAT_File.print(",");
  CANSAT_File.print("EUL_Y");                             CANSAT_File.print(",");
  CANSAT_File.print("EUL_Z");                             CANSAT_File.print(",");
  CANSAT_File.print("MAG_X");                             CANSAT_File.print(",");
  CANSAT_File.print("MAG_Y");                             CANSAT_File.print(",");
  CANSAT_File.print("MAG_Z");                             CANSAT_File.print(",");
  CANSAT_File.print("GPS_STATUS");                          CANSAT_File.print(",");
  CANSAT_File.print("LATITUDE");                          CANSAT_File.print(",");
  CANSAT_File.print("LONGITUDE");                         CANSAT_File.print(",");
  CANSAT_File.print("PRESSURE");                          CANSAT_File.print(",");
  CANSAT_File.print("TEMPERATURE");                       CANSAT_File.print(",");
  CANSAT_File.println("REAL_TIME");

  LoraSerial.print("ACC_X_b");                                LoraSerial.print(",");
  LoraSerial.print("ACC_Y_b");                                LoraSerial.print(",");
  LoraSerial.print("ACC_Z_b");                                LoraSerial.print(",");
  LoraSerial.print("GYRO_X");                                 LoraSerial.print(",");
  LoraSerial.print("GYRO_Y");                                 LoraSerial.print(",");
  LoraSerial.print("GYRO_Z");                                 LoraSerial.print(",");
  LoraSerial.print("EUL_X");                                  LoraSerial.print(",");
  LoraSerial.print("EUL_Y");                                  LoraSerial.print(",");
  LoraSerial.print("EUL_Z");                                  LoraSerial.print(",");
  LoraSerial.print("MAG_X");                                  LoraSerial.print(",");
  LoraSerial.print("MAG_Y");                                  LoraSerial.print(",");
  LoraSerial.print("MAG_Z");                                  LoraSerial.print(",");
  LoraSerial.print("GPS_STATUS");                               LoraSerial.print(",");
  LoraSerial.print("LATITUDE");                               LoraSerial.print(",");
  LoraSerial.print("LONGITUDE");                              LoraSerial.print(",");
  LoraSerial.print("PRESSURE");                               LoraSerial.print(",");
  LoraSerial.print("TEMPERATURE");                            LoraSerial.print(",");
  LoraSerial.println("REAL_TIME");

  //---------------------- DATA GATHERING ----------------------//

  LoraSerial.println("go");
  LoraSerial.println("3");
}

void loop(){
  static uint16_t Secondary_Time = 0;
  static uint16_t Tertiary_Time = 0;
  static uint32_t Real_Time_previous = 0;
  static uint32_t Real_Time = millis();
  static uint32_t Start_Time = millis();
  static float latitude = 0;
  static float longitude = 0;
  static float pressure = 0;
  static float temperature = 0;
  uint8_t gpsSTATUS = 0;
  uint16_t gpsHour = 0;
  uint16_t gpsMinute = 0;
  float gpsSecond = 0;


  Real_Time_previous = Real_Time;
  Real_Time = millis();
  Secondary_Time += Real_Time - Real_Time_previous;
  Tertiary_Time += Real_Time - Real_Time_previous;  
  Acc_Vector = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Gyro_Vector = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Eul_Vector = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Mag_Vector = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  
  if(Secondary_Time >= 500){
    Secondary_Time = 0;
    if(Tertiary_Time >= 2000){
      Tertiary_Time = 0;
      myWire.beginTransmission(GPS_I2C_ADDRESS);
      myWire.write(0x00); // Request from start
      myWire.endTransmission();
      myWire.requestFrom(GPS_I2C_ADDRESS, 64);
      if(myWire.available()){
        gps.encode(Wire.read());
      }
      if(gps.location.isValid()){
        gpsSTATUS = 1;
      }
      if (gps.location.isUpdated()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
      }
    }
    pressure = bme.readPressure();
    temperature = bme.readTemperature();
    LoraSerial.print(Acc_Vector.x());                                     LoraSerial.print(",");
    LoraSerial.print(Acc_Vector.y());                                     LoraSerial.print(",");
    LoraSerial.print(Acc_Vector.z());                                     LoraSerial.print(",");
    LoraSerial.print(Gyro_Vector.x());                                    LoraSerial.print(",");
    LoraSerial.print(Gyro_Vector.y());                                    LoraSerial.print(",");
    LoraSerial.print(Gyro_Vector.z());                                    LoraSerial.print(",");
    LoraSerial.print(Eul_Vector.x());                                     LoraSerial.print(",");
    LoraSerial.print(Eul_Vector.y());                                     LoraSerial.print(",");
    LoraSerial.print(Eul_Vector.z());                                     LoraSerial.print(",");
    LoraSerial.print(Mag_Vector.x());                                     LoraSerial.print(",");
    LoraSerial.print(Mag_Vector.y());                                     LoraSerial.print(",");
    LoraSerial.print(Mag_Vector.z());                                     LoraSerial.print(",");
    LoraSerial.print(gpsSTATUS);                                          LoraSerial.print(",");
    LoraSerial.print(latitude);                                           LoraSerial.print(",");
    LoraSerial.print(longitude);                                          LoraSerial.print(",");
    LoraSerial.print(pressure);                                           LoraSerial.print(",");
    LoraSerial.print(temperature);                                        LoraSerial.print(",");
    LoraSerial.println(Real_Time - Start_Time);           
    CANSAT_File.flush();           
  }            

  CANSAT_File.print(Acc_Vector.x());                                     CANSAT_File.print(",");
  CANSAT_File.print(Acc_Vector.y());                                     CANSAT_File.print(",");
  CANSAT_File.print(Acc_Vector.z());                                     CANSAT_File.print(",");
  CANSAT_File.print(Gyro_Vector.x());                                    CANSAT_File.print(",");
  CANSAT_File.print(Gyro_Vector.y());                                    CANSAT_File.print(",");
  CANSAT_File.print(Gyro_Vector.z());                                    CANSAT_File.print(",");
  CANSAT_File.print(Eul_Vector.x());                                     CANSAT_File.print(",");
  CANSAT_File.print(Eul_Vector.y());                                     CANSAT_File.print(",");
  CANSAT_File.print(Eul_Vector.z());                                     CANSAT_File.print(",");
  CANSAT_File.print(Mag_Vector.x());                                     CANSAT_File.print(",");
  CANSAT_File.print(Mag_Vector.y());                                     CANSAT_File.print(",");
  CANSAT_File.print(Mag_Vector.z());                                     CANSAT_File.print(",");
  CANSAT_File.print(gpsSTATUS);                                          CANSAT_File.print(",");
  CANSAT_File.print(latitude);                                           CANSAT_File.print(",");
  CANSAT_File.print(longitude);                                          CANSAT_File.print(",");
  CANSAT_File.print(pressure);                                           CANSAT_File.print(",");
  CANSAT_File.print(temperature);                                        CANSAT_File.print(",");
  CANSAT_File.println(Real_Time - Start_Time);

}
