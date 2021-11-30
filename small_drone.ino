double uz = 0;
#include <SD.h>
#include <SPI.h>
#include "SBUS.h"
#include "TeensyThreads.h"
#include <Eigen.h>
#include <Eigen/Geometry>
#include "src/sensors/vectornav.h"
#include "src/filters/LowPassFilter.h"
#include "src/filters/SGFilter.h"
#include "src/codegen/barrier.h"
#include "src/codegen/zBodyInWorld.h"

using namespace Eigen;
using namespace Cyberpod;

// whether or not to use optitrack for position, velocity, and yaw
bool use_opti = false;

//rotation for Quaternion
// Vector3d reference_eul = {-M_PI/2,0,PI};
Vector3d reference_eul = {M_PI,0,M_PI/2.};
Vector3d reference_eul_no_yaw = {M_PI,0,0};
Quaterniond reference_quatd,reference_quatd_no_yaw;
double reference_quat[4],reference_quat_no_yaw[4];
Matrix3d reference_R, reference_R_no_yaw;
Vector3d lla_ref;

//states
double quat_state[4];
double vel_body[3];
double vel_world[3];
double pos_world[3];
double rates_state[3];

//timing
int sbus_t1, sbus_t2; int max_sbus_t = 0;
int xbee_t1, xbee_t2;
int imu_t1, imu_t2;

double yaw0[1];

const int chipSelect = BUILTIN_SDCARD;

int speedtest;

// SBUS setup
bool isSbusAvailable = 0;
uint16_t channels[16];
bool failSafe;
bool lostFrame;
SBUS x8r(Serial2);
bool armed = false;
int sbus_thread_id;


// Lidar Setup
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
bool isLidarAvailable = 0;
int range;

//Sonar setup
int sonarPin = 14;
int sonarValue;
bool isSonarAvailable = 0;

//GPS Setup
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GPS myGPS;
#include <SoftwareSerial.h>
long lastTime = 0; //Simple local timer. Limits amount of I2C traffic to Ublox module.
long lastArmed = 0;
long lastFlush = 0;
byte minSat = 0;
long latitude;
long longitude;
long altitude;
int PDOP, northVel, eastVel, downVel;
byte SIV;
bool isGPSAvailable = 0;

//IMU Setup
bool tared = false;
float quats_f[4],rates_f[3],acc_f[3], mag_f[3], gps_vel_body_f[3], gps_vel_ned_f[3], temp_f, pres_f;
double gps_pos_d[4];
float quat_yaw0[4];
byte in[120];
union {unsigned short s; byte b[2];} checksum;
union {float f; char c[4]; int32_t i; uint32_t u;} tmp;
union {double d; char c[8];} tmp_8;
char dataMsg[159+24];
bool isIMUAvailable = 0;
int bad_imu_chksum = 0;

//blackbox setup
//char dataString[150];
String name;
File myfile;

//optitrack variables
double quats_f_opti_yaw[4];
byte message[20];
byte header[2];
float pos_f[3], yaw_f;
bool isXbeeAvailable = 0;
int xbeeCount = 0;
int xbeeT = micros();
int opti_vel_time;
int opti_pos_time;

// Low Pass Filter

LowPassFilter *lpassX;
LowPassFilter *lpassY;
LowPassFilter *lpassZ;
LowPassFilter *lpassYaw;

//desired angular rates
int lastAuto = millis();
int lastHover = millis();
bool isAuto = false;
uint16_t des_channels[16];
float des_throttle = 0;
float des_roll = 0;
float des_pitch = 0;
float des_yaw = 0;
Vector3d des_vel;
float des_yaw_rate = 0;
float des_roll_rate = 0;
float des_pitch_rate = 0;
Vector3d global_vel;
Vector3d body_vel;



//Eigen initialization
Matrix3d Rd;
Matrix3d Rdz;
Matrix3d Rdy;
Matrix3d Rdx;
Matrix3d R;
Vector3d eR;
Matrix3d tmp_3d;
Vector3d Od;
Vector3d omega;
Vector3d eO;
Vector3d rates_des;
Vector3d cmd;
Vector3d eul;

//Hover controller
double hoverThrust_ = 0.3;
bool hover_ = false;
bool integrator_initialized_ = false;
double uzInt_ = 0;
int CTRLtnm1_ = 0;
double hover_height_ = 0.5;
double KiVz_ = 0.15;//0.15;
double KpVz_ = 0.10;//.15;
double KdVz_ = .05;//.25;
int lastTime2 = 0;

//barrier_
bool barrier_ = false;
uint16_t des_channels_barrier[16];
int lastBarrier = millis();
int barrier_t1;
int barrier_t2;
double lambda[1];
double h[101];
double u[4];
double u_des[4];

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


void setup() {


  Serial.begin(115200);
  delay(100);
  Serial.println("Serial coms init");

  Serial.print("Initializing SBUS.....");
  x8r.begin();
  delay(10);
  Serial.println("X");


  // Serial.print("Initializing Lidar....");
  // Wire.begin();
  // sensor.setTimeout(500);
  // if (!sensor.init())
  // {
  //   Serial.println("Failed to detect and initialize sensor!");
  //   while (1) {}
  // }
  // sensor.startContinuous();
  // Serial.println("x");


  //Serial.print("Initializing blackbox...");
  //Serial8.begin(115200);
  //delay(100);
  //while(!Serial8) {};
  Serial.println("X");

  Serial.println("Initializing Sonar....X");

  // Serial.print("Initializing GPS....");
  // do {
  //   //Serial.println("GPS: trying 115200 baud");
  //   Serial5.begin(115200);
  //   if (myGPS.begin(Serial5) == true) {
  //       //Serial.println("GPS: connected at 115200 baud!");
  //       delay(100);
  //       break;
  //   } else {
  //       //myGPS.factoryReset();
  //       delay(2000); //Wait a bit before trying again to limit the Serial output
  //   }

  //   //Serial.println("GPS: trying 38400 baud");
  //   Serial5.begin(38400);
  //   if (myGPS.begin(Serial5) == true)
  //   {
  //       //Serial.println("GPS: connected at 38400 baud, switching to 115200");
  //       myGPS.setSerialRate(115200);
  //       delay(100);
  //       break;
  //   }

  //   delay(100);
  //   //Serial.println("GPS: trying 9600 baud");
  //   Serial5.begin(9600);
  //   if (myGPS.begin(Serial5) == true) {
  //       //Serial.println("GPS: connected at 9600 baud, switching to 115200");
  //       myGPS.setSerialRate(115200);
  //       delay(100);
  //   }

  //   delay(100);

  // } while(1);

  // myGPS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  // myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  // myGPS.saveConfiguration(); //Save the current settings to flash and BBR

  // myGPS.setUART1Output(COM_TYPE_UBX); //Set the UART port to output UBX only
  // myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)

  // myGPS.setNavigationFrequency(19);           //Set output to 10 times a second
  // byte rate = myGPS.getNavigationFrequency(); //Get the update rate of this module
  // myGPS.setAutoPVT(true); //Tell the GPS to "send" each solution
  // Serial.print("Current update rate:");
  // Serial.println(rate);

  // myGPS.saveConfiguration(); //Save the current settings to flash and BBR

  // Serial.println("X");

  // Serial.println("Searching for satelights");

  // while(1) {
  //   if(myGPS.getPVT()) {
  //     SIV = myGPS.getSIV();
  //     if(SIV >= minSat) {
  //       break;
  //     }
  //     else {
  //       Serial.print(",");
  //       Serial.print(SIV);
  //     }
  //   }
  // }

  initIMU();
  eul2quatZYX(reference_eul,reference_quatd);
    eul2quatZYX(reference_eul_no_yaw,reference_quatd_no_yaw);
  quat2rotm(reference_quatd,reference_R);
  quat2rotm(reference_quatd_no_yaw,reference_R_no_yaw);
  reference_quat[0] = reference_quatd.w();
  reference_quat[1] = reference_quatd.x();
  reference_quat[2] = reference_quatd.y();
  reference_quat[3] = reference_quatd.z();
  reference_quat_no_yaw[0] = reference_quatd_no_yaw.w();
  reference_quat_no_yaw[1] = reference_quatd_no_yaw.x();
  reference_quat_no_yaw[2] = reference_quatd_no_yaw.y();
  reference_quat_no_yaw[3] = reference_quatd_no_yaw.z();
  // quats_f[0] = 99.0;
  // while (quats_f[0] == 99.0)
  // {
  //   readIMU();
  // }
  //
  // Quaterniond tmp_quat;
  // tmp_quat.w() = quats_f[3];
  // tmp_quat.x() = quats_f[0];
  // tmp_quat.y() = quats_f[1];
  // tmp_quat.z() = quats_f[2];
  // Vector3d tmp_eul;
  // quat2eulZYX(tmp_quat,tmp_eul);
  // tmp_eul(0) = 0; tmp_eul(1) = 0; tmp_eul(2) = -tmp_eul(2);
  // eul2quatZYX(tmp_eul,tmp_quat);
  // quat_yaw0[0] = tmp_quat.w();
  // quat_yaw0[1] = tmp_quat.x();
  // quat_yaw0[2] = tmp_quat.y();
  // quat_yaw0[3] = tmp_quat.z();
  //
  // tared = true;


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  while (!SD.begin(chipSelect)) {
  //if (!SD.sdfs.begin(SdioConfig(DMA_SDIO))) {
    Serial.println("Card failed, or not present");
    delay(100);

  }
  Serial.println("card initialized.");

//  String test = SD.sdfs.ls();
//  Serial.println(test);
//  Serial.println(test.indexOf("Japan"));

  // increase SD card frequency
  //bool ok = SD.sdfs.begin(SdSpiConfig(chipSelect, SHARED_SPI, SD_SCK_MHZ(16)));

//  for(int i = 0; i < 100; i++) {
//    file.getFilename(directory[i]);
//    if(directory[i] == NULL) {
//      break;
//    }
//  }
//
//  for(int i = 0; i <

  Serial.print("initializing xbee...");
  Serial8.begin(115200);
  Serial8.setTimeout(10);
  Serial.println("x");

  lpassX = new LowPassFilter(.1, .1, 1/100.);
  lpassY = new LowPassFilter(.1, .1, 1/100.);
  lpassZ = new LowPassFilter(.1, .1, 1/100.);
  lpassYaw = new LowPassFilter(.1, .1, 1/100.);
  // lpassX = new SGFilter(4, 20);
  // lpassY = new SGFilter(4, 20);
  // lpassZ = new SGFilter(4, 20);
  // lpassYaw = new SGFilter(4, 20);
  des_vel.setZero();
  global_vel.setZero();





  delay(1000);
  threads.setSliceMicros(50);
  sbus_thread_id = threads.addThread(readSBUS_thread,nullptr,50000);
  // threads.addThread(readLidar_thread);
  // threads.addThread(readSonar_thread);
  // threads.addThread(readGPS_thread);
  threads.addThread(readIMU_thread);
  if (use_opti)
    threads.addThread(readXbee_thread);
  Serial.println("attaching threads");









}

void loop() {
  // put your main code here, to run repeatedly:

  //Threads::Mutex mylock;
  //mylock.lock();
  //these will go into threads
//  readSBUS();
//  isSbusAvailable = 0;
//  readLidar();
//  isLidarAvailable = 0;
//  readSonar();
//  isSonarAvailable = 0;
//  readGPS();
//  isGPSAvailable = 0;

  // write to SD card if somthing has changed
  //delay(100);

  // quad has just been armed, open sd log
//  if(!armed) {
  if(!armed && channels[4] > 1100 && channels[4] < 2000 && millis() - lastArmed > 500) {

     // check for log name
     quat2yaw(quats_f,yaw0);

      //SD.begin(chipSelect);
      for(int i = 0; i < 1024; i++) {
        name = "log"+String(i)+".txt";
        if(!SD.exists(name.c_str())) {
          break;
        }
      }


     //bool ok = SD.sdfs.begin(SdSpiConfig(chipSelect, SHARED_SPI, SD_SCK_MHZ(16)));
     //SD.sdfs.begin(SdioConfig(DMA_SDIO));
     //delay(50);


     armed = 1;
     //FsFile myfile = SD.sdfs.open(name, O_WRITE | O_CREAT);
     myfile = SD.open(name.c_str(), FILE_WRITE);
     //unsigned int len = myfile.fileSize();
     Serial.print(name);
     Serial.print(" started with ");
     //Serial.print(len);
     Serial.println(" bytes");
//     if (len > 0) {
//       // reduce the file to zero if it already had data
//       myfile.truncate();
//     }

//     if (myfile.preAllocate(106*1000*15*60)) {
//       Serial.print("  Allocate" +String(106*1000*15*60)+" ");
//     } else {
//       Serial.print("  unable to preallocate this file");
//     }


     lastArmed = millis();
     lastTime = millis();
     lastFlush = millis();
//     myfile.println("is anyone there?");
//     myfile.close();
//
//     while(1) {};

  }

  if(millis() - lastFlush > 500 && armed) {
    myfile.flush();
    lastFlush = millis();
    //Serial.println("flushing");
  }

  if (armed && channels[4] < 1000 && millis() - lastArmed > 500) {
    armed = 0;
    lastArmed = millis();
    //close SD card
    Serial.println("closing SD");
    Serial.println(channels[4]);
    // myfile.close();
  }

  if((isSbusAvailable || isLidarAvailable || isSonarAvailable || isGPSAvailable || isIMUAvailable || isXbeeAvailable) && armed) {
//    String dataString = "";
//    dataString += micros(); dataString += ",";
//    dataString += armed; dataString += ",";
//
//    dataString += isSbusAvailable; dataString += ",";
//    dataString += channels[0]; dataString += ",";
//    dataString += channels[1]; dataString += ",";
//    dataString += channels[2]; dataString += ",";
//    dataString += channels[3]; dataString += ",";
//    dataString += channels[4]; dataString += ",";
//    dataString += channels[5]; dataString += ",";
//    dataString += channels[6]; dataString += ",";
//    dataString += channels[7]; dataString += ",";
//
//    dataString += isLidarAvailable; dataString += ",";
//    dataString += String(range, 7);
//    dataString += ",";
//
//    dataString += isSonarAvailable; dataString += ",";
//    dataString += String(sonarValue, 7);
//    dataString += ",";
//
//    dataString += isGPSAvailable; dataString += ",";
//    dataString += String(latitude, 15);
//    dataString += ",";
//    dataString += String(longitude, 15);
//    dataString += ",";
//    dataString += String(altitude, 15);
//    dataString += ",";
//    dataString += SIV; dataString += ",";
//
//    dataString += isIMUAvailable; dataString += ",";
//    for (int i = 0; i < 4; i++)
//    {
//      dataString += String(quats_f[i], 7);
//      dataString += ",";
//    }
//    for (int i = 0; i < 3; i++)
//    {
//      dataString += String(rates_f[i], 7);
//      dataString += ",";
//    }
//    for (int i = 0; i < 3; i++)
//    {
//      dataString += String(acc_f[i], 7);
//      dataString += ",";
//    }
//    for (int i = 0; i < 3; i++)
//    {
//      dataString += String(mag_f[i], 7);
//      dataString += ",";
//    }
//    dataString += String(temp_f, 7);
//    dataString += ",";
//    dataString += String(pres_f, 7);
//
//    Serial.println(dataString);

    int idx = 0;
    dataMsg[idx] = 49; idx++;
    dataMsg[idx] = 50; idx++;

    Threads::Mutex mylock;
    mylock.lock();

    dataMsg[idx] = isSbusAvailable; idx++;
    for (int i = 0; i < 8; i++) {
      tmp.i = channels[i];
      for (int j = 0; j < 4; j++) {
        dataMsg[idx] = tmp.c[j];
        idx++;
      }
    }
    // dataMsg[idx] = isLidarAvailable; idx++;
    // tmp.i = range;
    // for (int j = 0; j < 4; j++) {
    //     dataMsg[idx] = tmp.c[j];
    //     idx++;
    // }
    // dataMsg[idx] = isSonarAvailable; idx++;
    // tmp.i = sonarValue;
    // for (int j = 0; j < 4; j++) {
    //     dataMsg[idx] = tmp.c[j];
    //     idx++;
    // }
    // dataMsg[idx] = isGPSAvailable; idx++;
    // tmp.i = latitude;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }
    // tmp.i = longitude;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }
    // tmp.i = altitude;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }
    //
    // tmp.i = northVel;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }
    //
    // tmp.i = eastVel;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }
    //
    // tmp.i = downVel;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }
    //
    // tmp.i = PDOP;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }
    //
    // dataMsg[idx] = SIV; idx++;

    dataMsg[idx] = isIMUAvailable; idx++;
    for (int i = 0; i < 4; i++) {
      tmp.f = (float) quat_state[i];
      for (int j = 0; j < 4; j++) {
        dataMsg[idx] = tmp.c[j];
        idx++;
      }
    }
    for (int i = 0; i < 3; i++) {
      tmp.f = (float) rates_state[i];
      for (int j = 0; j < 4; j++) {
        dataMsg[idx] = tmp.c[j];
        idx++;
      }
    }
    tmp_8.d = pos_world[0];
    if (use_opti)
      tmp_8.d = lpassX->data_.y;
    for (int j = 0; j < 8; j++) {
      dataMsg[idx] = tmp_8.c[j];
      idx++;
    }
    tmp_8.d = pos_world[1];
    if (use_opti)
      tmp_8.d = lpassY->data_.y;
    for (int j = 0; j < 8; j++) {
      dataMsg[idx] = tmp_8.c[j];
      idx++;
    }
    tmp_8.d = pos_world[2];
    if (use_opti)
      tmp_8.d = lpassZ->data_.y;
    for (int j = 0; j < 8; j++) {
      dataMsg[idx] = tmp_8.c[j];
      idx++;
    }
    tmp_8.d = lla_ref[0];
    for (int j = 0; j < 8; j++) {
      dataMsg[idx] = tmp_8.c[j];
      idx++;
    }
    // tmp.f = lpassY->data_.y;
    tmp_8.d = lla_ref[1];
    for (int j = 0; j < 8; j++) {
      dataMsg[idx] = tmp_8.c[j];
      idx++;
    }
    // tmp.f = lpassZ->data_.y;
    tmp_8.d = lla_ref[2];
    for (int j = 0; j < 8; j++) {
      dataMsg[idx] = tmp_8.c[j];
      idx++;
    }
    tmp.f = vel_world[0];
    if (use_opti)
      tmp.f = lpassX->data_.yDot;
    for (int j = 0; j < 4; j++) {
      dataMsg[idx] = tmp.c[j];
      idx++;
    }
    tmp.f = vel_world[1];
    if (use_opti)
      tmp.f = lpassY->data_.yDot;
    for (int j = 0; j < 4; j++) {
      dataMsg[idx] = tmp.c[j];
      idx++;
    }
    tmp.f = vel_world[2];
    if (use_opti)
      tmp.f = lpassZ->data_.yDot;
    for (int j = 0; j < 4; j++) {
      dataMsg[idx] = tmp.c[j];
      idx++;
    }
    // for (int i = 0; i < 3; i++) {
    //   tmp.f = acc_f[i];
    //   for (int j = 0; j < 4; j++) {
    //     dataMsg[idx] = tmp.c[j];
    //     idx++;
    //   }
    // }

    // for (int i = 0; i < 3; i++) {
    //   tmp.f = mag_f[i];
    //   for (int j = 0; j < 4; j++) {
    //     dataMsg[idx] = tmp.c[j];
    //     idx++;
    //   }
    // }
    //
    // tmp.f = temp_f;
    //   for (int j = 0; j < 4; j++) {
    //     dataMsg[idx] = tmp.c[j];
    //     idx++;
    //   }
    // tmp.f = pres_f;
    //   for (int j = 0; j < 4; j++) {
    //     dataMsg[idx] = tmp.c[j];
    //     idx++;
    //   }
    //Serial.println(idx);
    // dataMsg[idx] = isXbeeAvailable; idx++;
    //
    // tmp.f = lpassX->data_.y;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }
    // tmp.f = lpassY->data_.y;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }
    // tmp.f = lpassZ->data_.y;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }
    // tmp.f = lpassX->data_.yDot;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }
    // tmp.f = lpassY->data_.yDot;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }
    // tmp.f = lpassZ->data_.yDot;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }
    // tmp.f = yaw_f;
    // for (int j = 0; j < 4; j++) {
    //   dataMsg[idx] = tmp.c[j];
    //   idx++;
    // }

    // desired angular rates data
    dataMsg[idx] = isAuto*isSbusAvailable; idx++;
    for (int i = 0; i < 8; i++) {
      if (barrier_)
        tmp.i = des_channels_barrier[i];
      else
        tmp.i = des_channels[i];
      for (int j = 0; j < 4; j++) {
        dataMsg[idx] = tmp.c[j];
        idx++;
      }
    }
    for (int i = 0; i < 4; i++) {
      tmp.f = u[i];
      for (int j = 0; j < 4; j++) {
        dataMsg[idx] = tmp.c[j];
        idx++;
      }
    }
    tmp.f = lambda[0];
    for (int j = 0; j < 4; j++) {
      dataMsg[idx] = tmp.c[j];
      idx++;
    }

    //Serial.println(idx);
    tmp.i = micros();
    for (int j = 0; j < 4; j++) {
      dataMsg[idx] = tmp.c[j];
      idx++;
    }

    mylock.unlock();
    dataMsg[idx] = 60; idx++;
    dataMsg[idx] = 59; idx++;

    isSbusAvailable = 0;
    isLidarAvailable = 0;
    isSonarAvailable = 0;
    isGPSAvailable = 0;
    isIMUAvailable = 0;
    isXbeeAvailable = 0;

    lastTime = millis();


    //myfile.print("test csdfasdfasdfasdfasdfasd");
    //Serial.println(millis()-lastTime);
    //Threads::Mutex mylock;
    //mylock.lock();
    //Serial.println("writing to file");

    if(myfile) {
      myfile.write(dataMsg,159+24);

    //myfile.println("why is this not working?");
    }
    else {
      Serial.println("failed to write");
    }
//    Serial.println(millis() - lastTime);
//    //mylock.unlock();
//    Serial.println(dataString);
  }
  //mylock.unlock();
//  speedtest = micros();
  threads.yield();

//  Serial.print(micros()-lastTime);
//  Serial.print(",");
//  Serial.print(channels[0]);
//  Serial.print(",");
//  Serial.print(range);
//  Serial.print(",");
//  Serial.print(sonarValue);
//  Serial.print(",");
//  Serial.println(SIV);

  //delay(100);

}

void readSBUS() {
  if(x8r.read(&channels[0], &failSafe, &lostFrame)){

    if(lostFrame) {
      Serial.println("lost frame");
    }
    isSbusAvailable = 1;
    // write the SBUS packet to an SBUS compatible servo
    //Serial.println(channels[0]);
    int Ltime = micros();

    //turn on and off auto
    if(!isAuto && channels[6] > 1100 && channels[6] < 2000 && millis() - lastAuto > 500 ) {
      lastAuto = millis();
      Serial.println(channels[6]);
      isAuto = 1;
      for (int i = 0; i < 3; i++)
        lla_ref[i] = pos_world[i];
    }
    if(isAuto && channels[6]<500) {
      isAuto = 0;
    }
    // turn on and off hover
    if(!hover_ && channels[7] > 500 && channels[7] < 2000 && millis() - lastHover > 500 ) {
      lastHover = millis();
      Serial.println("Hover on!");
      hover_ = 1;
    }
    if(hover_ && channels[7]<500) {
      hover_ = 0;
      Serial.println("Hover off!");
    }

    // turn on and off hover
    if(!barrier_ && channels[7] > 1700 && channels[7] < 2000 && millis() - lastBarrier > 500 ) {
      lastBarrier = millis();
      Serial.println("Barrier on!");
      barrier_ = 1;
    }
    if(barrier_ && channels[7]<1700) {
      barrier_ = 0;
      Serial.println("Barrier off!");
    }

    //if (armed)
    //{

    if(armed && isAuto) {
      doAuto();
      x8r.write(des_channels);
      //Serial.println("Auto ENABLED");
    }
    else{
      x8r.write(channels);
    }
    //}
    int timeElapsed = micros() - Ltime;
//      Serial.println(timeElapsed);
  }
}

void doAuto() {

  // current auto, set angular rates to 0 keep throttle

  // des_throttle = 0;
  des_roll_rate = (channels[1]-172)/(1811.-172)*2-1;
  des_pitch_rate = (channels[2]-172)/(1811.-172)*2-1;
  //des_yaw = (channels[3]-172)/(1811.-172)*2-1;
  double tmp1[1];
  quat2yaw(quats_f,tmp1);
  Quaterniond quat;
  // quat.w() = quats_f[3]; quat.x() = quats_f[0]; quat.y() = quats_f[1]; quat.z() = quats_f[2];
  quat.w() = quat_state[0]; quat.x() = quat_state[1]; quat.y() = quat_state[2]; quat.z() = quat_state[3];
  quat2eulZYX(quat, eul);
  // des_yaw = -tmp1[0];
  des_yaw = eul[2];

  des_roll = des_roll_rate*des_roll_rate*sgn(des_roll_rate) * M_PI/2;
  des_pitch = des_pitch_rate*des_pitch_rate*sgn(des_pitch_rate) * M_PI/2;
  //des_yaw = (des_yaw) * M_PI/4.0;


  // fixed example
//  des_roll = M_PI/4.0;
//  des_pitch = - des_roll;
//  des_yaw = M_PI/3.0;

  // des_vel(0) = 0;
  // body_vel(0) = 0;

  // des_vel(1) = 0;
  // body_vel(1) = 0 ;

  // // set desired velocity to be from the stick inputs
  // des_vel(0) = des_pitch;
  // des_vel(1) = -des_roll;

  // double vxError = des_vel(0)-body_vel(0);
  // double vyError = des_vel(1)-body_vel(1);
  // double K2 = .5;
  // des_pitch = K2*(vxError-eul(1));
  // des_roll = K2*(vyError-eul(0));

  // double pitch_error =  K2*(-vxError-eul(1));
  // double roll_error = -K2*(-vyError-eul(0));

  // des_pitch = fmin(fmax(des_pitch,-M_PI/4),M_PI/4);
  // des_roll = fmin(fmax(des_roll,-M_PI/4),M_PI/4);

  double a = des_yaw;
  double b = des_pitch;
  double y = des_roll;


  // Create desired rotation matrix

  // Rd(0,0) = cos(a)*cos(b); Rd(0,1) = cos(a)*sin(b)*sin(y)-sin(a)*cos(y); Rd(0,2) = cos(a)*sin(b)*cos(y)+sin(a)*sin(y);
  // Rd(1,0) = sin(a)*cos(b); Rd(1,1) = sin(a)*sin(b)*sin(y)+cos(a)*cos(y); Rd(1,2) = sin(a)*sin(b)*cos(y)-cos(a)*sin(y);
  // Rd(2,0) = -sin(b); Rd(2,1) = cos(b)*sin(y); Rd(2,2) = cos(b)*cos(y);


  Rdz(0,0) = cos(a); Rdz(0,1) = -sin(a); Rdz(0,2) = 0;
  Rdz(1,0) = sin(a); Rdz(1,1) = cos(a); Rdz(1,2) = 0;
  Rdz(2,0) = 0; Rdz(2,1) = 0; Rdz(2,2) = 1;

  Rdy(0,0) = cos(b); Rdy(0,1) = 0; Rdy(0,2) = sin(b);
  Rdy(1,0) = 0; Rdy(1,1) = 1; Rdy(1,2) = 0;
  Rdy(2,0) = -sin(b); Rdy(2,1) = 0; Rdy(2,2) = cos(b);

  Rdx(0,0) = 1; Rdx(0,1) = 0; Rdx(0,2) = 0;
  Rdx(1,0) = 0; Rdx(1,1) = cos(y); Rdx(1,2) = -sin(y);
  Rdx(2,0) = 0; Rdx(2,1) = sin(y); Rdx(2,2) = cos(y);

  // Rd = Rdy;
  Rd = Rdz*Rdy*Rdx;
//
//  Serial.print("(");
//  Serial.print(Rd(0,0)); Serial.print(", "); Serial.print(Rd(0,1)); Serial.print(", "); Serial.print(Rd(0,2)); Serial.println(",");
//  Serial.print(Rd(1,0));Serial.print(", "); Serial.print(Rd(1,1)); Serial.print(", "); Serial.print(Rd(1,2)); Serial.println(",");
//  Serial.print(Rd(2,0));Serial.print(", "); Serial.print(Rd(2,1)); Serial.print(", "); Serial.print(Rd(2,2)); Serial.println(")");

  double K = 10;
  //double K = 2;

  //Matrix3d Rd = MatrixXd::Identity(3,3);
  //Rd = MatrixXd::Identity(3,3);

  y = M_PI;
  Rdx(0,0) = 1; Rdx(0,1) = 0; Rdx(0,2) = 0;
  Rdx(1,0) = 0; Rdx(1,1) = cos(y); Rdx(1,2) = -sin(y);
  Rdx(2,0) = 0; Rdx(2,1) = sin(y); Rdx(2,2) = cos(y);

  y = des_roll;
  // fixed example
  //  quat.w() = .9134; quat.x() =.1736; quat.y() =.3458; quat.z() =.1264;

  // New method to generate R, convert quats to EUL, then flip, then convert to rotation matrix

  // eul(0) = - eul(0); //yaw
  // eul(1) = - eul(1); //pitch
  // eul(2) = eul(2); //roll

  // eul2quatZYX(eul, quat);

  quat2rotm(quat, R);
  Matrix3d R_tmp = R;
  // R = Rdx*R*Rdx;

  tmp_3d = .5*(Rd.transpose()*R-R.transpose()*Rd);
  vee(tmp_3d,eR);

  // eR[0] = eR[0];
  // eR[1] = -eR[1];
  // eR[2] = -eR[2];

  Od = -K*eR;
  // omega = {rates_f[0],-rates_f[1],-rates_f[2]};
  omega = {.1,-.25,0.};
  eO = omega - R.transpose()*Rd*Od;

  double C = 200./360.*2*M_PI;
  // rates_des = -eO;
  rates_des = Od;

  //yaw rates
  rates_des[2] -= 4*((channels[3]-172)/(1811.-172)*2-1);
  // rates_des[0] = -K * roll_error;
  // rates_des[1] = -K * pitch_error;
  // rates_des[2] = des_yaw_rate;

  cmd;
  for (auto i = 0; i < 3; i++)
  {
    if (rates_des[i] > 0)
      cmd[i] = rates_des[i]/(.7*rates_des[i] + C);
    else
      cmd[i] = rates_des[i]/(.7*-1*rates_des[i] + C);
  }



//  delay(10);


  float des_throttle = (channels[0] - 172.)/((1811.-172.));
  float des_roll = fmin(fmax(cmd[0],-1),1);
  float des_pitch = fmin(fmax(cmd[1],-1),1);
  float des_yaw = fmin(fmax(-cmd[2],-1),1);
  // now convert to channel outputs

  des_channels[0] = channels[0];//(des_throttle*(1811-172))+172;
  des_channels[1] = ((des_roll+1)*((1811-172)/2))+172;
  des_channels[2] = ((des_pitch+1)*((1811-172)/2))+172;
  des_channels[3] = ((des_yaw+1)*((1811-172)/2))+172;
  // des_channels[1] = channels[1];
  // des_channels[2] = channels[2];
  // des_channels[3] = channels[3];


  for(int i = 4; i < 8;i++) {
    des_channels[i] = channels[i];
  }

  //need to make sure that FC is in ACRO mode
  des_channels[5] = 1700;

  //add hover
  double height_ = pos_world[2];
  double heightDot_ = vel_world[2];
  if (use_opti)
  {
    height_ =  lpassZ->data_.y;
    heightDot_ =  lpassZ->data_.yDot;
    saturateInPlace(heightDot_, -1.0, 1.0);
  }

  int tNow_ = micros();

  if(hover_)
  {
    if(integrator_initialized_==0)
    {
      uzInt_ = hoverThrust_;
      integrator_initialized_ = true;
      hover_height_ = height_;
    }
    else
    {
      if (heightDot_ < 0.2)
      {
        uzInt_ += KiVz_*1e-6*(tNow_ - CTRLtnm1_)*(hover_height_ - height_);
        saturateInPlace(uzInt_,hoverThrust_,hoverThrust_+0.05);
      }
    }
  }
  else
  {
    integrator_initialized_ = false;
  }
  // double uz = KpVz_*vzError + uzInt_/zBodyInWorld(2);
  double zB = zBodyInWorld(quat_state);
  if (zB < 0)
    zB = 2;
  else
    saturateInPlace(zB,.5,1);


  // double uz = des_throttle;
  if (hover_)
  {
    double pTerm = KpVz_*(hover_height_ - height_) ;
    double dTerm = - KdVz_*heightDot_;
    saturateInPlace(pTerm,-.1,.1); //.25 .25
    saturateInPlace(dTerm,-.1,.1); //.25 .25
    // saturateInPlace(pTerm,-.05,.05); // for indoor
    // saturateInPlace(dTerm,-.05,.05); // for indoor
    uz = uzInt_/zB + pTerm + dTerm;
    // saturateInPlace(uz,0,hoverThrust_ + .25);
    saturateInPlace(uz,0,hoverThrust_ + .1);
    des_channels[0] = uz*(1811-172)+172;
    // uz = KpVz_*(hover_height_ - height_);
  }
  if (barrier_)
  {
    for (int i = 0; i < 3; i++)
      {
        // rates_des[i] = ((channels[i+1]-172)/(1811.-172)*2-1);
        // rates_des[i] = -200.*rates_des[i]*(1./(-1.+(fabs(rates_des[i])*.7)))/360.*2.*M_PI;
      }
    // rates_des[2] = -rates_des[2];
    // uz = (channels[0] - 172.)/((1811.-172.));
  }

  u_des[0] = uz;
  u_des[1] = rates_des[0];
  u_des[2] = rates_des[1];
  u_des[3] = rates_des[2];

  //IMU orientation with optitrack yaw
  //remove imu yaw

  // for (int i = 0; i < 4; i++)
  //   quats_f_opti_yaw[i] = (double) quats_f[i];
  // Quaterniond tmp_quat;
  // Vector3d tmp_eul;
  // tmp_quat.w() = (double) quats_f[3];
  // tmp_quat.x() = (double) quats_f[0];
  // tmp_quat.y() = (double) quats_f[1];
  // tmp_quat.z() = (double) quats_f[2];
  // quat2eulZYX(tmp_quat,tmp_eul);
  // tmp_eul[0] = 0; tmp_eul[1] = 0; tmp_eul[2] = -tmp_eul[2];
  // double quat_tmp[4];
  // eul2quatZYX(tmp_eul,tmp_quat);
  // quat_tmp[0] = tmp_quat.w();
  // quat_tmp[1] = tmp_quat.x();
  // quat_tmp[2] = tmp_quat.y();
  // quat_tmp[3] = tmp_quat.z();
  //
  // double quat_math[4];
  // double quats_f_rotated[4];
  // quats_f_rotated[0] = (double) quats_f[3];
  // quats_f_rotated[1] = (double) quats_f[0];
  // quats_f_rotated[2] = (double) quats_f[1];
  // quats_f_rotated[3] = (double) quats_f[2];
  // quatmultiply(quat_tmp,quats_f_rotated,quat_math);
  // quats_f_opti_yaw[0] = (float) quat_math[1];
  // quats_f_opti_yaw[1] = (float) quat_math[2];
  // quats_f_opti_yaw[2] = (float) quat_math[3];
  // quats_f_opti_yaw[3] = (float) quat_math[0];
  //
  //
  // //add opti yaw
  // tmp_eul(0) = 0; tmp_eul(1) = 0; tmp_eul(2) = yaw_f;
  // eul2quatZYX(tmp_eul,tmp_quat);
  // quat_tmp[0] = tmp_quat.w();
  // quat_tmp[1] = tmp_quat.x();
  // quat_tmp[2] = tmp_quat.y();
  // quat_tmp[3] = tmp_quat.z();
  // quats_f_rotated[0] = quats_f_opti_yaw[3];
  // quats_f_rotated[1] = quats_f_opti_yaw[0];
  // quats_f_rotated[2] = quats_f_opti_yaw[1];
  // quats_f_rotated[3] = quats_f_opti_yaw[2];
  // quatmultiply(quat_tmp,quats_f_rotated,quat_math);
  // quats_f_opti_yaw[0] = (float) quat_math[1];
  // quats_f_opti_yaw[1] = (float) quat_math[2];
  // quats_f_opti_yaw[2] = (float) quat_math[3];
  // quats_f_opti_yaw[3] = (float) quat_math[0];

  Vector3d lla_tmp;
  Vector3d lla_current = {pos_world[0],pos_world[1],pos_world[2]};
  lla2x_aiden(lla_current,lla_ref,lla_tmp);


  double x0[13] = {lla_tmp[0],lla_tmp[1],lla_tmp[2],
                  quat_state[0],quat_state[1],quat_state[2],quat_state[3],
                  vel_world[0],vel_world[1],vel_world[2],
                  rates_f[0],rates_f[1],rates_f[2]};
  if (use_opti)
  {
    x0[0] = lpassX->data_.y;
    x0[1] = lpassY->data_.y;
    x0[2] = lpassZ->data_.y;
    x0[3] = quat_state[0];
    x0[4] = quat_state[1];
    x0[5] = quat_state[2];
    x0[6] = quat_state[3];
    x0[7] = lpassX->data_.yDot;
    x0[8] = lpassY->data_.yDot;
    x0[9] = lpassZ->data_.yDot;
    double tmp_v = lpassX->data_.yDot;
    saturateInPlace(tmp_v,-1.0,1.0);
    x0[7] = tmp_v;
    tmp_v = lpassY->data_.yDot;
    saturateInPlace(tmp_v,-1.0,1.0);
    x0[8] = tmp_v;
    tmp_v = lpassZ->data_.yDot;
    saturateInPlace(tmp_v,-1.0,1.0);
    x0[9] = tmp_v;
    x0[10] = rates_f[0];
    x0[11] = rates_f[1];
    x0[12] = rates_f[2];
  }


  double x[13];
  memcpy(x,x0,13*sizeof(double));
  if (barrier_)
  {
    barrier_t1 = micros();
    barrier(x,u_des,u,h,lambda);
    barrier_t2 = micros();

    for (auto i = 0; i < 3; i++)
    {
      if (u[i+1] > 0)
        cmd[i] = u[i+1]/(.7*u[i+1] + C);
      else
        cmd[i] = u[i+1]/(.7*-1*u[i+1] + C);
    }

    des_roll = fmin(fmax(cmd[0],-1),1);
    des_pitch = fmin(fmax(cmd[1],-1),1);
    des_yaw = fmin(fmax(-cmd[2],-1),1);
    des_channels[0] = u[0]*(1811-172)+172;
    des_channels[1] = ((des_roll+1)*((1811-172)/2))+172;
    des_channels[2] = ((des_pitch+1)*((1811-172)/2))+172;
    des_channels[3] = ((des_yaw+1)*((1811-172)/2))+172;

    for (int i = 0; i < 8; i++)
    {
      des_channels_barrier[i] = des_channels[i];
    }
  }
  CTRLtnm1_ = micros();


  // if(hover_)
  // {
  //   if(integrator_initialized_==0)
  //   {
  //     CTRLtnm1_ = micros();
  //     uz = 0;
  //     integrator_initialized_ = true;
  //   }
  //   else
  //   {
  //     if ((tNow_ - CTRLtnm1_) > 50e4)
  //     {
  //       uz += 0.05;
  //       CTRLtnm1_ = tNow_;
  //     }
  //   }
  // }
  // else
  // {
  //   integrator_initialized_ = false;
  // }
  // if (uz > 1.0)
  //   uz = 0.0;
  // des_channels[0] = uz*(1811-172)+172;
  sbus_t2 = micros()-sbus_t1;
  if (max_sbus_t < sbus_t2)
    max_sbus_t = sbus_t2;

  if ((millis() - lastTime2) > 100)
  {
    // Serial.print("quat: (");
    // Serial.print(quats_f[0]); Serial.print(", ");
    // Serial.print(quats_f[1]); Serial.print(", ");
    // Serial.print(quats_f[2]); Serial.print(", ");
    // Serial.print(quats_f[3]); Serial.println(") ");
    //
    // Serial.print("R_tmp: (");
    // Serial.print(R_tmp(0,0)); Serial.print(", "); Serial.print(R_tmp(0,1)); Serial.print(", "); Serial.print(R_tmp(0,2)); Serial.println(",");
    // Serial.print(R_tmp(1,0));Serial.print(", "); Serial.print(R_tmp(1,1)); Serial.print(", "); Serial.print(R_tmp(1,2)); Serial.println(",");
    // Serial.print(R_tmp(2,0));Serial.print(", "); Serial.print(R_tmp(2,1)); Serial.print(", "); Serial.print(R_tmp(2,2)); Serial.println(")");
    //
    // Serial.print("R: (");
    // Serial.print(R(0,0)); Serial.print(", "); Serial.print(R(0,1)); Serial.print(", "); Serial.print(R(0,2)); Serial.println(",");
    // Serial.print(R(1,0));Serial.print(", "); Serial.print(R(1,1)); Serial.print(", "); Serial.print(R(1,2)); Serial.println(",");
    // Serial.print(R(2,0));Serial.print(", "); Serial.print(R(2,1)); Serial.print(", "); Serial.print(R(2,2)); Serial.println(")");
    // //
    // Serial.print("Rd: (");
    // Serial.print(Rd(0,0)); Serial.print(", "); Serial.print(Rd(0,1)); Serial.print(", "); Serial.print(Rd(0,2)); Serial.println(",");
    // Serial.print(Rd(1,0));Serial.print(", "); Serial.print(Rd(1,1)); Serial.print(", "); Serial.print(Rd(1,2)); Serial.println(",");
    // Serial.print(Rd(2,0));Serial.print(", "); Serial.print(Rd(2,1)); Serial.print(", "); Serial.print(Rd(2,2)); Serial.println(")");
    //
    // Serial.print("tmp_3d: (");
    // Serial.print(tmp_3d(0,0)); Serial.print(", "); Serial.print(tmp_3d(0,1)); Serial.print(", "); Serial.print(tmp_3d(0,2)); Serial.println(",");
    // Serial.print(tmp_3d(1,0));Serial.print(", "); Serial.print(tmp_3d(1,1)); Serial.print(", "); Serial.print(tmp_3d(1,2)); Serial.println(",");
    // Serial.print(tmp_3d(2,0));Serial.print(", "); Serial.print(tmp_3d(2,1)); Serial.print(", "); Serial.print(tmp_3d(2,2)); Serial.println(")");
    //
    // Serial.print("heading:"); Serial.print(yaw_f);
    // Serial.print("Glob Vel: ("); Serial.print(global_vel(0)); Serial.print(",");
    // Serial.print(global_vel(1)); Serial.print(",");
    // Serial.print(global_vel(2)); Serial.print(")");
    // Serial.print("Body Vel: ("); Serial.print(body_vel(0)); Serial.print(",");
    // Serial.print(body_vel(1)); Serial.print(",");
    // Serial.print(body_vel(2)); Serial.print(")");
    // Serial.print("Des angles: (");
    // Serial.print(y); Serial.print(",");
    // Serial.print(b); Serial.print(",");
    // Serial.println(a);
    // Serial.print("angular rates: (");
    // Serial.print(omega[0]); Serial.print(",");
    // Serial.print(omega[1]); Serial.print(",");
    // Serial.print(omega[2]);
    // Serial.print(") Desired rates: (");
    // Serial.print(rates_des[0]); Serial.print(",");
    // Serial.print(rates_des[1]); Serial.print(",");
    // Serial.print(rates_des(2  ));
    // Serial.print(") Commands: (");
    // Serial.print(cmd[0]); Serial.print(",");
    // Serial.print(cmd[1]); Serial.print(",");
    // Serial.println(cmd[2]); //Serial.print(") C: "); Serial.println(C);
    //
    // Serial.print("Height: "); Serial.print(height_);
    // Serial.print("uzint: "); Serial.print(uzInt_);
    // Serial.print("uz: "); Serial.print(uz);
    // Serial.print("channel: "); Serial.println(des_channels[0]);

    // Serial.print("u_des: (");
    // for (int i = 0; i < 4; i++)
    //   {Serial.print(u_des[i]); Serial.print(",");}
    // Serial.print(") u: (");
    // for (int i = 0; i < 4; i++)
    //   {Serial.print(u[i]); Serial.print(",");}
    // Serial.print(") channels: (");
    // for (int i = 0; i < 4; i++)
    //   {Serial.print(des_channels[i]); Serial.print(",");}


    Quaterniond tmp_quat; Vector3d tmp_eul;
    tmp_quat.w() = quat_state[0]; tmp_quat.x() = quat_state[1]; tmp_quat.y() = quat_state[2]; tmp_quat.z() = quat_state[3];
    quat2eulZYX(tmp_quat,tmp_eul);
    Serial.print(") uz: (");
    Serial.print(uz);
    Serial.print(") h: (");
    Serial.print(height_);
    Serial.print(") hDot: (");
    Serial.print(heightDot_);
    Serial.print(") x: (");
    for (int i = 0; i < 3; i++)
      {Serial.print(x0[i]); Serial.print(",");}
    Serial.print(") v: (");
    // for (int i = 0; i < 3; i++)
    //   {Serial.print(vel_world[i]); Serial.print(",");}
    for (int i = 0; i < 3; i++)
      {Serial.print(x0[i+7]); Serial.print(",");}
    Serial.print(") vel body: (");

    // for (int i = 0; i < 3; i++)
    //   {Serial.print(vel_body[i]); Serial.print(",");}
    Serial.print(") eul: (");

    for (int i = 0; i < 3; i++)
      {Serial.print(tmp_eul[i]); Serial.print(",");}
    Serial.print(")");

    tmp_quat.w() = quats_f[3]; tmp_quat.x() = quats_f[0]; tmp_quat.y() = quats_f[1]; tmp_quat.z() = quats_f[2];
    quat2eulZYX(tmp_quat,tmp_eul);
    Serial.print(") eul(old): (");

    for (int i = 0; i < 3; i++)
      {Serial.print(tmp_eul[i]); Serial.print(",");}
    Serial.println(")");

    // Serial.print("barrier time (us): "); Serial.println(barrier_t2 - barrier_t1);
    Serial.print("sbus time (us): "); Serial.println(sbus_t2);
    Serial.print("max sbus time (us): "); Serial.println(max_sbus_t);
    // Serial.print("imu time (us): "); Serial.println(imu_t2);


  lastTime2 = millis();
  }

}

inline void saturateInPlace(double &in, double min, double max)
{
    if(min>max)
    {
        double tmp = max;
        max = min;
        min = tmp;
    }

    if(in>max)
        in = max;
    else if(in<min)
        in = min;
}

inline void saturateInPlace(float &in, double min, double max)
{
    if(min>max)
    {
        double tmp = max;
        max = min;
        min = tmp;
    }

    if(in>max)
        in = max;
    else if(in<min)
        in = min;
}

void rad2Command(float rads) {

  // convert to degrees
  float degs = rads * (360/(2*3.1415926535));


}


void vee(Matrix3d &in, Vector3d &out)
{
  out[0] = in(2,1);
  out[1] = in(0,2);
  out[2] = in(1,0);
}

void quat2yaw(float *quats_f, double *yaw)
{


  Quaterniond q;
  q.w() = quats_f[3]; q.x() = quats_f[0]; q.y() = quats_f[1]; q.z() = quats_f[2];
//  yaw[0] = atan2(2.0*(quat[1]*quat[2] + quat[3]*quat[0]), quat[3]*quat[3] - quat[0]*quat[0] - quat[1]*quat[1] + quat[2]*quat[2]);
  //yaw[0] = atan2(2.0*(q[0]*q[1] + q[3]*q[2]), q[3]*q[3] + q[0]*q[0] - q[1]*q[1] - q[2]*q[2]);
  double eul_tmp = 2.0*(q.y()*q.y());
  yaw[0] = atan2(2.0 * q.w() * q.z() + 2.0 * q.x() * q.y(), (1.0 - eul_tmp) - 2.0 * (q.z() * q.z()));
  //Serial.println(yaw[0]);
}

inline void quat2eulZYX(const Quaterniond &q,
                              Vector3d &eul)
{
	double eul_tmp = 2.0 * (q.y() * q.y());
	eul(0) = atan2(2.0 * q.w() * q.x() + 2.0 * q.y() * q.z(), (1.0 - 2.0 * (q.x() * q.x())) - eul_tmp);

	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (abs(sinp) >= 1)
		eul(1) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		eul(1) = asin(sinp);

	eul(2) = atan2(2.0 * q.w() * q.z() + 2.0 * q.x() * q.y(), (1.0 - eul_tmp) - 2.0 * (q.z() * q.z()));
}

inline void eul2quatZYX(const Eigen::Vector3d &eul,
                              Eigen::Quaterniond &q)
{
	double cy = cos(eul(2) * 0.5);
	double sy = sin(eul(2) * 0.5);
	double cp = cos(eul(1) * 0.5);
	double sp = sin(eul(1) * 0.5);
	double cr = cos(eul(0) * 0.5);
	double sr = sin(eul(0) * 0.5);

	q.w() = cy * cp * cr + sy * sp * sr;
	q.x() = cy * cp * sr - sy * sp * cr;
	q.y() = sy * cp * sr + cy * sp * cr;
	q.z() = sy * cp * cr - cy * sp * sr;
}

inline void quat2rotm(const Quaterniond &q,
Matrix3d &rotm)
{
    rotm(0,0) = 1-2*q.y()*q.y()-2*q.z()*q.z();
    rotm(0,1) = 2*q.x()*q.y()-2*q.z()*q.w();
    rotm(0,2) = 2*q.x()*q.z()+2*q.y()*q.w();
    rotm(1,0) = 2*q.x()*q.y()+2*q.z()*q.w();
    rotm(1,1) = 1-2*q.x()*q.x()-2*q.z()*q.z();
    rotm(1,2) = 2*q.y()*q.z()-2*q.x()*q.w();
    rotm(2,0) = 2*q.x()*q.z()-2*q.y()*q.w();
    rotm(2,1) = 2*q.y()*q.z()+2*q.x()*q.w();
    rotm(2,2) = 1-2*q.x()*q.x()-2*q.y()*q.y();
}

void readGPS() {
  if (myGPS.getPVT()) {
    latitude = myGPS.getLatitude();
    longitude = myGPS.getLongitude();
    altitude = myGPS.getAltitude();
    SIV = myGPS.getSIV();
    PDOP = myGPS.getPDOP();
    northVel = myGPS.getNedNorthVel();
    eastVel = myGPS.getNedEastVel();
    downVel = myGPS.getNedDownVel();

    isGPSAvailable = 1;
  }
}

void readXbee() {

  // print out the number of messages per seccond

  if(millis() - xbeeT > 1000) {
    xbeeT = millis();
    Serial.println(xbeeCount);
    xbeeCount = 0;
  }

  if(Serial8.available() > 0) {
    //Serial8.readBytes(message, 20);

    //read the header to determine package size
    Serial8.readBytes(header, 2);
    // position information
    if(header[0] == 49 && header[1] == 50) {
        Serial8.readBytes(message,18);

        if(message[16] == 60 && message[17] == 59) {
          for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++)
            {
              tmp.c[j] = message[j+i*4];
            }
            pos_f[i] = tmp.f;
            if(i == 0) {
              lpassX->update(micros(),(float) pos_f[i]);
              global_vel(0) = lpassX->data_.yDot;
            }
            if(i == 1) {
              lpassY->update(micros(),(float) pos_f[i]);
              global_vel(1) = lpassY->data_.yDot;
            }
            if(i == 2) {
              lpassZ->update(micros(),(float) pos_f[i]);
              global_vel(2) = lpassZ->data_.yDot;
            }
          }

          for (int i = 0; i < 4; i++) {tmp.c[i] = message[12+i];}
          yaw_f = tmp.f;
          lpassYaw->update((float) (double)(micros())*1e-6,(float) yaw_f);
          //Serial.println("message recvied");
        }
        else {
          Serial.println("badxbee data");
          // do we need to clear the buffer here ?
        }
    }  //velocity and heading command
    else if(header[0] == 51 && header[1] == 52) {
      Serial8.readBytes(message, 10);

      if(message[8] == 62 && message[9] == 61) {

        for (int j = 0; j < 4; j++) {
          tmp.c[j] = message[j];
        }
        des_vel(0) = tmp.f;

        for (int j = 4; j < 8; j++) {
          tmp.c[j-4] = message[j];
        }
        des_yaw_rate = -tmp.f;

        //Serial.println(des_vel(0));

      }
      else {
        Serial.println("bad xbee data");
      }


    }

    // Serial8.readBytes(message, 20);
    // if ((message[0] == 49 && message[1] == 50 && message[18] == 60 && message[19] == 59))
    // {
    // // good xbee data

    // for (int i = 0; i < 3; i++)
    // {
    //   for (int j = 0; j < 4; j++)
    //   {
    //     tmp.c[j] = message[j+i*4+2];
    //   }
    //   pos_f[i] = tmp.f;
    //   if(i == 0) {
    //     lpassX->update((float) (double)(micros())*1e-6,(float) pos_f[i]);
    //   }
    //   if(i == 1) {
    //     lpassY->update((float) (double)(micros())*1e-6,(float) pos_f[i]);
    //   }
    //   if(i == 2) {
    //     lpassZ->update((float) (double)(micros())*1e-6,(float) pos_f[i]);
    //   }
    // }




    isXbeeAvailable = 1;
    xbeeCount++;

    // convert from global to local velocity
    float a = yaw_f;
    Matrix3d Rdz;
    Rdz(0,0) = cos(a); Rdz(0,1) = -sin(a); Rdz(0,2) = 0;
    Rdz(1,0) = sin(a); Rdz(1,1) = cos(a); Rdz(1,2) = 0;
    Rdz(2,0) = 0; Rdz(2,1) = 0; Rdz(2,2) = 1;
    //convert quat 2

    body_vel = Rdz*global_vel;

    //print filtered velcity data

    // Serial.print(lpassX->data_.yDot); Serial.print(",");
    // Serial.print(lpassY->data_.yDot); Serial.print(",");
    // Serial.print(lpassZ->data_.`yDot`); Serial.print(",");
    // Serial.print(lpassYaw->data_.yDot); Serial.println("");


    // Serial.print("RAW: ");
    // Serial.print(lpassX->data_.yDot); Serial.print(",");
    // Serial.print(lpassY->data_.yDot); Serial.print(",");
    // Serial.print(lpassZ->data_.yDot); Serial.print(",");
    // Serial.print(lpassYaw->data_.yDot); Serial.println("");

    // Serial.println(pos_f[0]);
//      Serial.print(message[0]); Serial.print(", ");
//      Serial.print(message[1]); Serial.print(", ");
//      Serial.print(message[18]); Serial.print(", ");
//      Serial.println(message[19]);
//        Serial.println("bad xbee data");



  }
}


void initIMU()
{
  Serial.println("Initializing VN-100 Serial (115200)");
  Serial1.begin(115200);
  delay(100);
  Serial.println("turning off output!!!!!!!");
  Serial1.print("$VNASY,0*XX\r\n");
  delay(100);
  Serial.println("resetting");
  Serial1.print("$VNWRG,06,0*XX\r\n");
  delay(100);
  Serial.println("Setting output packet rate");
  // Serial1.print("$VNWRG,75,2,1,01,0530*XX\r\n");
  Serial1.print("$VNWRG,75,2,2,21,00F0,0008*XX\r\n");
  delay(100);
  Serial.println("Setting baud rate");
  Serial1.print("$VNWRG,05,921600*XX\r\n");
  delay(100);
  Serial1.flush();
  delay(100);

  Serial.println("Initializing VN-100 Serial (921600)");
  Serial1.begin(921600);
  delay(100);
  Serial1.print("$VNRST*4D\r\n");
  delay(3000);
  Serial.println("turning off output");
  Serial1.print("$VNASY,0*XX\r\n");
  delay(100);
  Serial.println("resetting");
  Serial1.print("$VNWRG,06,0*XX\r\n");
  delay(100);
  Serial.println("Setting output packet rate");
  // Serial1.print("$VNWRG,75,2,1,01,0530*XX\r\n");
  Serial1.print("$VNWRG,75,2,2,21,00F0,0008*XX\r\n");
  delay(100);
  Serial.println("starting");
  Serial1.print("$VNASY,1*XX\r\n");
  delay(100);
  while (Serial1.available() > 0) {
    Serial1.read();
  }
  Serial.println("Initialized VN-100 Serial");
}


void readXbee_thread() {
  while (1) {
    Threads::Mutex mylock;
    mylock.lock();
    xbee_t1 = micros();
    readXbee();
    xbee_t2 = micros()-xbee_t1;
    mylock.unlock();
    threads.yield();
  }

}

void readSBUS_thread()
{
  while (1)
  {
    Threads::Mutex mylock;
    mylock.lock();
    sbus_t1 = micros();
    readSBUS();
    mylock.unlock();
    threads.yield();
  }
}

void readGPS_thread()
{
  while (1)
  {
    readGPS();
    threads.yield();
  }
}

void readIMU_thread()
{
  while (1)
  {
    Threads::Mutex mylock;
    mylock.lock();
    imu_t1 = micros();
    readIMU();
    imu_t2 = micros()-imu_t1;
    mylock.unlock();
    threads.yield();
  }
}

unsigned short calculate_imu_crc(byte data[], unsigned int length)
  {
    unsigned int i;
    unsigned short crc = 0;
    for(i=0; i<length; i++){
      crc = (byte)(crc >> 8) | (crc << 8);
      crc ^= data[i];
      crc ^= (byte)(crc & 0xff) >> 4;
      crc ^= crc << 12;
      crc ^= (crc & 0x00ff) << 5;
    }
    return crc;
  }

int check_sync_byte(void)
  {
    for (int i = 0; i < 6; i++) {
      Serial1.readBytes(in, 1);
      if (in[0] == 0xFA) {
        return 1;
      }
    }
    return 0;
  }

void read_imu_data_100(void) {
    Serial1.readBytes(in, 65);

    checksum.b[0] = in[64];
    checksum.b[1] = in[63];

    // uint8_t euler[12]; rpy
    uint8_t quat[16];
    uint8_t acc[12];
    uint8_t rates[12];
    uint8_t mag[12];
    uint8_t temp[4];
    uint8_t pres[4];

    if (calculate_imu_crc(in, 63) == checksum.s) {
      for (int i = 0; i < 4; i++) {
        quat[i] = in[3 + i];
        quat[4+i] = in[7 + i];
        quat[8+i] = in[11 + i];
        quat[12+i] = in[15 + i];
        rates[i] = in[19 + i];
        rates[4+i] = in[23 + i];
        rates[8+i] = in[27 + i];
        acc[i] = in[31 + i];
        acc[4+i] = in[35 + i];
        acc[8+i] = in[39 + i];
        mag[i] = in[43 + i];
        mag[i+4] = in[47 + i];
        mag[i+8] = in[51 + i];
        temp[i] = in[55 + i];
        pres[i] = in[59 + i];
      }
      isIMUAvailable = 1;
    }
    else {
      bad_imu_chksum++;
      //Serial.print("bad imu chksums: "); Serial.println(bad_imu_chksum);
      return;
    }

    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        tmp.c[j] = quat[j+i*4];
      }
      quats_f[i] = tmp.f;
    }

    if (tared)
    {
      double quat_tmp[4];
      for (int i = 0; i < 4; i++)
        quat_tmp[i] = (double) quat_yaw0[i];
      double quat_math[4];
      double quats_f_rotated[4];
      quats_f_rotated[0] = (double) quats_f[3];
      quats_f_rotated[1] = (double) quats_f[0];
      quats_f_rotated[2] = (double) quats_f[1];
      quats_f_rotated[3] = (double) quats_f[2];
      quatmultiply(quat_tmp,quats_f_rotated,quat_math);
      quats_f[0] = (float) quat_math[1];
      quats_f[1] = (float) quat_math[2];
      quats_f[2] = (float) quat_math[3];
      quats_f[3] = (float) quat_math[0];
    }

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        tmp.c[j] = rates[j+i*4];
      }
      rates_f[i] = tmp.f;
    }
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        tmp.c[j] = acc[j+i*4];
      }
      acc_f[i] = tmp.f;

    }
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        tmp.c[j] = mag[j+i*4];
      }
      mag_f[i] = tmp.f;
    }

    for (int j = 0; j < 4; j++)
      {
        tmp.c[j] = temp[j];
      }
      temp_f = tmp.f;

    for (int j = 0; j < 4; j++)
      {
        tmp.c[j] = pres[j];
      }
      pres_f = tmp.f;

  }

  void read_imu_data_200(void) {
      Serial1.readBytes(in, 83);

      checksum.b[0] = in[82];
      checksum.b[1] = in[81];

      // uint8_t euler[12]; rpy
      uint8_t quat[16];
      // uint8_t acc[12];
      uint8_t rates[12];
      uint8_t pos[24];
      uint8_t velned[12];
      uint8_t velbody[12];

      if (calculate_imu_crc(in, 81) == checksum.s) {
        for (int i = 0; i < 4; i++) {
          quat[i] = in[2+3 + i];
          quat[4+i] = in[2+7 + i];
          quat[8+i] = in[2+11 + i];
          quat[12+i] = in[2+15 + i];
          rates[i] = in[2+19 + i];
          rates[4+i] = in[2+23 + i];
          rates[8+i] = in[2+27 + i];
          pos[i] = in[2+31 + i];
          pos[4+i] = in[2+35 + i];
          pos[8+i] = in[2+39 + i];
          pos[12+i] = in[2+43 + i];
          pos[16+i] = in[2+47 + i];
          pos[20+i] = in[2+51 + i];
          velned[i] = in[2+55 + i];
          velned[4+i] = in[2+59 + i];
          velned[8+i] = in[2+63 + i];
          velbody[i] = in[2+67 + i];
          velbody[4+i] = in[2+71 + i];
          velbody[8+i] = in[2+75 + i];
        }
        isIMUAvailable = 1;
      }
      else {
        bad_imu_chksum++;
        // Serial.print("bad imu chksums: "); Serial.println(bad_imu_chksum);
        return;
      }

      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          tmp.c[j] = quat[j+i*4];
        }
        quats_f[i] = tmp.f;
      }

      //rotate quats
      double quats_f_rotated[4] = {quats_f[3],quats_f[0],quats_f[1],quats_f[2]};
      quatmultiply(reference_quat_no_yaw,quats_f_rotated,quat_state);
      double drew[4];
      memcpy(drew,quat_state,4*sizeof(double));
      quatmultiply(drew,reference_quat,quat_state);
      if (use_opti)
      {
        Quaterniond quat_state_d, quat_opti_yaw_d, quat_opti_state_inv_d;
        quat_state_d.w() = quat_state[0];
        quat_state_d.x() = quat_state[1];
        quat_state_d.y() = quat_state[2];
        quat_state_d.z() = quat_state[3];
        Vector3d eul_state_d, eul_opti_state_inv_d, eul_opti_yaw_d;
        quat2eulZYX(quat_state_d,eul_state_d);

        eul_opti_state_inv_d.setZero(); eul_opti_state_inv_d(2) = -eul_state_d(2);
        eul_opti_yaw_d.setZero(); eul_opti_yaw_d(2) = yaw_f;
        eul2quatZYX(eul_opti_state_inv_d,quat_opti_state_inv_d);
        eul2quatZYX(eul_opti_yaw_d,quat_opti_yaw_d);
        quat_state_d = quat_opti_yaw_d*quat_opti_state_inv_d*quat_state_d;
        quat_state[0] = quat_state_d.w();
        quat_state[1] = quat_state_d.x();
        quat_state[2] = quat_state_d.y();
        quat_state[3] = quat_state_d.z();
      }

      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          tmp.c[j] = rates[j+i*4];
        }
        rates_f[i] = tmp.f;
      }
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 8; j++)
        {
          tmp_8.c[j] = pos[j+i*8];
        }
        gps_pos_d[i] = tmp_8.d;

      }
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          tmp.c[j] = velned[j+i*4];
        }
        gps_vel_ned_f[i] = tmp.f;
      }
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 4; j++)
        {
          tmp.c[j] = velbody[j+i*4];
        }
        gps_vel_body_f[i] = tmp.f;
      }

      //rotate rates, pos, and vel
      Vector3d tmp;

      for (int i = 0; i < 3; i++)
        tmp[i] = rates_f[i];
      tmp = reference_R*tmp;
      for (int i = 0; i < 3; i++)
        rates_state[i] = tmp[i];

      for (int i = 0; i < 3; i++)
        pos_world[i] = gps_pos_d[i];

      for (int i = 0; i < 3; i++)
        tmp[i] = gps_vel_body_f[i];
      tmp = reference_R.transpose()*tmp;
      for (int i = 0; i < 3; i++)
        vel_body[i] = tmp[i];

      vel_world[0] = gps_vel_ned_f[0];
      vel_world[1] = -gps_vel_ned_f[1];
      vel_world[2] = -gps_vel_ned_f[2];
    }

int readIMU(void)
{
  int check = 0;
  if (Serial1.available() > 4) {
    check = check_sync_byte();
  }
  if (check == 1) {
    read_imu_data_200();

    return 1;
  }
  return 0;
}

int lla2x_aiden(const Eigen::Ref<const Eigen::Vector3d> lla,
                const Eigen::Ref<const Eigen::Vector3d> llaRef,
                Eigen::Ref<Eigen::Vector3d> nwu)
{

  //set z
  nwu(2) = lla(2)-llaRef(2);

  nwu(0) = M_PI/180*(lla(0)-llaRef(0))* 6378137.;
  nwu(1) = M_PI/180*(lla(1)-llaRef(1))*cos(llaRef(1)*2*M_PI/180)*6378137.;



}

constexpr static double earth_a = 6378137.;  // semi-major axis
constexpr static double earth_f = 1. / 298.257223563;  // flattening
constexpr static double earth_e2 = earth_f * (2. - earth_f);  // eccentricity squared
constexpr static double earth_b = earth_a * (1. - earth_f);  // semi-minor axis

/***************************************************************************
 * \brief Converts geographic latitude, longitude and altitude (lla) into
 * a north-west-up frame: frame with z axis normal to the WGS 84 ellipsoid
 * and with the x vector pointing toward the geographic north pole
 ***************************************************************************/
inline void lla2nwu(
  const Eigen::Ref<const Eigen::Vector3d> lla,
  const Eigen::Ref<const Eigen::Vector3d> llaRef,
  Eigen::Ref<Eigen::Vector3d> nwu)
{
  Eigen::Vector3d ecefRef;
  lla2ecef(llaRef, ecefRef);

  Eigen::Vector3d ecef;
  lla2ecef(lla, ecef);

  const double cosLatRef = cos(llaRef[0]);
  const double sinLatRef = sin(llaRef[0]);
  const double cosLongRef = cos(llaRef[1]);
  const double sinLongRef = sin(llaRef[1]);

  Eigen::Vector3d x;
  x[0] = -sinLatRef * cosLongRef;
  x[1] = -sinLatRef * sinLongRef;
  x[2] = cosLatRef;

  Eigen::Vector3d y;
  y[0] = sinLongRef;
  y[1] = -cosLongRef;
  y[2] = 0.;

  Eigen::Vector3d z;
  z[0] = cosLatRef * cosLongRef;
  z[1] = cosLatRef * sinLongRef;
  z[2] = sinLatRef;

  Eigen::Matrix3d RotEcef2Nwu;
  RotEcef2Nwu.row(0) = x;
  RotEcef2Nwu.row(1) = y;
  RotEcef2Nwu.row(2) = z;

  nwu = RotEcef2Nwu * (ecef - ecefRef);
}

inline void lla2ecef(
  const Eigen::Ref<const Eigen::Vector3d> lla,
  Eigen::Ref<Eigen::Vector3d> ecef)
{
  Eigen::Vector3d llr;
  geographic2geocentric(lla, llr);
  const double cosThetaPrime = cos(llr[0]);
  const double sinThetaPrime = sin(llr[0]);

  ecef[0] = llr[2] * cosThetaPrime * cos(llr[1]);
  ecef[1] = llr[2] * cosThetaPrime * sin(llr[1]);
  ecef[2] = llr[2] * sinThetaPrime;
}

inline void geographic2geocentric(
  const Eigen::Ref<const Eigen::Vector3d> lla,
  Eigen::Ref<Eigen::Vector3d> llr)
{
  const double theta = atan((tan(lla[0]) * earth_b * earth_b) / (earth_a * earth_a));
  const double cosTheta = cos(theta);
  const double sinTheta = sin(theta);

  const double r = 1. /
    sqrt(
    (cosTheta * cosTheta) / (earth_a * earth_a) + (sinTheta * sinTheta) /
    (earth_b * earth_b));

  const double thetaPrime =
    atan((r * sinTheta + lla[2] * sin(lla[0])) / (r * cosTheta + lla[2] * cos(lla[0])));

  const double rPrime = (r * cosTheta + lla[2] * cos(lla[0])) / cos(thetaPrime);

  llr[0] = thetaPrime;
  llr[1] = lla[1];
  llr[2] = rPrime;
}
