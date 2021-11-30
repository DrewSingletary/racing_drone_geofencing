#include "VectorNav.h"

union {float f; char c[4]; int32_t i; uint32_t u;} tmp;
union {double d; char c[8];} tmp_8;

namespace Cybercortex
{
  VectorNav::VectorNav(HardwareSerial &port,
                       const uint32_t &baud,
                       const uint32_t &receiveTimeout):
  port_(port),
  baud_(baud),
  receiveTimeout_(receiveTimeout)
  {}


  VectorNav::~VectorNav()
  {}

  int32_t VectorNav::init(bool resetSettings)
  {
    port_.begin(baud_);
    port_.setTimeout(receiveTimeout_);
    VectorNav::initSerial();
    Serial.println("VectorNav initialized!");
    return 1;
  }

  void VectorNav::reset(void)
  {
      delay(5000);
  }

  void VectorNav::initSerial(void)
  {
    Serial.println("Initializing VN-100 Serial (115200)");
    port_.begin(115200);
    delay(100);
    Serial.println("turning off output!!!!!!!");
    port_.print("$VNASY,0*XX\r\n");
    delay(100);
    Serial.println("resetting");
    port_.print("$VNWRG,06,0*XX\r\n");
    delay(100);
    Serial.println("Setting output packet rate");
    // port_.print("$VNWRG,75,2,1,01,0530*XX\r\n");
    port_.print("$VNWRG,75,2,2,21,00F0,0008*XX\r\n");
    delay(100);
    Serial.println("Setting baud rate");
    port_.print("$VNWRG,05,921600*XX\r\n");
    delay(100);
    port_.flush();
    delay(100);

    Serial.println("Initializing VN-100 Serial (921600)");
    port_.begin(921600);
    delay(100);
    port_.print("$VNRST*4D\r\n");
    delay(3000);
    Serial.println("turning off output");
    port_.print("$VNASY,0*XX\r\n");
    delay(100);
    Serial.println("resetting");
    port_.print("$VNWRG,06,0*XX\r\n");
    delay(100);
    Serial.println("Setting output packet rate");
    // port_.print("$VNWRG,75,2,1,01,0530*XX\r\n");
    port_.print("$VNWRG,75,2,2,21,00F0,0008*XX\r\n");
    delay(100);
    Serial.println("starting");
    port_.print("$VNASY,1*XX\r\n");
    delay(100);
    while (port_.available() > 0) {
      port_.read();
    }
    Serial.println("Initialized VN-100 Serial");
  }



  // Calculate the 16-bit CRC for the given ASCII or binary message.
  unsigned short VectorNav::calculate_imu_crc(byte data[], unsigned int length)
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

  int VectorNav::check_sync_byte(void)
  {
    for (int i = 0; i < 6; i++) {
      port_.readBytes(in, 1);
      if (in[0] == 0xFA) {
        return 1;
      }
    }
    return 0;
  }

  void VectorNav::read_imu_data(void) {
    port_.readBytes(in, 83);

    checksum.b[0] = in[82];
    checksum.b[1] = in[81];

    uint8_t quat[16];
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
    }
    else {
      bad_imu_chksum_++;
      // Serial.print("bad imu chksums: "); Serial.println(bad_imu_chksum);
      return;
    }

    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        tmp.c[j] = quat[j+i*4];
      }
      data_.quats[i] = tmp.f;
    }

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        tmp.c[j] = rates[j+i*4];
      }
      data_.rates[i] = tmp.f;
    }

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 8; j++)
      {
        tmp_8.c[j] = pos[j+i*8];
      }
      data_.pos[i] = tmp_8.d;
    }

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        tmp.c[j] = velned[j+i*4];
      }
      data_.vel_world[i] = tmp.f;
    }

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        tmp.c[j] = velbody[j+i*4];
      }
      data_.vel_body[i] = tmp.f;
    }
    return;
  }

  int32_t VectorNav::update(void)
  {
    int check = 0;
    if (port_.available() > 4) {
      check = check_sync_byte();
    }
    if (check == 1) {
      read_imu_data();
      return 1;
    }
    return 0;
  }

}
