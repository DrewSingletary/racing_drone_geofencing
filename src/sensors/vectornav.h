#ifndef VectorNav_h
#define VectorNav_h

#include "Arduino.h"
#include <vector>

namespace Cybercortex
{

  class VectorNav
  {
  public:
      enum class COMMAND : uint8_t
      {
      	CMD_READ = 0x01,
      	CMD_WRITE = 0x02,
      	CMD_FLASH = 0x03,
      	CMD_RESTORE = 0x04,
      	CMD_RESET = 0x06,
      	CMD_MAG_DISTURBANCE = 0x08,
      	CMD_ACCEL_DISTURBANCE = 0x09,
      	CMD_SET_GYRO_BIAS = 0x0C
      };

      struct data
      {
      	double rates[3];
        double quats[4];
        double acc[3];
        double pos[3];
        double vel_body[3];
        double vel_world[3];
      };

  public:
      data data_;
      int bad_imu_chksum_ = 0;
      VectorNav(HardwareSerial &port,
  	      const uint32_t &baud,
  	      const uint32_t &receiveTimeout);
      virtual ~VectorNav();

      void initSerial(void);
      virtual int32_t init(bool resetSettings = false);
      virtual int32_t update(void);
      void reset(void);
      void tare(void);

      elapsedMicros timeSinceTX;

      byte in[100];  // array to save data send from the IMU

      union {unsigned short s; byte b[2];} checksum;

  protected:
      HardwareSerial &port_;
      const uint32_t baud_;
      const int receiveTimeout_;

      bool newMsgCheck(void);
      int32_t getRawData(void);
      int32_t getFusionedData(void);

      void readMessage(const uint32_t &len, std::vector<uint8_t> &data);
      int32_t sendMessage(const COMMAND &cmd, const std::vector<uint8_t> &data);
      int32_t sendMessage(const COMMAND &cmd);
      int check_sync_byte(void);
      void read_imu_data(void);
      unsigned short calculate_imu_crc(byte data[], unsigned int length);

  };
}
#endif
