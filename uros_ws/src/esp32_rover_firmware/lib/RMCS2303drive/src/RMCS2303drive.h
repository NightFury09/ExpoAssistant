#ifndef RMCS2303drive_h
#define RMCS2303drive_h

#include "Arduino.h"

#ifdef ESP32
#include <HardwareSerial.h>
#else
#include "SoftwareSerial.h"
#endif

class RMCS2303
{
  public:
    void begin(HardwareSerial *hport, long baudrate);
#ifndef ESP32
    void begin(SoftwareSerial *sport, long baudrate);
#endif
    void Serial0(long baudrate);
    void Serial_selection(byte x);
    
    // Motor Control
    byte Enable_Analog_Mode(byte slave_id);
    byte Disable_Analog_Mode(byte slave_id);
    byte Enable_Digital_Mode(byte slave_id, byte dir);
    byte Disable_Digital_Mode(byte slave_id, byte dir);
    byte Absolute_position(byte slave_id, long int count);
    byte Speed(byte slave_id, int SPEED);
    long int Speed_Feedback(byte slave_id);
    byte Brake_Motor(byte slave_id, byte dir);
    byte EncoderPosition_LPR(byte slave_id, int LPR);
    byte Disable_Position_Mode(byte slave_id);
    byte EncoderPosition_enable(byte slave_id);
    byte Absolute_move(byte slave_id, long int count);
    byte Acceleration(byte slave_id, int acceleration);
    byte ProportionalGain(byte slave_id, int PP_GAIN);
    byte IntegralGain(byte slave_id, int PI_GAIN);
    byte FeedforwardGain(byte slave_id, int VF_GAIN);
    long int Position_Feedback(byte slave_id);
    byte SAVE(byte slave_id);
    byte RESET(byte slave_id);
    byte ESTOP(byte slave_id);
    byte STOP(byte slave_id);
    byte SET_HOME(byte slave_id);
    byte Restart(byte slave_id);
    byte WRITE_PARAMETER(byte slave_id, int INP_CONTROL_MODE, int PP_GAIN, int PI_GAIN, int VF_GAIN, int LPR, int acceleration, int SPEED);
    byte READ_PARAMETER(byte slave_id);

  private:
    // Pointers must be per-instance to avoid UART cross-talk!
    HardwareSerial *port;
#ifndef ESP32
    SoftwareSerial *softport;
#endif
    byte Sprt;

    // Helper functions
    byte WriteSingleRegister(byte slave_id, int address, unsigned int value);
    String Modbus_string(byte slave_id, byte FC, int address, unsigned int data);
    String print_hex(byte number);
    String ReadSingleRegister(byte slave_id, int address, unsigned int No_register);
    long int READ_DEVICE_MODBUS_ADDRESS(byte slave_id);
    long int READ_INP_CONTROL_BYTE(byte slave_id);
    long int READ_INP_MODE_BYTE(byte slave_id);
    long int READ_PP_GAIN_BYTE(byte slave_id);
    long int READ_PI_GAIN_BYTE(byte slave_id);
    long int READ_VF_GAIN_BYTE(byte slave_id);
    long int READ_LINES_PER_ROT(byte slave_id);
    long int READ_TRP_ACL_WORD(byte slave_id);
    long int READ_TRP_SPD_WORD(byte slave_id);
    void hexToBytes(byte *byteArray, const char *hexString);
    byte nibble(char c);
    byte LRC(String s, byte len);
    long int value(String input);
};

#endif
