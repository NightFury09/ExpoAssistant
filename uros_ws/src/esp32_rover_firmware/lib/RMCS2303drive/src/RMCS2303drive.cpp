#include "RMCS2303drive.h"
#include "Arduino.h"

const byte MaxByteArraySize = 8;

void RMCS2303::begin(HardwareSerial *hport, long baudrate) {
    port = hport;
    Sprt = 0;
    // Note: port->begin() should be called in setup() to avoid side effects
}

#ifndef ESP32
void RMCS2303::begin(SoftwareSerial *sport, long baudrate) {
    softport = sport;
    Sprt = 1;
}
#endif

void RMCS2303::Serial0(long baudrate) {
    // Legacy support, avoid using for UROS
}

void RMCS2303::Serial_selection(byte x) {   
    Sprt = x;
}

byte RMCS2303::Enable_Analog_Mode(byte slave_id) {
    return WriteSingleRegister(slave_id, 2, 1);
}

byte RMCS2303::Disable_Analog_Mode(byte slave_id) {
    return WriteSingleRegister(slave_id, 2, 0);
}

byte RMCS2303::Enable_Digital_Mode(byte slave_id, byte dir) {
    int data = (dir == 0) ? 257 : 265;
    return WriteSingleRegister(slave_id, 2, data);
}

byte RMCS2303::Disable_Digital_Mode(byte slave_id, byte dir) { 
    int data = (dir == 0) ? 256 : 264;
    return WriteSingleRegister(slave_id, 2, data);
}

byte RMCS2303::Absolute_position(byte slave_id, long int count) {  
    if (Absolute_move(slave_id, count)) {
        return EncoderPosition_enable(slave_id);
    }
    return 0;
}

byte RMCS2303::Speed(byte slave_id, int SPEED) {  
    return WriteSingleRegister(slave_id, 14, SPEED);
}

long int RMCS2303::Speed_Feedback(byte slave_id) {     
    String result = ReadSingleRegister(slave_id, 24, 1);
    if (result == "0") return 0;
    unsigned long int q = value(result);
    return (q > 32765) ? (long int)q - 65535 : (long int)q;
}

byte RMCS2303::Brake_Motor(byte slave_id, byte dir) { 
    int data = (dir == 0) ? 260 : 268;
    return WriteSingleRegister(slave_id, 2, data);
}

byte RMCS2303::EncoderPosition_LPR(byte slave_id, int LPR) {
    return WriteSingleRegister(slave_id, 10, LPR);
}

byte RMCS2303::Disable_Position_Mode(byte slave_id) {
    return WriteSingleRegister(slave_id, 2, 512);
}

byte RMCS2303::EncoderPosition_enable(byte slave_id) {   
    return WriteSingleRegister(slave_id, 2, 513);
}
     
byte RMCS2303::Absolute_move(byte slave_id, long int count) { 
    unsigned long int data;
    if (count >= 0) data = count;
    else data = 4294967295UL - (unsigned long)abs(count);
    
    unsigned int Hidata = data >> 16;
    unsigned int Lodata = data & 0xFFFF;
    
    if (WriteSingleRegister(slave_id, 16, Lodata)) {
        return WriteSingleRegister(slave_id, 18, Hidata);
    }
    return 0;
}

byte RMCS2303::Acceleration(byte slave_id, int acceleration) {
    return WriteSingleRegister(slave_id, 12, acceleration);
}

byte RMCS2303::ProportionalGain(byte slave_id, int PP_GAIN) {
    return WriteSingleRegister(slave_id, 4, PP_GAIN);
}

byte RMCS2303::IntegralGain(byte slave_id, int PI_GAIN) {
    return WriteSingleRegister(slave_id, 6, PI_GAIN);
}

byte RMCS2303::FeedforwardGain(byte slave_id, int VF_GAIN) {
    return WriteSingleRegister(slave_id, 8, VF_GAIN);
}

long int RMCS2303::Position_Feedback(byte slave_id) {     
    String result = ReadSingleRegister(slave_id, 20, 2);
    if (result == "0") return 0;
    
    char a[9];
    char *endptr;
    result.substring(7, 15).toCharArray(a, 9);
    
    for (int i = 0; i < 4; i++) {
        char c = a[i]; a[i] = a[i+4]; a[i+4] = c; 
    }
    unsigned long q = strtoul(a, &endptr, 16);
    return (q > 2147483647) ? (long int)q - 4294967295UL : (long int)q;
}	
  
byte RMCS2303::WriteSingleRegister(byte slave_id, int address, unsigned int value) {   
    String result = Modbus_string(slave_id, 6, address, value);
    if (Sprt == 0) {    
        port->print(result);
        delay(5); // Minimum wait for master to release line
        unsigned long start = millis();
        String response = "";
        while ((millis() - start) < 30) { // Max 30ms wait for response
            if (port->available()) {
                response = port->readStringUntil('\n');
                break;
            }
        }
        if (response.length() < 10) return 0;
        return LRC(response.substring(1, 16), 6);
    }
    return 0;
}
    
String RMCS2303::Modbus_string(byte slave_id, byte FC, int address, unsigned int data) {
    byte AddLo = (address & 0xFF);
    byte AddHi = (address >> 8) & 0xFF;
    byte dataLo = (data & 0xFF);
    byte dataHi = (data >> 8) & 0xFF;
    
    byte sum = 0;
    sum -= slave_id;
    sum -= FC;
    sum -= AddLo;
    sum -= AddHi;
    sum -= dataLo;
    sum -= dataHi;

    return ":" + print_hex(slave_id) + print_hex(FC) + print_hex(AddHi) + 
           print_hex(AddLo) + print_hex(dataHi) + print_hex(dataLo) + 
           print_hex(sum) + "\r\n";
}

String RMCS2303::print_hex(byte number) {
    String value = String(number, HEX);
    value.toUpperCase();
    if (value.length() == 1) value = "0" + value;
    return value;
}

String RMCS2303::ReadSingleRegister(byte slave_id, int address, unsigned int No_register) {   
    String result = Modbus_string(slave_id, 3, address, No_register);
    if (Sprt == 0) { 
        port->print(result);
        delay(5);
        unsigned long start = millis();
        String response = "";
        while ((millis() - start) < 30) {
            if (port->available()) {
                response = port->readStringUntil('\n');
                break;
            }
        }
        if (response.length() < 10) return "0";
        if (LRC(response.substring(1, 18), 7)) return response;
    }
    return "0";
}
    
byte RMCS2303::LRC(String s, byte len) {
    const char* str = s.c_str();
    byte sum = 0;
    for (int i = 0; i < len; i++) {
        char buf[3] = {str[i*2], str[i*2+1], '\0'};
        sum += (byte)strtol(buf, NULL, 16);
    }
    byte actualLRC = (byte)strtol(s.substring(len*2).c_str(), NULL, 16);
    byte calculatedLRC = (byte)((~sum) + 1);
    return (actualLRC == calculatedLRC) ? 1 : 0;
}

long int RMCS2303::value(String input) { 
    if (input.length() < 10) return 0;
    char *endptr;
    String hexVal = input.substring(7, input.length() - 3);
    return strtol(hexVal.c_str(), &endptr, 16);
}

// Stubs for parameters to avoid binary bloat
byte RMCS2303::SAVE(byte slave_id) { return WriteSingleRegister(slave_id, 0, ((slave_id << 8) | 0xFF)); }
byte RMCS2303::WRITE_PARAMETER(byte s, int c, int p, int i, int v, int l, int a, int sp) { return 0; }
byte RMCS2303::READ_PARAMETER(byte s) { return 0; }
byte RMCS2303::RESET(byte s) { return 0; }
byte RMCS2303::ESTOP(byte s) { return 0; }
byte RMCS2303::STOP(byte s) { return 0; }
byte RMCS2303::SET_HOME(byte s) { return 0; }
byte RMCS2303::Restart(byte s) { return 0; }
