/*
Connections of Drive and Arduino
Serial Port 0 is not used to connect to drive because its connected to USB-Serial and used to show information on console.

For Arduino Uno Software serial needs to be used as there is only one hardware serial port and its connected to USB-Serial. 
   Drive to Arduino UNO/Nano connections
   GND         -      GND
   RXD         -      D3
   TXD         -      D2

For arduino mega and other arduinos with multiple hardware serial port, any port other than 0 can be selected to connect the drive.

   Drive to Arduino Mega2560 connections
   GND         -      GND
   RXD         -      Tx1/Tx2/Tx3
   TXD         -      Rx1/Rx2/Rx3

*	In this mode Speed, Direction, Acceleration and Position of motor can be controlled.
*	The position control mode is suitable in applications where exact movement of motor is required. This can be used in precision applications like machine control, motion control or robotics. 

* For more information see : https://robokits.co.in/motor-drives-drivers/encoder-dc-servo/rhino-dc-servo-driver-50w-compatible-with-modbus-uart-ascii-for-encoder-dc-servo-motor

*/


#include<RMCS2303drive.h>

RMCS2303 rmcs;                      //object for class RMCS2303

SoftwareSerial myserial(2,3);     //Software Serial port For Arduino Uno. Comment out if using Mega.

//Parameter Settings "Refer datasheet for details" - 
byte slave_id=11;                    //Choose the slave id of connected drive.
int INP_CONTROL_MODE=513;           //IMPORTANT: refer datasheet and set value(integer) according to application 
int PP_gain=32;
int PI_gain=16;
int VF_gain=32;
int LPR=334;
int acceleration=5000;
int speed=8000;

long int Current_position;
long int Current_Speed;

void setup()
{
   rmcs.Serial_selection(1);       //Serial port selection:0-Hardware serial,1-Software serial
   rmcs.Serial0(9600);             //set baudrate for usb serial to monitor data on serial monitor
   Serial.println("RMCS-2303 Position control mode demo\r\n\r\n");

   //rmcs.begin(&Serial1,9600);    //Uncomment if using hardware serial port for mega2560:Serial1,Serial2,Serial3 and set baudrate. Comment this line if Software serial port is in use
   rmcs.begin(&myserial,9600);     //Uncomment if using software serial port. Comment this line if using hardware serial.
   rmcs.WRITE_PARAMETER(slave_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);    //Uncomment to write parameters to drive. Comment to ignore.
   rmcs.READ_PARAMETER(slave_id);
   
}

void loop()
{
   Serial.println("Sending absolute position command to -50000");
   rmcs.Absolute_position(slave_id,-50000);   //enter position count with direction (CW:+ve,CCW:-ve) 
   
   while(1)       //Keep reading positions. Exit when reached.
   {
      Current_position=rmcs.Position_Feedback(slave_id); //Read current encoder position 
      Current_Speed=rmcs.Speed_Feedback(slave_id);       //Read current speed
      Serial.print("Position Feedback :\t");
      Serial.print(Current_position);
      Serial.print("\t\tSpeed Feedback :\t");
      Serial.println(Current_Speed);

      delay(100);
      if(Current_position==-50000)
      {
         Serial.println("Position -50000 reached.");
         break;
      }
   }
   delay(2000);
   
   
   Serial.println("Sending absolute position command to 50000");
   rmcs.Absolute_position(slave_id,50000);   //enter position count with direction (CW:+ve,CCW:-ve) 
   
   while(1)       //Keep reading positions. Exit when reached.
   {
      Current_position=rmcs.Position_Feedback(slave_id); //Read current encoder position 
      Current_Speed=rmcs.Speed_Feedback(slave_id);       //Read current speed
      Serial.print("Position Feedback :\t");
      Serial.print(Current_position);
      Serial.print("\t\tSpeed Feedback :\t");
      Serial.println(Current_Speed);
      delay(100);
      if(Current_position==50000)
      {
         Serial.println("Position 50000 reached.");
         break;
      }
   }
   delay(2000);

   
   Serial.println("Disabling motor.");
   rmcs.Disable_Position_Mode(slave_id);            //Disable postion control mode
   delay(1000);
}
 
