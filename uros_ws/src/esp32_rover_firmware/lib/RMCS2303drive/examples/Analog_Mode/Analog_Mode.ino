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
   
*	In this mode the speed of the Rhino DC Servo motor can be controlled by an externally connected Potentiometer. 
*	User can increase or decrease the speed manually based on requirement. 
*	The drive will provide full torque at all speeds within the range.
*	However the potentiometer has to be connected via a voltage divider to provide a maximum of 3.3 volts so as to not damage the drive. 
*	Also the Enable, Brake and direction pins have to be connected as per configuration requirements

* For more information see : https://robokits.co.in/motor-drives-drivers/encoder-dc-servo/rhino-dc-servo-driver-50w-compatible-with-modbus-uart-ascii-for-encoder-dc-servo-motor

*/

#include<RMCS2303drive.h>

RMCS2303 rmcs;                      //object for class RMCS2303
SoftwareSerial myserial(2,3);     //Software Serial port For Arduino Uno. Comment out if using Mega.
byte slave_id=7;

void setup()
{
   rmcs.Serial_selection(1);       //Serial port selection:0-Hardware serial,1-Software serial
   rmcs.Serial0(9600);             //set baudrate for usb serial to monitor data on serial monitor
   Serial.println("RMCS-2303 Analog Mode\r\n\r\n");   

   //rmcs.begin(&Serial1,9600);    //Uncomment if using hardware serial port for mega2560:Serial1,Serial2,Serial3 and set baudrate. Comment this line if Software serial port is in use
   rmcs.begin(&myserial,9600);     //Uncomment if using software serial port. Comment this line if using hardware serial.
   //rmcs.WRITE_PARAMETER(slave_id,INP_CONTROL_MODE,PP_gain,PI_gain,VF_gain,LPR,acceleration,speed);    //Uncomment to write parameters to drive.
   rmcs.READ_PARAMETER(slave_id);
}

void loop(void)
{
   rmcs.Enable_Analog_Mode(slave_id);      //To enable motor in analog control mode 
   rmcs.SAVE(slave_id);
   Serial.println("Drive set to analog mode.\r\nNow drive will work with potentiometer/analog input signal.\r\n\r\n");   
   //rmcs.Disable_Analog_Mode(slave_id);   //Uncomment disable function To disable motor in Analog Mode
   while(1)                                //Infinite loop after task is done.
   {
      Serial.print(".");
      delay(1000);
   }
}
