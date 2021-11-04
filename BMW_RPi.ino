/*=========================================================================*/
/*                   v8 uses the Seeed Studio CAN-BUS Libray               */
/*                 Works with either the 32u4 BLE or the M0 BLE            */
/*                                                                         */
/*                   Written by James Tanner - DO NOT SHARE                */
/*                  https://www.facebook.com/TheMeroving1an                */
/*                                  IAmOrion                               */
/*=========================================================================*/

/*=========================================================================*/
/*                        CAN_BUS Headers & Settings                       */
/*=========================================================================*/

#include <mcp_can.h>
#include <HID-Project.h>
#include <HID-Settings.h>

/*=========================================================================*/
/*                       Include Settings & Variables                      */
/*=========================================================================*/

#include "Settings.h"
#include "Variables.h"

/*=========================================================================*/
/*                               iDrive CAN Codes                          */
/*=========================================================================*/

#include "iDrive.h"

/*=========================================================================*/
/*                         Adafruit Bluefruit Headers                      */
/*=========================================================================*/

#include <Arduino.h>

/*=========================================================================*/
/*                               Everything Else                           */
/*=========================================================================*/

MCP_CAN CAN1(9);
MCP_CAN CAN2(10);

const String FilterStr = "filter";
int FilterID = 627; //CIC ID for controller initialization message

/*=========================================================================*/
/*                                 void setup()                            */
/*=========================================================================*/

void setup() {

  CAN1.begin(CAN_100KBPS);
  CAN2.begin(CAN_500KBPS);

  Serial.begin(9600);
  
  Serial.println("Serial initialized.");
  
}

/*=========================================================================*/
/*                                  void loop()                            */
/*=========================================================================*/

void loop() {
    
  while(Serial.available()>0){

    String SerialMessage = Serial.readString();

    const char *MessageChar = SerialMessage.c_str();
    const char *FilterChar = FilterStr.c_str();

    if(strstr(MessageChar, FilterChar) != NULL) {

      String CANFilterString = FilterStr.substring(6);
      FilterID = strtoul(CANFilterString.c_str(), NULL, 16);    
      Serial.println("Filter set to: ");
      Serial.print(FilterID, HEX);
      Serial.println();
      
    }

    else {

      String CANIndexString = SerialMessage.substring(0,3);
      unsigned int CANIndex = strtoul(CANIndexString.c_str(), NULL, 16);
      String CANLenghtString = SerialMessage.substring(4,5);
      int CANLenght = CANLenghtString.toInt();
      
      unsigned char Bytes[CANLenght] = {};
      String CANMessageString;
      uint8_t CANAddress = CANIndex;
      
      int index = 1;
      int counter = 0;
      
      while (counter < CANLenght * 3) {
        
        CANMessageString = SerialMessage.substring(6+counter,8+counter);
        unsigned int value = strtoul(CANMessageString.c_str(), NULL, 16);    
        Bytes[index] = value;
        counter = counter + 3;
        index = index + 1;
        
      }
  
      Serial.println("Sending CAN bus message:");
      Serial.print("ID: ");
      Serial.println(CANAddress);
      Serial.print("Data: ");
  
      int indexcounter = 1;
      
      while (indexcounter < CANLenght+1) {
        
        Serial.print(Bytes[indexcounter]);
        Serial.print(" ");
        indexcounter++;
        
      }
  
      Serial.println(" ");
      unsigned char buf1[CANLenght] = { Bytes[1], Bytes[2], Bytes[3], Bytes[4], Bytes[5], Bytes[6], Bytes[7], Bytes[8] };
      CAN1.sendMsgBuf(CANAddress, 0, CANLenght, buf1);
      Serial.println("---------------------------");
      
    }

  }

  // Initial Inits
  if (!(RotaryInitSuccess)) { iDriveInit(); }  
  if (!(TouchpadInitDone) && (RotaryInitSuccess)) { iDriveTouchpadInit(); TouchpadInitDone = true; }
  if (!(PollInit) && (TouchpadInitDone) && (RotaryInitSuccess)) { do_iDrivePoll(); PollInit = true; }
  if (!(LightInitDone)) { if (previousMillis == 0) { previousMillis = millis(); }; iDriveLightInit(); }
  if (!(controllerReady) && (RotaryInitSuccess) && (TouchpadInitDone) && (PollInit)) {  if (CoolDownMillis == 0) { CoolDownMillis = millis(); }; if (millis() - CoolDownMillis > controllerCoolDown) { controllerReady = true; } }
  
  iDrivePoll(iDrivePollTime);
  iDriveLight(iDriveLightTime);

  if(CAN_MSGAVAIL == CAN2.checkReceive()) {
    unsigned char len = 0; unsigned char buf[8];
    CAN2.readMsgBuf(&len, buf);
    unsigned long canId = CAN2.getCanId();
    decodeCanBus500(canId, len, buf);
  }

  if(CAN_MSGAVAIL == CAN1.checkReceive()) {
    unsigned char len = 0; unsigned char buf[8];
    CAN1.readMsgBuf(&len, buf);
    unsigned long canId = CAN1.getCanId();
    decodeCanBus100(canId, len, buf);
  }
  
}

void iDriveInit() {
  //ID: 273, Data: 8 1D E1 0 F0 FF 7F DE 4
  const int msglength = 8; unsigned char buf[8] =  { 0x1D, 0xE1, 0x0, 0xF0, 0xFF, 0x7F, 0xDE, 0x4 };  
  CAN2.sendMsgBuf(MSG_OUT_ROTARY_INIT, 0, msglength, buf);
  RotaryInitPositionSet = false;
}

void iDriveTouchpadInit() {
  //ID: BF, Length: 8, Data: 21 0 0 0 11 0 0 0
  const int msglength = 8; unsigned char buf[8] =  { 0x21, 0x0, 0x0, 0x0, 0x11, 0x0, 0x0, 0x0 };
  CAN2.sendMsgBuf(MSG_IN_TOUCH, 0, msglength, buf);
}

void iDriveLightInit() {
  do_iDriveLight(); if (millis() - previousMillis > iDriveInitLightTime) { LightInitDone = true; }
}

void iDrivePoll(unsigned long milliseconds) {
  static unsigned long lastiDrivePing = 0;
  if (millis() - lastiDrivePing >= milliseconds)
  {
    lastiDrivePing += milliseconds;
    do_iDrivePoll();
  }
}

void do_iDrivePoll() {
  //ID: 501, Data Length: 8, Data: 1 0 0 0 0 0 0 0
  const int msglength = 8; unsigned char buf[8] =  { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  CAN2.sendMsgBuf(MSG_OUT_POLL, 0, msglength, buf);
}

void iDriveLight(unsigned long milliseconds) {
  static unsigned long lastiDriveLight = 0;
  if (millis() - lastiDriveLight >= milliseconds)
  {
    lastiDriveLight += milliseconds;
    do_iDriveLight();
  }
}

void do_iDriveLight() {
  //ID: 202, Data: 2 FD 0 == Light ON, D: 202, Data: 2 FE 0 == Light OFF
  unsigned char buf[2] = { 0x00, 0x0 };
  const int msglength = 2; if (iDriveLightOn) { buf[0] = 0xFD; } else { buf[0] = 0xFE; }
  CAN2.sendMsgBuf(MSG_OUT_LIGHT, 0, msglength, buf);
}

void decodeCanBus100(unsigned long canId, unsigned char len, unsigned char buf[8]) {

  if (canId == FilterID) {
    
    Serial.println("-----------------------------");
    Serial.print("Get data from ID: 0x");
    Serial.println(canId, HEX);

    for (int i = 0; i < len; i++) { // print the data
      
      Serial.print(buf[i], HEX);
      Serial.print("\t");
      
    }
    
    Serial.println();

    unsigned char msg[4] = { 0xE1, 0x9D, buf[7], 0xFF };
    CAN1.sendMsgBuf(0x277, 0, 4, msg);

    Serial.println("Controller responded. Byte 3 = ");
    Serial.print(buf[7]);
    Serial.println();
    
  }

}

void decodeCanBus500(unsigned long canId, unsigned char len, unsigned char buf[8]) {

  if (canId == MSG_IN_ROTARY_INIT) {
    RotaryInitSuccess = true;
  }

  switch (canId) {
    case MSG_IN_INPUT:
      {
        CAN1.sendMsgBuf(canId, 0, len, buf);
      }
      break;

    case MSG_IN_ROTARY:
      {
        CAN1.sendMsgBuf(canId, 0, len, buf);
      }
      break;

    case MSG_IN_TOUCH:
      {
        byte fingers = buf[4];
        
        int touch_count = 0;
        switch (fingers) {
          case FINGER_REMOVED:
            touch_count = 0;
            break;
          case SINGLE_TOUCH:
            touch_count = 1;
            break;
          case MULTI_TOUCH:
            touch_count = 2;
            break;
          default:
            touch_count = 1;
            break;
        }
        TouchPadMouse(buf[1], buf[3], buf[5], buf[7], buf[0], buf[2], buf[6], touch_count);
      }
      break;
      
    case MSG_IN_STATUS:
      {
        if (buf[4] == MSG_STATUS_NO_INIT) {
          RotaryInitSuccess = false; LightInitDone = false; previousMillis = 0; CoolDownMillis = 0; TouchpadInitIgnoreCounter = 0;
        }
      }
      break;
    default:
      break;
  }
  
}

int touch_time = 0;
  
void TouchPadMouse(byte x, byte y, byte x2, byte y2, byte counter, byte xLR, byte x2LR, int touchcount) {
  /*
  *  X is 0 - 255 | 0 - 255 (Left to Center, Center to Right) | X Home (0) = LEFT & Center
  *  Y is 0 - 30 (Bottom to Top) | Y Home (0) = BOTTOM
  *  
  *  The 0xBF Response is built of the following:
  *  Byte[0] = Counter
  *  Byte[1] = X Pos (Single Touch)
  *  Byte[2] = X Left or Right -> The X Axis is split in 2, with left to center being 0-255, then center to right being 0-255. Byte 2 is 0 for left range, 1 for right range
  *  Byte[3] = Y Pos (Single Touch)
  *  Byte[4] = Touch Count -> Eg, how many fingers are touchng the touchpad.  Can detect 1,2,3 or even 4 - but for practicality we only want single or multi (2) max
  *  Byte[5] = X2 Pos (Multi Touch)
  *  Byte[6] = X2 Left or Right (As Above)
  *  Byte[7] = Y2 Pos (Multi Touch)
  */
  
  xLR = xLR & 0x0F; x2LR = x2LR & 0x0F;
  int pos_x = ((int)x), pos_y = ((int)y), x_LR = (int)xLR;

  if (touchcount > 0) {
    touch_time = touch_time + 1;
    if (TouchpadInitIgnoreCounter > TouchpadInitIgnoreCount) {
      #ifdef MOUSE_V1      
        if (x_LR == 0) { pos_x = map(pos_x, 0, 255, mouse_low_range_v2, mouse_center_range_v2); } else if (x_LR == 1) { pos_x = map(pos_x, 0, 255, mouse_center_range_v2, mouse_high_range_v2); } else { //Serial.print(F("Doh - x_LR = "));
          //Serial.println(x_LR);
          }
        pos_y = map(pos_y, 0, 30, mouse_low_range_v2, 90);
    
        if (!(touching)) { touching = true; PreviousX = pos_x; PreviousY = pos_y; ResultX = 0; ResultY = 0; }
    
        //Serial.print(F("2) X: ")); Serial.print(x); Serial.print(F(", pos_x: ")); Serial.print(pos_x); Serial.print(F(", Y: ")); Serial.print(y); Serial.print(F(", pos_y: ")); Serial.println(pos_y);
    
        if ((pos_x != 0) || (pos_y != 0)) {
          int RawX = pos_x - PreviousX, RawY = pos_y - PreviousY;
          pos_x += ResultX; pos_y += ResultY;
          int XFinal = RawX, YFinal = RawY;
  
          if (controllerReady) {          
            Mouse.move((((int8_t)XFinal)/2), ((((int8_t)(YFinal*6))*-1)/2)); 
          }

          if ((PreviousX == pos_x) && (PreviousY == pos_y)) {
            //Mouse.click();           
          }
          
          ResultX = RawX - XFinal; ResultY = RawY - YFinal;
          PreviousX = pos_x; PreviousY = pos_y;
        }      
      #endif
      #ifdef MOUSE_V2
        int8_t pos_x_mapped = 0, pos_y_mapped = 0, mousemove_x = 0, mousemove_y = 0;
        if (x_LR == 0) { pos_x_mapped = map(pos_x, 0, 255, mouse_low_range, mouse_center_range); } else if (x_LR == 1) { pos_x_mapped = map(pos_x, 0, 255, mouse_center_range, mouse_high_range); } else { //Serial.print(F("Doh - x_LR = ")); Serial.println(x_LR); }
        pos_y_mapped = map(pos_y, 0, 30, mouse_low_range, mouse_high_range);
    
        if ((pos_x_mapped == 0) && ((pos_y_mapped == 0))) {
          mousemove_x = 0; mousemove_y = 0;
        } else {
          mousemove_x = pos_x_mapped; mousemove_y = pos_x_mapped;
          if (mousemove_x > 0) { mousemove_x = ((pos_x_mapped/4)+8); } else if (mousemove_x < 0) { mousemove_x = ((pos_x_mapped/4)-8); }
          if (mousemove_y > 0) { mousemove_y = ((pos_y_mapped/4)+8); } else if (mousemove_x < 0) { mousemove_y = ((pos_y_mapped/4)-8); }
        }
      #endif
      }
	else {			
      if (RotaryInitSuccess) {		  
		TouchpadInitIgnoreCounter++;		
	  }
	}
	} 
  else {
	if (touch_time < 5) {
		Mouse.click();
	}
  touch_time = 0;
	touching = false;
  }
}
