#include <ESP8266WiFi.h>
#include <Wire.h>
#include <EEPROM.h>

#include "rc.h"
#include "OSC.h"


#define WIFI_CHANNEL 4

//#define ARMSWITCH

volatile boolean recv;
volatile int peernum = 0;


#define ACCRESO 4096
#define CYCLETIME 4
#define MINTHROTTLE 1050
#define MIDRUD 1519
#define THRCORR 19

enum ang { ROLL,PITCH,YAW };

static int16_t gyroADC[3];
static int16_t accADC[3];
static int16_t gyroData[3];
static float angle[2]    = {0,0};  
extern int calibratingA;

#define ROL 0
#define PIT 1
#define THR 2
#define RUD 3
#define AU1 4
#define AU2 5
static int16_t rcCommand[] = {0,0,0};

#define GYRO     0
#define STABI    1
#define RTH      2
static int8_t flightmode;
static int8_t oldflightmode;

boolean armed = false;
uint8_t armct = 0;

void initWifiAP() {
  //Serial.println("Setting up WiFi AP...");
  if (WiFi.softAP("espCopter", "12345678")) {
    Serial.println("Wifi AP set up successfully");
  }
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                    IPAddress(255, 255, 255, 0));
}

void processOSCMsg() {
  if (OSCpage == 1) 
 {
    rcValue[THR] = (uint16_t)(1000+OSCfader[0]*10000);
	rcValue[PIT]=(uint16_t)(1000+OSCxy1_y*10000);
	rcValue[ROL]=(uint16_t)(1000+OSCxy1_x*10000);
	rcValue[RUD ] = (uint16_t)(1000+OSCfader[1]*10000);  
  rcValue[AU1] = (uint16_t)(1000+OSCfader[2]*10000); 
    }
  OSC_MsgRead();
    
  }

void initServo();

void setup() 
{
  Serial.begin(115200); Serial.println();

  MPU6050_init();
  MPU6050_readId(); // must be 0x68, 104dec
  
  EEPROM.begin(64);
  if (EEPROM.read(63) != 0x55) Serial.println("Need to do ACC calib");
  ACC_Read();
  
  OSC_init();

  initWifiAP();

  delay(1000); 
  initServo();

}



uint32_t rxt; // receive time, used for falisave

void loop() 
{
	OSC_MsgRead();

	 uint32_t now,diff; 
  
  //now = millis(); // actual time

  if (OSCnewMessage)
  {
   OSCnewMessage = 0;
		processOSCMsg();

    if      (rcValue[AU1] < 1300) flightmode = GYRO;
    else if (rcValue[AU1] > 1700) flightmode = RTH;
    else                          flightmode = STABI;   
    if (oldflightmode != flightmode)
    {
      zeroGyroI();
      oldflightmode = flightmode;
    }

    #if defined (ARMSWITCH)
      if (armed) 
      {
        if (rcValue[AU2] <= 1400) { armed = false; armct = 0; }
        rcValue[THR]    -= THRCORR;
        rcCommand[ROLL]  = rcValue[ROL] - MIDRUD;
        rcCommand[PITCH] = rcValue[PIT] - MIDRUD;
        rcCommand[YAW]   = rcValue[RUD] - MIDRUD;
      }  
      else if (rcValue[AU2] >= 1600)
      {  
        if (rcValue[THR] < MINTHROTTLE) armct++;
        if (armct >= 25) armed = true;
      } 
    #else
      if (armed) 
      {
        rcValue[THR]    -= THRCORR;
        rcCommand[ROLL]  = rcValue[ROL] - MIDRUD;
        rcCommand[PITCH] = rcValue[PIT] - MIDRUD;
        rcCommand[YAW]   = rcValue[RUD] - MIDRUD;
      }  
      else
      {  
        if (rcValue[THR] < MINTHROTTLE) armct++;
        if (armct >= 25) armed = true;
      }
    #endif 
    
    //Serial.println(rcValue[AU2]    );
    //Serial.print(rcValue[THR]    ); Serial.print("  ");
    //Serial.print(rcCommand[ROLL] ); Serial.print("  ");
    //Serial.print(rcCommand[PITCH]); Serial.print("  ");
    //Serial.print(rcCommand[YAW]  ); Serial.println();

    //diff = now - rxt;
    //Serial.print(diff); Serial.println();
    rxt = millis();

    if (peernum > 0) 
    {
      //t_angle();
      t_gyro();
      //t_acc();

      //esp_now_send(NULL, (u8*)hello, sizeof(hello)); // NULL means send to all peers
    }
  }

  Gyro_getADC();
  //Serial.print(gyroADC[0]); Serial.print("  ");
  //Serial.print(gyroADC[1]); Serial.print("  ");
  //Serial.print(gyroADC[2]); Serial.println("  ");
  
  ACC_getADC();
  //Serial.print(accADC[0]); Serial.print("  ");
  //Serial.print(accADC[1]); Serial.print("  ");
  //Serial.print(accADC[2]); Serial.println("  ");

  getEstimatedAttitude();
  Serial.print(angle[0]); Serial.print("  ");
  Serial.print(angle[1]); Serial.println("  ");

  pid();

  mix();

  writeServo();
  
  // Failsave part
  if (now > rxt+90)
  {
    rcValue[THR] = MINTHROTTLE;
    rxt = now;
    //Serial.println("FS");
  }

  // parser part
  if (Serial.available())
  {
    char ch = Serial.read();
    // Perform ACC calibration
    if (ch == 'A')
    { 
      Serial.println("Doing ACC calib");
      calibratingA = 64; // CALSTEPS
      while (calibratingA != 0)
      {
        delay(CYCLETIME);
        ACC_getADC(); 
      }
      ACC_Store();
      Serial.println("ACC calib Done");
    }
  }
  
  delay(CYCLETIME-1);
  //diff = millis() - now;
  //Serial.print(diff); Serial.println();
}


