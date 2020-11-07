#include <Arduino.h>
#include <Servo.h>
#include "Wire.h"
#include "I2CKeyPad.h"
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <pins_arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h> 
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <DNSServer.h>
#include <WiFiManager.h>  
#include <ArduinoJson.h>


#define cls             10
#define up              11
#define down            12
#define exit            13
#define ok              14
#define dot             15

#define Min_Throttle  40  //46
#define Max_Throttle  156  //156

#define lcd_puts_XY(x,y,msg) {\
 lcd.setCursor(x,y);\
 lcd.print(msg);\
}
    

//#define BrushlessMotorPin 15      //D8 is GPIO15
#define BrushlessMotorPin D0
//**********************Encoder Variable*****************************
#define Encoder_DT    D5
#define Encoder_CLK   D6
int16_t  Encoder_quad = 0;
int16_t Encoder_previous_data;
void ICACHE_RAM_ATTR Encoder_read();
/********************************************************************/
const uint8_t KEYPAD_ADDRESS  = 0x26;
const uint8_t LCD_ADDRESS     = 0x27;
I2CKeyPad         keyPad;
LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4);
Servo esc;  // create servo object to control a servo

/********************************************************************/
const char info[] ={"\r\n***************************************\r\n"
                        "************* AeroPendulum ************\r\n"
                        "******* Seyyed Amir Ali Masoumi *******\r\n"
                        "**** PhoneNumber: +98 930 927 1137 ****\r\n"
                        "********* Telegram: @Mashroti *********\r\n"
                        "****** Email: Mashroty@gmail.com ******\r\n"
                        "***************************************\r\n"};

struct Status
{
	uint8_t motor	  : 1;
	uint8_t matlab	: 1;
	uint8_t whiles	: 1;
	uint8_t PID_whil: 1;
	uint8_t PID_OK	: 1;
	uint8_t Get_Num	: 1;
	uint8_t get_data: 1;
	uint8_t Online	: 1;
	uint8_t Offline	: 1;
  uint8_t exe		  : 1;
}Status;

struct PID{
	int8_t SetPoint;
	uint8_t time;
	double Kp;
	double Ki;
	double Kd;
}PID={25,0,2,0,0};

uint8_t zaman;
String token,code;
String angleAdderss;
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
char GET_KEY  (void);
void MATLAB   (void);
void startup (void);
void key_analyze(void);
void Online(void);
void data_send_online(uint16 *time, int16_t *angle, uint16 i);
void Offline(void);
void show_param(void);
void PID_Controll(void);
void PID_setting(void);
void Get_Number (char *NUMBER);

/*void Volume(void);
void Verify_Unique(void);
void Process_UART_Data(char* Data);

void WiFi_Setting(void);*/


/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/

void setup()
{
  Serial.begin(115200);
  Serial.setRxBufferSize(100);
  Serial.println();
  //Serial.println(__FILE__);
  //***************************** Encoder Setup ************************************
  pinMode(Encoder_DT,  INPUT);
  pinMode(Encoder_CLK, INPUT);
  Encoder_previous_data = digitalRead(Encoder_DT) << 1 | digitalRead(Encoder_CLK);
  attachInterrupt(Encoder_DT,  Encoder_read, CHANGE);
  attachInterrupt(Encoder_CLK, Encoder_read, CHANGE);
  //Serial.println("\r\nEncoder init");
  //***************************** Keypad I2C ***************************************
  Wire.begin();
  Wire.setClock(100000);
  if (keyPad.begin(KEYPAD_ADDRESS) == false)
  {
    //Serial.println("\nERROR: cannot communicate to keypad.\nPlease reboot.\n");
    lcd_puts_XY(5,0,"cannot");
    lcd_puts_XY(2,1,"communicate");
    lcd_puts_XY(3,2,"to keypad");
    lcd_puts_XY(1,3,"Please reboot");
    while(1){delay(1);}
  }
  //Serial.println("\r\nkeypad init");
  //***************************** LCD I2C ******************************************
  lcd.init();
  lcd.home();
  lcd.backlight();
  //Serial.println("\r\nlcd init");
  //***************************** Speed Controller *********************************
  esc.attach(BrushlessMotorPin); // attaches the servo on GPIO15(D8) to the servo object
  esc.write(0);   // tell servo to go to position (0 - 180)
  
  startup();

  Status.Online =0;
  Status.Offline =0;
  Status.exe =0;
  Status.motor =0;
}
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
void loop()
{
  uint8_t Num = 20;


  while (1)
  {
    if((Num > 0 && Num <=5 ) || Num == 20)
    {
      lcd.clear();
      if(Num == 1) 
      {
        Status.Online =1;
        Online();
        Status.Online =0;
      }
      else if(Num == 2)	MATLAB();
      else if(Num == 3)	
      {
        Status.Offline = 1;
        Offline();
        Status.Offline = 0;
      }
      /*
      else if(Num == 4)	Volume();
      else if(Num == 5)	WiFi_Setting();
 
      */

      lcd.clear();
      lcd_puts_XY(0,0,"1> Online");
      lcd_puts_XY(0,1,"2> Matlab");
      lcd_puts_XY(0,2,"3> Offline");
      lcd_puts_XY(0,3,"4> Win App");
    }
    Num = GET_KEY();
    delay(1);
  }
}
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
void startup(void)
{
	lcd.clear();
	uint8_t Num = GET_KEY();

	if(Num!='N' && Num!='F')
	{
	  lcd_puts_XY(1,0,"AmirAliMasoumi");
	  lcd_puts_XY(1,1,"Tel: @Mashroti");
	  lcd_puts_XY(0,2,"+98 930 927 1137");
	  lcd_puts_XY(2,3,"AeroPendulum");
	  delay(6000);
	}
	else
	{
	  lcd_puts_XY(2,0,"AeroPendulum");
	  lcd_puts_XY(3,1,"Motor Init");
	  lcd_puts_XY(2,2,"Please Wait");
	  for(uint8_t i=0 ;i<15 ;i++)
	  {
		  lcd_puts_XY(i,3,"\xff>");
		  delay(300);
	  }
	}
  Serial.println();
	Serial.println(info);
}
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
void MATLAB   (void)
{
  char buffer[15];
  bool data = false;
  uint8_t pwm, count=0;
  unsigned long time = millis();   //for turn off the motor after two seconds
	
  union u_type
  {
		double pid_exit;
		uint8_t u[8];
	}var;

  Status.whiles = 1;
  lcd.noBacklight();

  while (Status.whiles)
  {
    key_analyze();
    
    if(Serial.available())
    {
      char c = Serial.read();  //gets one byte from serial buffer
      buffer[count++] = c; //makes the string readString
      buffer[count] = 0;

      if(c == '\n')
      {
        data = true;
        count = 0;
      }

      //delay(1);  //slow looping to allow buffer to fill with next character
    }
    delay(1);
    if(data)
    {
      if (buffer[0]==50 && buffer[1]==52)
      {
        float zavye = Encoder_quad/4;
        Serial.println(zavye,2);
      }
      else if (buffer[0]==40 && buffer[1]==42)
      {
        var.u[0] = buffer[2];
				var.u[1] = buffer[3];
				var.u[2] = buffer[4];
				var.u[3] = buffer[5];
				var.u[4] = buffer[6];
				var.u[5] = buffer[7];
				var.u[6] = buffer[8];
				var.u[7] = buffer[9];	//change received data to int16_t

        pwm = var.pid_exit + Min_Throttle;
        if (pwm < Min_Throttle) pwm = Min_Throttle;
        if (pwm > Max_Throttle) pwm = Max_Throttle;
        esc.write(pwm);
        time = millis();
      }      
      data = false;
    }

    //for turn off the motor after two seconds
    if(millis() - time > 2000)  esc.write(Min_Throttle);

  }
  lcd.backlight();
}
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
void Online(void)
{
  uint8_t i=12;

  Status.whiles = 1;

  lcd_puts_XY(0,0,"Please Wait");
  lcd_puts_XY(0,2,"Connecting to :");
  lcd_puts_XY(0,3,WiFi.SSID());
  
  delay(500);

  while (WiFi.status() != WL_CONNECTED && Status.whiles) 
  {
    lcd_puts_XY(i,0,'.');
    delay(500);
    i++;
    if(i>14) 
    {
      i = 12;
      lcd_puts_XY(12,0,"    ");
      delay(500);
    }
    key_analyze();
  }
  if(!Status.whiles)  return;
  Status.whiles = 1;

  HTTPClient http;    //Declare object of class HTTPClient
  DynamicJsonDocument doc(200);

  String payload;
  int httpCode;

  while(Status.whiles)
  {
    http.begin("http://ap.damoon.pro/api/ap/status");            //Specify request destination
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");    //Specify content-type header
  
    httpCode = http.POST("status=0");   //Send the request
    payload = http.getString();    //Get the response payload
    delay(1000);
    Serial.println(httpCode);   //Print HTTP return code
    Serial.println(payload);    //Print request response payload
    
    http.end();  //Close connection
    
    if(payload.indexOf("kp") > 0)
    {
      DeserializationError error = deserializeJson(doc, payload);
      if (error) 
      {
        Serial.print(F("deserializeJson failed"));
      }   
      else
      {
        PID.Kp = doc["kp"];
        PID.Ki = doc["ki"];
        PID.Kd = doc["kd"];
        PID.SetPoint = doc["sp"];
        PID.time = doc["time"];
        const char *token_char = doc["token"];

        token = token_char;
        token += '/'; 

        const char *code_char = doc["code"]; 
        code = code_char;

        angleAdderss = "http://ap.damoon.pro/api/ap/angle/";
        angleAdderss += token;
        angleAdderss += code; 

        PID_Controll();
      }  
    }

    key_analyze();
  }

}
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
void data_send_online(uint16 *time, int16_t *angle, uint16 i)
{
  HTTPClient http;
  String payload;
  int httpCode;

  String data = "angle=";
  for(uint16 x=0 ; x<i ; x++)
  {
    data += angle[x];
    data += '@';
    data += time[x];
  }
  http.begin(angleAdderss); 
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  httpCode = http.POST(data);
  payload = http.getString();
  Serial.println(httpCode);
  Serial.println(payload);
}
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
void Offline(void)
{
  String buffer;
  Status.whiles = 1;

  show_param();

  while (Status.whiles)
  {
    key_analyze();
    delay(1);
  }
  
}
void show_param(void)
{
  lcd.clear();
  lcd_puts_XY(0,0,"SetPoint=");
  lcd.print(PID.SetPoint);
  lcd_puts_XY(0,1,"KP=");
  lcd.print(PID.Kp);
  lcd_puts_XY(0,2,"KI=");
  lcd.print(PID.Ki);
  lcd_puts_XY(0,3,"KD=");
  lcd.print(PID.Kd);
}
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
char prev_key;
void key_analyze(void)
{
  uint8_t key = GET_KEY();
  if(key != prev_key)
  {
    prev_key = key;
    switch (key)
    {
      case exit:
        Status.whiles = 0;
        break;
      case 0:
        Encoder_quad = 0;
        break; 
      case cls:
        Status.motor ^= 1;
        if(Status.motor && Status.Offline) PID_Controll();
        break;   
      case ok:
        if(!Status.motor && Status.Offline) PID_setting();
        break; 
      default:
        break;
    }    
  }

}
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
char GET_KEY(void)
{
  //char keys[] = "123A456B789C*0#DNF";  // N = Nokey, F = Fail
  char keys[  ]={ 1, 2, 3, 10,
                  4, 5, 6, 11,
                  7, 8, 9, 12,
                  15, 0,14, 13, 'N', 'F'};
  uint8_t idx = keyPad.getKey();
  return keys[idx];
}
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
void ICACHE_RAM_ATTR Encoder_read()
{
  uint8_t current_data = digitalRead(Encoder_DT) << 1 | digitalRead(Encoder_CLK);
 
  if( current_data == Encoder_previous_data )
    return;
 
  if( bitRead(current_data, 0) == bitRead(Encoder_previous_data, 1) )
    Encoder_quad -= 1;
  else
    Encoder_quad += 1;
  Encoder_previous_data = current_data; 
}

/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
void PID_Controll(void)
{
	double Error=0, Previous_error=0;
	double p_term=0, d_term=0, i_term=0, dt=0;
  float Angle = 0;
	int16_t Value=0, PWM=0;
	uint32_t timePrev, TickStart = millis();
	uint8_t throttle =  Max_Throttle - Min_Throttle;

  //online variable
  uint16  i = 0;
  uint16  time_online [1000];
  int16_t angle_online[1000];
  unsigned long time_tick_online = millis();
  unsigned long time_to_send = millis();
  
  lcd.clear();

	while(Status.motor & Status.whiles)
	{
	  Angle = Encoder_quad/4.0;

    timePrev	=	TickStart;
    TickStart	=	millis();
    dt			= 	((float)(TickStart - timePrev) / 1000.00);

    Error		=	PID.SetPoint - Angle;

    p_term = PID.Kp * Error;

    i_term		=	(PID.Ki * (Error + Previous_error) * dt / 2) + i_term ;
    if(i_term > throttle) i_term = throttle;
    if(i_term <-throttle) i_term = -throttle;

    d_term = (Error - Previous_error) / dt * PID.Kd;
    if(d_term > throttle)	d_term = throttle;
    if(d_term <-throttle)	d_term = -throttle;

    PWM			=	(int16_t)(p_term + i_term + d_term);

    if (PWM > throttle)	PWM = throttle;
    if (PWM < 0)		PWM = 0;

    Value = PWM + Min_Throttle;
    esc.write(Value);

    Previous_error = Error;


    key_analyze();

    
    if(Status.Online)
    {
      angle_online[i] = (int16_t) Angle;
      time_online[i]  = millis() - time_tick_online;

      if(time_online[i] >= PID.time*1000)
			{
        Status.motor = 0;
        Status.whiles = 0;
        PID.time = 0;
      }
      i++;
      if(millis() - time_to_send >= 1000 || Status.motor == 0)
			{
        data_send_online(time_online, angle_online, i);
        time_to_send = millis();
        i = 0;
      }
    }

    if(Status.Offline)
    {
      lcd_puts_XY(0,0,"SP=");
      lcd.print(PID.SetPoint); 
      lcd.print("   ");  
      lcd_puts_XY(0,2,"PV=");
      lcd.print(Angle,2);
      lcd.print("   ");
    }
    delay(1);
	}
  Status.whiles = 1;
  Status.motor = 0;
  esc.write(0);
  if(Status.Offline) show_param();
}
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
void PID_setting(void)
{
  uint8_t Prev_Num=ok,Num;
	char Buffer[20];
	char NUMBER[9];
	int8_t i=0;
	double x;

	lcd.clear();

	do
	{
		lcd_puts_XY(4,0,"SetPoint");
		lcd_puts_XY(7,1,"KP");
		lcd_puts_XY(7,2,"KI");
		lcd_puts_XY(7,3,"KD");

		Status.whiles=1; Status.PID_whil=0;
		while(Status.whiles)
		{
			Num = GET_KEY();

			if(Num != Prev_Num)
			{
				lcd_puts_XY(2,i," ");
				lcd_puts_XY(13,i," ");

				if(Num==up)i--;
				if(Num==down)i++;
				if(i<0)i=3;
				if(i>3)i=0;

				lcd_puts_XY(2,i,">");
				lcd_puts_XY(13,i,"<");
				Prev_Num = Num;

				if(Num == ok){Status.whiles=0;Status.PID_whil=1;}
				if(Num == exit)Status.whiles=0;
			}
      delay(1);
		}
		if(Status.PID_whil)
		{
			lcd.clear();

			if(i==0)
			{
				lcd_puts_XY(0,0,"---->SET SP<----");
				sprintf(Buffer,"Last SP= %02d ",PID.SetPoint);
				lcd_puts_XY(0,2,Buffer);
				lcd_puts_XY(1,3,"NEW SP= ");
			}
			if(i==1)
			{
				lcd_puts_XY(0,0,"---->SET KP<----");
				sprintf(Buffer,"Last KP= %07.4f  ",PID.Kp);
				lcd_puts_XY(0,2,Buffer);
				lcd_puts_XY(1,3,"NEW KP= ");
			}
			if(i==2)
			{
				lcd_puts_XY(0,0,"---->SET KI<----");
				sprintf(Buffer,"Last KI= %07.4f  ",PID.Ki);
				lcd_puts_XY(0,2,Buffer);
				lcd_puts_XY(1,3,"NEW KI= ");
			}
			if(i==3)
			{
				lcd_puts_XY(0,0,"---->SET KD<----");
				sprintf(Buffer,"Last KD= %07.4f  ",PID.Kd);
				lcd_puts_XY(0,2,Buffer);
				lcd_puts_XY(1,3,"NEW KD= ");
			}

			Get_Number(NUMBER);
			if(Status.PID_OK)
			{
				Prev_Num = ok;
				x = atof(NUMBER);
				if(x > 999) x = 999;
				switch(i)
				{
					case 0:
						if(x > 140) x = 140;
						if(x < 0) x = 0;
						PID.SetPoint = x;
					break;
					case 1:
						PID.Kp = x;
					break;
					case 2:
						PID.Ki = x;
					break;
					case 3:
						PID.Kd = x;
					break;
				}
			}else Prev_Num = exit;

		}
	}while(Status.whiles);

	Status.whiles=1;
	lcd.clear();
  prev_key = exit;
  show_param();
}
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
void Get_Number (char *NUMBER)
{
  char Prev_Num=ok ,Num;
  uint8_t i;

  for(i=0;i<7;i++)*(NUMBER+i)=' ';
  i=0;

  Status.Get_Num=1;
  while(Status.Get_Num)
  {
    Num = GET_KEY();
    if(Prev_Num != Num && Num !='N' && Num !='F')
    {
      switch (Num)
      {
        case cls:
            if(i>0)i--;
            NUMBER[i] = ' ';
            lcd.setCursor(9+i, 3);
            lcd.print(' ');
        break;

        case ok:
          Status.Get_Num = 0;
            Status.PID_OK = 1;
        break;

        case exit:
          Status.Get_Num = 0;
            Status.PID_OK = 0;
        break;

        case up:
        break;

        case down:
        break;

        case dot:
          if(i<6)
          {
            NUMBER[i] = '.';
            lcd.setCursor(9+i, 3);
            lcd.print(NUMBER[i]);
            i++;
          }
        break;

        default:
          if(i<7)
          {
            NUMBER[i] = Num + '0';
            lcd.setCursor(9+i, 3);
            lcd.print(NUMBER[i]);
            i++;
          }
        break;
      };

    }
    Prev_Num = Num;
    delay(1);
  }
  Status.whiles=1;
  lcd.clear();
}