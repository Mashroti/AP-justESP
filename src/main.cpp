#include <Arduino.h>
#include <Servo.h>
#include "Wire.h"
#include "I2CKeyPad.h"
#include <LiquidCrystal_I2C.h>
#include <string.h>
#include <pins_arduino.h>


#define cls             10
#define up              11
#define down            12
#define exit            13
#define ok              14
#define dot             15

#define Min_Throttle  46
#define Max_Throttle  156

#define lcd_puts_XY(x,y,msg) {\
 lcd.setCursor(x,y);\
 lcd.print(msg);\
}
    

//#define BrushlessMotorPin 15      //D8 is GPIO15
#define BrushlessMotorPin D0
//**********************Encoder Variable*****************************
#define Encoder_DT    14
#define Encoder_CLK   12
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
	uint8_t exe		  : 1;
}Status;

/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
char GET_KEY  (void);
void MATLAB   (void);
void startup (void);
void Process_UART_Data(char* Data);
char GET_KEY(void);
void KeyPad(void);
void Volume(void);
void Verify_Unique(void);
void PID_setting(void);
void Get_Number (char *NUMBER);
void key_analyze(void);
void Online(void);
void WiFi_Setting(void);


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
  esc.write(Min_Throttle);   // tell servo to go to position (0 - 180)
  
  startup();
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
           if(Num == 2)	MATLAB();
      else if(Num == 3)	KeyPad();
      else if(Num == 4)	Volume();
      else if(Num == 5)	WiFi_Setting();
      else if(Num == 1) KeyPad();
      

      lcd.clear();
      lcd_puts_XY(0,0,"1> Online");
      lcd_puts_XY(0,1,"2> Matlab");
      lcd_puts_XY(0,2,"3> Offline");
      lcd_puts_XY(0,3,"4> Win App");
    }
    Num = GET_KEY();
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
	  for(uint8_t i=0 ;i<19 ;i++)
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
    while (!data) 
    {
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

        delay(2);  //slow looping to allow buffer to fill with next character
      }
      delay(1);
    }
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
      }      
      data = false;
    }
  }
  lcd.backlight();
}
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
void key_analyze(void)
{
  uint8_t key = GET_KEY();
  switch (key)
  {
    case exit:
      Status.whiles = 0;
      esc.write(Min_Throttle);
      break;
    case 0:
      Encoder_quad = 0;
      break; 
    case cls:
      Status.motor ^= 1;
      if(Status.motor == 0) esc.write(Min_Throttle);
      break;   

    default:
      break;
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

