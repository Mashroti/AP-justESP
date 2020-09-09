#include <Arduino.h>
#include <Servo.h>
#include "Wire.h"
#include "I2CKeyPad.h"
#include <LiquidCrystal_I2C.h>
#include <string.h>


#define BrushlessMotorPin 15      //D8 is GPIO15

//**********************Encoder Variable*****************************
#define Encoder_DT    9
#define Encoder_CLK   10
int16_t  Encoder_quad = 0;
int16_t Encoder_previous_data;
void ICACHE_RAM_ATTR Encoder_read();
/********************************************************************/

const uint8_t KEYPAD_ADDRESS  = 0x23;
const uint8_t LCD_ADDRESS     = 0x27;
I2CKeyPad         keyPad;
LiquidCrystal_I2C lcd(LCD_ADDRESS, 20, 4);
Servo esc;  // create servo object to control a servo

/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
char GET_KEY(void);




/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  //***************************** LCD I2C ******************************************
  lcd.init();
  lcd.home();
  lcd.backlight();
  //***************************** Encoder Setup ************************************
  pinMode(Encoder_DT,  INPUT);
  pinMode(Encoder_CLK, INPUT);
  Encoder_previous_data = digitalRead(Encoder_DT) << 1 | digitalRead(Encoder_CLK);
  attachInterrupt(Encoder_DT,  Encoder_read, CHANGE);
  attachInterrupt(Encoder_CLK, Encoder_read, CHANGE);
  //***************************** Keypad I2C ***************************************
  Wire.begin();
  Wire.setClock(100000);
  if (keyPad.begin(KEYPAD_ADDRESS) == false)
  {
    Serial.println("\nERROR: cannot communicate to keypad.\nPlease reboot.\n");
    while(1);
  }
  //***************************** Speed Controller *********************************
  esc.attach(BrushlessMotorPin); // attaches the servo on GPIO15(D8) to the servo object
  esc.write(0);   // tell servo to go to position (0 - 180)
}
/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/

void loop()
{

  delay(500);
}

/***************************************************************************
****************************************************************************
****************************************************************************
***************************************************************************/
char GET_KEY(void)
{
  char keys[] = "123A456B789C*0#DNF";  // N = Nokey, F = Fail
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

