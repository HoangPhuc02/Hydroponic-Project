#ifndef __LCD_MENU_H
#define __LCD_MENU_H
/* Write by Phuc Phan Hoang*/

#include "stdio.h"
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "Sensor_Actuator.h"
#include <String>




//#define MAX_TITLE_LENGTH 16   //Max chacracter on a row here example on 16x2 LCD

#define MAX_TITLE_LENGTH 20   //Max chacracter on a row here example on 125x64 OLed
#define MAX_MENU_ITEM 4 
struct Button
{
    /* data */
};

//extern sensor_init_t sensor;
// typedef void (*fp_ActivationOn)(char,char);
// typedef void (*fp_ActivationOff)(char,char);
// typedef void (*fp_SensorDisplay)(char,char);

typedef struct Linker 
{
    Linker *pre;
    uint8_t maxSelect; 
    uint8_t cur_cursor;
    char Title[MAX_TITLE_LENGTH];
    char MenuList[MAX_MENU_ITEM][MAX_TITLE_LENGTH];
    Linker *List[MAX_MENU_ITEM];
    
    void (*fp_ActivationOn)( uint8_t, uint8_t);
    void (*fp_ActivationOff)( uint8_t, uint8_t);
    void (*fp_SensorDisplay)( uint8_t , String[]);
    /* data */
}Menu;

/*char MenuList1[MAX_TITLE_LENGTH];Linker *List1;
    char MenuList2[MAX_TITLE_LENGTH];Linker *List2;
    char MenuList3[MAX_TITLE_LENGTH];Linker *List3;
    char MenuList4[MAX_TITLE_LENGTH];Linker *List4;*/


#define i2c_Address 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
//#define i2c_Address 0x3d //initialize with the I2C addr 0x3D Typically Adafruit OLED's

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO



#define LOGO8_CURSOR_HEIGHT 8
#define LOGO8_CURSOR_WIDTH  8
static const unsigned char logo8_cursor[] =
{ B00000000, 
  B11000000, 
  B11110000, 
  B11111100, 
  B11111111, 
  B11111100, 
  B11110000, 
  B11000000,  

};
/**TODO Add feature*/
/* Display init */
/* Add menu fuction*/

extern Adafruit_SH1106G display ;

extern Menu MainMenu,SensorMenu,WaterSensorMenu,LandSensorMenu,ActuatorMenu,ServoMenu,RelayMenu,LedMenu,SensorShowInfo;
// Main menu



void MenuInit(void);
void MenuDisplay(Menu *menu, uint8_t select);
void ActuatorsActivation( uint8_t Device,  uint8_t State);
void SensorDisplay(uint8_t Sensor_Type, String sensor_data[]);


#endif