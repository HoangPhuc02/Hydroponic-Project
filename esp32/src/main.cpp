/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>

#include <stdio.h>
#include <String>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_flash.h"


#include <RTClib.h>
// #include <Adafruit_SH110X.h>
// #include <Adafruit_GFX.h>

#include <FirebaseESP32.h>
// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

#include "WiFi.h"

#include "Update.h"
#include "LittleFS.h"
#include "SD.h"

#include <Wire.h>
#include <SPI.h>


#include <lcdMenu.h>
#include <UART.h>
#include <Sensor_Actuator.h>

// Set LED_BUILTIN if it is not defined by Arduino framework

#define LED 13

// Config button
#define BUTTON_GPIO_UP 34
#define BUTTON_GPIO_DOWN 35
#define BUTTON_GPIO_ENTER 32
#define BUTTON_GPIO_EXIT 26
#define GPIO_INPUT_PIN_SEL ((1ULL << BUTTON_GPIO_UP) | (1ULL << BUTTON_GPIO_DOWN) | (1ULL << BUTTON_GPIO_ENTER) | (1ULL << BUTTON_GPIO_EXIT))
// interrupt flag define
#define ESP_INTR_FLAG_DEFAULT 0
// debounce time define
#define DEBOUNCE_TIME 200 // 0.2S

// queue handler for button
static QueueHandle_t gpio_evt_queue = NULL;

// last time flag
static uint64_t last_time = 0;

// FUNCTION
static void IRAM_ATTR gpio_isr_handler(void *arg);

static void gpio_task_example(void *arg);
static void configure_button(void);

/*=======================OLED DECLARE============================*/
/* Config i2c for it*/
Menu *menu = &MainMenu;
/* Uncomment the initialize the I2C address , uncomment only one, If you get a totally blank screen try the other*/

/*===============================================================*/


QueueHandle_t actuator_control_queue = NULL;

QueueHandle_t uart2_queue = NULL;



uint8_t actuator_command[2];






/*
 0 : hardware
 1 : firebase
*/


/*======================== Wifi Declare =========================*/

const char *ssid = "MI 8 Pro";      // The SSID (name) of the Wi-Fi network you want to connect to
const char *password = "123456789"; // The password of the Wi-Fi network

// const char *ssid = "hmmm";      // The SSID (name) of the Wi-Fi network you want to connect to
// const char *password = "phucphuc"; // The password of the Wi-Fi network

// const char *ssid = "Symbol Coffee Tea";      // The SSID (name) of the Wi-Fi network you want to connect to
// const char *password = "Symbolcoffee"; // The password of the Wi-Fi network

// const char *ssid = "HíHíHí";      // The SSID (name) of the Wi-Fi network you want to connect to
// const char *password = "hihihi858585"; // The password of the Wi-Fi network

void wifi_init();

/*===============================================================*/
String sensor_data[MAX_SENSOR_DATA] = {"","","","","",""};

/**
 * @brief sensor list
 * temp
 * humi
 * moisture
 * light
 * water level
 * PH
 */

Actuator_Init_t actuators[MAX_DEVICE];

// char actuator_name[3][6] = {"Relay","Servo","Led"};
// char actuator_state[2][4] = {"Off","On"};
/*=================== Real Time Clock Declare ===================*/
RTC_DS3231 rtc;
void rtc_init();
void rtc_set_alarm();
/*===============================================================*/

/*====================== Firebase Declare =======================*/

/* 2. Define the API Key */
#define API_KEY "AIzaSyDgReLUPXPzDwMjOO5U5Y66RuTCFwUBJR0"

/* 3. Define the RTDB URL */
#define DATABASE_URL "https://aquasys-e55d4-default-rtdb.firebaseio.com/" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app

/* 4. Define the user Email and password that alreadey registerd or added in your project */
#define USER_EMAIL "phuchocnhung@gmail.com"
#define USER_PASSWORD "phucphuc"

// Define Firebase Data object
FirebaseData fbdo;
FirebaseData stream;

FirebaseAuth auth;
FirebaseConfig config;
FirebaseJson json;




String parentPath = "/Actuators";
String childPath[2] = {"/Actuators_water","/Actuators_environment"};

String callBackPath;
int callBackNum;

String actuatorsPAth[MAX_DEVICE] = {"/Actuators_water/0","/Actuators_water/2","/Actuators_environment/0"};
String ledPath = "/Actuators_environment/0";
String relayPath = "/Actuators_water/0";
String servoPath = "/Actuators_water/2";

SemaphoreHandle_t semaphore_firebase_is_free;

#define TURN_BY_HW  0
#define TURN_BY_FB  1
/*
0 : hw
*/
uint8_t actuator_turn_hw_or_fb[MAX_DEVICE] = {TURN_BY_HW,TURN_BY_HW,TURN_BY_HW};

volatile bool dataChanged = false;

unsigned long sendDataPrevMillis = 0;

unsigned long count = 0;


QueueHandle_t firebase_actuator_queue = NULL;
QueueHandle_t firebase_sensor_queue = NULL;

uint8_t flag_send_uart = 0;

void show_sensor_display(String sensor_data[]);
void firebase_update_data_task(void *pvParameters);
void firebase_handle_data_changed(void *pvParameters);

void streamCallback(MultiPathStreamData stream);
void streamTimeoutCallback(bool timeout);
void spareCommandChange(void);

void firebase_init(void);
void update_state_fb(uint8_t device, uint8_t state);

void control_actuators(uint8_t device, uint8_t state);

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  semaphore_firebase_is_free = xSemaphoreCreateBinary();
  //actuator_cmd = xQueueCreate(5, sizeof(actuator_command));
  pinMode (LED, OUTPUT);
  digitalWrite (LED, HIGH); 
  Serial.begin (115200);

  configure_button();
  actuator_init();
  MenuInit();
  uart_init();
  wifi_init();
  firebase_init();
  
  //rtc_init();
}

// the loop function runs over and over again forever
void loop() {
  // digitalWrite (LED, HIGH);  // turn the LED on (HIGH is the voltage level)
  // delay (1000);  // wait for a second
  // digitalWrite (LED, LOW); // turn the LED off by making the voltage LOW
  // delay (1000);  // wait for a second0...
  // Serial.print ("loop() running in core ");
  // Serial.println (xPortGetCoreID());
  if (dataChanged)
        {
            dataChanged = false;
            // When stream data is available, do anything here...
        }

        // After calling stream.keepAlive, now we can track the server connecting status
        if (!stream.httpConnected())
        {
            // Server was disconnected!
        }

    if(flag_send_uart == 1)
    {
        control_actuators(actuator_command[0],actuator_command[1]);
        flag_send_uart = 0;
    }
}

/*==================== Button Function ==========================*/
static void gpio_isr_handler(void *arg)
{
    unsigned int gpio_num = (unsigned int)arg;
    uint64_t cur_time = esp_timer_get_time();
    if ((cur_time - last_time) / 1000 >= DEBOUNCE_TIME)
    {
        last_time = cur_time;
        xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
    }
}
static void configure_button(void)
{
    /* Set the GPIO as a push/pull output */
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    // bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    // start gpio task
    xTaskCreatePinnedToCore(gpio_task_example, "gpio_task_example", 8192, NULL, configMAX_PRIORITIES, NULL, 0);
    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

    // hook isr handler for specific gpio pin
    gpio_isr_handler_add((gpio_num_t)BUTTON_GPIO_DOWN, gpio_isr_handler, (void *)BUTTON_GPIO_DOWN);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add((gpio_num_t)BUTTON_GPIO_UP, gpio_isr_handler, (void *)BUTTON_GPIO_UP);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add((gpio_num_t)BUTTON_GPIO_ENTER, gpio_isr_handler, (void *)BUTTON_GPIO_ENTER);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add((gpio_num_t)BUTTON_GPIO_EXIT, gpio_isr_handler, (void *)BUTTON_GPIO_EXIT);

    // printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());
}
static void gpio_task_example(void *arg)
{
    static const char *BUTTON_TAG = "BUTTON";
    unsigned int io_num;
    //esp_log_level_set(BUTTON_TAG, ESP_LOG_INFO);
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY))
        {
          
            Serial.printf("GPIO[%u] intr, val: %d\n", io_num, gpio_get_level((gpio_num_t)io_num));
            // printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
            if (io_num == BUTTON_GPIO_DOWN)
            {
                menu->cur_cursor = (menu->cur_cursor == menu->maxSelect - 1) ? 0 : menu->cur_cursor + 1;
            }
            else if (io_num == BUTTON_GPIO_UP)
            {
                menu->cur_cursor = (menu->cur_cursor == 0) ? menu->maxSelect - 1 : menu->cur_cursor - 1;
            }
            else if (io_num == BUTTON_GPIO_ENTER)
            {
                uint8_t cursor = menu->cur_cursor;
                if (menu->List[cursor] != NULL)
                {
                    menu = menu->List[cursor];
                }
                else if (cursor == 0 && menu->fp_ActivationOn != NULL && actuators[menu->pre->cur_cursor].ActuatorState == ACTUATOR_OFF) // check if it cursors to activate on actuator
                {
                    menu->fp_ActivationOn((uint8_t)(menu->pre->cur_cursor), (uint8_t)ACTUATOR_ON);
                }
                else if (cursor == 1 && menu->fp_ActivationOff != NULL && actuators[menu->pre->cur_cursor].ActuatorState == ACTUATOR_ON) // check if it cursors to activate off actuator
                {
                    menu->fp_ActivationOff((uint8_t)(menu->pre->cur_cursor), (uint8_t)ACTUATOR_OFF);
                }
                else if(menu->fp_SensorDisplay != NULL)
                {
                    show_sensor_display(sensor_data);
                }
            }
            else if (io_num == BUTTON_GPIO_EXIT)
            {
                menu = (menu->pre == NULL) ? menu : menu->pre;
            }
            MenuDisplay(menu, menu->cur_cursor);
        }
    }
}
/*===========================================================*/



/*=================== Menu OLED function ===================*/
// it declare in menuLCD lib as weak so you can  redeclare
void ActuatorsActivation(uint8_t Device, uint8_t State)
{
    actuator_turn_hw_or_fb[Device] = TURN_BY_HW;
    actuator_command[0] = Device;
    actuator_command[1] = State;
    flag_send_uart = 1;
    //xQueueSend()
    //control_actuators(Device, State);
 
}
void SensorDisplay(uint8_t Sensor_Type, String sensor_data[])
{

}

void show_sensor_display(String sensor_data[])
{
   
  if(menu == &LandSensorMenu)
  {
    menu = &SensorShowInfo;
    menu->pre = &LandSensorMenu;
    strcpy(menu->Title,(const char*)LandSensorMenu.MenuList[LandSensorMenu.cur_cursor]);
    Serial.println(sensor_data[LandSensorMenu.cur_cursor]);
    strcpy(menu->MenuList[0]+3,sensor_data[LandSensorMenu.cur_cursor].c_str());

  }
  else if(menu == &WaterSensorMenu)
  {
    menu = &SensorShowInfo;
    menu->pre = &WaterSensorMenu;
    strcpy(menu->Title,(const char*)WaterSensorMenu.MenuList[WaterSensorMenu.cur_cursor]);
    Serial.println(sensor_data[WaterSensorMenu.cur_cursor+4]);
    strcpy(menu->MenuList[0]+3,sensor_data[WaterSensorMenu.cur_cursor+4].c_str());

  }
}

void update_sensor_data_from_uart(void)
{
     xSemaphoreGive(semaphore_firebase_is_free);
    if(menu ==  &SensorShowInfo)
    {
        if(menu->pre == &LandSensorMenu)
        {
            strcpy(menu->MenuList[0]+3,sensor_data[LandSensorMenu.cur_cursor].c_str());
        }
        else if(menu->pre == &WaterSensorMenu)
        {
            strcpy(menu->MenuList[0]+3,sensor_data[WaterSensorMenu.cur_cursor+4].c_str());
        }
        MenuDisplay(menu, menu->cur_cursor);
    }
}
/*==========================================================*/

/*================= Real Time Clock Function ===============*/
void rtc_init()
{
    if (!rtc.begin())
    {
        Serial.printf("Couldn't find RTC");
        while (1)
            vTaskDelay(10);
    }
    else
         Serial.printf( "Find RTC");

    // Check if RTC lost power and if so, set the time
    if (rtc.lostPower())
    {
         Serial.printf("RTC lost power, let's set the time!");
        // this will adjust to the date and time at compilation
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    // we don't need the 32K Pin, so disable it
    rtc.disable32K();

    // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
    // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);
}
/*==========================================================*/

/*=====================Wifi Firebase=======================*/

void wifi_init()
{
    //nvs_flash_init(); // TODO explain why we need this mother fuckeerrrrrrrr
    WiFi.begin(ssid, password);
    Serial.printf("WIFI", "Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("You're now fucking connect to wifi");
    // ESP_LOGI("WIFI",(string)WiFi.localIP());
}

/*=========================================================*/


/*====================== Firebase Function =======================*/
void firebase_init(void)
{
    /* Assign the api key (required) */
    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the user sign in credentials */
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

  // Comment or pass false value when WiFi reconnection will control by your code or third party library e.g. WiFiManager
  Firebase.reconnectNetwork(true);

  // Since v4.4.x, BearSSL engine was used, the SSL buffer need to be set.
  // Large data transmission may require larger RX buffer, otherwise connection issue or data read time out can be occurred.
  fbdo.setBSSLBufferSize(4096 /* Rx buffer size in bytes from 512 - 16384 */, 1024 /* Tx buffer size in bytes from 512 - 16384 */);

  // The WiFi credentials are required for Pico W
  // due to it does not have reconnect feature.


  // Or use legacy authenticate method
  // config.database_url = DATABASE_URL;
  // config.signer.tokens.legacy_token = "<database secret>";

  // To connect without auth in Test Mode, see Authentications/TestMode/TestMode.ino

  Firebase.begin(&config, &auth);

  // You can use TCP KeepAlive For more reliable stream operation and tracking the server connection status, please read this for detail.
  // https://github.com/mobizt/Firebase-ESP8266#enable-tcp-keepalive-for-reliable-http-streaming
  // You can use keepAlive in ESP8266 core version newer than v3.1.2.
  // Or you can use git version (v3.1.2) https://github.com/esp8266/Arduino
  // stream.keepAlive(5, 5, 1);
if (!Firebase.beginMultiPathStream(stream, parentPath))
    Serial.printf("sream begin error, %s\n\n", stream.errorReason().c_str());

  Firebase.setMultiPathStreamCallback(stream, streamCallback, streamTimeoutCallback);

  //  firebase_actuator_queue =  xQueueCreate(5, sizeof(Actuator_Init_t));
   // firebase_sensor_queue = xQueueCreate(12,5);
    // Create a semaphore to synchronize data updates
    
  //  xTaskCreatePinnedToCore(firebase_update_data_task, "firebase_update_data_task",16384, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    xTaskCreatePinnedToCore(firebase_update_data_task, "firebase_update_data_task",8192, NULL, configMAX_PRIORITIES, NULL, 1);
   // xTaskCreatePinnedToCore(firebase_handle_data_changed, "firebase_handle_data_changed",1024, NULL, configMAX_PRIORITIES -1, NULL, 1);
    // xQueueSend(firebase_actuator_queue, &actuators[0], portMAX_DELAY);
    // xQueueSend(firebase_actuator_queue, &actuators[1], portMAX_DELAY);
    // xQueueSend(firebase_actuator_queue, &actuators[2], portMAX_DELAY);
}


/*
 actuator_config(&actuators[0], Relay, ACTUATOR_OFF);        
    actuator_config(&actuators[1], Servo, ACTUATOR_OFF),
    actuator_config(&actuators[2], Led, ACTUATOR_OFF),
*/
void firebase_handle_data_changed(void *pvParameters)
{
    while(1)
    {
        if (dataChanged)
        {
            dataChanged = false;
            // When stream data is available, do anything here...
        }

        // After calling stream.keepAlive, now we can track the server connecting status
        if (!stream.httpConnected())
        {
            // Server was disconnected!
        }
    }
    vTaskDelete(NULL);
}
void firebase_update_data_task(void *pvParameters)
{
    while(1)
    {
    if( xSemaphoreTake(semaphore_firebase_is_free, 10))
      {
        sendDataPrevMillis = millis();
        Serial.println( millis());
        Serial.println("send");

      // Serial.printf("Set int... %s\n", Firebase.setInt(fbdo, F("/Actuators_environment/0/status"), count) ? "ok" : fbdo.errorReason().c_str());
//                 Firebase.setString(fbdo, F("/Sensors_environment/1/value"),sensor_data[0]); 
//                 Firebase.setString(fbdo, F("/Sensors_environment/0/value"),sensor_data[1])); 
//                 Firebase.setString(fbdo, F("/Sensors_environment/3/value"),sensor_data[2]); 
//                 Firebase.setString(fbdo, F("/Sensors_environment/2/value"),sensor_data[3]); 
//                 Firebase.setString(fbdo, F("/Sensors_water/1/value"),sensor_data[4]) ;
//                 Firebase.setString(fbdo, F("/Sensors_water/0/value"),sensor_data[5]) ;

        json.set("Humi/", sensor_data[0]);
        json.set("Light/",sensor_data[2]);
        json.set("Moisture/", sensor_data[3]);
        json.set("Temp/", sensor_data[1]);
        json.set("Water_level/", sensor_data[4]);
        json.set("pH/", sensor_data[5]);
        Serial.printf("Set json... %s\n", Firebase.set(fbdo, F("/val_sensor"), json) ? "ok" : fbdo.errorReason().c_str());
        Serial.println( millis());
        //xSemaphoreGive(semaphore_firebase_is_free);
        vTaskDelay(10000);
      }
    }
     vTaskDelete(NULL);
}
void streamCallback(MultiPathStreamData stream)
{
  size_t numChild = sizeof(childPath) / sizeof(childPath[0]);

  for (size_t i = 0; i < numChild; i++)
  {
    if (stream.get(childPath[i]))
    {
      Serial.printf("path: %s, event: %s, type: %s, value: %s%s", stream.dataPath.c_str(), stream.eventType.c_str(), stream.type.c_str(), stream.value.c_str(), i < numChild - 1 ? "\n" : "");
      callBackPath = stream.dataPath;
      callBackNum =  atoi(stream.value.c_str());
      
    }
  }

  Serial.println();
    
  // This is the size of stream payload received (current and max value)
  // Max payload size is the payload size under the stream path since the stream connected
  // and read once and will not update until stream reconnection takes place.
  // This max value will be zero as no payload received in case of ESP8266 which
  // BearSSL reserved Rx buffer size is less than the actual stream payload.
  //Serial.printf("Received stream payload size: %d (Max. %d)\n\n", stream.payloadLength(), stream.maxPayloadLength());

  // Due to limited of stack memory, do not perform any task that used large memory here especially starting connect to server.
  // Just set this flag and check it status later.
  dataChanged = true;
  spareCommandChange();
}
void spareCommandChange(void)
{
    uint8_t device;
    for(int i = 0 ; i < MAX_DEVICE; i++)
    {
        if(strstr(callBackPath.c_str(),actuatorsPAth[i].c_str())!= NULL)
        {
            device = i;
        }
    }
    if(strstr(callBackPath.c_str(),"status" )!= NULL)
    {
    
        actuator_turn_hw_or_fb[device] = TURN_BY_FB;
        control_actuators(device,callBackNum);
    }
}
void streamTimeoutCallback(bool timeout)
{
  if (timeout)
    Serial.println("stream timed out, resuming...\n");

  if (!stream.httpConnected())
    Serial.printf("error code: %d, reason: %s\n\n", stream.httpCode(), stream.errorReason().c_str());
}


void control_actuators(uint8_t device, uint8_t state)
{
    set_state_actuator(&actuators[device], (Actuator_State_t)state);
    if(actuator_turn_hw_or_fb[device] == TURN_BY_FB) 
        Serial.println("turn by fb");
    //TODO update firebase;
    else if(actuator_turn_hw_or_fb[device] == TURN_BY_HW) 
    {
        update_state_fb( device, state);
        Serial.println("turn by hw");
    }
    //TODO update firebase;
    xQueueSend(actuator_control_queue, &actuators[device], portMAX_DELAY);
}

void update_state_fb(uint8_t device, uint8_t state)
{
    while(1)
    {
        if(Firebase.ready())
        {
        String dataPath1 = parentPath + actuatorsPAth[device]+ "/flag";
        String dataPath2 = parentPath + actuatorsPAth[device]+ "/status";
        Serial.printf("Set bool... %s\n",Firebase.setBool(fbdo, dataPath1 ,false)? "ok" : fbdo.errorReason().c_str());
        Serial.printf("Set int... %s\n",Firebase.setInt(fbdo, dataPath2 ,state)? "ok" : fbdo.errorReason().c_str());
        break;
        }
    }
}
// void firebase_check_actuator_state(Actuator_Init_t * actuator, uint8_t state )
// {
//     for(uint8_t i = 0 ; i < MAX_DEVICE; i++)
//     {
//         if(actuators[i]->ActuatorState != *state[i])
//         ActuatorsActivation(i, *state[i]);
//     }
// }
/*================================================================*/

// /*======================= SENSOR function ========================*/
// void config_data_sensor(char* data, void* sensor)
// {
    
// }
// /*================================================================*/