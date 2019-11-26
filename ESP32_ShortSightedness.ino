
/*
    ESP32-近视仪
    -20190808
*/

#include <Wire.h>
#include <BH1750.h>
#include <U8g2lib.h>
#include <VL53L0X.h>
#include <Ticker.h>  //Ticker Library
 
#include <WiFi.h>
#include <Update.h>

#include "TridentTD_ESP32NVS.h"  //NVS

#include "BLE\BLEDevice.h" 
#include "BLE\BLEServer.h"
#include "BLE\BLEUtils.h"
#include "BLE\BLE2902.h"

#include <sdkconfig.h >
#include <esp_sleep.h>
#include <esp_pm.h>
#include "dx_lcd_map.h"



//WiFiClient client;

//@-Display for Use HT1621
//#include <HT1621.h> // include our library

//@-新增I2C
TwoWire Wire2 = TwoWire(3);
//#include <AT24CX.h>  


//@-按键
#define Button1_Pin            32
#define Button2_Pin            33
#define Button3_Pin            27
#define Button4_Pin            26
//@-光感和测距供用同一I2C总线
#define VL53L0X_BH1750_SCL_Pin 22
#define VL53L0X_BH1750_SDA_Pin 21
//@-OLED的I2C总线
#define OLED_SCL_Pin           5
#define OLED_SDA_Pin           18
//@-系统功能Pin
#define Power_Ctl_Pin          2
#define LED_Pin                17
#define BAT_Pin                36
#define Tone_Pin               23
#define Check_DX1_Pin          14
#define Check_DX2_Pin          12

//@-测距功能选择高精度
#define HIGH_ACCURACY
// #define LONG_RANGE

//@-无人检测
#define Check_Preson_Size 10
#define person_no  0
#define person_yes 1
#define person_unknow 3

//@-菜单相关
#define Menu_Max_Size 4

//@-主菜单显示相关
#define MainPage_DisMode_2         2
#define MainPage_DisMode_3         3
#define MainPage_DisMode_EVENT     4   

//@-设备事件
#define DEV_EVENT_NONE                 0
#define DEV_EVENT_Tomato_25min_TimerUp 1
#define DEV_EVENT_Tomato_5min_TimerUp  2
#define DEV_EVENT_DISTANCE_OVER_LIMT   3
#define DEV_EVENT_LIGHT_OVER_LIMT      4
#define DEV_EVENT_CLOSE                5
#define DEV_EVENT_DISTANCE_IGNORE_30   6
#define DEV_EVENT_LIGHT_IGNORE_30      7

#define DEV_EVENT_BATTERY_LOW          8
#define DEV_EVENT_BATTERY_OK           9
#define DEV_EVENT_CHARGE_IN            10
#define DEV_EVENT_CHARGE_OUT           11

//@-定时相关
#define Timer_5min   1 * 60
#define Timer_25min  25 * 60

//@-测光等级相关
#define Light_Level_High   1
#define Light_Level_Normal 2
#define Light_Level_Low    3

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


//#define AT24C32_SCL_Pin 16
//#define AT24C32_SDA_Pin 4
// EEPROM object
//A0-A1-A2-High
//AT24C32 mem(7);


//@-创建HT1621设备
//HT1621 lcd; // create an "lcd" object

/*
  BH1750 can be physically configured to use two I2C addresses:
    - 0x23 (most common) (if ADD pin had < 0.7VCC voltage)
    - 0x5C (if ADD pin had > 0.7VCC voltage)
  Library uses 0x23 address as default, but you can define any other address.
  If you had troubles with default value - try to change it to 0x5C.
*/
//@-创建测光传感器
BH1750 lightMeter(0x23);
//@-测光超限tick
int Light_Alarm_Tick = 0;                //@--测光超限tick
//@-测光超限忽略tick
int Light_Alarm_Ignore_Tick = 0;         //@--测光超限忽略tick

//@-创建测距传感器
VL53L0X DistanceSensor;
//@-测距超限tick
int Distance_Alarm_Tick = 0;             //@--测距超限tick
//@-测距超限忽略tick
int Distance_Alarm_Ignore_Tick = 0;      //@--测距超限忽略tick


//@-创建显示设备
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL_Pin, /* data=*/ OLED_SDA_Pin, /* reset=*/ U8X8_PIN_NONE);   // Adafruit Feather M0 Basic Proto + FeatherWing OLED
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL_Pin, /* data=*/ OLED_SDA_Pin, /* reset=*/ U8X8_PIN_NONE);

//@-光强
float lux = 350;
int   lux_level = Light_Level_Normal;

//@-测距
float distance = 350;
int   distance_limit = 300;

//@-tick相关
int lux_tick = 0;
int distance_tick = 0;
int bat_check_tick = 0;
int bat_tick = 0;
int oled_tick = 0;
int ble_tick = 0;

//@-系统脉冲
Ticker Tick1;
Ticker Tick2;
int tick = 0;
int System_Second = 0;
bool flash_flag = false;
bool Logo_Flag = true;

//@-检测有无人相关
int check_person_index = 0;
int check_have_person[Check_Preson_Size];
int check_no_person_time = 0;
int check_have_person_time = 0;

//@-屏幕关闭标志
int  Screen_Close_Flag = 0;       //@-0:不关  1:关闭中  2:已关闭
bool Screen_Close_Lock = false;  //@-屏幕关闭锁

//@-键盘相关
int Power_Button_State = 0;     //@-电源键状态
int Power_PressOFF_Tick = 0;       //@-电源键长按状态
bool Power_Press_Close_Flag = false; //@-电源键启动关机功能标志
int USB_Button_State = 1;       //@-充电状态
int USB_Button_State_Copy = 1;  //@-充电状态Copy
bool USB_Button_State_Change = false;  //@-充电状态改变
int  USB_Button_State_Change_ShowTick = 500; //@-充电状态改变展示时间tick
int None_Button_Press_Tick = 0; //@-没有按键按下     
int Button1_State = 0;          //@-按键1状态
int Button2_State = 0;          //@-按键2状态
int Button3_State = 0;          //@-按键3状态
int Button4_State = 0;          //@-按键4状态
int Button4_Press_Tick = 0;     //@-按键4长按状态监测
bool Button4_LongPress_Lock = false;  //@-按键4长按锁标志
bool Key_Flag = false;


//------------------------------OTA-------------------------------------------------
//// Variables to validate
//// response from S3
//int contentLength = 0;
//bool isValidContentType = false;
 
// Your SSID and PSWD that the chip needs
// to connect to

const char* ssid = "8879";
const char* password = "blackbug381";

//const char* ssid       = "wuyiyi";
//const char* password   = "10238831";

int OTA_Tick = 0;
int OTA_Updata_Tick = 0;
bool OTA_Start = false;

//@13-多任务
TaskHandle_t Task1;

////const char* ssid = "8879";
////const char* password = "blackbug381";
// 
//// S3 Bucket Config
//String host = "www.dx1023.com"; // Host => bucket-name.s3.region.amazonaws.com
//int port = 80; // Non https. For HTTPS 443. As of today, HTTPS doesn't work.
//String bin = "/media/images/ESP32_ShortSightedness.ino.esp32.bin"; // bin file name with a slash in front.
//----------------------------------------------------------------------------------


//@-锂电电量检测相关
int Signal = 0;
float BAT_V = 0;
float BAT_BB[10];
float BAT_B = 0;
float BAT_Sum = 0;
int  BAT_Info_ShowTick = 0;

//@-设备事件
int Dev_event  = DEV_EVENT_NONE;  //0:无事件  1:番茄管理

//@-菜单操作相关
int Menu_Index = 0;       //@-0:主菜单  1:防疲劳  2:告警方式  3:测距设置  4:环境光强
int Menu_Option_Set = 0;  //@-0:主界面  1:防疲劳配置  2:告警方式配置  3:测距设置  4:环境光强

//@-防疲劳功能相关
int Studay_Pro_Index = 0;       //@-0:防疲劳30min  1:防疲劳45min  2:防疲劳60min
int Studay_Pro_Index_temp = 0;  //@-0:防疲劳30min  1:防疲劳45min  2:防疲劳60min
bool Studay_Pro_Flag = false;   //@-防疲劳功能项配置标志

//@-番茄功能相关
int Tomato_Pro_Index = 0;               //@-0:番茄钟25min  1:速算5min   2:计时器
int Tomato_Pro_Index_temp = 0;          //@-0:番茄钟25min  1:速算5min   2:计时器
bool Tomato_Pro_Flag = false;           //@-番茄管理功能项配置标志
bool Tomato_Run = false;                //@-番茄管理运行标志
bool Tomato_Timer_Pause = false;        //@-番茄管理-计时器暂停标志
int  Tomato_25min_Timer = Timer_25min;  //@-25min定时器
int  Tomato_5min_Timer = Timer_5min;    //@-5min定时器
int  Tomato_Timer = 0;                  //@-计时器
bool Tomato_Page_Lock = false;          //@-番茄功能上锁标志
int  Tomato_Page_Lock_Tick = 0;         //@-番茄功能上锁时间tick
int  Tomato_Alarm_Tick = 0;             //@-番茄功能超时tick
int  Power_OFF_Tick = 10;               //@-设备关机倒计时10s

//@-告警方式功能相关
int Alarm_Pro_Index = 0;       //@-0:声+屏  1:屏   
int Alarm_Pro_Index_temp = 0;  //@-0:声+屏  1:屏   
bool Alarm_Pro_Flag = false;   //@-告警设置功能项配置标志

//@-测距设置功能相关
int Distance_Pro_Index = 0;       //@-0:20cm  1:30cm  2:40cm  3:50cm  4:60cm
int Distance_Pro_Index_temp = 0;  //@-0:20cm  1:30cm  2:40cm  3:50cm  4:60cm
bool Distance_Pro_Flag = false;   //@-测距设置功能项配置标志

//@-环境光强功能相关
bool Light_Pro_Flag = false;      //@-环境光强功能项配置标志

//@-主页面显示相关
int MainPage_DisMode = MainPage_DisMode_3;        //@-主页面显示模式   MainPage_DisMode_2：2项显示   MainPage_DisMode_3：3项显示  
int MainPage_DisMode_Save = MainPage_DisMode_3;   //@-主页面显示模式保存
bool MainPage_DisMode_Save_Lock = false;          //@-显示模式保存锁
int MainPage_Exe_DisMode = 1;                     //@-主页面扩展显示项  0：无
 
//@-随机数相关
int random_num;

//@-logo界面相关
int Logo_Run_Tick = 0;
const unsigned char *logo_name[35]={col_logo1,col_logo2,col_logo3,col_logo4,col_logo5, col_logo6,col_logo7,col_logo8,col_logo9,col_logo10,
                                   col_logo11,col_logo12,col_logo13,col_logo14,col_logo15, col_logo16,col_logo17,col_logo18,col_logo19,col_logo20,
                                   col_logo21,col_logo22,col_logo23,col_logo24,col_logo25, col_logo26,col_logo27,col_logo28,col_logo29,col_logo30,
                                   col_logo31,col_logo32,col_logo33,col_logo34,col_logo35};


//@-BLE相关
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint8_t txValue = 0;
uint8_t RXbuffer[256];
char  BLE_Msg[256];
String BLE_Msg_Input;
bool BLE_Rece_Flag = false;
uint8_t SendData[10];


//@-ESP32深度睡眠
void sleep_start()
{
  esp_deep_sleep_start();  
}

//@-检测有无人
void check_person()
{
    bool flag = true;
  
    //@-检测数组index自增
    check_person_index = check_person_index + 1;
    if(check_person_index >= Check_Preson_Size)
    {
        check_person_index = 0; 
    } 
    
    //@-检测到没有人
    if(distance > 8000)
    {
        check_have_person[check_person_index] = person_no;
        
        check_no_person_time = check_no_person_time + 1;
        if(check_no_person_time > 1200)  //@-120s检测到没有人，关机
        {
            if(Screen_Close_Lock == false)
            {
              Screen_Close_Lock = true;
              Screen_Close_Flag = 1;
            }
            
            if(Screen_Close_Flag == 2)
            {
              sleep_start();
            }
        }
    }
    //@-检测到有人
    else 
    {
       check_have_person[check_person_index] = person_yes;

        //@-检测数组都是有人
        for(int temp = 0; temp < 9; temp++)
        {
            //@-数组中有无人数据
            if(check_have_person[temp] == person_no)
            {  
                flag = false;
                break;
            }
        }

       //@-最近10次都检测到有人
       if(flag == true)
       {
          
          check_have_person_time = check_have_person_time + 1;
          if(check_have_person_time > 6) //@-1min中检测到有人 
          {
            //@-无人记时清零
            check_no_person_time = 0;
          }
       }
    }
}

//@-1秒tick
void Second_test()
{
    //@-Tomato_Run
    if(Tomato_Run == true)
    {
      //@-Tomato运行方式-25min番茄钟
      if(Tomato_Pro_Index == 0)
      {
        Tomato_25min_Timer = Tomato_25min_Timer - 1;
        if(Tomato_25min_Timer <= 0)
        {
          Tomato_Run = false;
          Tomato_25min_Timer = Timer_25min;
          //@-设备关机优先级最高
          if(Dev_event != DEV_EVENT_CLOSE)
          Dev_event = DEV_EVENT_Tomato_25min_TimerUp;  //@-25min时间到-事件立即赋值
        }
      }
      //@-Tomato运行方式-5min速算
      else if(Tomato_Pro_Index == 1)
      {
        Tomato_5min_Timer = Tomato_5min_Timer - 1;
        if(Tomato_5min_Timer <= 0)
        {
          Tomato_Run = false;
          Tomato_5min_Timer = Timer_5min;
          //@-设备关机优先级最高
          if(Dev_event != DEV_EVENT_CLOSE)
          Dev_event = DEV_EVENT_Tomato_5min_TimerUp;  //@-5min时间到-事件立即赋值
        }
      }
      //@-Tomato运行方式-计时器
      else if(Tomato_Pro_Index == 2)
      {
        //@-计时器功能没有暂停
        if(Tomato_Timer_Pause == false)
        {
          Tomato_Timer = Tomato_Timer + 1;
          if(Tomato_Timer >= 5940) //@-计时器大于99min停止
          {
            Tomato_Run = false;   //@-计时器停止工作 
          }
        }
      }
    }
}


//@-0.1秒tick挂载接口
void Second_Create()
{
  tick = tick + 1;
 
  //@-tick秒脉冲
  if(tick > 100)
  {
    tick = 0;
    System_Second = System_Second + 1;


    //@-功能操作在logo后启动
    if(System_Second > 5)
    {
        Logo_Flag = false;

        //@-检测有无人
        check_person();

        // Serial.print("no person:");
        // Serial.println(check_no_person_time);
        // Serial.print("have person[9]:");
        // Serial.println(check_have_person[9]);
        // Serial.println(Screen_Close_Flag);

        // Serial.println(Dev_event);
        // Serial.println(Tomato_Timer_Pause);

        //@-番茄功能上锁
        if(Tomato_Page_Lock == true)
        {
          Tomato_Page_Lock_Tick = Tomato_Page_Lock_Tick + 1;
          if(Tomato_Page_Lock_Tick > 2)
          {
            Tomato_Page_Lock = false;
            Tomato_Page_Lock_Tick = 0;
          }
        }

        //@-监测到有定时器到-告警超过30s后停止告警
        if((Dev_event == DEV_EVENT_Tomato_25min_TimerUp) || (Dev_event == DEV_EVENT_Tomato_5min_TimerUp))
        {
            Tomato_Alarm_Tick = Tomato_Alarm_Tick + 1;
            if(Tomato_Alarm_Tick > 30)
            {
              Tomato_Alarm_Tick = 0;
              if((Dev_event == DEV_EVENT_Tomato_25min_TimerUp) || (Dev_event == DEV_EVENT_Tomato_5min_TimerUp))
              {
                Dev_event = DEV_EVENT_NONE;
              }
            }
        }
        //@-关机事件
        else if(Dev_event == DEV_EVENT_CLOSE)
        {
          Power_OFF_Tick = Power_OFF_Tick - 1;
          if(Power_OFF_Tick == 0)
          {
            //@-执行关机
            digitalWrite(Power_Ctl_Pin, LOW); 
          }
        }

        //@-测距超限事件
        else if(Dev_event == DEV_EVENT_DISTANCE_OVER_LIMT)
        {
          Distance_Alarm_Tick = Distance_Alarm_Tick + 1;
          if(Distance_Alarm_Tick > 30)
          {
            Distance_Alarm_Tick = 0;
            if(Dev_event == DEV_EVENT_DISTANCE_OVER_LIMT)
            Dev_event = DEV_EVENT_DISTANCE_IGNORE_30;  //@-测距超限事件暂时忽略30s
          }
        }
        //@-测距功能暂时忽略30s事件
        else if(Dev_event == DEV_EVENT_DISTANCE_IGNORE_30)
        {
          Distance_Alarm_Ignore_Tick = Distance_Alarm_Ignore_Tick + 1;
          if(Distance_Alarm_Ignore_Tick > 30)
          {
            Distance_Alarm_Ignore_Tick = 0;
            if(Dev_event == DEV_EVENT_DISTANCE_IGNORE_30)
            Dev_event = DEV_EVENT_NONE;
          }
        }

        //@-测光超限事件
        else if(Dev_event == DEV_EVENT_LIGHT_OVER_LIMT)
        {
          Light_Alarm_Tick = Light_Alarm_Tick + 1;
          if(Light_Alarm_Tick > 30)
          {
            Light_Alarm_Tick = 0;
            if(Dev_event == DEV_EVENT_LIGHT_OVER_LIMT)
            Dev_event = DEV_EVENT_LIGHT_IGNORE_30;  //@-测光超限事件暂时忽略30s
          }
        }
        //@-测光功能暂时忽略30s事件
        else if(Dev_event == DEV_EVENT_LIGHT_IGNORE_30)
        {
          Light_Alarm_Ignore_Tick = Light_Alarm_Ignore_Tick + 1;
          if(Light_Alarm_Ignore_Tick > 30)
          {
            Light_Alarm_Ignore_Tick = 0;
            if(Dev_event == DEV_EVENT_LIGHT_IGNORE_30)
            Dev_event = DEV_EVENT_NONE;
          }
        }

        
    }

    //@-开始20s后可以启动关机
    if(System_Second > 20)
    {
      Power_Press_Close_Flag = true;
    }

    //@-运行指示
    if(flash_flag == false)  
    {
        flash_flag = true;
        // digitalWrite(LED_Pin, HIGH); 
        // ledcWrite(1, 100); 
    }  
    else if(flash_flag == true)  
    {
        flash_flag = false;
        // digitalWrite(LED_Pin, LOW); 
        // ledcWrite(1, 0); 
    }  
    
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();


      if( rxValue.length() < 256)
      {
          memset(BLE_Msg, 0, sizeof(BLE_Msg));
          for (int i = 0; i < rxValue.length(); i++)
          {
            BLE_Msg[i] = rxValue[i];
            BLE_Msg_Input.setCharAt(i,rxValue[i]);
          }
      }

      // R = rxValue[0];
      // G = rxValue[1];
      // B = rxValue[2];
      
      // if(rxValue[3] == '0')
      // Led_Num =0;

      //@-鸣音
      ledcWrite(0, 90); 
      delay(20);
      ledcWrite(0, 0); 
      
      BLE_Rece_Flag = true;

      Serial.println("BLE Rev");
      
      if(rxValue == "ON")
      {
//        Serial.println("Turning ON the led");
        // digitalWrite(LedPin,HIGH);
      }
      if(rxValue == "OFF")
      { 
//        Serial.println("Turning OFF the led");
        // digitalWrite(LedPin,LOW);
      }
      
    }
};

void BLE_Init()
{
   // Create the BLE Device
  BLEDevice::init("DX_Plus_M5");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
                    CHARACTERISTIC_UUID_TX,
                    BLECharacteristic::PROPERTY_NOTIFY
                  );
                      
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                       CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

      // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}


//@-系统初始化
void setup(){

  //@1-初始化
  for(int temp=0; temp >= Check_Preson_Size; temp++)
  {
    check_have_person[temp] = person_unknow;
  }

  //@-产生随机数种子
  randomSeed(analogRead(BAT_Pin));

  //@-串口初始化
  Serial.begin(115200);

  //@-接管电源控制
  pinMode(Power_Ctl_Pin, OUTPUT);
  digitalWrite(Power_Ctl_Pin, HIGH); 

  //@-用户LED
  pinMode(LED_Pin, OUTPUT);

  //@-鸣音器控制-PWM_Channel PWM_Ferquency PWM_Resoution
  ledcSetup(0, 700, 8);
  ledcAttachPin(Tone_Pin, 0);
  ledcWrite(0, 0);

  //@-配置键盘上拉输入Pin
  pinMode(Check_DX1_Pin, INPUT|PULLUP);
  pinMode(Check_DX2_Pin, INPUT|PULLUP);
  pinMode(Button1_Pin, INPUT|PULLUP);
  pinMode(Button2_Pin, INPUT|PULLUP);
  pinMode(Button3_Pin, INPUT|PULLUP);
  pinMode(Button4_Pin, INPUT|PULLUP);

  //@-BLE初始化
  BLE_Init();

  //@-E2PROM初始化
//  Wire2.begin(AT24C32_SDA_Pin, AT24C32_SCL_Pin, 4000000);

  //@-I2C初始化
  Wire.begin(VL53L0X_BH1750_SDA_Pin, VL53L0X_BH1750_SCL_Pin, 400000); // SDA (21), SCL (22) on ESP32, 400 kHz rate

  /*
    BH1750 has six different measurement modes. They are divided in two groups;
    continuous and one-time measurements. In continuous mode, sensor continuously
    measures lightness value. In one-time mode the sensor makes only one
    measurement and then goes into Power Down mode.
    Each mode, has three different precisions:
      - Low Resolution Mode - (4 lx precision, 16ms measurement time)
      - High Resolution Mode - (1 lx precision, 120ms measurement time)
      - High Resolution Mode 2 - (0.5 lx precision, 120ms measurement time)
    By default, the library uses Continuous High Resolution Mode, but you can
    set any other mode, by passing it to BH1750.begin() or BH1750.configure()
    functions.
    [!] Remember, if you use One-Time mode, your sensor will go to Power Down
    mode each time, when it completes a measurement and you've read it.
    Full mode list:
      BH1750_CONTINUOUS_LOW_RES_MODE
      BH1750_CONTINUOUS_HIGH_RES_MODE (default)
      BH1750_CONTINUOUS_HIGH_RES_MODE_2
      BH1750_ONE_TIME_LOW_RES_MODE
      BH1750_ONE_TIME_HIGH_RES_MODE
      BH1750_ONE_TIME_HIGH_RES_MODE_2
  */

  // begin returns a boolean that can be used to detect setup problems.
  //@-光感初始化
 if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
   Serial.println(F("BH1750 Advanced begin"));
 }
 else {
   Serial.println(F("Error initialising BH1750"));
 }


  delay(200);

  DistanceSensor.setAddress(0x29);
  DistanceSensor.init();
  DistanceSensor.setTimeout(500); //500
  
  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  DistanceSensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  DistanceSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  DistanceSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  #endif

  #if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  DistanceSensor.setMeasurementTimingBudget(20000);  // minimum timing budget 20 ms
  #elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  // DistanceSensor.setMeasurementTimingBudget(50000);  //200000
  DistanceSensor.setMeasurementTimingBudget(200000);  //200000
  #endif

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  // DistanceSensor.startContinuous(500); //最原始没有值
  DistanceSensor.startContinuous(10); //最原始没有值


  //@-初始化 Ticker every 1s
  Tick1.attach(0.01, Second_Create);
  Tick2.attach(1, Second_test);

  //@5-初始化显示
  u8g2.begin();
  u8g2.enableUTF8Print();    // enable UTF8 support for the Arduino print() function
//  u8g2.setFont(u8g2_font_10x20_tn ); 
  u8g2.setFontDirection(0);


  //@6-扫描I2C接口
//  scanPorts();

  // read and write byte
//  Serial.println("Write 42 to address 12");
//  mem.write(12, 138);
//  Serial.println("Read byte from address 12 ...");
//  byte b = mem.read(12);
//  Serial.print("... read: ");
//  Serial.println(b, DEC);
//  Serial.println();

  //@1-NVS初始化只做一次
  // NVS.begin();
  // NVS.setInt("Studay_Pro", 0);
  // NVS.setInt("Tomato_Pro", 0);
  // NVS.setInt("Alarm_Pro", 0);
  // NVS.setInt("Distance_Pro", 0);
  // NVS.close();
  //@1-NVS初始化读一次
  NVS.begin();
  Studay_Pro_Index = NVS.getInt("Studay_Pro");
  Tomato_Pro_Index = NVS.getInt("Tomato_Pro");
  Alarm_Pro_Index = NVS.getInt("Alarm_Pro");
  Distance_Pro_Index = NVS.getInt("Distance_Pro");
  NVS.close();

  //@-初始设置测距限制值
  distance_limit_set();

  //@-产生随机数
  random_num = random(35); //产生随机数

//  String  ssid_t        = NVS.getString("SSID");
//  String  password_t    = NVS.getString("PASSWORD");
//
//  Serial.printf("ssid        : %s\n", ssid_t.c_str());
//  Serial.printf("password    : %s\n", password_t.c_str());
  // NVS.close();


  //@-low power
//  esp_bluedroid_disable();
//  esp_bt_controller_disable();
//  esp_wifi_stop();

// esp_pm_configure(ESP32_DEFAULT_CPU_FREQ_80);

}

//@-开机欢迎界面
void logo()
{
    Logo_Run_Tick = Logo_Run_Tick + 2;
    if(Logo_Run_Tick > 108)
    Logo_Run_Tick = 0;

    u8g2.clearBuffer();

    //@-设置字体
    u8g2.setFont( u8g2_font_t0_16b_tr ); //12px

    // logo_name = &col_logo1;
    
    u8g2.drawXBMP(43+Logo_Run_Tick , 3 , 20 , 20 , logo_name[random_num]);

    u8g2.drawXBMP(0 , 3 , 41 , 23 , col_qidian);
    

    // u8g2.setCursor(10, 30);
    // // sprintf(temp, "%.0f", lux); 
    // u8g2.print("Logo"+String(random_num));

    u8g2.sendBuffer();
}

//@-主页面
void main_page()
{
    char temp[20];

    u8g2.clearBuffer();

    //@-设置字体
    // u8g2.setFont( u8g2_font_t0_16b_tr ); //10px
    u8g2.setFont( u8g2_font_6x13_t_hebrew ); //9PX
     

    if(MainPage_DisMode == MainPage_DisMode_3)
    {
      //@-光强
      if(lux_level == Light_Level_High)
      {
        u8g2.drawXBMP( 10 , 0 , 18 , 17 , col_sun);
        u8g2.drawXBMP( 6 , 21 , 27 , 13 , col_mingliang);
      }
      else if(lux_level == Light_Level_Normal)
      {
        u8g2.drawXBMP( 10 , 0 , 18 , 18 , col_smile_small);
        u8g2.drawXBMP( 6 , 21 , 27 , 13 , col_zhengchang);
      }
      else if(lux_level == Light_Level_Low)
      {
        u8g2.drawXBMP( 10 , 0 , 18 , 18 , col_sad_small);
        u8g2.drawXBMP( 6 , 21 , 27 , 13 , col_guoan);
      }

      // u8g2.setCursor(10, 32);
      // sprintf(temp, "%.0f", lux); 
      // u8g2.print(String(temp));

      //@-分割线
      u8g2.drawLine(41, 0, 41, 32);

      //@-距离
      u8g2.drawXBMP(51 , 0 , 18 , 18 , col_distance);
      if(distance > 800)
      {
        u8g2.drawXBMP( 47 , 21 , 27 , 13 , col_guoyuan);
      }
      else
      {
        u8g2.setCursor(55, 32);
        sprintf(temp, "%.0f", distance/10); 
        u8g2.print(String(temp));
      }

      //@-分割线
      u8g2.drawLine(84, 0, 84, 32);

      //@-番茄管理
      if((MainPage_Exe_DisMode == 1))
      {
        int temp_min;
        int temp_sec;

        //@2-规避B4短按造成的番茄功能切换
        if((Button4_State == HIGH))
        {
          //@-充电状态没有改变
          if(USB_Button_State_Change == false)
          {
            if(Tomato_Pro_Index == 0)
            {
              u8g2.drawXBMP( 100 , 0 , 18 , 18 , col_tomato_small );
              temp_sec = Tomato_25min_Timer % 60;
              temp_min = Tomato_25min_Timer / 60;
            }
            else if(Tomato_Pro_Index == 1)
            {
              u8g2.drawXBMP( 100 , 0 , 18 , 18 , col_susuan_small );
              temp_sec = Tomato_5min_Timer % 60;
              temp_min = Tomato_5min_Timer / 60;
            }
            else if(Tomato_Pro_Index == 2)
            {
              u8g2.drawXBMP( 100 , 0 , 18 , 18 , col_clock_small );
              temp_sec = Tomato_Timer % 60;
              temp_min = Tomato_Timer / 60; 
            }
            //@-设置字体
            u8g2.setFont( u8g2_font_t0_11_tr ); //8px
            u8g2.setCursor(94, 32);
            sprintf(temp, "%2d:%2d",temp_min,temp_sec); 
            u8g2.print(String(temp));
          }
          //@-充电状态改变
          else if(USB_Button_State_Change == true)
          {
            //@-USB状态改变显示tick
            USB_Button_State_Change_ShowTick = USB_Button_State_Change_ShowTick - 1;
            if(USB_Button_State_Change_ShowTick <= 0)
            USB_Button_State_Change = false;

            //@-设置字体
            u8g2.setFont( u8g2_font_t0_11_tr ); //8px

            //@-番茄功能显示
            if(Tomato_Pro_Index == 0)
            {
              temp_sec = Tomato_25min_Timer % 60;
              temp_min = Tomato_25min_Timer / 60;
            }
            else if(Tomato_Pro_Index == 1)
            {
              temp_sec = Tomato_5min_Timer % 60;
              temp_min = Tomato_5min_Timer / 60;
            }
            else if(Tomato_Pro_Index == 2)
            {
              temp_sec = Tomato_Timer % 60;
              temp_min = Tomato_Timer / 60; 
            }
            u8g2.setCursor(91, 10);
            sprintf(temp, "%2d:%2d",temp_min,temp_sec); 
            u8g2.print(String(temp));

            //@-分割线
            u8g2.drawLine(94, 14, 120, 14);

            //@-USB充电状态显示
            if(USB_Button_State_Copy == LOW)  //USB接入
            {
              if((BAT_B>=0)&&(BAT_B<30))
              {
                u8g2.drawXBMP( 96 , 20 , 24 , 12 , col_bat_charge );
              }
              else if((BAT_B>=30)&&(BAT_B<40))
              {
                u8g2.drawXBMP( 96 , 20 , 24 , 12 , col_bat_val3 );
              }
              else if((BAT_B>=40)&&(BAT_B<110))
              {
                u8g2.drawXBMP( 96 , 20 , 24 , 12 , col_bat_val4 );
              }
              else if((BAT_B>=110)&&(BAT_B<=130))
              {
                if(flash_flag == true)
                u8g2.drawXBMP( 96 , 20 , 24 , 12 , col_bat_val5 );
                else if(flash_flag == false)
                u8g2.drawXBMP( 88 , 19 , 38 , 13 , col_yichongman );
              }
            }
            else if(USB_Button_State_Copy == HIGH)  //USB移除
            {
              if((BAT_B>=0)&&(BAT_B<20))
              {
                if(flash_flag == true)
                u8g2.drawXBMP( 96 , 20 , 24 , 12 , col_bat_val1 );
                else if(flash_flag == false)
                u8g2.drawXBMP( 88 , 19 , 38 , 13 , col_qingchongdian );
              }
              else if((BAT_B>=20)&&(BAT_B<30))
              {
                if(flash_flag == true)
                u8g2.drawXBMP( 96 , 20 , 24 , 12 , col_bat_val2 );
                else if(flash_flag == false)
                u8g2.drawXBMP( 88 , 19 , 38 , 13 , col_qingchongdian );
              }
              else if((BAT_B>=30)&&(BAT_B<40))
              {
                u8g2.drawXBMP( 96 , 20 , 24 , 12 , col_bat_val3 );
              }
              else if((BAT_B>=40)&&(BAT_B<80))
              {
                u8g2.drawXBMP( 96 , 20 , 24 , 12 , col_bat_val4 );
              }
              else if((BAT_B>=80)&&(BAT_B<=130))
              {
                u8g2.drawXBMP( 96 , 20 , 24 , 12 , col_bat_val5 );
              }
            }
          }

        }
        else if((Button4_State == LOW))
        {
          // if(Tomato_Run == true)
          // u8g2.drawXBMP( 100 , 0 , 27 , 13 , col_tingzhi);
          // else if(Tomato_Run == false)
          // u8g2.drawXBMP( 100 , 0 , 27 , 13 , col_qidong);

          //@-B4长按显示
          if((Button4_Press_Tick >= 0) && (Button4_Press_Tick <= 3))
          {
            //@-番茄功能为计时器功能且已运行
            if((Tomato_Pro_Index == 2) && (Tomato_Run == true))
            {
              if(Tomato_Timer_Pause == true)
              u8g2.drawXBMP( 100 , 0 , 27 , 13 , col_zanting);
              else if(Tomato_Timer_Pause == false)
              u8g2.drawXBMP( 100 , 0 , 27 , 13 , col_qidong);
            }
            
            u8g2.drawEllipse(112, 20, 2, 2, U8G2_DRAW_ALL);
            u8g2.drawEllipse(112, 20, 1, 1, U8G2_DRAW_ALL);
          }
          else if((Button4_Press_Tick > 3) && (Button4_Press_Tick <= 6))
          {
            u8g2.drawEllipse(112, 20, 6, 6, U8G2_DRAW_ALL);
            u8g2.drawEllipse(112, 20, 5, 5, U8G2_DRAW_ALL);
          }
          else if((Button4_Press_Tick > 6) && (Button4_Press_Tick <= 9))
          {
            u8g2.drawEllipse(112, 20, 10, 10, U8G2_DRAW_ALL);
            u8g2.drawEllipse(112, 20, 9, 9, U8G2_DRAW_ALL);
          }
          else if(Button4_Press_Tick > 9)
          {
            if(Tomato_Run == true)
            u8g2.drawXBMP( 100 , 17 , 27 , 13 , col_qidong);
            else if(Tomato_Run == false)
            u8g2.drawXBMP( 100 , 17 , 27 , 13 , col_tingzhi);
          }

        }
      }
    }
    // else if(MainPage_DisMode == MainPage_DisMode_2)
    // {
    //   //@-光强
    //   u8g2.drawXBMP( 0 , 5 , 28 , 27 , col_sun_big);
    //   u8g2.setCursor(32, 16);
    //   sprintf(temp, "%.0f", lux); 
    //   u8g2.print(String(temp));
    //   u8g2.setCursor(35, 32);
    //   u8g2.print("lx");

    //   //@-分割线
    //   u8g2.drawLine(63, 0, 63, 32);

    //   //@-距离
    //   u8g2.drawXBMP(67 , 0 , 28 , 28 , col_distance_big);
    //   u8g2.setCursor(100, 16);
    //   sprintf(temp, "%.0f", distance/10); 
    //   u8g2.print(String(temp));
    //   u8g2.setCursor(103, 32);
    //   u8g2.print("cm");
    // }
    else if(MainPage_DisMode == MainPage_DisMode_EVENT)
    {
      //@-距离超限
      if(Dev_event == DEV_EVENT_DISTANCE_OVER_LIMT)
      {
          u8g2.setCursor(13, 32);
          u8g2.print("distance");
      }
      //@-光强超限
      else if(Dev_event == DEV_EVENT_LIGHT_OVER_LIMT)
      {
          u8g2.setCursor(13, 32);
          u8g2.print("light");
      }
      //@-25min定时到
      else if(Dev_event == DEV_EVENT_Tomato_25min_TimerUp)
      {
          u8g2.setCursor(13, 32);
          u8g2.print("25min");
      }
      //@-5min定时到
      else if(Dev_event == DEV_EVENT_Tomato_5min_TimerUp)
      {
          u8g2.setCursor(13, 32);
          u8g2.print("5min");
      }
      //@-关机事件
      else if(Dev_event == DEV_EVENT_CLOSE)
      {
          u8g2.setCursor(13, 32);
          u8g2.print("Close:"+String(Power_OFF_Tick));
      }
    }
    u8g2.sendBuffer();
}

//@-防疲劳页面
void studay_page()
{
    u8g2.clearBuffer();

    u8g2.drawXBMP(15 , 3 , 20 , 20 , col_studay);
    u8g2.drawXBMP(40 , 0 , 74 , 23 , col_xuexiguanli);

    u8g2.drawEllipse(55, 28, 1, 1, U8G2_DRAW_ALL);
    u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.drawEllipse(73, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.drawEllipse(82, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.sendBuffer();
  
}

//@-防疲劳设置
void studay_set()
{
    
    u8g2.clearBuffer();

    u8g2.drawXBMP(15 , 3 , 20 , 20 , col_studay);
    u8g2.drawXBMP(40 , 0 , 41 , 23 , col_pilao);

    u8g2.setCursor(83, 18);
    if(Studay_Pro_Index_temp == 0)
    {
      u8g2.print("30min");
      u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(55, 28, 1, 1, U8G2_DRAW_ALL);

      u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(73, 28, 2, 2, U8G2_DRAW_ALL);
    }
    else if(Studay_Pro_Index_temp == 1)
    {
      u8g2.print("45min");
      u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);

      u8g2.drawEllipse(64, 28, 1, 1, U8G2_DRAW_ALL);
      u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);

      u8g2.drawEllipse(73, 28, 2, 2, U8G2_DRAW_ALL);
    }
    else if(Studay_Pro_Index_temp == 2)
    {
      u8g2.print("60min");
      u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);

      u8g2.drawEllipse(73, 28, 1, 1, U8G2_DRAW_ALL);
      u8g2.drawEllipse(73, 28, 2, 2, U8G2_DRAW_ALL);
    }

    u8g2.sendBuffer();

}

//@-告警方式设置页面
void alarm_page()
{
    u8g2.clearBuffer();

    u8g2.drawXBMP(15 , 3 , 20 , 20 , col_alarm);
    u8g2.drawXBMP(40 , 0 , 74 , 23 , col_gaojingfangshi);

    u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.drawEllipse(64, 28, 1, 1, U8G2_DRAW_ALL);
    u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.drawEllipse(73, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.drawEllipse(82, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.sendBuffer();
  
}

//@-告警方式设置
void alarm_set()
{
    u8g2.clearBuffer();

    u8g2.drawXBMP(15 , 3 , 20 , 20 , col_alarm);

    if(Alarm_Pro_Index_temp == 0)
    {
      u8g2.drawXBMP(40 , 0 , 41 , 23 , col_shengpin);

      u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(55, 28, 1, 1, U8G2_DRAW_ALL);

      u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);
    }
    else if(Alarm_Pro_Index_temp == 1)
    {
      u8g2.drawXBMP(40 , 0 , 41 , 23 , col_pinmu);

      u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);

      u8g2.drawEllipse(64, 28, 1, 1, U8G2_DRAW_ALL);
      u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);
    }
    u8g2.sendBuffer();
}

//@-测距设置页面
void distance_page()
{
    u8g2.clearBuffer();

    u8g2.drawXBMP(15 , 3 , 18 , 18 , col_distance);
    u8g2.drawXBMP(40 , 0 , 74 , 23 , col_cejushezhi);

    u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.drawEllipse(73, 28, 1, 1, U8G2_DRAW_ALL);
    u8g2.drawEllipse(73, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.drawEllipse(82, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.sendBuffer();
}

//@-测距设置
void distance_set()
{
    u8g2.clearBuffer();

    u8g2.drawXBMP(15 , 3 , 18 , 18 , col_distance);
    u8g2.drawXBMP(40 , 0 , 41 , 23 , col_ceju);

    u8g2.setCursor(83, 18);
    if(Distance_Pro_Index_temp == 0)
    {
      u8g2.print("20cm");
      u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(55, 28, 1, 1, U8G2_DRAW_ALL);

      u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(73, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(82, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(91, 28, 2, 2, U8G2_DRAW_ALL);
    }
    else if(Distance_Pro_Index_temp == 1)
    {
      u8g2.print("30cm");
      u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);

      u8g2.drawEllipse(64, 28, 1, 1, U8G2_DRAW_ALL);
      u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);

      u8g2.drawEllipse(73, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(82, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(91, 28, 2, 2, U8G2_DRAW_ALL);
    }
    else if(Distance_Pro_Index_temp == 2)
    {
      u8g2.print("40cm");
      u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);

      u8g2.drawEllipse(73, 28, 1, 1, U8G2_DRAW_ALL);
      u8g2.drawEllipse(73, 28, 2, 2, U8G2_DRAW_ALL);

      u8g2.drawEllipse(82, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(91, 28, 2, 2, U8G2_DRAW_ALL);
    }
    else if(Distance_Pro_Index_temp == 3)
    {
      u8g2.print("50cm");
      u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(73, 28, 2, 2, U8G2_DRAW_ALL);

      u8g2.drawEllipse(82, 28, 1, 1, U8G2_DRAW_ALL);
      u8g2.drawEllipse(82, 28, 2, 2, U8G2_DRAW_ALL);

      u8g2.drawEllipse(91, 28, 2, 2, U8G2_DRAW_ALL);
    }
    else if(Distance_Pro_Index_temp == 4)
    {
      u8g2.print("60cm");
      u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(73, 28, 2, 2, U8G2_DRAW_ALL);
      u8g2.drawEllipse(82, 28, 2, 2, U8G2_DRAW_ALL);
      
      u8g2.drawEllipse(91, 28, 1, 1, U8G2_DRAW_ALL);
      u8g2.drawEllipse(91, 28, 2, 2, U8G2_DRAW_ALL);
    }

    u8g2.sendBuffer();
}

//@-修改距离限制值
void distance_limit_set()
{
  if(Distance_Pro_Index == 0)
  {
    Distance_Pro_Index_temp = 0;
    distance_limit = 200;
  }
  else if(Distance_Pro_Index == 1)
  {
    Distance_Pro_Index_temp = 1;
    distance_limit = 300;
  }
  else if(Distance_Pro_Index == 2)
  {
    Distance_Pro_Index_temp = 2;
    distance_limit = 400;
  }
  else if(Distance_Pro_Index == 3)
  {
    Distance_Pro_Index_temp = 3;
    distance_limit = 500;
  }
  else if(Distance_Pro_Index == 4)
  {
    Distance_Pro_Index_temp = 4;
    distance_limit = 600;
  }
}

//@-环境光强检测页面
void light_page()
{
    u8g2.clearBuffer();

    u8g2.drawXBMP(15 , 3 , 18 , 18 , col_light);
    u8g2.drawXBMP(40 , 0 , 74 , 23 , col_huanjingguangqiang);

    u8g2.drawEllipse(55, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.drawEllipse(64, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.drawEllipse(73, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.drawEllipse(82, 28, 1, 1, U8G2_DRAW_ALL);
    u8g2.drawEllipse(82, 28, 2, 2, U8G2_DRAW_ALL);

    u8g2.sendBuffer();
}

//@-环境光强检测功能
void light_set()
{
    char temp[20];
    
    u8g2.clearBuffer();

    u8g2.drawXBMP(15 , 3 , 18 , 18 , col_light);
    u8g2.drawXBMP(40 , 0 , 41 , 23 , col_guangqiang);

    u8g2.setCursor(45, 32);
    sprintf(temp, "%.0f lux", lux); 
    u8g2.print(String(temp));

    u8g2.sendBuffer();
}

//@-OTA
void Check_OTA(void * parameter)
{
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {

      OTA_Tick =  OTA_Tick + 1;
      if(OTA_Tick > 10)
      {
        OTA_Start = false;
        break;
      }
      
      delay(500);
  }

  // Execute OTA Update
  execOTA();
  
}

//@-键盘扫描
void Button_Check()
{
    String Temp3 = "None";

//    touchRead(T0)

    Power_Button_State = digitalRead(Check_DX1_Pin);
    USB_Button_State = digitalRead(Check_DX2_Pin);
    Button1_State = digitalRead(Button1_Pin);
    Button2_State = digitalRead(Button2_Pin);
    Button3_State = digitalRead(Button3_Pin);
    Button4_State = digitalRead(Button4_Pin);
//    Button1_State = touchRead(T9);


//    Serial.println("Start OTA ..."+String(Button1_State));

    //@-电源键
    if(Power_Button_State == LOW)
    {
        Serial.println("Power");

        //@-电源键长按检测
        if(Power_Press_Close_Flag == true)
        {
          Power_PressOFF_Tick = Power_PressOFF_Tick + 1;
          if(Power_PressOFF_Tick > 10)
          {
            //digitalWrite(Power_Ctl_Pin, LOW); 
            //@-触发关机事件
            Dev_event = DEV_EVENT_CLOSE;
          }
        }
    }
    else
    {
      Power_PressOFF_Tick = 0;
    }
    

    //@-USB充电
    if(USB_Button_State_Copy != USB_Button_State)
    {
        USB_Button_State_Copy = USB_Button_State;
        USB_Button_State_Change = true;
        USB_Button_State_Change_ShowTick = 500;

        // if(USB_Button_State_Copy == LOW)
        // {
        //   Serial.println("USB In");
        //   // if(Dev_event != DEV_EVENT_NONE)
        //   // DEV_EVENT_CHARGE_IN
        // }

        // else if(USB_Button_State_Copy == HIGH)
        // {
        //   Serial.println("USB Out");
        // }

    }

    //@-B1
    if((Button1_State == LOW) && (Key_Flag == false))
//     if(Button1_State == 79)
    {
        Temp3 = "B1";

        Key_Flag = true;

        Serial.println("Key1");

        digitalWrite(LED_Pin, HIGH); 

        // if(OTA_Start == false)
        // {
        //     OTA_Start = true;

        //     Serial.println("Start OTA ...");
            
        //     xTaskCreatePinnedToCore(
        //     Check_OTA,          /* Task function. */
        //     "Check_OTA",        /* String with name of task. */
        //     10000,            /* Stack size in bytes. */
        //     NULL,             /* Parameter passed as input of the task */
        //     1,                /* Priority of the task. */
        //     &Task1,
        //     0); 
        // }
    }
    //@-B2
    else if((Button2_State == LOW) && (Key_Flag == false))
    {
        Temp3 = "B2";
        Key_Flag = true;

        ledcWrite(0, 150); 

        Serial.println("Key2");
    }


    //@-B3
    else if((Button3_State == LOW) && (Key_Flag == false))
    {
        Temp3 = "B3";
        Key_Flag = true;

        //@-关闭无按键操作
        None_Button_Press_Tick = 0;

        //@-设备没有关机事件
        if(Dev_event != DEV_EVENT_CLOSE)
        {
          if((Dev_event != DEV_EVENT_Tomato_25min_TimerUp) && (Dev_event != DEV_EVENT_Tomato_5min_TimerUp))
          {
            //@-显示模式不在扩展模式和番茄功能显示不上锁的情况下可按键
            if((MainPage_DisMode != MainPage_DisMode_EVENT) && (Tomato_Page_Lock != true))
            {
              //@-主菜单
              if(Menu_Option_Set == 0)
              {
                Menu_Index = Menu_Index + 1;
                if(Menu_Index > Menu_Max_Size)
                Menu_Index = 0;
              }
              //@-防疲劳
              else if(Menu_Option_Set == 1)
              {
                Studay_Pro_Index_temp = Studay_Pro_Index_temp + 1;
                if(Studay_Pro_Index_temp > 2)
                Studay_Pro_Index_temp = 0;
              }
              //@-告警方式
              else if(Menu_Option_Set == 2)
              {
                Alarm_Pro_Index_temp = Alarm_Pro_Index_temp + 1;
                if(Alarm_Pro_Index_temp > 1)
                Alarm_Pro_Index_temp = 0;
              }
              //@-测距设置
              else if(Menu_Option_Set == 3)
              {
                Distance_Pro_Index_temp = Distance_Pro_Index_temp + 1;
                if(Distance_Pro_Index_temp > 4)
                Distance_Pro_Index_temp = 0;
              }
            }
          }
          else if((Dev_event == DEV_EVENT_Tomato_25min_TimerUp) || (Dev_event == DEV_EVENT_Tomato_5min_TimerUp))
          {
            Dev_event = DEV_EVENT_NONE;
          }
        }
        else
        {
          //@-取消关机事件
          Dev_event = DEV_EVENT_NONE;
          Power_OFF_Tick = 10;
        }
        
        Serial.println("Key3");
    }
    //@-B4
    else if((Button4_State == LOW) && (Key_Flag == false))
    {
        Temp3 = "B4";
        Key_Flag = true;

        //@-关闭无按键操作
        None_Button_Press_Tick = 0;

        //@-设备没有关机事件
        if(Dev_event != DEV_EVENT_CLOSE)
        {
          if((Dev_event != DEV_EVENT_Tomato_25min_TimerUp) && (Dev_event != DEV_EVENT_Tomato_5min_TimerUp))
          {
            //@-主界面
            if(Menu_Index == 0)
            {
                //@-番茄钟没有运行
                if(Tomato_Run == false)
                {
                  Tomato_Pro_Index = Tomato_Pro_Index + 1;
                  if(Tomato_Pro_Index > 2)
                  Tomato_Pro_Index = 0;

                  Serial.println(Tomato_Pro_Index);
                  // NVS Save Tomato_Pro_Index
                  NVS.begin();
                  NVS.setInt("Tomato_Pro", Tomato_Pro_Index);
                  NVS.close();
                }

                //@-番茄功能为计时器功能且已运行
                if((Tomato_Pro_Index == 2) && (Tomato_Run == true))
                {
                  if(Tomato_Timer_Pause == false)
                  Tomato_Timer_Pause = true;
                  else if(Tomato_Timer_Pause == true)
                  Tomato_Timer_Pause = false;

                  ledcWrite(0, 90); 
                  delay(20);
                  ledcWrite(0, 0); 
                }
            }
            //@-防疲劳
            else if(Menu_Index == 1)
            {
                if(Studay_Pro_Flag == false)
                {
                  Studay_Pro_Flag = true;  
                  Menu_Option_Set = 1;
                }
                else if(Studay_Pro_Flag == true)
                {
                  Studay_Pro_Flag = false;
                  Menu_Option_Set = 0; //@-返回上级菜单
                  Studay_Pro_Index = Studay_Pro_Index_temp;
                  Serial.println(Studay_Pro_Index);
                  // NVS Save Studay_Pro_Index
                  NVS.begin();
                  NVS.setInt("Studay_Pro", Studay_Pro_Index);
                  NVS.close();
                }
            }
            //@-告警方式
            else if(Menu_Index == 2)
            {
                if(Alarm_Pro_Flag == false)
                {
                  Alarm_Pro_Flag = true;
                  Menu_Option_Set = 2;
                }
                else if(Alarm_Pro_Flag == true)
                {
                  Alarm_Pro_Flag = false;
                  Menu_Option_Set = 0; //@-返回上级菜单
                  Alarm_Pro_Index = Alarm_Pro_Index_temp;
                  Serial.println(Alarm_Pro_Index);
                  // NVS Save Alarm_Pro_Index
                  NVS.begin();
                  NVS.setInt("Alarm_Pro", Alarm_Pro_Index);
                  NVS.close();
                }
            }
            //@-测距设置
            else if(Menu_Index == 3)
            {
                if(Distance_Pro_Flag == false)
                {
                  Distance_Pro_Flag = true;
                  Menu_Option_Set = 3;
                }
                else if(Distance_Pro_Flag == true)
                {
                  Distance_Pro_Flag = false;
                  Menu_Option_Set = 0; //@-返回上级菜单
                  Distance_Pro_Index = Distance_Pro_Index_temp;
                  //@-修改测距限制值
                  distance_limit_set();
                  Serial.println(Distance_Pro_Index);
                  // NVS Save Distance_Pro_Index
                  NVS.begin();
                  NVS.setInt("Distance_Pro", Distance_Pro_Index);
                  NVS.close();
                }
            }
            //@-环境光强
            else if(Menu_Index == 4)
            {
                if(Light_Pro_Flag == false)
                {
                  Light_Pro_Flag = true;
                  Menu_Option_Set = 4;
                }
                else if(Light_Pro_Flag == true)
                {
                  Light_Pro_Flag = false;
                  Menu_Option_Set = 0; //@-返回上级菜单
                }
            }

          }
          else if((Dev_event == DEV_EVENT_Tomato_25min_TimerUp) || (Dev_event == DEV_EVENT_Tomato_5min_TimerUp))
          {
            Dev_event = DEV_EVENT_NONE;
          }
        }
        else
        {
          //@-取消关机事件
          Dev_event = DEV_EVENT_NONE;
          Power_OFF_Tick = 10;
        }

        // Serial.println("Key4");
    }
    //@-B4-长按
    else if((Button4_State == LOW))
    {
        //@-关闭无按键操作
        None_Button_Press_Tick = 0;

        //@-按键长按检测
        Button4_Press_Tick = Button4_Press_Tick + 1;
        if(Button4_Press_Tick > 10)
        {
            if(Button4_LongPress_Lock == false)
            {
              Button4_LongPress_Lock = true;

              Studay_Pro_Flag = false;
              Tomato_Pro_Flag = false;
              Alarm_Pro_Flag = false;
              Menu_Index = 0;      //@-返回主菜单
              Menu_Option_Set = 0; //@-返回主菜单
              // Serial.println("Key4--->long");

              //@-执行番茄管理
              if(Tomato_Run == false)
              {
                //@-消除短按B4番茄钟模式切换的影响
                Tomato_Pro_Index = Tomato_Pro_Index - 1;
                if(Tomato_Pro_Index < 0)
                Tomato_Pro_Index = 2;

                Serial.println(Tomato_Pro_Index);
                // NVS Save Tomato_Pro_Index
                NVS.begin();
                NVS.setInt("Tomato_Pro", Tomato_Pro_Index);
                NVS.close();

                MainPage_DisMode = MainPage_DisMode_3;
                MainPage_Exe_DisMode = 1;
                //@-清除定时器到事件
                if((Dev_event == DEV_EVENT_Tomato_25min_TimerUp) || (Dev_event == DEV_EVENT_Tomato_5min_TimerUp))
                {
                  Dev_event = DEV_EVENT_NONE;
                }
                Tomato_25min_Timer = Timer_25min;
                Tomato_5min_Timer = Timer_5min;
                Tomato_Timer = 0;
                Tomato_Timer_Pause = false;
                Tomato_Alarm_Tick = 0;
                ledcWrite(0, 30); 
                delay(80);
                ledcWrite(0, 0); 
                // Serial.println("TO-T");
                Tomato_Run = true;
              }
              else if(Tomato_Run == true)
              {

                MainPage_DisMode = MainPage_DisMode_3;
                MainPage_Exe_DisMode = 1;
                Tomato_25min_Timer = Timer_25min;
                Tomato_5min_Timer = Timer_5min;
                Tomato_Timer = 0;
                ledcWrite(0, 80); 
                delay(80);
                ledcWrite(0, 0); 
                // Serial.println("TO-F");
                Tomato_Run = false;
              }
            }
        }
    }

    //@-所有功能按键均没有按下
    if((Button1_State == HIGH)&&(Button2_State == HIGH)&&(Button3_State == HIGH)&&(Button4_State == HIGH))
    {
        Key_Flag = false;
        Button4_Press_Tick = 0;
        Button4_LongPress_Lock = false;

        //@-无按键按下返回主菜单
        None_Button_Press_Tick = None_Button_Press_Tick + 1;
        if(None_Button_Press_Tick > 200)
        {
          None_Button_Press_Tick = 0;
          Studay_Pro_Flag = false;
          Tomato_Pro_Flag = false;
          Alarm_Pro_Flag = false;
          Menu_Index = 0;      //@-返回主菜单
          Menu_Option_Set = 0; //@-返回主菜单
          Serial.println("none key");
        }

        // digitalWrite(LED_Pin, LOW); 
        ledcWrite(0, 0); 
    }
}

//@-设备事件处理
void dev_event_check()
{
  //@1-测距超限-在主页面时检测
  if((distance < distance_limit) && (Menu_Index == 0))
  {
      //@-监测事件不能覆盖
      if(((Dev_event == DEV_EVENT_NONE) && (Dev_event != DEV_EVENT_DISTANCE_IGNORE_30)) || (Dev_event == DEV_EVENT_LIGHT_IGNORE_30))
      {
        Dev_event = DEV_EVENT_DISTANCE_OVER_LIMT;
      }
  }
  else if(distance > distance_limit)
  {
      if(Dev_event == DEV_EVENT_DISTANCE_OVER_LIMT)
      {
        Dev_event = DEV_EVENT_NONE;
        Distance_Alarm_Tick = 0;
      }
  }

  //@2-光强超限
  if(lux < 150)
  {
      // //@-监测事件不能覆盖
      // if(((Dev_event == DEV_EVENT_NONE) && (Dev_event != DEV_EVENT_LIGHT_IGNORE_30)) || (Dev_event == DEV_EVENT_DISTANCE_IGNORE_30))
      // {
      //   // Dev_event = DEV_EVENT_LIGHT_OVER_LIMT;  
      // }
      lux_level = Light_Level_Low;
  }
  else if((lux >= 150) && (lux <= 500))
  {
      // if(Dev_event == DEV_EVENT_LIGHT_OVER_LIMT)
      // {
      //   Dev_event = DEV_EVENT_NONE;
      // }
      lux_level = Light_Level_Normal;
  }
  else if((lux > 500))
  {
      lux_level = Light_Level_High;
  }

  //@-查询设备事件标志
  if((Dev_event != DEV_EVENT_NONE) && (Dev_event != DEV_EVENT_DISTANCE_IGNORE_30) && (Dev_event != DEV_EVENT_LIGHT_IGNORE_30))
  {
      //@-保存现有显示模式
      if(MainPage_DisMode_Save_Lock == false)
      {
        MainPage_DisMode_Save_Lock = true;
        MainPage_DisMode_Save = MainPage_DisMode;
      }
      //@-显示模式切换成事件显示模式
      MainPage_DisMode = MainPage_DisMode_EVENT;
      Menu_Index = 0;      //@-返回主菜单
      Menu_Option_Set = 0; //@-返回主菜单

      //@-距离超限
      if(Dev_event == DEV_EVENT_DISTANCE_OVER_LIMT)
      {

      }
      //@-光强过暗
      else if(Dev_event == DEV_EVENT_LIGHT_OVER_LIMT)
      {

      }
      //@-25min定时到
      else if(Dev_event == DEV_EVENT_Tomato_25min_TimerUp)
      {

      }
      //@-5min定时到
      else if(Dev_event == DEV_EVENT_Tomato_5min_TimerUp)
      {

      }
      //@-关机事件
      else if(Dev_event == DEV_EVENT_CLOSE)
      {

      }

     //@-在事件模式下启动告警提示
     if(MainPage_DisMode == MainPage_DisMode_EVENT)
     {
        //@-非关机事件物理告警
        if(Dev_event != DEV_EVENT_CLOSE)
        {
          //@-告警方式选择
          if(Alarm_Pro_Index == 0)
          {
              digitalWrite(LED_Pin, HIGH); 
              ledcWrite(0, 30); 
              delay(80);
              digitalWrite(LED_Pin, LOW); 
              ledcWrite(0, 0); 
          }
          else if(Alarm_Pro_Index == 1)
          {
              digitalWrite(LED_Pin, HIGH); 
              delay(80);
              digitalWrite(LED_Pin, LOW); 
          }
        }
     }
  }
  else
  {
    digitalWrite(LED_Pin, LOW); 
    ledcWrite(0, 0); 

    //@-恢复无事件前的显示模式
    if(MainPage_DisMode_Save_Lock == true)
    {
      MainPage_DisMode_Save_Lock = false;
      MainPage_DisMode = MainPage_DisMode_Save;
    }
  }
}

//@-主循环
void loop() {

  //@-tick自增
  lux_tick = lux_tick + 1;
  distance_tick = distance_tick + 1;
  bat_check_tick = bat_check_tick + 1;
  oled_tick = oled_tick + 1;
  ble_tick = ble_tick + 1;

//  OTA_Updata_Tick = OTA_Updata_Tick + 1;

  //@-电池电压：3.17V~4.02V
  if(bat_check_tick > 300)
  {
    bat_check_tick = 0;
    Signal = analogRead(BAT_Pin);
    BAT_V = ((Signal * 3.3 * 2)/4096) + 0.4;
    // BAT_BB[bat_tick] = (BAT_V-3.17)/0.0079;
    BAT_B = (BAT_V-3.17)/0.0079;
    // bat_tick = bat_tick + 1;
    // if(bat_tick >= 9)
    // {
    //   bat_tick = 0;
    //   BAT_Sum = 0;
    //   for(int i=0; i<10; i++)
    //   BAT_Sum = BAT_Sum + BAT_BB[i];
    //   BAT_B = BAT_Sum / 10;
    // }
    // Serial.println("AD");
    Serial.println(BAT_B);

    //@-查询电池电压
    if((BAT_B < 35) || (BAT_B > 110))
    {
      if(USB_Button_State_Change == false)
      {
        BAT_Info_ShowTick = BAT_Info_ShowTick + 1;
        if(BAT_Info_ShowTick > 8)
        {
          USB_Button_State_Change = true;
          USB_Button_State_Change_ShowTick = 500;
        }
      }
      else if(USB_Button_State_Change == true)
      {
        BAT_Info_ShowTick = 0;
      }
    }
  }

  //@-光强监测
  if(lux_tick > 50)
  {
      lux_tick = 0;
      lux = lightMeter.readLightLevel();  
      //  Serial.println("LUX");
  }

  //@—距离监测
  if(distance_tick > 10)
  {
  //    DistanceSensor.startContinuous();
    distance_tick = 0;
    distance= DistanceSensor.readRangeContinuousMillimeters();
  //    DistanceSensor.stopContinuous();
  }

  if(Logo_Flag == false)
  {
    //@-键盘处理
    Button_Check();

    //@-OLED显示
    if(oled_tick > 20)
    {
      if(Screen_Close_Flag == 0)
      {
        if(Menu_Option_Set == 0)
        {
          if(Menu_Index == 0)
          main_page();
          else if(Menu_Index == 1)
          studay_page();
          else if(Menu_Index == 2)
          alarm_page();
          else if(Menu_Index == 3)
          distance_page();
          else if(Menu_Index == 4)
          light_page();
        }
        else if(Menu_Option_Set == 1)
        {
          studay_set();
        }
        else if(Menu_Option_Set == 2)
        {
          alarm_set();
        }
        else if(Menu_Option_Set == 3)
        {
          distance_set();
        }
        else if(Menu_Option_Set == 4)
        {
          light_set();
        }
      }
      else if(Screen_Close_Flag == 1)
      {
          u8g2.clearBuffer();
          u8g2.sendBuffer();
          Screen_Close_Flag = 2;
      }
    }

    //@-设备事件处理
    dev_event_check();
  }
  else
  {
    logo();
  }


  //@-BLE发送
  if(ble_tick > 20)
  {
    ble_tick = 0;
    if(deviceConnected) 
    {
      
        SendData[0] = (int)(((int) distance)%256);
        SendData[1] = (int)(((int) distance)/256);
        
        pTxCharacteristic->setValue(SendData, 2);
        pTxCharacteristic->notify();
    }
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
  // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
  

  //@-系统延时
  delay(20);

}
