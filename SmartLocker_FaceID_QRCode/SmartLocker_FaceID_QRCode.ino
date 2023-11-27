#include "esp_camera.h"
#include <WiFi.h>
#include "fd_forward.h"
#include "fr_flash.h"
#include "fr_forward.h"
#include <EEPROM.h>
#include "soc/soc.h"          
#include "soc/rtc_cntl_reg.h"  

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const char* ssid = "TINY_WIFI";
const char* password = "phamvanthang";

IPAddress staticIP(192, 168, 137, 137);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 137, 1);
IPAddress dns(192, 168, 137, 1);

#define I2C_SDA   15
#define I2C_SCL   13
TwoWire I2CSensors = TwoWire(0);
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;
int AngleX, AngleY, AngleZ;

String lockerStatus = "-1";
String QRCodeResult = "x@x";
String conditionOpenCameraQR = "";
String controlLockerbyQr = "0";

String regisQr_1 = "";
String regisQr_2 = "";
String regisQr_3 = "";
String regisQr_4 = "";

String pickLocker = "";

String disableLocker1 = "";
String disableLocker2 = "";
String disableLocker3 = "";
String disableLocker4 = "";
String emergenUnlocker1 = "";
String emergenUnlocker2 = "";
String emergenUnlocker3 = "";
String emergenUnlocker4 = "";

int connect_times = 0;

#define RELAY_1     12
#define RELAY_2     1
#define RELAY_3     14
#define RELAY_4     3
#define LED         4
#define BELL        33


#define ENROLL_CONFIRM_TIMES 2
#define FACE_ID_SAVE_NUMBER 4
face_id_list id_list = {0};

void startCameraServer();
void createTaskQRCodeReader();
void handle_QR();

void read_eeprom_qr_locker();
void delete_eeprom(int qrLockerNumber);

void read_eeprom_disable_locker();
void delete_eeprom_disable_locker(int disableLockerNumber);
void delete_face(int faceNumber);
void read_axis();
boolean matchFace1 = false;
boolean matchFace2 = false;
boolean matchFace3 = false;
boolean matchFace4 = false;
boolean activateRelay1 = false;
boolean activateRelay2 = false;
boolean activateRelay3 = false;
boolean activateRelay4 = false;
boolean activateRelay5 = false;
boolean activateRelay6 = false;
boolean activateRelay7 = false;
boolean activateRelay8 = false;
boolean activateRelay9 = false;
boolean activateRelay10 = false;
boolean activateRelay11 = false;
boolean activateRelay12 = false;
boolean activateRelay13 = false;
boolean activateRelay14 = false;
boolean activateRelay15 = false;
boolean activateRelay16 = false;
bool stateRelayBell = false;
unsigned long timer = 0;
unsigned long oldMillis = 0;
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(RELAY_4, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(BELL, OUTPUT);
  
  digitalWrite(RELAY_1, LOW); 
  digitalWrite(RELAY_2, LOW); 
  digitalWrite(RELAY_3, LOW); 
  digitalWrite(RELAY_4, LOW); 
  digitalWrite(LED, LOW);
  digitalWrite(BELL, LOW);
  
  Serial.begin(115200);
  
  
  Serial.setDebugOutput(true);
  Serial.println();

  if(WiFi.config(staticIP, gateway, subnet, dns, dns) == false)
  {
    Serial.println("Configuration failed");
  }
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    connect_times++;
    if(connect_times > 20)
    {
      ESP.restart();
    }
  }
  Serial.println("");
  Serial.println("WiFi connected");
  ledcAttachPin(4, 4);  
  ledcSetup(4, 5000, 8);
  ledcWrite(4,5);
  delay(200);
  ledcWrite(4,0);
  delay(200);
  ledcWrite(4,5);
  delay(200);
  ledcWrite(4,0);
  delay(200);
  ledcWrite(4,5);
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  
  
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 15;
    config.fb_count = 1;
  
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

  read_eeprom_qr_locker();
  read_eeprom_disable_locker();
  
  startCameraServer();
  
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
  Serial.print("Subnet mask: ");
  Serial.println(WiFi.subnetMask());
  Serial.print("Gateway IP: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print("DNS 1: ");
  Serial.println(WiFi.dnsIP(0));
  Serial.print("DNS 2: ");
  Serial.println(WiFi.dnsIP(1));

  createTaskQRCodeReader();
  disableCore0WDT();
  connect_times = 0;
  
  Serial.println(F("MPU6050 test"));
  I2CSensors.begin(I2C_SDA, I2C_SCL, 100000);
  if (!mpu.begin(0x68, &I2CSensors)) 
  {
  Serial.println("Sensor init failed");
  while (1)
    yield();
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void loop() {
  static long prevMillis = 0;

  mpu.getEvent(&a, &g, &temp);
  handle_QR();
  
  if (matchFace1 == true && activateRelay1 == false)
  {
    activateRelay1 = true;
    digitalWrite(RELAY_1, HIGH);
    prevMillis = millis();    
  }
  
  if (activateRelay1 == true && millis()- prevMillis > 8000)
  {
    activateRelay1 = false;
    matchFace1 = false;
    digitalWrite(RELAY_1, LOW);  
  }

  if (matchFace2 == true && activateRelay2 == false)
  {
    activateRelay2 = true;
    digitalWrite(RELAY_2, HIGH);
    prevMillis = millis();    
  }
  
  if (activateRelay2 == true && millis()- prevMillis > 8000)
  {
    activateRelay2 = false;
    matchFace2 = false;
    digitalWrite(RELAY_2, LOW);  
  }

  if (matchFace3 == true && activateRelay3 == false)
  {
    activateRelay3 = true;
    digitalWrite(RELAY_3, HIGH);
    prevMillis = millis();    
  }
  
  if (activateRelay3 == true && millis()- prevMillis > 8000)
  {
    activateRelay3 = false;
    matchFace3 = false;
    digitalWrite(RELAY_3, LOW);  
  }

  if (matchFace4 == true && activateRelay4 == false)
  {
    activateRelay4 = true;
    digitalWrite(RELAY_4, HIGH);
    prevMillis = millis();    
  }
  
  if (activateRelay4 == true && millis()- prevMillis > 8000)
  {
    activateRelay4 = false;
    matchFace4 = false;
    digitalWrite(RELAY_4, LOW);  
  }
  
  if(controlLockerbyQr == "1" && pickLocker == "LOCKER_1" && activateRelay5 == false)
  {
    activateRelay5 = true;
    digitalWrite(RELAY_1, HIGH);
    prevMillis = millis(); 
  }
  if(activateRelay5 == true && millis()- prevMillis > 8000)
  {
    controlLockerbyQr = "0";
    activateRelay5 = false;
    digitalWrite(RELAY_1, LOW); 
  }

  if(controlLockerbyQr == "1" && pickLocker == "LOCKER_2" && activateRelay6 == false)
  {
    activateRelay6 = true;
    digitalWrite(RELAY_2, HIGH);
    prevMillis = millis(); 
  }
  if(activateRelay6 == true && millis()- prevMillis > 8000)
  {
    controlLockerbyQr = "0";
    activateRelay6 = false;
    digitalWrite(RELAY_2, LOW); 
  }

  if(controlLockerbyQr == "1" && pickLocker == "LOCKER_3" && activateRelay7 == false)
  {
    activateRelay7 = true;
    digitalWrite(RELAY_3, HIGH);
    prevMillis = millis();  
  }
  if(activateRelay7 == true && millis()- prevMillis > 8000)
  {
    controlLockerbyQr = "0";
    activateRelay7 = false;
    digitalWrite(RELAY_3, LOW); 
  }

  if(controlLockerbyQr == "1" && pickLocker == "LOCKER_4" && activateRelay8 == false)
  {
    activateRelay8 = true;
    digitalWrite(RELAY_4, HIGH);
    prevMillis = millis(); 
  }
  if(activateRelay8 == true && millis()- prevMillis > 8000)
  {
    controlLockerbyQr = "0";
    activateRelay8 = false;
    digitalWrite(RELAY_4, LOW); 
  }
  
  if(QRCodeResult == regisQr_1 && activateRelay9 == false)
  {
    lockerStatus = -10;
    activateRelay9 = true;
    digitalWrite(RELAY_1, HIGH);
    QRCodeResult = "x@x";
    regisQr_1 = "";
    delete_eeprom(1);
    prevMillis = millis(); 
  }
  if(activateRelay9 == true && millis()- prevMillis > 8000)
  {
    activateRelay9 = false;
    digitalWrite(RELAY_1, LOW); 
  }

  if(QRCodeResult == regisQr_2 && activateRelay10 == false)
  {
    lockerStatus = -11;
    activateRelay10 = true;
    digitalWrite(RELAY_2, HIGH);
    QRCodeResult = "x@x";
    regisQr_2 = "";
    delete_eeprom(2);
    prevMillis = millis(); 
  }
  if(activateRelay10 == true && millis()- prevMillis > 8000)
  {
    activateRelay10 = false;
    digitalWrite(RELAY_2, LOW); 
  }

  if(QRCodeResult == regisQr_3 && activateRelay11 == false)
  {
    digitalWrite(RELAY_3, HIGH);
    QRCodeResult = "x@x";
    regisQr_3 = "";
    delete_eeprom(3);
    prevMillis = millis(); 
    lockerStatus = -12;
    activateRelay11 = true;
  }
  if(activateRelay11 == true && millis()- prevMillis > 8000)
  {
    activateRelay11 = false;
    digitalWrite(RELAY_3, LOW); 
  }

  if(QRCodeResult == regisQr_4 && activateRelay12 == false)
  {  
    digitalWrite(RELAY_4, HIGH);
    QRCodeResult = "x@x";
    regisQr_4 = "";
    delete_eeprom(4);
    prevMillis = millis(); 
    lockerStatus = -13;
    activateRelay12 = true;
  }
  if(activateRelay12 == true && millis()- prevMillis > 8000)
  {
    activateRelay12 = false;
    digitalWrite(RELAY_4, LOW); 
  }

  if (emergenUnlocker1 == "1" && activateRelay13 == false)
  {
    disableLocker1 = "0";
    activateRelay13 = true;
    digitalWrite(RELAY_1, HIGH);   

    id_list.head = 0;
    Serial.println("Deleting Face: LOCKER 1"); 
    delete_face_id_in_flash(&id_list);
    id_list.count = 0;
    
    prevMillis = millis();
  }
  
  if (activateRelay13 == true && millis()- prevMillis > 8000)
  {
    activateRelay13 = false;
    emergenUnlocker1 = "0";
    digitalWrite(RELAY_1, LOW); 
     
  }

  if (emergenUnlocker2 == "1" && activateRelay14 == false)
  {
    disableLocker2 = "0";
    activateRelay14 = true;
    digitalWrite(RELAY_2, HIGH);   

    id_list.head = 1;
    Serial.println("Deleting Face: LOCKER 2"); 
    delete_face_id_in_flash(&id_list);
    id_list.count = 0;
    
    prevMillis = millis();  
  }
  
  if (activateRelay14 == true && millis()- prevMillis > 8000)
  {
    activateRelay14 = false;
    emergenUnlocker2 = "0";
    digitalWrite(RELAY_2, LOW);  
  }

  if (emergenUnlocker3 == "1" && activateRelay15 == false)
  {
    disableLocker3 = "0";
    activateRelay15 = true;
    digitalWrite(RELAY_3, HIGH);

    id_list.head = 2;
    Serial.println("Deleting Face: LOCKER 3"); 
    delete_face_id_in_flash(&id_list);
    id_list.count = 0;
    
    prevMillis = millis();    
  }
  
  if (activateRelay15 == true && millis()- prevMillis > 8000)
  {
    activateRelay15 = false;
    emergenUnlocker3 = "0";
    digitalWrite(RELAY_3, LOW);  
  }

  if (emergenUnlocker4 == "1" && activateRelay16 == false)
  {
    disableLocker4 = "0";
    activateRelay16 = true;
    digitalWrite(RELAY_4, HIGH);

    id_list.head = 3;
    Serial.println("Deleting Face: LOCKER 4"); 
    delete_face_id_in_flash(&id_list);
    id_list.count = 0;

    prevMillis = millis();    
  }
  
  if (activateRelay16 == true && millis()- prevMillis > 8000)
  {
    activateRelay16 = false;
    emergenUnlocker4 = "0";
    digitalWrite(RELAY_4, LOW);  
  }

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);  
    Serial.print(".");
    ledcWrite(4,0);
    connect_times++;
    if(connect_times > 5)
    {
      ESP.restart();
    }
  }   
  if((millis()-timer) > 300)
  {
    read_axis();
    timer = millis();  
  }
  if((AngleX < 170 || AngleY < 170) && stateRelayBell == false)
  {
    stateRelayBell = true;
    digitalWrite(BELL, HIGH);
    
    oldMillis = millis();
  }
  if(stateRelayBell == true && millis() - oldMillis > 5000)
  {
    stateRelayBell= false;
    digitalWrite(BELL, LOW);
    
  }    
}

void read_axis() { 
  float rawX, rawY, rawZ;
  float acX, acY, acZ;
  float x, y, z;
  
  rawX = a.acceleration.x;
  rawY = a.acceleration.y;
  rawZ = a.acceleration.z;

  acX = rawX/4096.0;
  acY = rawY/4096.0;
  acZ = rawZ/4096.0;

  x= RAD_TO_DEG * (atan2(-acY, -acZ)+PI);
  y= RAD_TO_DEG * (atan2(-acX, -acZ)+PI);
  z= RAD_TO_DEG * (atan2(-acY, -acX)+PI);

  AngleX = abs(map(x,360,180,180,0));
  AngleY = abs(map(y,360,180,180,0));
  AngleZ = abs(map(z,360,180,180,0));
  Serial.print("AngleX : ");
  Serial.print(AngleX);
  Serial.print(", AngleY : ");
  Serial.print(AngleY);
  Serial.print(", AngleZ : ");
  Serial.println(AngleZ);
}
