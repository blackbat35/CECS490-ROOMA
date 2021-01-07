#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiServer.h>
#include <ESP32Servo.h>
#include <WiFiUdp.h>
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"

WiFiClient client;
WiFiServer server(8880);
unsigned char cmd[6];
Servo myservo;
long duration, cm ;
uint8_t text;

int pos = 0;
int minUs = 500;
int maxUs = 2500;
#define trigPin 3
#define echoPin 16

#define ServoPin 12

#define IN1  13
#define IN2  15
#define IN3  14
#define IN4  2

#define LED2 4

#define PART_BOUNDARY "123456789000000000000987654321"
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

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
  }
  return res;
}



void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
}
void setup() {
 WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 ESP32PWM::allocateTimer(2);
 myservo.attach(ServoPin, minUs, maxUs); 
 
 Serial.begin(9600);
 Serial.setDebugOutput(false);

 pinMode(LED2, OUTPUT); // Flash pin

 // Setup pins for ultrasonic sensor
 pinMode(trigPin, OUTPUT);
 pinMode(echoPin, INPUT);
 
 // Setup pins for L298N Motor Drive
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, LOW);
 digitalWrite(IN3, LOW);
 digitalWrite(IN4, LOW);
 

 // Setup for camera module
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
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 

  // Change the quality of the camera module 
  // Adjust the value of jpeg_quality from 10 to 60
  if(psramFound()){
    config.frame_size   = FRAMESIZE_SVGA;
    config.jpeg_quality = 20;
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count     = 1;
  }
  
  // Camera init
  esp_err_t err = esp_camera_init(&config);
 // Create an access point
 WiFi.mode(WIFI_AP);
 WiFi.softAP("myboard"); 
 server.begin();
 startCameraServer(); // Streaming
}

void loop() {
  client = server.available();
  
  digitalWrite(LED2,LOW);
  
  if(client) {
    Serial.println("Connected!");
    while(client.connected()){
    while(!client.connected()){
      if(!client.connected()) break;
      delay(1);
    }
    char a = client.read();
    
      switch(a)
      { 
        // Drawing Map of the Environment
        // Servo degrees change from 0 to 180       
        case 'O': for (pos = 0; pos <= 180; pos ++) {
                  myservo.write(pos);
                  delayMicroseconds(15);
                  digitalWrite(trigPin, LOW);
                  delayMicroseconds(5);
                  digitalWrite(trigPin, HIGH);
                  delayMicroseconds(10);
                  digitalWrite(trigPin, LOW);
                  pinMode(echoPin, INPUT);
                  duration = pulseIn(echoPin, HIGH);
                  cm = (duration/2) / 29.1;
                  if (cm>=200) cm=200;
                  
                  //Serial.println(pos);
                  //Serial.println(cm);
                  client.println(pos);
                  client.println(cm);
                  delay(400);
        }
        break;
        
        // Servo degrees change from 180 to 0   
        case 'P': for (pos = 180; pos >= 0; pos --) {
                  myservo.write(pos);
                  delayMicroseconds(15);         
                  digitalWrite(trigPin, LOW);
                  delayMicroseconds(5);
                  digitalWrite(trigPin, HIGH);
                  delayMicroseconds(10);
                  digitalWrite(trigPin, LOW);
                  pinMode(echoPin, INPUT);
                  duration = pulseIn(echoPin, HIGH);
                  cm = (duration/2) / 29.1;
                  if (cm>=200) cm=200;
                 
                  //Serial.println(pos);
                  //Serial.println(cm);
                  client.println(pos);
                  client.println(cm);
                  delay(400);
        }
        break;

        // Control L298N Motor Driver Module
        // Foward
        case 'W': digitalWrite(IN1, HIGH);
                  digitalWrite(IN2, LOW);
                  digitalWrite(IN3, HIGH);
                  digitalWrite(IN4, LOW);
                  Serial.println("Foward");
        break;
         // Reverse
        case 'S': digitalWrite(IN1, LOW);
                  digitalWrite(IN2, HIGH);
                  digitalWrite(IN3, LOW);
                  digitalWrite(IN4, HIGH);
                  Serial.println("Reverse");
        break;
         // Left
        case 'A': digitalWrite(IN1, LOW);
                  digitalWrite(IN2, LOW);
                  digitalWrite(IN3, HIGH);
                  digitalWrite(IN4, LOW);
                  Serial.println("Left");
        break;
         // Right
        case 'D': digitalWrite(IN1, HIGH);
                  digitalWrite(IN2, LOW);
                  digitalWrite(IN3, LOW);
                  digitalWrite(IN4, LOW);
                  Serial.println("Right");
        break;
         // Stoprobot
        case 'Z': digitalWrite(IN1, LOW);
                  digitalWrite(IN2, LOW);
                  digitalWrite(IN3, LOW);
                  digitalWrite(IN4, LOW);
                  Serial.println("Stoprobot");
        break;

        // Servo
        /*
        case 'G': if(Serial.available()){
                      digitalWrite(IN1,HIGH);
                      text = client.read();
                      if(text == 255)
                        digitalWrite(IN2,HIGH);
                      Serial.println(text);
        }             
        break;
        */
        case 'G':myservo.write(0);
                  delayMicroseconds(50);
        break;
        case 'H':myservo.write(45);
                  delayMicroseconds(50);
        break;  
        case 'J':myservo.write(90);
                  delayMicroseconds(50);
        break;   
        case 'K':myservo.write(135);
                  delayMicroseconds(50);
        break;
        case 'L':myservo.write(180);
                  delayMicroseconds(50);
        break;
               
        // Turn on/off flash
        // Turn off
        case 'Q': digitalWrite(LED2,LOW); 
        break;
        // Turn on
        case 'E': digitalWrite(LED2,HIGH);
        break;
      }
    }
  }
  
}
