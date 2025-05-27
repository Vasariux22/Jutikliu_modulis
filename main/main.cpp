#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <stdio.h>
#include <errno.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/sdspi_host.h"
#include <sys/socket.h>

#include "wifi.h"
#include "ButtonHandler.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include <MAX17055.h>

#include "battery.h"
#include "MLX90632.h"
#include "LSM6DSOSensor.h"
#include "MAX86916_cm.h"
#include "ppg_basic.h"
#include "menu.h"
#include "imu.h"
#include "temp.h"
#include "scheduler.h"
#include "sd_card.h"


uint64_t laikas;
int safeguard_counter = 0;
int reset_counter = 0;
int numeris;
int bat_vel;
int kiek;
int brightness = 0;
int fadeAmount = 5;

uint16_t gautas_crc, calc_crc;
unsigned long timecounter;

bool inMenuMode = false;
bool lastWiFiState = false;
TaskHandle_t tcpTaskHandle = NULL; 

TwoWire PPGWire(1);

uint16_t activeSampleRateHz = 0;

static unsigned long lastPPG = 0;
static unsigned long packetStartTime = 0;
bool wifiEnabled = false;

uint16_t imuSampleRateHz = 52;
unsigned long lastIMU = 0;



// Oled
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


MAX17055 BAT;
float resistSensorValue;
float SOC;
float capacity;
float RepCap;
float voltage;
float current;
float TTE;
int update_status;


char last_message[60];
void displayMessage(const char *message)
{
  oled.clearDisplay();
  oled.setTextSize(2);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 10);
  oled.println(message);
  oled.display();
}

void tcpTask(void *) {
  while (true) {
      if (wifiRunning && !tcpConnected) {
          if (!connectToTCPServer()) {
              ESP_LOGW("TCP", "Try again");
              vTaskDelay(pdMS_TO_TICKS(3000));
              continue;
          }
      }

      if (tcpConnected) {
          char buf[64];
          int ret = recv(sock, buf, sizeof(buf), 0);

          if (ret == 0 || (ret < 0 && errno != EWOULDBLOCK && errno != EAGAIN)) {
              ESP_LOGW("TCP", "Error (errno=%d)", errno);
              close(sock);
              sock = -1;
              tcpConnected = false;
          }
      }

      vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Battery info function
void displayBatteryInfo() {
  oled.clearDisplay();
  oled.drawBitmap(87, 0, Img1.data, Img1.width, Img1.height, WHITE);
  if (current > 0) {
    oled.drawBitmap(95, 6, Img2.data, Img2.width, Img2.height, WHITE);
  }

  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 10);

  {
    oled.print("U:");
    oled.print(voltage, 2);
    oled.println("V");

    oled.print("I:");
    oled.print(current, 2);
    oled.println("mA");

    oled.setCursor(98, 20);
    oled.print(TTE, 1);
    oled.print("h");
  }

  oled.setCursor(0, 30);
  oled.print(last_message);

  oled.setCursor(105, 7);
  oled.setTextSize(1);
  oled.print(static_cast<int>(SOC));

  oled.setCursor(0, 57);
  oled.print("Temperature: ");
  oled.print(tempC);
  oled.print("C");

  oled.display();
}

// Buttons
ButtonHandler DOWNButton(PB_DOWN);
ButtonHandler UPButton(PB_UP);
ButtonHandler statusButton(PB_OK);


extern "C" void app_main()
{
  initArduino();
  Serial.begin(115200);
  esp_log_level_set("gpio", ESP_LOG_ERROR);
  packetStartTime = millis();

  //Ijungimo islaikymas
  pinMode(PWR_EN, OUTPUT);
  digitalWrite(PWR_EN, HIGH);

  //Krovimo rezimas
  pinMode(CRG_MODE, OUTPUT);
  digitalWrite(CRG_MODE, LOW);

  // Mygtukai
  pinMode(PB_UP, INPUT_PULLDOWN);
  pinMode(PB_OK, INPUT_PULLDOWN);
  pinMode(PB_DOWN, INPUT_PULLDOWN);

  pinMode(USB_MUX, OUTPUT);
  digitalWrite(USB_MUX, LOW); // SD CARD -> USB reader
  
  // SD korteles maitinimas
  pinMode(SD_EN, OUTPUT);
  digitalWrite(SD_EN, HIGH);

  spi_bus_config_t bus_config = {
      .mosi_io_num = GPIO_NUM_11,
      .miso_io_num = GPIO_NUM_13,
      .sclk_io_num = GPIO_NUM_12,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 4000,
  };
  esp_err_t ret = spi_bus_initialize(SPI2_HOST, &bus_config, SDSPI_DEFAULT_DMA);
  if (ret != ESP_OK) {
      ESP_LOGE("MAIN", "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
  }


  //I2C
  PPGWire.begin(5, 6, 400000);
  Wire.begin(41, 40, 400000); 

  //oled
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)){ 
    printf("SSD1306 init failed");
  }
  printf("SSD1306 oled init ok");

  delay(2000); 
  oled.clearDisplay();
  oled.setTextSize(2); 
  oled.setTextColor(WHITE);
  oled.setCursor(0, 10); 
  oled.println("o");
  oled.display();

  //ppg
  PPGUserConfig ppgConfig = getDefaultPPGConfig();
  activeSampleRateHz = desiredRate; 
  ppg_basic_init(&PPGWire, ppgConfig);
  
  //imu
  IMUUserConfig imuConfig = getDefaultIMUConfig();
  imu_init(&PPGWire, imuConfig); 

  //temp
  temp_init(&PPGWire);

  SensorScheduler_init(&PPGWire, activeSampleRateHz, imuSampleRateHz, 2);

  if (BAT.init(500, 0.01, false) == ESP_OK) {
    resistSensorValue = BAT.getResistSensor();
    SOC = BAT.getSOC();
    capacity = BAT.getCapacity();
    voltage = BAT.getInstantaneousVoltage();
    current = BAT.getInstantaneousCurrent();
    TTE = BAT.getTimeToEmpty();
    resistSensorValue = BAT.getResistSensor();
    SOC = BAT.getSOC();
    capacity = BAT.getCapacity();
    displayBatteryInfo();
  }
  else {
    Serial.println(F("BMS read failed"));
  }

  // loop()
  while (true) {
    if ((millis() - timecounter) > 5) { 
      timecounter = millis();

      //Baterija
      bat_vel++; 
      if ((bat_vel > 40) && !inMenuMode) {
        bat_vel = 0;
        voltage = BAT.getInstantaneousVoltage();
        current = BAT.getInstantaneousCurrent();
        RepCap = BAT.getRepCapacity();
        TTE = BAT.getTimeToEmpty();
        SOC = BAT.getSOC();
        displayBatteryInfo();
      }
      
      // Mygtukai
      DOWNButton.update();
      UPButton.update();
      statusButton.update();

      // Meniu
      if (!inMenuMode) {
        if (DOWNButton.isPressed()) {
            enterMenu(); 
            SensorScheduler_deinit();
        }
      } 
      else {
        menu_update();
        if (menu_isReady()) {
            ppgConfig = menu_getConfig();
            activeSampleRateHz = desiredRate;
            ppg_basic_init(&PPGWire, ppgConfig);

            IMUUserConfig imuConfig = menu_getIMUConfig();
            imuSampleRateHz = imuConfig.sampleRateHz; 
            imu_init(&PPGWire, imuConfig); 

            SensorScheduler_init(&PPGWire, activeSampleRateHz, imuSampleRateHz, 2);
            inMenuMode = false; 
        }
      }
      // Wi-Fi
      if (wifiEnabled != lastWiFiState) {
        if (wifiEnabled) {
          enableWiFi(); 

          if (tcpTaskHandle == NULL) {
              xTaskCreatePinnedToCore(tcpTask, "tcpTask", 4096, NULL, 5, &tcpTaskHandle, 1);
          }
        } 
        else {
          disableWiFi(); 

          if (tcpTaskHandle != NULL) {
            vTaskDelete(tcpTaskHandle);
            tcpTaskHandle = NULL;
          }
        }
        lastWiFiState = wifiEnabled; 
      }

      // Isjungimas
      if (statusButton.isLongPressed(3000)) {
        displayMessage("Shutting down...");
        delay(1000);
        digitalWrite(PWR_EN, LOW);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}


