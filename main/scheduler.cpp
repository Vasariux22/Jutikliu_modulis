#include "scheduler.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include "Arduino.h"
#include "wifi.h"
#include "MLX90632.h"
#include "menu.h"
#include "sd_card.h"

static SemaphoreHandle_t i2cMutex = nullptr;

TaskHandle_t ppgTaskHandle   = NULL;
TaskHandle_t imuTaskHandle   = NULL;
TaskHandle_t tempTaskHandle   = NULL;
TaskHandle_t pktTaskHandle   = NULL;

static bool log_file_open = false;
float tempC = NAN; 


struct __attribute__((packed)) SensorPacket {
  uint32_t device_id;
  uint64_t timestamp_ms;
  uint16_t ppg_count;
  uint16_t imu_count;
  uint16_t temp_count;
  uint8_t  ppg_channels;
  uint8_t  data[];
};

#define MAX_PAYLOAD_SIZE  (MAX_PPG_SAMPLES * sizeof(PPGSample) + MAX_IMU_SAMPLES * sizeof(IMUSample) + MAX_TEMP_SAMPLES * sizeof(TempSample))

#define MAX_PACKET_SIZE   (sizeof(SensorPacket) + MAX_PAYLOAD_SIZE)

static uint8_t sensorPacketBuffer[MAX_PACKET_SIZE];

static QueueHandle_t ppgDataQueue;
static QueueHandle_t imuDataQueue;
static QueueHandle_t tmpDataQueue;
static TwoWire*      _ppgBus;
static volatile uint32_t g_ppgHz;
static volatile uint16_t g_imuHz;

static PPGSample ppgBuf[MAX_PPG_SAMPLES];
static IMUSample imuBuf[MAX_IMU_SAMPLES];
static TempSample tmpBuf[MAX_TEMP_SAMPLES];

static void buildAndSendPacket(size_t pc, size_t ic, size_t tc) {
  if (!tcpConnected) return;

  uint8_t ppg_channels = __builtin_popcount(activePPGChannels);

  size_t ppg_sample_size = ppg_channels * sizeof(uint32_t);
  size_t payload_size = pc * ppg_sample_size + ic * sizeof(IMUSample) + tc * sizeof(TempSample);

  size_t total_size = sizeof(SensorPacket) + payload_size;
  if (total_size > MAX_PACKET_SIZE) return;

  SensorPacket* pkt = reinterpret_cast<SensorPacket*>(sensorPacketBuffer);

  pkt->device_id    = 1234;
  pkt->timestamp_ms = esp_timer_get_time() / 1000;
  pkt->ppg_count    = pc;
  pkt->imu_count    = ic;
  pkt->temp_count   = tc;
  pkt->ppg_channels = activePPGChannels;

  uint8_t* ptr = pkt->data;

  for (size_t i = 0; i < pc; ++i) {
    const PPGSample& s = ppgBuf[i];

    if (activePPGChannels & (1 << 0)) { memcpy(ptr, &s.ir, sizeof(uint32_t)); ptr += 4; }
    if (activePPGChannels & (1 << 1)) { memcpy(ptr, &s.red, sizeof(uint32_t)); ptr += 4; }
    if (activePPGChannels & (1 << 2)) { memcpy(ptr, &s.green, sizeof(uint32_t)); ptr += 4; }
    if (activePPGChannels & (1 << 3)) { memcpy(ptr, &s.blue, sizeof(uint32_t)); ptr += 4; }
}

  if (ic > 0) {
    memcpy(ptr, imuBuf, ic * sizeof(IMUSample));
    ptr += ic * sizeof(IMUSample);
  }

  if (tc > 0) {
    memcpy(ptr, tmpBuf, tc * sizeof(TempSample));
    ptr += tc * sizeof(TempSample);
  }

  sendPacket(pkt, total_size);
}

static void PPGTask(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10);  // 100 Hz

  for (;;) {
    vTaskDelayUntil(&last, period);

    PPGSample sample;

    if (xSemaphoreTake(i2cMutex, 0)) {
      bool ok = ppg_basic_read(_ppgBus, &sample.ir, &sample.red, &sample.green, &sample.blue);
      xSemaphoreGive(i2cMutex);
      if (!ok) continue;
      xQueueSend(ppgDataQueue, &sample, 0); 
    }
  }
}

static void IMUTask(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000);  // 1 Hz

  for (;;) {
    vTaskDelayUntil(&last, period);
    int count = 0;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
      count = imu_read_fifo(imuBuf, MAX_IMU_SAMPLES);
      xSemaphoreGive(i2cMutex);
    }
    for (int i = 0; i < count; ++i) {
      xQueueSend(imuDataQueue, &imuBuf[i], 0);
    }
  }
}



static void TempTask(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(10000);  // 0.1 Hz

  for (;;) {
    vTaskDelayUntil(&last, period);
    TempSample s;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
      s.temperatureC = temp_get();
      xSemaphoreGive(i2cMutex);
    }
    xQueueSend(tmpDataQueue, &s, 0);
  }
}

static void PacketTask(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(1000);

  for (;;) {
      vTaskDelayUntil(&last, period);

      size_t pc = 0, ic = 0, tc = 0;

      PPGSample ps;
      while (pc < MAX_PPG_SAMPLES && xQueueReceive(ppgDataQueue, &ps, 0) == pdTRUE)
          ppgBuf[pc++] = ps;
      
      IMUSample im;
      while (ic < MAX_IMU_SAMPLES && xQueueReceive(imuDataQueue, &im, 0) == pdTRUE)
          imuBuf[ic++] = im;
      
      TempSample ts;
      while (tc < MAX_TEMP_SAMPLES && xQueueReceive(tmpDataQueue, &ts, 0) == pdTRUE)
          tmpBuf[tc++] = ts;

        if (tc > 0) {
          tempC = tmpBuf[tc - 1].temperatureC;
        }

      if (sdEnabled && !log_file_open) {
        startNewSDLogFile(); 
        log_file_open = true;
      }
      if (!sdEnabled && log_file_open) {
        stopSDLogFile();
        log_file_open = false;
      }

      // for (size_t i = 0; i < pc; ++i) {
      //   ESP_LOGI("PktTask", "PPG[%d] IR=%lu RED=%lu GREEN=%lu BLUE=%lu",
      //             (int)i,
      //             (unsigned long)ppgBuf[i].ir,
      //             (unsigned long)ppgBuf[i].red,
      //             (unsigned long)ppgBuf[i].green,
      //             (unsigned long)ppgBuf[i].blue);
      // }
          
      ESP_LOGI("PktTask", "PACKET START — PPG=%u, IMU=%u, TEMP=%u", (unsigned)pc, (unsigned)ic, (unsigned)tc);

      //for (size_t i = 0; i < ic; ++i) {
        // ESP_LOGI("PktTask", "IMU[%d] ACC=[%ld %ld %ld] GYR=[%ld %ld %ld]",
        //   (int)i,
        //   (long)imuBuf[i].acc[0], (long)imuBuf[i].acc[1], (long)imuBuf[i].acc[2],
        //   (long)imuBuf[i].gyr[0], (long)imuBuf[i].gyr[1], (long)imuBuf[i].gyr[2]);
        // }
        

      // static int counter = 0;
      // if (++counter >= 10) { 
      //   counter = 0;
      //   ESP_LOGI("Stack", "PPG  stack free: %u bytes", uxTaskGetStackHighWaterMark(ppgTaskHandle)  * sizeof(StackType_t));
      //   ESP_LOGI("Stack", "IMU  stack free: %u bytes", uxTaskGetStackHighWaterMark(imuTaskHandle)  * sizeof(StackType_t));
      //   ESP_LOGI("Stack", "TEMP stack free: %u bytes", uxTaskGetStackHighWaterMark(tempTaskHandle) * sizeof(StackType_t));
      //   ESP_LOGI("Stack", "PKT  stack free: %u bytes", uxTaskGetStackHighWaterMark(pktTaskHandle)  * sizeof(StackType_t));
      // }


      if ((pc + ic + tc) > 0 && tcpConnected) {
          buildAndSendPacket(pc, ic, tc);
      }

      if (log_file_open && (pc + ic + tc) > 0) {
        sd_card_write_samples(pc, ic, tc, ppgBuf, imuBuf, tmpBuf);
      }
    
  }
}

void SensorScheduler_init(TwoWire* bus, uint32_t pHz, uint16_t iHz, uint16_t /*t*/) {
  _ppgBus = bus;
  g_ppgHz = pHz;
  g_imuHz = iHz;
  i2cMutex = xSemaphoreCreateMutex();
  configASSERT(i2cMutex);

  vTaskDelay(pdMS_TO_TICKS(500));

  xTaskCreatePinnedToCore(PPGTask,   "PPGTask",  4096, nullptr, configMAX_PRIORITIES-2, &ppgTaskHandle, 1);
  xTaskCreatePinnedToCore(IMUTask,   "IMUTask",  4096, nullptr, configMAX_PRIORITIES-3, &imuTaskHandle, 1);
  xTaskCreatePinnedToCore(TempTask,  "TempTask", 4096, nullptr, configMAX_PRIORITIES-3, &tempTaskHandle, 1);
  xTaskCreatePinnedToCore(PacketTask,"PktTask", 8192, nullptr, configMAX_PRIORITIES-4, &pktTaskHandle, 1);

  ppgDataQueue = xQueueCreate(MAX_PPG_SAMPLES, sizeof(PPGSample));
  configASSERT(ppgDataQueue);
  
  imuDataQueue = xQueueCreate(MAX_IMU_SAMPLES, sizeof(IMUSample));
  configASSERT(imuDataQueue);
  
  tmpDataQueue = xQueueCreate(MAX_TEMP_SAMPLES, sizeof(TempSample));
  configASSERT(tmpDataQueue);
  
}

void SensorScheduler_deinit() {
  if (ppgTaskHandle != NULL) {
      vTaskDelete(ppgTaskHandle);
      ppgTaskHandle = NULL;
  }
  if (imuTaskHandle != NULL) {
      vTaskDelete(imuTaskHandle);
      imuTaskHandle = NULL;
  }
  if (pktTaskHandle != NULL) {
      vTaskDelete(pktTaskHandle);
      pktTaskHandle = NULL;
  }
  if (tempTaskHandle != NULL) {
    vTaskDelete(tempTaskHandle);
    tempTaskHandle = NULL;
  }

  if (ppgDataQueue) vQueueDelete(ppgDataQueue);
  if (imuDataQueue) vQueueDelete(imuDataQueue);
  if (tmpDataQueue) vQueueDelete(tmpDataQueue);

}
