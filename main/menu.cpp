#include "menu.h"
#include <Adafruit_SSD1306.h>
#include "ButtonHandler.h"
#include "sd_card.h"


bool sdEnabled = false;

extern Adafruit_SSD1306 oled;
extern ButtonHandler UPButton, DOWNButton, statusButton;

static uint8_t selectedHome = 0;
static uint8_t selectedMain = 0;
static uint8_t selectedImu = 0;
static uint8_t editingLEDIndex = 0;
uint8_t activePPGChannels = 0b1111; 
PPGUserConfig latestPPGConfig = getDefaultPPGConfig();

static bool menuReady = false;
extern bool inMenuMode;

enum MenuState {
    MENU_HOME,
    MENU_MAIN,
    MENU_IMU,
    MENU_WIFI,
    MENU_SD,
    MENU_LED_SELECT,
    MENU_LED_PW_SELECT,
    MENU_LED_POWER,
    MENU_LED_POWER_EDIT,
    MENU_LED_RANGE,
    MENU_IMU_ACCEL,
    MENU_IMU_GYRO

};

 
static MenuState menuState = MENU_HOME;
static uint8_t submenuItem = 0;

PPGUserConfig getDefaultPPGConfig() {
    PPGUserConfig cfg;
    cfg.sampleRateReg = 0x01;
    cfg.ledEnabled[0] = cfg.ledEnabled[1] = cfg.ledEnabled[2] =  true;
    cfg.ledEnabled[3] = true;
    cfg.ledWidth = 0x00;
    for (int i = 0; i < 4; i++) {
        cfg.ledPower[i] = 0xFF;
        cfg.ledRange[i] = 0;
    }
    return cfg;
}
PPGUserConfig defaults = getDefaultPPGConfig();

IMUUserConfig getDefaultIMUConfig() {
    IMUUserConfig cfg;
    cfg.sampleRateHz = 26;
    cfg.accelScale   = 4;
    cfg.gyroScale    = 250;
    return cfg;
}

static IMUUserConfig currentIMUConfig = getDefaultIMUConfig();

IMUUserConfig menu_getIMUConfig() {
    return currentIMUConfig;
}

static bool ledEnabled[4] = {
    defaults.ledEnabled[0],
    defaults.ledEnabled[1],
    defaults.ledEnabled[2],
    defaults.ledEnabled[3]
};
static uint8_t pulseWidth = defaults.ledWidth;
uint16_t desiredRate = 100; 
static uint8_t ledPower[4] = {
    defaults.ledPower[0],
    defaults.ledPower[1],
    defaults.ledPower[2],
    defaults.ledPower[3]
};
static uint8_t ledRange[4] = {
    defaults.ledRange[0],
    defaults.ledRange[1],
    defaults.ledRange[2],
    defaults.ledRange[3]
};

const char* getPWLabel(uint8_t pw) {
    switch (pw) {
        case 0: return "70us";
        case 1: return "120us";
        case 2: return "220us";
        case 3: return "420us";
        default: return "Invalid";
    }
}

float calcLEDCurrent_mA(uint8_t powerRegVal, uint8_t range) {
    const float stepSizes[] = {0.2f, 0.4f, 0.6f, 0.8f};
    if (range > 3) return 0.0f;
    return powerRegVal * stepSizes[range];
}

void increaseLEDPower(uint8_t& powerReg, uint8_t range, float step_mA = 0.8f) {
    const float stepSizes[] = {0.2f, 0.4f, 0.6f, 0.8f};
    if (range > 3) return;

    float step = stepSizes[range];
    float current_mA = powerReg * step;
    float next_mA = current_mA + step_mA;

    if (next_mA > step * 255) next_mA = 0.0f;

    powerReg = (uint8_t)(next_mA / step + 0.5f);
}

void decreaseLEDPower(uint8_t& powerReg, uint8_t range, float step_mA = 0.8f) {
    const float stepSizes[] = {0.2f, 0.4f, 0.6f, 0.8f};
    if (range > 3) return;

    float step = stepSizes[range];
    float current_mA = powerReg * step;
    float next_mA = current_mA - step_mA;

    if (next_mA < 0.0f) next_mA = step * 255;

    powerReg = (uint8_t)(next_mA / step + 0.5f);
}

void enterMenu() {
    inMenuMode = true;
    menuReady = false;
    menuState = MENU_HOME;
    selectedHome = 0;
    selectedMain = 0;
    selectedImu = 0;
    submenuItem = 0;
    drawMenu();
}

const uint8_t accelScales[] = {2, 4, 8, 16};
const int numAccel = sizeof(accelScales) / sizeof(accelScales[0]);

const uint16_t gyroScales[] = {125, 250, 500, 1000, 2000};
const int numGyro = sizeof(gyroScales) / sizeof(gyroScales[0]);

uint16_t validRates[4];
int numValidRates = 0;

void drawMenu() {
    oled.clearDisplay();
    oled.setCursor(0, 0);
    oled.setTextSize(1);
    if (menuState == MENU_HOME) {
        const char* options[] = {
            "PPG",
            "IMU",
            "Wi-Fi",
            "SD card",
            "Back"
        };

        for (int i = 0; i < 5; i++) {
            oled.setTextColor(i == selectedHome ? BLACK : WHITE, i == selectedHome ? WHITE : BLACK);
            oled.println(options[i]);
        }
    
        oled.display();
        return;
    }
    else if (menuState == MENU_SD) {
        oled.setTextSize(2);
        oled.setTextColor(WHITE);
        oled.setCursor(0, 0);
        oled.println("SD Card");
        oled.setTextSize(1);
        oled.setCursor(0, 20);
        oled.printf("SD log is %s\n", sdEnabled ? "ON" : "OFF");
        oled.display();
    }
    else if (menuState == MENU_WIFI) {
        oled.setTextSize(2);
        oled.setTextColor(WHITE);
        oled.setCursor(0, 0);
        oled.println("Wi-Fi");
        oled.setTextSize(1);
        oled.setCursor(0, 20);
        oled.printf("Wi-Fi is %s\n", wifiEnabled ? "ON" : "OFF");
        oled.display();
    }

    else if (menuState == MENU_IMU) {
        for (int i = 0; i < 3; i++) {
            oled.setTextColor(i == selectedImu ? BLACK : WHITE, i == selectedImu ? WHITE : BLACK);

            switch (i) {
                case 0: oled.printf("Accel: %dg\n", currentIMUConfig.accelScale); break;
                case 1: oled.printf("Gyro: %d dps\n", currentIMUConfig.gyroScale); break;
                case 2: oled.println("Start"); break;
                
            }
        }
    } 

    else if (menuState == MENU_MAIN) {
        for (int i = 0; i < 6; i++) {
            oled.setTextColor(i == selectedMain ? BLACK : WHITE, i == selectedMain ? WHITE : BLACK);
            switch (i) {
                case 0: {
                    oled.print("LEDs > "); 
                    const char* ledNames[] = {"IR", "RD", "GR", "BL"};
                    bool first = true;
                    for (int i = 0; i < 4; i++) {
                        if (ledEnabled[i]) {
                            if (!first) oled.print("+");
                            oled.print(ledNames[i]);
                            first = false;
                        }
                    }
                    oled.println();
                    break;
                }
                case 1: oled.print("PW > "); oled.println(getPWLabel(pulseWidth)); break;
                case 2: oled.println("LED Power >"); break;
                case 3: oled.println("LED Range >"); break;
                case 4: oled.println("Start"); break;
            }
        }
    } 
    else if (menuState == MENU_LED_SELECT) {
        const char* ledNames[] = {"IR", "RED", "GREEN", "BLUE"};
        for (int i = 0; i < 5; i++) {
            oled.setTextColor(i == submenuItem ? BLACK : WHITE, i == submenuItem ? WHITE : BLACK);
            if (i < 4)
                oled.printf("%s: %s\n", ledNames[i], ledEnabled[i] ? "ON" : "OFF");
            else
                oled.println("Back");
        }
    }   
    else if (menuState == MENU_LED_PW_SELECT) {
        for (int i = 0; i < 5; i++) {
            oled.setTextColor(i == submenuItem ? BLACK : WHITE, i == submenuItem ? WHITE : BLACK);
            if (i < 4) oled.println(getPWLabel(i));
            else oled.println("Back");
        }
    }   

    else if (menuState == MENU_LED_POWER) {
        const char* ledNames[] = {"IR", "RED", "GREEN", "BLUE"};
        for (int i = 0; i < 5; i++) {
            oled.setTextColor(i == submenuItem ? BLACK : WHITE, i == submenuItem ? WHITE : BLACK);
            if (i < 4) oled.printf("%s Power: %.1f mA\n", ledNames[i], calcLEDCurrent_mA(ledPower[i], ledRange[i]));
            else oled.println("Back");
        }
    } 

    else if (menuState == MENU_LED_POWER_EDIT) {
        oled.setCursor(0, 0);
        oled.setTextSize(1);
        oled.setTextColor(WHITE);
        const char* ledNames[] = {"IR", "RED", "GREEN", "BLUE"};
        float current = calcLEDCurrent_mA(ledPower[editingLEDIndex], ledRange[editingLEDIndex]);
        oled.printf("%s Power:\n", ledNames[editingLEDIndex]);
        oled.printf("HEX: 0x%02X\n", ledPower[editingLEDIndex]);
        oled.printf("Current: %.1f mA\n", current);
    }
    
    else if (menuState == MENU_LED_RANGE) {
        const char* ledNames[] = {"IR", "RED", "GREEN", "BLUE"};
        const char* ledRangeOptions[] = {"50mA", "100mA", "150 mA", "200 mA"};
        for (int i = 0; i < 5; i++) {
            oled.setTextColor(i == submenuItem ? BLACK : WHITE, i == submenuItem ? WHITE : BLACK);
            if (i < 4) oled.printf("%s Range: %s\n", ledNames[i], ledRangeOptions[ledRange[i]]);
            else oled.println("Back");
        }
    }

    else if (menuState == MENU_IMU_ACCEL) {
        oled.println("Accel Scale:");
        for (int i = 0; i < numAccel + 1; i++) {
            oled.setTextColor(i == submenuItem ? BLACK : WHITE, i == submenuItem ? WHITE : BLACK);
            if (i < numAccel)
                oled.printf(" %dg\n", accelScales[i]);
            else
                oled.println("Back");
        }
    }

    else if (menuState == MENU_IMU_GYRO) {
        oled.println("Gyro Scale:");
        for (int i = 0; i < numGyro + 1; i++) {
            oled.setTextColor(i == submenuItem ? BLACK : WHITE, i == submenuItem ? WHITE : BLACK);
            if (i < numGyro)
                oled.printf(" %d dps\n", gyroScales[i]);
            else
                oled.println("Back");
        }
    }
    

    oled.display();
}


void menu_update() {
    if (menuReady) return;
    if (menuState == MENU_HOME){
        if (DOWNButton.isPressed()) { 
            selectedHome = (selectedHome + 1) % 5; drawMenu();
        }
        if (UPButton.isPressed()) { 
            selectedHome = (selectedHome + 4) % 5; drawMenu(); 
        }
        if (statusButton.isPressed()) {
            switch (selectedHome) {
                case 0: menuState = MENU_MAIN; submenuItem = 0; break;
                case 1: menuState = MENU_IMU; submenuItem = 0; break;
                case 2: menuState = MENU_WIFI; submenuItem = 0; break;
                case 3: menuState = MENU_SD; submenuItem = 0; break;
                case 4: menuReady = true; break;
            }
            drawMenu();
        }
            
    }
    else if (menuState == MENU_SD) {
        if (statusButton.isPressed()) { 
            sdEnabled = !sdEnabled;
    
            if (sdEnabled) {
                digitalWrite(USB_MUX, HIGH);
                delay(100); 
                if (sd_card_init() != ESP_OK) {
                    ESP_LOGE("MENU", "SD init failed after mux HIGH");
                    sdEnabled = false;
                    digitalWrite(USB_MUX, LOW);
                }
            } 
            else {
                stopSDLogFile();
                digitalWrite(USB_MUX, LOW);
            }
            drawMenu();
        }
    
        if (DOWNButton.isPressed() || UPButton.isPressed()) {
            menuState = MENU_HOME;
            drawMenu();
        }
    }
    
    else if (menuState == MENU_WIFI) {
        if (statusButton.isPressed()) {
            wifiEnabled = !wifiEnabled;
            drawMenu();
        }
        if (DOWNButton.isPressed() || UPButton.isPressed()) {
            menuState = MENU_HOME;
            drawMenu();
        }
    }
    else if (menuState == MENU_IMU){
        if (DOWNButton.isPressed()) { 
            selectedImu = (selectedImu + 1) % 3; drawMenu();
        }
        if (UPButton.isPressed()) { 
            selectedImu = (selectedImu + 2) % 3; drawMenu(); 
        }
        if (statusButton.isPressed()) {
            switch (selectedImu) {
                case 0: menuState = MENU_IMU_ACCEL; submenuItem = 0; break;
                case 1: menuState = MENU_IMU_GYRO; submenuItem = 0; break;
                case 2: menuReady = true; break;
            }
            drawMenu();
        }
            
    }
    else if (menuState == MENU_MAIN) {
        if (DOWNButton.isPressed()) { selectedMain = (selectedMain + 1) % 5; drawMenu(); }
        if (UPButton.isPressed()) { selectedMain = (selectedMain + 4) % 5; drawMenu(); }

        if (statusButton.isPressed()) {
            switch (selectedMain) {
                case 0: menuState = MENU_LED_SELECT; submenuItem = 0; break;
                case 1: menuState = MENU_LED_PW_SELECT; submenuItem = pulseWidth; break;
                case 2: menuState = MENU_LED_POWER; submenuItem = 0; break;
                case 3: menuState = MENU_LED_RANGE; submenuItem = 0; break;
                case 4: menuReady = true; break;
            }
            drawMenu();
        }
    } 
    else if (menuState == MENU_LED_SELECT) {
        if (UPButton.isPressed()) {
            submenuItem = (submenuItem + 4) % 5;
            drawMenu();
        }
    
        if (DOWNButton.isPressed()) {
            submenuItem = (submenuItem + 1) % 5;
            drawMenu();
        }
    
        if (statusButton.isPressed()) {
            if (submenuItem < 4) {
                ledEnabled[submenuItem] = !ledEnabled[submenuItem];
            } else {
                menuState = MENU_MAIN;
            }
            drawMenu();
        }
    }
    else if (menuState == MENU_LED_PW_SELECT) {
        if (DOWNButton.isPressed()) { submenuItem = (submenuItem + 1) % 5; drawMenu(); }
        if (UPButton.isPressed()) { submenuItem = (submenuItem + 4) % 5; drawMenu(); }
    
        if (statusButton.isPressed()) {
            if (submenuItem < 4) pulseWidth = submenuItem;
            menuState = MENU_MAIN;
            drawMenu();
        }
    }
    else if (menuState == MENU_LED_POWER) {
        if (UPButton.isPressed()) {
            submenuItem = (submenuItem + 4) % 5;
            drawMenu();
        }
    
        if (DOWNButton.isPressed()) {
            submenuItem = (submenuItem + 1) % 5;
            drawMenu();
        }
    
        if (statusButton.isPressed()) {
            if (submenuItem < 4) {
                editingLEDIndex = submenuItem;
                menuState = MENU_LED_POWER_EDIT;
            } else {
                menuState = MENU_MAIN;
            }
            drawMenu();
        }
    }
    else if (menuState == MENU_LED_POWER_EDIT) {
        if (UPButton.isPressed()) {
            increaseLEDPower(ledPower[editingLEDIndex], ledRange[editingLEDIndex]);
            drawMenu();
        }
    
        if (DOWNButton.isPressed()) {
            decreaseLEDPower(ledPower[editingLEDIndex], ledRange[editingLEDIndex]);
            drawMenu();
        }
    
        if (statusButton.isPressed()) {
            menuState = MENU_LED_POWER;
            drawMenu();
        }
    }
    
    else if (menuState == MENU_LED_RANGE) {
        if (DOWNButton.isPressed()) { submenuItem = (submenuItem + 1) % 5; drawMenu(); }
        if (UPButton.isPressed()) { submenuItem = (submenuItem + 4) % 5; drawMenu(); }

        if (statusButton.isPressed()) {
            if (submenuItem < 4) {
                ledRange[submenuItem] = (ledRange[submenuItem] + 1) % 4;
            } else {
                menuState = MENU_MAIN;
            }
            drawMenu();
        }
    }
    else if (menuState == MENU_IMU_ACCEL) {
        if (DOWNButton.isPressed()) { submenuItem = (submenuItem + 1) % (numAccel + 1); drawMenu(); }
        if (UPButton.isPressed())   { submenuItem = (submenuItem + numAccel) % (numAccel + 1); drawMenu(); }
    
        if (statusButton.isPressed()) {
            if (submenuItem < numAccel)
                currentIMUConfig.accelScale = accelScales[submenuItem];
            menuState = MENU_IMU;
            drawMenu();
        }
    }
    else if (menuState == MENU_IMU_GYRO) {
        if (DOWNButton.isPressed()) { submenuItem = (submenuItem + 1) % (numGyro + 1); drawMenu(); }
        if (UPButton.isPressed())   { submenuItem = (submenuItem + numGyro) % (numGyro + 1); drawMenu(); }
    
        if (statusButton.isPressed()) {
            if (submenuItem < numGyro)
                currentIMUConfig.gyroScale = gyroScales[submenuItem];
            menuState = MENU_IMU;
            drawMenu();
        }
    }
    
}

bool menu_isReady() {
    return menuReady;
}

uint8_t mapSampleRateToReg(uint16_t rate) {
    switch (rate) {
        case 50: return 0x00;
        case 100: return 0x01;
        case 200: return 0x02;
        case 400: return 0x03;
        default: return 0x01;
    }
}

PPGUserConfig menu_getConfig() {
    PPGUserConfig cfg;

    cfg.sampleRateReg = mapSampleRateToReg(desiredRate);
    cfg.ledWidth = pulseWidth;

    for (int i = 0; i < 4; i++) {
        cfg.ledEnabled[i] = ledEnabled[i];
        cfg.ledPower[i] = ledPower[i];
        cfg.ledRange[i] = ledRange[i];
    }

    activePPGChannels = 0; 
    for (int i = 0; i < 4; i++) {
        if (cfg.ledEnabled[i]) {
            activePPGChannels |= (1 << i); 
        }
    }
    
    latestPPGConfig = cfg; 
    return cfg;
}



