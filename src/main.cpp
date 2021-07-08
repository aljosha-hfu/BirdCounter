#include <ADCSampler.h>
#include <Arduino.h>
#include <DACOutput.h>
#include <FreeRTOS.h>
#include <I2SMEMSSampler.h>
#include <I2SOutput.h>
#include <SDCard.h>
#include <WAVFileReader.h>
#include <WAVFileWriter.h>
#include <WiFi.h>
#include <esp_now.h>
#include <stdio.h>

#include "SPIFFS.h"
#include "config.h"

boolean play_blue_tit_sound = false;
boolean play_red_kite_sound = false;
boolean play_common_buzzard_sound = false;
boolean play_european_robin_sound = false;

int blue_tit_button_touch_time = 0;
int red_kite_button_touch_time = 0;
int common_buzzard_button_touch_time = 0;
int european_robin_button_touch_time = 0;

int threshold = 15;
int touch_time_min = 10;

int go_to_sleep = 10000;

int highscore = 0;  // TODO update when new highscore received

// binary array
int num[10][4] = {{0, 0, 0, 0},   // 0
                  {0, 0, 0, 1},   // 1
                  {0, 0, 1, 0},   // 2
                  {0, 0, 1, 1},   // 3
                  {0, 1, 0, 0},   // 4
                  {0, 1, 0, 1},   // 5
                  {0, 1, 1, 0},   // 6
                  {0, 1, 1, 1},   // 7
                  {1, 0, 0, 0},   // 8
                  {1, 0, 0, 1}};  // 9

void play(Output *output, const char *fname) {
    int16_t *samples = (int16_t *)malloc(sizeof(int16_t) * 1024);
    // open the file on the sdcard
    FILE *fp = fopen(fname, "rb");
    // create a new wave file writer
    WAVFileReader *reader = new WAVFileReader(fp);
    output->start(reader->sample_rate());
    // read until theres no more samples
    while (true) {
        int samples_read = reader->read(samples, 1024);
        if (samples_read == 0) {
            break;
        }
        output->write(samples, samples_read);
        // Serial.println("Played samples");
    }
    // stop the input
    output->stop();
    fclose(fp);
    delete reader;
    free(samples);
    // Serial.println("Finished playing");
}

void main_task(void *param) {
    Serial.println("Starting up");

    Serial.println("Mounting SPIFFS on /sdcard");
    SPIFFS.begin(true, "/sdcard");

    Output *output = new DACOutput(I2S_NUM_0);

    while (true) {
        // Checking how long a button was pressed to prevent false alarms:
        int blue_tit_button_touch = touchRead(BLUE_TIT_BUTTON);
        int red_kite_button_touch = touchRead(RED_KITE_BUTTON);
        int common_buzzard_button_touch = touchRead(COMMON_BUZZARD_BUTTON);
        int european_robin_button_touch = touchRead(EUROPEAN_ROBIN_BUTTON);

        if (!play_blue_tit_sound) {
            if (blue_tit_button_touch < threshold) {
                if (blue_tit_button_touch_time >= touch_time_min) {
                    play_blue_tit_sound = true;
                    blue_tit_button_touch_time = 0;
                } else {
                    blue_tit_button_touch_time++;
                }
            } else {
                blue_tit_button_touch_time = 0;
            }
        }
        if (!play_red_kite_sound) {
            if (red_kite_button_touch < threshold) {
                if (red_kite_button_touch_time >= touch_time_min) {
                    play_red_kite_sound = true;
                    red_kite_button_touch_time = 0;
                } else {
                    red_kite_button_touch_time++;
                }
            } else {
                red_kite_button_touch_time = 0;
            }
        }
        if (!play_common_buzzard_sound) {
            if (common_buzzard_button_touch < threshold) {
                if (common_buzzard_button_touch_time >= touch_time_min) {
                    play_common_buzzard_sound = true;
                    common_buzzard_button_touch_time = 0;
                } else {
                    common_buzzard_button_touch_time++;
                }
            } else {
                common_buzzard_button_touch_time = 0;
            }
        }
        if (!play_european_robin_sound) {
            if (european_robin_button_touch < threshold) {
                if (european_robin_button_touch_time >= touch_time_min) {
                    play_european_robin_sound = true;
                    european_robin_button_touch_time = 0;
                } else {
                    european_robin_button_touch_time++;
                }
            } else {
                european_robin_button_touch_time = 0;
            }
        }

        if (play_blue_tit_sound) {
            play_blue_tit_sound = false;
            Serial.println("Playing Blue Tit sound");
            play(output, "/sdcard/Blaumeise.wav");
            go_to_sleep = 10000;
        } else if (play_red_kite_sound) {
            play_red_kite_sound = false;
            Serial.println("Playing Red Kite sound");
            play(output, "/sdcard/Rotmilan.wav");
            go_to_sleep = 10000;
        } else if (play_common_buzzard_sound) {
            play_common_buzzard_sound = false;
            Serial.println("Playing Common Buzzard sound");
            play(output, "/sdcard/Maeusebussard.wav");
            go_to_sleep = 10000;
        } else if (play_european_robin_sound) {
            play_european_robin_sound = false;
            Serial.println("Playing European Robin sound");
            play(output, "/sdcard/Rotkehlchen.wav");
            go_to_sleep = 10000;
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}


void displayHighScore() {
    if (highscore > 99) {
        highscore = 99;
    }
    int first_digit = highscore / 10;
    int second_digit = highscore % 10;

    digitalWrite(bcdPinA, num[first_digit][0]);
    digitalWrite(bcdPinB, num[first_digit][1]);
    digitalWrite(bcdPinC, num[first_digit][2]);
    digitalWrite(bcdPinD, num[first_digit][3]);

    digitalWrite(bcdPinE, num[second_digit][0]);
    digitalWrite(bcdPinF, num[second_digit][1]);
    digitalWrite(bcdPinG, num[second_digit][2]);
    digitalWrite(bcdPinH, num[second_digit][3]);
}

void touchCallbackFunc() {}
void callback() {}

typedef struct struct_message {
    int a;
} struct_message;
struct_message espNowData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    memcpy(&espNowData, incomingData, sizeof(espNowData));
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("Int: ");
    Serial.println(espNowData.a);
}

void setup() {
    Serial.begin(115200);
    // Bird
    xTaskCreate(main_task, "Main", 4096, NULL, 0, NULL);

    // set pins to output
    pinMode(bcdPinA, OUTPUT);
    pinMode(bcdPinB, OUTPUT);
    pinMode(bcdPinC, OUTPUT);
    pinMode(bcdPinD, OUTPUT);

    pinMode(bcdPinE, OUTPUT);
    pinMode(bcdPinF, OUTPUT);
    pinMode(bcdPinG, OUTPUT);
    pinMode(bcdPinH, OUTPUT);

    pinMode(LED_PIN, OUTPUT);

    touchAttachInterrupt(BLUE_TIT_BUTTON, callback, 40);
    touchAttachInterrupt(RED_KITE_BUTTON, callback, 40);
    touchAttachInterrupt(COMMON_BUZZARD_BUTTON, callback, 40);
    touchAttachInterrupt(EUROPEAN_ROBIN_BUTTON, callback, 40);
    esp_sleep_enable_touchpad_wakeup();

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);

    delay(1000);
}

void loop() {
    digitalWrite(LED_PIN, HIGH);
    displayHighScore();
    delay(1000);
    go_to_sleep--;
    if (go_to_sleep < 1) {
        esp_deep_sleep_start();
    }
}
