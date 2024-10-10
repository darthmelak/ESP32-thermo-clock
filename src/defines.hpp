#ifndef defines_hpp
#define defines_hpp

#include <Arduino.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C
#define NTP_SERVER "europe.pool.ntp.org"
#define CLOCK_TIMEZONE "CET-1CEST,M3.5.0,M10.5.0/3"

#define RANDOM_SEED_PIN 36
#define PIR_PIN 34
#define SERVO_PIN 25
#define SERVO_SW_PIN 26

struct DisplayOffsets {
    int line_1_x = 0;
    int clock_x = 0;
    int clock_y = 0;

    void randomize() {
        line_1_x = random(0, 60);
        clock_x = random(0, 32);
        clock_y = random(0, 32);
    }
};

#endif
