/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

//
// StampFly Flight Control Main Module
//
// Desigend by Kouhei Ito 2023~2024
//
// 2024-08-11 StampFly 自己開発用のスケルトンプログラム制作開始
#include "main_loop.hpp"
#include "motor.hpp"
#include "rc.hpp"
#include "pid.hpp"
#include "sensor.hpp"
#include "led.hpp"
#include "telemetry.hpp"
#include "button.hpp"
#include "buzzer.h"
#include "stampfly.hpp"

// 割り込み関数
// Intrupt function
hw_timer_t* timer = NULL;
void IRAM_ATTR onTimer() {
    StampFly.flag.loop = 1;
}

// Initialize StampFly
void init_copter(void) {
    // Initialize Mode
    StampFly.flag.mode = INIT_MODE;
    // Initialaze LED function
    led_init();
    // Initialize Serial communication
    USBSerial.begin(115200);
    delay(1500);
    USBSerial.printf("Start StampFly! Skeleton\r\n");

    motor_init();
    sensor_init();
    rc_init();

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 2500, true);
    timerAlarmEnable(timer);
    // init button G0
    init_button();
    setup_pwm_buzzer();
    USBSerial.printf("Finish StampFly init!\r\n");
    start_tone();
}

// Main loop
void loop_400Hz(void) {
    uint32_t now_time;

    // 割り込みにより400Hzで以降のコードが実行
    while (StampFly.flag.loop == 0);
    StampFly.flag.loop = 0;

    now_time = micros();
    StampFly.times.old_elapsed_time = StampFly.times.elapsed_time;
    StampFly.times.elapsed_time = 1e-6 * (now_time - StampFly.times.start_time);
    StampFly.times.interval_time = StampFly.times.interval_time - StampFly.times.old_elapsed_time;
    
    // Read Sensor Value
    sensor_read(&StampFly.sensor);
    // LED Drive
    led_drive();

    // Begin Mode select
    if (StampFly.flag.mode == INIT_MODE) {
        motor_stop();
        StampFly.counter.offset = 0;
        StampFly.flag.mode = AVERAGE_MODE;
        return;

    } else if (StampFly.flag.mode == AVERAGE_MODE) {
        // Gyro offset Estimate
        if (StampFly.counter.offset < AVERAGENUM) {
            sensor_calc_offset_avarage();
            StampFly.counter.offset++;
            return;
        }
        // Mode change
        StampFly.flag.mode   = PARKING_MODE;
        StampFly.times.start_time = micros();
        return;
    } else if (StampFly.flag.mode == PARKING_MODE) {
        //ここに自分のコードを書く
        StampFly.counter.loop++;
    }

    //// Telemetry
    telemetry();
    StampFly.flag.oldmode = StampFly.flag.mode;  // Memory now mode
    // End of Loop_400Hz function
}
