// ============================================================
//  Helene Roboterarm – micro-ROS Edition  (v2)
//  Original code: FG MuST, TU Darmstadt
//
//  Changes compared to CAN version:
//    - CAN (ESP32CAN) + SCPI completely removed
//    - micro-ROS: 3 publishers (force_x/y/z) + 1 subscriber (cmd)
//    - OLED shows forces instead of CAN payload
//
//  Improvements v2:
//    [1] Serial separation: micro-ROS → Serial2 (TX=17, RX=16)
//                         Debug      → Serial  (USB, UART0)
//    [2] rmw_uros_ping_agent instead of delay(2000) + runtime reconnect
//    [3] Mutex for g_force_x/y/z between loop (Core 0)
//        and OLED task (Core 0, preemptive RTOS)
// ============================================================

// ---------- Includes -----------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include <ADS1220.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// micro-ROS
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>   // for rmw_uros_ping_agent
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/u_int8.h>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ---------- Defines ------------------------------------------
#define PIN_ADC1_DRDY   23
#define PIN_ADC1_CS     18
#define PIN_LED          2

// [1] Serial2 pins for micro-ROS transport (freely selectable)
//     Do not use GPIO 9/10 – they are connected to flash!
#define UROS_SERIAL_TX  17
#define UROS_SERIAL_RX  16
#define UROS_BAUDRATE   576000   // higher baudrate recommended for micro-ROS

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   32
#define OLED_RESET       4
#define SCREEN_ADDRESS  0x3C

#define TIME_MS_PUBLISH  19   // ~50 Hz

// micro-ROS helper macros
#define RCCHECK(fn)     { rcl_ret_t _rc = fn; if(_rc != RCL_RET_OK){ uros_error_handler(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t _rc = fn; (void)_rc; }

// ---------- Image (Helene) -----------------------------------
static const unsigned char PROGMEM logo_bmp[] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x03,0xc0,0x00,0x00,0x0f,0xf0,0x00,0x00,0x3f,0xf0,
    0x00,0x0e,0x67,0xf0,0x03,0xfc,0x17,0xf8,0x0f,0xb8,0x3f,0xf8,0x0c,0x1e,0xff,0xf8,
    0x0d,0xff,0xff,0xf8,0x0f,0xff,0xe0,0x30,0x07,0xff,0xe0,0x00,0x06,0xff,0xf0,0x00,
    0x10,0xc7,0xfc,0x00,0x1b,0x81,0xff,0x00,0x1f,0x00,0x77,0xc0,0x0b,0x00,0x03,0xe0,
    0x00,0x00,0x03,0xf0,0x00,0x00,0x03,0xf0,0x00,0x00,0x03,0xe0,0x00,0x00,0x0f,0xc0,
    0x00,0x07,0xff,0xc0,0x00,0x0f,0xff,0xe0,0x00,0x0f,0xff,0xf0,0x00,0x0f,0xff,0xf0,
    0x00,0x05,0xff,0xe0,0x00,0x0f,0xff,0xe0,0x00,0x0f,0xff,0xe0,0x00,0x0f,0xff,0xe0,
    0x00,0x0f,0xff,0xe0,0x00,0x0f,0xff,0xf0,0x00,0x0f,0xff,0xe0,0x00,0x00,0x00,0x00
};

static const char* g_names[] = {
    "Helene","FG: MuST","Sven Suppelt","Felix Herbst",
    "Romal Chadda","Jan Hinrichs","Dennis Roth",
    "Esan Sundaralingam","Eric Pohl","Philipp Witulla",
    "Prof. Kupnik","Technische","Universitaet","Darmstadt"
};

// ---------- Objekte ------------------------------------------
ADS1220          obj_ADC1;
Adafruit_SSD1306 obj_display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TaskHandle_t     Task_Oled_Hnd;

// ---------- micro-ROS Entitäten ------------------------------
rcl_node_t           node;
rclc_support_t       support;
rcl_allocator_t      allocator;
rclc_executor_t      executor;

rcl_publisher_t      pub_force_x;
rcl_publisher_t      pub_force_y;
rcl_publisher_t      pub_force_z;
rcl_subscription_t   sub_cmd;

std_msgs__msg__Float32 msg_force_x;
std_msgs__msg__Float32 msg_force_y;
std_msgs__msg__Float32 msg_force_z;
std_msgs__msg__UInt8   msg_cmd;

// ---------- Global variables --------------------------------
volatile bool g_adc_newdata = false;

// Raw values (written only by loop, no mutex needed)
float g_v_north = 0.0f, g_v_south = 0.0f;
float g_v_east  = 0.0f, g_v_west  = 0.0f;
int   g_current_mux = 0;

// [3] Forces: written by loop, read by OLED task → mutex
float g_force_x = 0.0f;
float g_force_y = 0.0f;
float g_force_z = 0.0f;
SemaphoreHandle_t g_force_mutex;

// Written by subscriber (executor runs in loop, Core 0 → no extra mutex needed)
uint8_t g_operation_ident = 0;

// [2] Connection state
bool g_uros_connected = false;

// ---------- Prototypen ---------------------------------------
bool uros_init();
void uros_destroy();

// ---------- Error handling ----------------------------------
// Called only on a hard initialization failure.
// In normal operation, connection loss is detected by ping.
void uros_error_handler()
{
    Serial.println("[uROS] Fehler – reinitializing...");
    g_uros_connected = false;
    // Kurz blinken zur Signalisierung
    for (int i = 0; i < 5; i++) {
        digitalWrite(PIN_LED, HIGH); delay(100);
        digitalWrite(PIN_LED, LOW);  delay(100);
    }
}

// ---------- ISR ----------------------------------------------
void IRAM_ATTR isr()
{
    g_adc_newdata = true;
}

// ---------- Subscriber-Callback ------------------------------
void cmd_subscription_callback(const void* msgin)
{
    const std_msgs__msg__UInt8* msg = (const std_msgs__msg__UInt8*)msgin;
    g_operation_ident = msg->data;
    Serial.printf("[CMD] operation_ident = %u\n", g_operation_ident);
}

// ---------- OLED Task (Core 0) -------------------------------
// [Improvement 3] Runs on Core 0 – same core as loop(),
// but RTOS is preemptive. Mutex protects read access to g_force_*.
void Task_Oled(void* parameter)
{
    long millis_timedisp = 0;
    for (;;) {
        if (millis() - millis_timedisp > 100) {
            millis_timedisp = millis();

            // Atomically copy forces
            float fx, fy, fz;
            if (xSemaphoreTake(g_force_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                fx = g_force_x;
                fy = g_force_y;
                fz = g_force_z;
                xSemaphoreGive(g_force_mutex);
            } else {
                // Mutex timeout: keep old value, no crash
                fx = fy = fz = 0.0f;
            }

            obj_display.clearDisplay();
            obj_display.setTextSize(1);
            obj_display.setTextColor(SSD1306_WHITE);

            obj_display.setCursor(0, 0);
            obj_display.print("Fx:"); obj_display.print(fx, 3);
            obj_display.setCursor(64, 0);
            obj_display.print("Fy:"); obj_display.print(fy, 3);

            obj_display.setCursor(0, 8);
            obj_display.print("Fz:"); obj_display.print(fz, 3);

            obj_display.setCursor(0, 16);
            obj_display.print("Op-ID: ");
            obj_display.print(g_operation_ident);

            obj_display.setCursor(0, 24);
            obj_display.print(g_uros_connected ? "uROS OK" : "uROS --");

            obj_display.display();
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // instead of yield() – cleaner under FreeRTOS
    }
}

// ---------- micro-ROS Init / Destroy -------------------------
// [2] Extracted so reconnect in loop is possible

bool uros_init()
{
    allocator = rcl_get_default_allocator();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
        return false;
    if (rclc_node_init_default(&node, "helene_force_node", "", &support) != RCL_RET_OK)
        return false;

    if (rclc_publisher_init_default(&pub_force_x, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "helene/force_x") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(&pub_force_y, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "helene/force_y") != RCL_RET_OK) return false;

    if (rclc_publisher_init_default(&pub_force_z, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "helene/force_z") != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(&sub_cmd, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
            "helene/cmd") != RCL_RET_OK) return false;

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK)
        return false;
    if (rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd,
            &cmd_subscription_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

    return true;
}

void uros_destroy()
{
    // Release resources for clean reconnect
    RCSOFTCHECK(rcl_publisher_fini(&pub_force_x, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_force_y, &node));
    RCSOFTCHECK(rcl_publisher_fini(&pub_force_z, &node));
    RCSOFTCHECK(rcl_subscription_fini(&sub_cmd, &node));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node));
    RCSOFTCHECK(rclc_support_fini(&support));
}

// =============================================================
//  SETUP
// =============================================================
void setup()
{
    // [1] Serial0 = debug only (USB)
    Serial.begin(115200);

    // Pins
    pinMode(PIN_ADC1_CS, OUTPUT);
    pinMode(PIN_LED,     OUTPUT);
    digitalWrite(PIN_LED,     HIGH);
    digitalWrite(PIN_ADC1_CS, HIGH);

    delay(500);

    // Start OLED
    if (!obj_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }

    // Intro animation
    int  l_namesize  = 0;
    int  l_totalsize = sizeof(g_names);
    Serial.println(F("Helene Roboterarm – micro-ROS v2"));
    while (l_totalsize > 0) {
        l_totalsize -= sizeof(g_names[l_namesize]);
        Serial.println(g_names[l_namesize]);
        l_namesize++;
    }

    boolean l_switchnames = esp_random() % 2;
    obj_display.clearDisplay();
    obj_display.setTextSize(1);
    obj_display.setTextColor(SSD1306_WHITE);

    for (int i = 0; i < l_namesize * 8 - 31; i++) {
        obj_display.clearDisplay();
        obj_display.setCursor(0, -i);
        for (int ii = 0; ii < l_namesize; ii++) {
            if (ii == 2 || ii == 3)
                obj_display.println(g_names[ii + l_switchnames * (5 - 2 * ii)]);
            else
                obj_display.println(g_names[ii]);
        }
        obj_display.drawBitmap(127 - 34, 0, logo_bmp, 32, 32, 1);
        obj_display.display();
        if (i == 0) delay(1000);
        delay(120);
    }
    delay(1000);

    // Configure ADC
    obj_ADC1.begin(PIN_ADC1_DRDY, PIN_ADC1_CS);
    obj_ADC1.setMUX(0);
    obj_ADC1.setGain(128);
    obj_ADC1.setOperatingMode('T');
    obj_ADC1.setAnalogReference('E');
    obj_ADC1.setDatarate(180);
    obj_ADC1.startContinuousMeas(true);
    delay(10);

    attachInterrupt(PIN_ADC1_DRDY, isr, FALLING);

    // [3] Create mutex
    g_force_mutex = xSemaphoreCreateMutex();
    configASSERT(g_force_mutex != NULL);

    // OLED task on Core 0 (same core as loop – preemptive RTOS)
    xTaskCreatePinnedToCore(
        Task_Oled, "Task_Oled", 10000, NULL, 1, &Task_Oled_Hnd, 0);

    // [1] Serial2 as micro-ROS transport
    //     Agent command on PC:
    //     ros2 run micro_ros_agent micro_ros_agent serial \
    //          --dev /dev/ttyUSB0 -b 576000
    Serial2.begin(UROS_BAUDRATE, SERIAL_8N1, UROS_SERIAL_RX, UROS_SERIAL_TX);
    set_microros_serial_transports(Serial2);

    // [2] Wait for agent response (LED blinks while waiting)
    Serial.println("[uROS] Warte auf Agent...");
    while (rmw_uros_ping_agent(200, 1) != RMW_RET_OK) {
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
        delay(100);
    }
    Serial.println("[uROS] Agent gefunden – initialisiere...");

    if (!uros_init()) {
        Serial.println("[uROS] Init failed!");
        uros_error_handler();
    } else {
        g_uros_connected = true;
        Serial.println("[uROS] Ready.");
    }

    digitalWrite(PIN_LED, LOW);
    disableCore0WDT();
}

// =============================================================
//  LOOP  (Core 0)
// =============================================================
long g_ms_last_publish = 0;

void loop()
{
    // [2] Verbindungsstatus prüfen – bei Verlust neu verbinden
    if (!g_uros_connected) {
        Serial.println("[uROS] Connection lost – waiting for agent...");
        uros_destroy();

        while (rmw_uros_ping_agent(200, 1) != RMW_RET_OK) {
            digitalWrite(PIN_LED, !digitalRead(PIN_LED));
            delay(100);
        }
        if (uros_init()) {
            g_uros_connected = true;
            digitalWrite(PIN_LED, LOW);
            Serial.println("[uROS] Reconnect successful.");
        }
        return; // Restart loop iteration
    }

    // 1) Process ADC data
    if (g_adc_newdata) {
        float raw_val = obj_ADC1.Read_Data();

        switch (g_current_mux) {
            case 0: g_v_north = raw_val; g_current_mux = 3; break;
            case 3: g_v_south = raw_val; g_current_mux = 5; break;
            case 5: g_v_east  = raw_val; g_current_mux = 1; break;
            case 1: g_v_west  = raw_val; g_current_mux = 0; break;
        }
        obj_ADC1.setMUX(g_current_mux);

        // [3] Write forces under mutex
        float fx = g_v_east  - g_v_west;
        float fy = g_v_north - g_v_south;
        float fz = (g_v_north + g_v_south + g_v_east + g_v_west) / 4.0f;

        if (xSemaphoreTake(g_force_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            g_force_x = fx;
            g_force_y = fy;
            g_force_z = fz;
            xSemaphoreGive(g_force_mutex);
        }

        // [1] Debug on Serial0 (USB) – no conflict with micro-ROS
        Serial.printf("Fx=%.3f Fy=%.3f Fz=%.3f\n", fx, fy, fz);

        g_adc_newdata = false;
    }

    // 2) Publish forces at ~50 Hz
    if (millis() - g_ms_last_publish >= TIME_MS_PUBLISH) {
        g_ms_last_publish = millis();

        // Read local copy under mutex
        if (xSemaphoreTake(g_force_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            msg_force_x.data = g_force_x;
            msg_force_y.data = g_force_y;
            msg_force_z.data = g_force_z;
            xSemaphoreGive(g_force_mutex);
        }

        if (rcl_publish(&pub_force_x, &msg_force_x, NULL) != RCL_RET_OK ||
            rcl_publish(&pub_force_y, &msg_force_y, NULL) != RCL_RET_OK ||
            rcl_publish(&pub_force_z, &msg_force_z, NULL) != RCL_RET_OK) {
            // Publish error → mark connection as lost
            g_uros_connected = false;
        }
    }

    // 3) Executor (subscriber callbacks)
    if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)) != RCL_RET_OK) {
        g_uros_connected = false;
    }
}