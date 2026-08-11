#include <Arduino.h>
#include "Motor.h" // TMC5160 stepper driver library
#include <AS5048A.h>
#include <ESP32CAN.h>
#include "ws2812.h"
#include <EEPROM.h>
#include <PID_v1.h>

// micro-ROS Main Libraries
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Standard ROS 2 Message Types
#include <sensor_msgs/msg/joint_state.h> 
#include <geometry_msgs/msg/vector3.h> // for publishing Fx, Fy, Fz from the force sensor with 3 axes
#include <std_msgs/msg/u_int8.h>

// Error handling macros for micro-ROS functions with motor stop on error
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ tmc.set_velocity(0); while(1){ delay(100); }}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define PI 3.1415926535897932384626433832795
#define PIN_CS_TMC 15
#define PIN_EN_TMC 16
#define PIN_CS_INT_AMS 17
#define PIN_CS_EXT_AMS 18
#define PIN_NEOPIXEL 32
#define PIN_JOINT_ID_BIT3 33
#define PIN_JOINT_ID_BIT2 25
#define PIN_JOINT_ID_BIT1 26
#define PIN_JOINT_ID_BIT0 27
#define EEPROM_SIZE 64
#define EEPROM_ADDRESS_ENABLE_STARTUP 1    // If 1, homing is enabled at startup
#define EEPROM_ADDRESS_AS5048_OFFSET 10    // Magnet Offset saved at Addresses 10-13 as long
#define TIME_MS_TIMEOUT_NULLPUNKTFART 7500 // Maximum amount of time per axis for homing sequence
#define TIME_MS_PUBLISH_FREQUENCY 19       // ~50Hz publishing rate
#define TIME_MS_TIMEOUT_STOP 300           // Failsafe: timeout to switch off motors if PC disconnects

int g_this_joint = 1;
// Transmission array kept for internal master homing sequence reference
float g_motor_transmission[6] = {3.5, -84.48, -37.77, -4.75, -3, 1};
int g_as_sign[6] = {1, 1, -1, 1, 1, 1};
boolean g_high_motor_current[6] = {1, 1, 1, 0, 1, 0};
int g_enable_Nullpunktfahrt = 0;

long g_target_velocities[6];
long g_actual_velocities[6];
long g_actual_angles[6];
float g_target_velocities_rad_s[6]; // Storage array for target velocities in rad/s from MoveIt
int g_state = 0; 
long g_ms_time_differenze_publish = 0;
uint8_t g_led_rgb[3] = {1, 1, 1};
boolean g_tx_frame_master2slave_send = false; 
int g_init_order[6] = {2, 3, 4, 5, 1, 6};
uint8_t g_ma2sl_led_red = 0;
boolean g_first_command_received = false; // Interlock safety latch to prevent startup spikes before initialization
unsigned long g_last_command_time = 0;    // Global timestamp tracking for safety watchdog

// Joint Name Strings matching URDF exactly (q1 - q6)
const char* joint_names_mapping[6] = {"q1", "q2", "q3", "q4", "q5", "q6"};

// CAN Communication Structures
typedef union { 
  struct {
    long angle;
    long velocity;
  };
  uint8_t data[8];
} convert_ll2c;
convert_ll2c g_slave2master;

typedef union { 
  struct {
    uint8_t operation_ident; // 0 = off, 1 = normal operation, 2 = Do homing
    uint8_t reserved;
    uint8_t led_green;
    uint8_t led_blue;
    long target_velocity; // Keeps standard architecture definition for slaves compatibility
  };
  uint8_t data[8];
} convert_iiiil2c;
convert_iiiil2c g_master2slave;

typedef union { 
  struct {
    uint8_t operation; // 0: idle, 1: save offset, 2: start calibration...
    uint8_t answer_to_this;
    uint16_t current_offset_value;
    uint16_t tobesaved_offset_value;
    uint16_t raw_magnet_angle;
  };
  uint8_t data[8];
} convert_config;
convert_config g_platine2config;

typedef union { 
  struct {
    int16_t Fx;
    int16_t Fy;
    int16_t Fz;
  };
  uint8_t data[8];
} convert_f2c;
convert_f2c g_meas2master;

CAN_device_t CAN_cfg;

// Hardware Entities
Motor tmc = Motor(PIN_CS_TMC, PIN_EN_TMC, 10000, 10000, false);
AS5048A obj_angleSensor(PIN_CS_EXT_AMS);
rgbVal *obj_pixels;
double g_Input_obj_init_PID, g_Output_obj_init_PID, g_Setpoint_obj_init_PID;
PID obj_init_PID(&g_Input_obj_init_PID, &g_Output_obj_init_PID, &g_Setpoint_obj_init_PID, 30, 0.05, 0, P_ON_E, DIRECT);

// micro-ROS Infrastructure Entities
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;

// Publishers
rcl_publisher_t pub_joint_states; // Single Unified Publisher required by Robot State Broadcaster
rcl_publisher_t pub_meas;         // Publishes Fx, Fy, Fz from the force sensor with 3 axes

// Subscribers
rcl_subscription_t sub_joint_commands; // Intercepts velocity/position profiles pushed by MoveIt
rcl_subscription_t sub_ledblue;
rcl_subscription_t sub_reserved;
rcl_subscription_t sub_ledgreen;

// Message Instances
sensor_msgs__msg__JointState msg_pub_joint_states;   // Outgoing telemetry to RViz
sensor_msgs__msg__JointState msg_sub_joint_commands; // Incoming references from MoveIt
geometry_msgs__msg__Vector3 msg_force;               // Fx, Fy, Fz from the force sensor with 3 axes
std_msgs__msg__UInt8 msg_sub_ledblue;
std_msgs__msg__UInt8 msg_sub_reserved;
std_msgs__msg__UInt8 msg_sub_ledgreen;

// Global Static Buffers for JointState memory mapping (Input and Output)
double g_pub_joint_pos[6];
double g_pub_joint_vel[6];
double g_pub_joint_eff[6];
rosidl_runtime_c__String g_pub_joint_names[6];
char g_pub_name_strings[6][20];

double g_sub_joint_pos[6];
double g_sub_joint_vel[6];
double g_sub_joint_eff[6];
rosidl_runtime_c__String g_sub_joint_names[6];
char g_sub_name_strings[6][20];

// Function Prototypes
void this_axis_do_Nullpunktfahrt(boolean l_thisjointisatgoal, long l_starttime);

// --- micro-ROS Input Callbacks ---
void sub_joint_commands_callback(const void * msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  
  // Quick defensive check against uninitialized packages
  if (msg->velocity.size == 0) return;
  
  g_first_command_received = true;
  g_last_command_time = millis(); // Refresh watchdog timer on every incoming command
  
  // Fast asynchronous memory copy to prevent incoming serial port buffer overflow
  int limit = (msg->velocity.size < 6) ? msg->velocity.size : 6;
  for (int i = 0; i < limit; i++) {
    g_target_velocities_rad_s[i] = msg->velocity.data[i];
  }

  // Ping micro-ROS Agent to ensure connection is still alive
  RCSOFTCHECK(rmw_uros_ping_agent(10, 1));
}

void sub_blue_callback(const void * msgin) {
  const std_msgs__msg__UInt8 * led_msg = (const std_msgs__msg__UInt8 *)msgin;
  g_master2slave.led_blue = led_msg->data;
}

void sub_green_callback(const void * msgin) {
  const std_msgs__msg__UInt8 * led_msg = (const std_msgs__msg__UInt8 *)msgin;
  g_master2slave.led_green = led_msg->data;
}

void sub_reserved_callback(const void * msgin) {
  const std_msgs__msg__UInt8 * res_msg = (const std_msgs__msg__UInt8 *)msgin;
  g_master2slave.reserved = res_msg->data;
}

void setup() {
  tmc.disable_motor();
  pinMode(PIN_CS_INT_AMS, OUTPUT);
  pinMode(PIN_CS_EXT_AMS, OUTPUT);
  pinMode(PIN_JOINT_ID_BIT3, INPUT_PULLUP);
  pinMode(PIN_JOINT_ID_BIT2, INPUT_PULLUP);
  pinMode(PIN_JOINT_ID_BIT1, INPUT_PULLUP);
  pinMode(PIN_JOINT_ID_BIT0, INPUT_PULLUP);
  digitalWrite(PIN_CS_INT_AMS, HIGH);
  digitalWrite(PIN_CS_EXT_AMS, HIGH);
  delay(1);

  g_this_joint = !digitalRead(PIN_JOINT_ID_BIT0) * 1 + !digitalRead(PIN_JOINT_ID_BIT1) * 2 + !digitalRead(PIN_JOINT_ID_BIT2) * 4 + !digitalRead(PIN_JOINT_ID_BIT3) * 8;

  ws2812_init(PIN_NEOPIXEL, LED_WS2812B);
  obj_pixels = (rgbVal *)malloc(sizeof(rgbVal) * 1); 
  obj_pixels[0] = makeRGBVal(0, 0, 0);               
  ws2812_setColors(1, obj_pixels);

  if (g_this_joint != 1) {
    while (true) {
      obj_pixels[0] = makeRGBVal(255, 0, 0); ws2812_setColors(1, obj_pixels); delay(200);
      obj_pixels[0] = makeRGBVal(0, 0, 0);   ws2812_setColors(1, obj_pixels); delay(2000);
    }
  }

  EEPROM.begin(EEPROM_SIZE);
  g_enable_Nullpunktfahrt = EEPROM.read(EEPROM_ADDRESS_ENABLE_STARTUP);
  uint16_t l_ASoffset = EEPROM.readShort(EEPROM_ADDRESS_AS5048_OFFSET);

  if (g_enable_Nullpunktfahrt != 0 && g_enable_Nullpunktfahrt != 1) {
    g_enable_Nullpunktfahrt = 0;
    EEPROM.write(EEPROM_ADDRESS_ENABLE_STARTUP, 0);
    EEPROM.commit();
  }
  if (l_ASoffset > 16384) {
    l_ASoffset = 0;
    EEPROM.writeShort(EEPROM_ADDRESS_AS5048_OFFSET, 0);
    EEPROM.commit();
  }

  CAN_cfg.speed = CAN_SPEED_200KBPS;
  CAN_cfg.tx_pin_id = GPIO_NUM_5;
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(100, sizeof(CAN_frame_t));
  ESP32Can.CANInit();

  for (int i = 0; i < g_this_joint; i++) {
    obj_pixels[0] = makeRGBVal(0, 0, 255); ws2812_setColors(1, obj_pixels); delay(200);
    obj_pixels[0] = makeRGBVal(0, 0, 0);   ws2812_setColors(1, obj_pixels); delay(200);
  }
  tmc.enable_motor();
  tmc.set_current(g_high_motor_current[g_this_joint - 1]);
  for (int i = g_this_joint; i < 6; i++) {
    delay(400);
  }
  obj_pixels[0] = makeRGBVal(255, 0, 0);
  ws2812_setColors(1, obj_pixels);

  obj_angleSensor.init();
  obj_angleSensor.setZeroPosition(l_ASoffset);
  
  boolean l_boolean_isnotzero = false;
  for(int i = 0; i < 20; i++) { 
    if(obj_angleSensor.getRawRotation() != 0) l_boolean_isnotzero = true;
  }
  if(!l_boolean_isnotzero) { 
    while (true) {
      obj_pixels[0] = makeRGBVal(255, 0, 0); ws2812_setColors(1, obj_pixels); delay(200);
      obj_pixels[0] = makeRGBVal(0, 0, 0);   ws2812_setColors(1, obj_pixels); delay(200);
      obj_pixels[0] = makeRGBVal(255, 0, 0); ws2812_setColors(1, obj_pixels); delay(200);
      obj_pixels[0] = makeRGBVal(0, 0, 0);   ws2812_setColors(1, obj_pixels); delay(2000);
    }
  }

  delay(1500); 

  if (g_enable_Nullpunktfahrt == 1) {
    obj_pixels[0] = makeRGBVal(0, 255, 0); 
  } else {
    obj_pixels[0] = makeRGBVal(0, 0, 255); 
  }
  ws2812_setColors(1, obj_pixels);
  delay(250);
  obj_pixels[0] = makeRGBVal(0, 0, 0);
  ws2812_setColors(1, obj_pixels);
  g_Setpoint_obj_init_PID = 0;

  if (g_enable_Nullpunktfahrt == 1) {
    boolean l_finished = false;
    int l_nullpunktfahrt_jointid = 1;
    while (!l_finished) { 
      int l_actualjoint = g_init_order[l_nullpunktfahrt_jointid - 1];
      if (l_actualjoint == g_this_joint) {
        boolean l_thisjointisatgoal = false;
        long l_starttime = millis();
        this_axis_do_Nullpunktfahrt(l_thisjointisatgoal, l_starttime);
      } 
      else if (l_actualjoint > 1 && l_actualjoint < 7) {
        g_master2slave.operation_ident = 2; 
        g_master2slave.target_velocity = 0;
        CAN_frame_t tx_frame;
        tx_frame.FIR.B.FF = CAN_frame_std;
        tx_frame.MsgID = l_actualjoint;
        tx_frame.FIR.B.DLC = 8;
        memcpy(tx_frame.data.u8, g_master2slave.data, 8);
        ESP32Can.CANWriteFrame(&tx_frame);
        
        xQueueReset(CAN_cfg.rx_queue); 
        boolean l_thisjointisatgoal = false;
        long l_starttime = millis(); 
        while (!l_thisjointisatgoal && millis() - l_starttime < TIME_MS_TIMEOUT_NULLPUNKTFART) {
          
          CAN_frame_t rx_frame;
          while (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
            if (rx_frame.FIR.B.RTR != CAN_RTR && rx_frame.MsgID == (10 + l_actualjoint)) {
              l_thisjointisatgoal = true; 
            }
          }
        }
      }
      l_nullpunktfahrt_jointid++;
      if (l_nullpunktfahrt_jointid > 6) l_finished = true;
    }
  }

  // --- micro-ROS Initialization ---
  Serial.begin(460800); 
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_INFO);
  set_microros_serial_transports(Serial); 

  allocator = rcl_get_default_allocator();

  // --- Visual Feedback for micro-ROS Agent Connection ---
  obj_pixels[0] = makeRGBVal(255, 100, 0);
  ws2812_setColors(1, obj_pixels);

  // --- Wait for micro-ROS Agent to be available ---
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
    obj_pixels[0] = makeRGBVal(255, 100, 0); ws2812_setColors(1, obj_pixels); delay(250);
    obj_pixels[0] = makeRGBVal(0, 0, 0);     ws2812_setColors(1, obj_pixels); delay(250);
  }

  // --- micro-ROS Agent Connected ---
  obj_pixels[0] = makeRGBVal(0, 255, 0); // green LED to indicate successful connection
  ws2812_setColors(1, obj_pixels);
  delay(500);

  // -- micro-ROS Node and Executor Initialization ---
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "helene_esp_master", "", &support));

  // --- Dynamic Memory Allocation Mapping for Outgoing JointState (Publish) ---
  msg_pub_joint_states.position.data = g_pub_joint_pos;
  msg_pub_joint_states.position.size = 6;
  msg_pub_joint_states.position.capacity = 6;

  msg_pub_joint_states.velocity.data = g_pub_joint_vel;
  msg_pub_joint_states.velocity.size = 6;
  msg_pub_joint_states.velocity.capacity = 6;

  msg_pub_joint_states.effort.data = g_pub_joint_eff;
  msg_pub_joint_states.effort.size = 6;
  msg_pub_joint_states.effort.capacity = 6;

  msg_pub_joint_states.name.data = g_pub_joint_names;
  msg_pub_joint_states.name.size = 6;
  msg_pub_joint_states.name.capacity = 6;

  for(int i = 0; i < 6; i++) {
    strcpy(g_pub_name_strings[i], joint_names_mapping[i]);
    msg_pub_joint_states.name.data[i].data = g_pub_name_strings[i];
    msg_pub_joint_states.name.data[i].size = strlen(g_pub_name_strings[i]);
    msg_pub_joint_states.name.data[i].capacity = 20;
  }

  // --- Dynamic Memory Allocation Mapping for Incoming JointState (Subscribe) ---
  msg_sub_joint_commands.position.data = g_sub_joint_pos;
  msg_sub_joint_commands.position.size = 0;
  msg_sub_joint_commands.position.capacity = 6;

  msg_sub_joint_commands.velocity.data = g_sub_joint_vel;
  msg_sub_joint_commands.velocity.size = 0;
  msg_sub_joint_commands.velocity.capacity = 6;

  msg_sub_joint_commands.effort.data = g_sub_joint_eff;
  msg_sub_joint_commands.effort.size = 0;
  msg_sub_joint_commands.effort.capacity = 6;

  msg_sub_joint_commands.name.data = g_sub_joint_names;
  msg_sub_joint_commands.name.size = 0;
  msg_sub_joint_commands.name.capacity = 6;
  
  for(int i = 0; i < 6; i++) {
    msg_sub_joint_commands.name.data[i].data = g_sub_name_strings[i];
    msg_sub_joint_commands.name.data[i].size = 0;
    msg_sub_joint_commands.name.data[i].capacity = 20;
  }

  RCCHECK(rclc_publisher_init_default(&pub_joint_states, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "esp_joint_states"));
  RCCHECK(rclc_publisher_init_default(&pub_meas, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "raw_meas_vector"));

  RCCHECK(rclc_subscription_init_default(&sub_joint_commands, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "helene_trajectory_controller/joint_commands"));
  RCCHECK(rclc_subscription_init_default(&sub_ledblue, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "helene_led_blue"));
  RCCHECK(rclc_subscription_init_default(&sub_ledgreen, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "helene_led_green"));
  RCCHECK(rclc_subscription_init_default(&sub_reserved, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "helene_reserved"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_joint_commands, &msg_sub_joint_commands, &sub_joint_commands_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_ledblue, &msg_sub_ledblue, &sub_blue_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_ledgreen, &msg_sub_ledgreen, &sub_green_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_reserved, &msg_sub_reserved, &sub_reserved_callback, ON_NEW_DATA));

  // Synchronize micro-ROS internal clock with the PC Agent node
  const int timeout_ms = 1000;
  rmw_uros_sync_session(timeout_ms);

  obj_init_PID.SetMode(AUTOMATIC);
  obj_init_PID.SetOutputLimits(-15000, 15000); 
  
  g_Setpoint_obj_init_PID = (double)(obj_angleSensor.getRotation() * g_as_sign[g_this_joint - 1]);
  g_last_command_time = millis(); // Initialize watchdog timestamp
}

void loop() {

  CAN_frame_t rx_frame;

  // empty the CAN queue in a NON blocking way (timeout 0): all the frames in
  // arrival (angles/velocities of the slaves, configuration responses and
  // force sensor responses with 3 axes) are handled here, in a single cycle,
  // so as not to block the main loop.
  while (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 0) == pdTRUE) {
    if (rx_frame.FIR.B.RTR != CAN_RTR && rx_frame.MsgID >= 10 && rx_frame.MsgID <= 19) {
      memcpy(g_slave2master.data, rx_frame.data.u8, 8);
      g_actual_angles[rx_frame.MsgID - 11] = g_slave2master.angle;
      g_actual_velocities[rx_frame.MsgID - 11] = g_slave2master.velocity;
    } 
    else if (rx_frame.FIR.B.RTR != CAN_RTR && rx_frame.MsgID == 20 + g_this_joint) {
      memcpy(g_platine2config.data, rx_frame.data.u8, 8);
      boolean l_thisjointisatgoal = false;
      long l_starttime = millis();
      
      switch (g_platine2config.operation) {
        case 1:
          EEPROM.writeShort(EEPROM_ADDRESS_AS5048_OFFSET, g_platine2config.tobesaved_offset_value);
          EEPROM.commit();
          obj_angleSensor.setZeroPosition(g_platine2config.tobesaved_offset_value);
          break;
        case 2: tmc.disable_motor(); break;
        case 3: tmc.enable_motor(); break;
        case 4:
          g_enable_Nullpunktfahrt = 1;
          EEPROM.write(EEPROM_ADDRESS_ENABLE_STARTUP, 1);
          EEPROM.commit();
          break;
        case 5:
          g_enable_Nullpunktfahrt = 0;
          EEPROM.write(EEPROM_ADDRESS_ENABLE_STARTUP, 0);
          EEPROM.commit();
          break;
        case 7:
          this_axis_do_Nullpunktfahrt(l_thisjointisatgoal, l_starttime);
          break;
        default: break;
      }
      
      if (g_platine2config.answer_to_this == 1) {
        CAN_frame_t l_tx_frame;
        g_platine2config.current_offset_value = EEPROM.readShort(EEPROM_ADDRESS_AS5048_OFFSET);
        g_platine2config.raw_magnet_angle = (g_platine2config.operation == 6) ? obj_angleSensor.getRotation() : obj_angleSensor.getRawRotation();
        
        memcpy(l_tx_frame.data.u8, g_platine2config.data, 8);
        l_tx_frame.FIR.B.FF = CAN_frame_std;
        l_tx_frame.MsgID = 27;
        l_tx_frame.FIR.B.DLC = 8;
        ESP32Can.CANWriteFrame(&l_tx_frame);
      }
    }
    // Response from the force sensor with 3 axes to the request sent below
    if (rx_frame.FIR.B.RTR != CAN_RTR && rx_frame.MsgID == 0x20) {
      memcpy(g_meas2master.data, rx_frame.data.u8, 6);
    }
  }

  // --- Unified Publishing & Actuation Execution Block (~50Hz) ---
  if (millis() - g_ms_time_differenze_publish >= TIME_MS_PUBLISH_FREQUENCY) {
    g_ms_time_differenze_publish = millis();

    // 0. Request a new sample from the force sensor with 3 axes (non-blocking).
    //    the answer (MsgID 0x20) is collected in the next iteration by the cycle
    //    xQueueReceive above, together with all other CAN frames.
    CAN_frame_t tx_frame_force;
    tx_frame_force.FIR.B.FF = CAN_frame_std;
    tx_frame_force.MsgID = 0x20;
    tx_frame_force.FIR.B.DLC = 1;
    tx_frame_force.data.u8[0] = 0;
    ESP32Can.CANWriteFrame(&tx_frame_force);

    // Check if the PC has disconnected or stopped sending commands (Failsafe Watchdog)
    if (g_first_command_received && (millis() - g_last_command_time > TIME_MS_TIMEOUT_STOP)) {
      // Emergency Stop: Shutdown the local motor immediately
      tmc.set_velocity(0);
      
      // Notify all downstream CAN Slaves to shut down or zero their velocities
      g_master2slave.operation_ident = 0; // 0 = Turn off/Safe mode
      g_master2slave.target_velocity = 0;
      
      CAN_frame_t stop_frame;
      for (int joint = 1; joint < 6; joint++) {
        stop_frame.FIR.B.FF = CAN_frame_std;
        stop_frame.MsgID = joint + 1;
        stop_frame.FIR.B.DLC = 8;
        memcpy(stop_frame.data.u8, g_master2slave.data, 8);
        ESP32Can.CANWriteFrame(&stop_frame);
      }
      
      // Update telemetry LEDs to signal a Watchdog Trip (Solid Red)
      g_master2slave.led_green = 0;
      g_master2slave.led_blue = 0;
      g_ma2sl_led_red = 255;
    }
    // Normal operation: commands are fresh and active
    else if (g_first_command_received) {
      g_master2slave.operation_ident = 1; // CRITICAL FIX: Explicitly set to 1 for Normal Operation Mode!
      g_ma2sl_led_red = 0;                // Clear error led color
      
      CAN_frame_t tx_frame;
      for (int joint = 1; joint < 6; joint++) {
        tx_frame.FIR.B.FF = CAN_frame_std;
        tx_frame.MsgID = joint + 1;
        tx_frame.FIR.B.DLC = 8;
        
        long target_speed_ticks = (g_target_velocities_rad_s[joint] / (2.0 * PI)) * 16384.0;
        g_master2slave.target_velocity = target_speed_ticks;
        memcpy(tx_frame.data.u8, g_master2slave.data, 8);
        ESP32Can.CANWriteFrame(&tx_frame);
      }    

      // Actuate Local Driver (Joint 1 Master) in velocity mode
      long master_target_speed_ticks = (g_target_velocities_rad_s[0] / (2.0 * PI)) * 16384.0;
      long max_safe_ticks = 2000; 
      if (master_target_speed_ticks > max_safe_ticks) master_target_speed_ticks = max_safe_ticks;
      if (master_target_speed_ticks < -max_safe_ticks) master_target_speed_ticks = -max_safe_ticks;
      
      tmc.set_velocity(master_target_speed_ticks * g_motor_transmission[0]);
    }
  
    // 1. Dispatch the 3 components of the force sensor (Fx, Fy, Fz)
    msg_force.x = (double)g_meas2master.Fx;
    msg_force.y = (double)g_meas2master.Fy;
    msg_force.z = (double)g_meas2master.Fz;
    RCSOFTCHECK(rcl_publish(&pub_meas, &msg_force, NULL));
    
    // 2. Sync Epoch System Timestamp from Agent to prevent RViz model flickering/jitters
    int64_t time_ns = rmw_uros_epoch_nanos();
    if (time_ns > 0) {
      msg_pub_joint_states.header.stamp.sec = time_ns / 1000000000LL;
      msg_pub_joint_states.header.stamp.nanosec = time_ns % 1000000000LL;
    } else {
      msg_pub_joint_states.header.stamp.sec = millis() / 1000;
      msg_pub_joint_states.header.stamp.nanosec = (millis() % 1000) * 1000000;
    }
    
    // 3. Map multi-joint data profiles into standard physical units (Radians)
    g_actual_angles[0] = obj_angleSensor.getRotation() * g_as_sign[g_this_joint - 1];
    
    for(int i = 0; i < 6; i++) {
      msg_pub_joint_states.position.data[i] = ((double)g_actual_angles[i] / 16384.0) * (2.0 * PI);
      
      if(i == 0) {
        msg_pub_joint_states.velocity.data[i] = ((double)tmc.get_vactual() / g_motor_transmission[0] / 16384.0) * (2.0 * PI);
      } else {
        msg_pub_joint_states.velocity.data[i] = ((double)g_actual_velocities[i] / 16384.0) * (2.0 * PI);
      }
      msg_pub_joint_states.effort.data[i] = 0.0; 
    }
    
    // 4. Stream synchronized states downstream
    RCSOFTCHECK(rcl_publish(&pub_joint_states, &msg_pub_joint_states, NULL));
    
    // RGB Telemetry Status Update 
    if (g_led_rgb[0] != g_ma2sl_led_red || g_led_rgb[1] != g_master2slave.led_green || g_led_rgb[2] != g_master2slave.led_blue) {
      g_led_rgb[0] = g_ma2sl_led_red;
      g_led_rgb[1] = g_master2slave.led_green;
      g_led_rgb[2] = g_master2slave.led_blue;
      obj_pixels[0] = makeRGBVal(g_led_rgb[0], g_led_rgb[1], g_led_rgb[2]);
      ws2812_setColors(1, obj_pixels);
    }
  }

  // Spin executor at 1ms to prioritize serial message parsing reproductivity
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}

void this_axis_do_Nullpunktfahrt(boolean l_thisjointisatgoal, long l_starttime) {
  obj_init_PID.SetMode(AUTOMATIC);
  obj_init_PID.SetOutputLimits(-5000, 5000);
  g_Setpoint_obj_init_PID = 8192;
  
  while (!l_thisjointisatgoal && millis() - l_starttime < TIME_MS_TIMEOUT_NULLPUNKTFART) {
    obj_pixels[0] = makeRGBVal(0, 125 + 125 * sin(millis() / 50), 0); 
    ws2812_setColors(1, obj_pixels);
    
    g_Input_obj_init_PID = double((obj_angleSensor.getRotation() * g_as_sign[g_this_joint - 1] + 8192));
    obj_init_PID.Compute();
    tmc.set_velocity(long(g_Output_obj_init_PID * g_motor_transmission[g_this_joint - 1]));
    
    if (abs(obj_angleSensor.getRotation()) < 5 && abs(tmc.get_vtarget()) < abs(25 * g_motor_transmission[g_this_joint - 1])) {
      l_thisjointisatgoal = true; 
    }
    delay(10);
  }
  tmc.set_velocity(0);
  obj_pixels[0] = makeRGBVal(0, 0, 0);
  ws2812_setColors(1, obj_pixels);
}