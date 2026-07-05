#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

// Oggetti micro-ROS
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2 // Il LED di bordo dell'ESP32 per feedback visivo

// Callback del timer: viene eseguita ogni secondo
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCL_UNUSED(last_call_time);
  if (timer != NULL) {
    // Incrementa il contatore e pubblica
    msg.data++;
    rcl_publish(&publisher, &msg, NULL);
    
    // Fai lampeggiare il LED ad ogni invio riuscito
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Configura il trasporto seriale di micro-ROS (Serial standard dell'ESP32)
  // Usiamo un baudrate alto (460800) ottimo per la robotica
  Serial.begin(460800); 
  set_microros_serial_transports(Serial);

  delay(2000); // Dai tempo alla seriale di stabilizzarsi

  allocator = rcl_get_default_allocator();

  // 1. Inizializza il supporto (Handshake con l'Agent)
  // Se l'Agent non è avviato sul PC, l'ESP32 rimarrà bloccato qui.
  rclc_support_init(&support, 0, NULL, &allocator);

  // 2. Crea il Nodo
  rclc_node_init_default(&node, "esp32_test_node", "", &support);

  // 3. Crea il Publisher sul topic "/micro_ros_test"
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/micro_ros_test");

  // 4. Crea un timer che scatta ogni 1000 millisecondi (1 secondo)
  const unsigned int timer_timeout = 1000;
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    &timer_callback);

  // 5. Crea l'Executor e aggiungi il timer
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  msg.data = 0;
}

void loop() {
  // Fai girare l'executor per gestire il timer
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}