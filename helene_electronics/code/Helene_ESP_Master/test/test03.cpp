#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>     // Quello corretto!

// Macro per il controllo degli errori senza blocco totale
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ micro_ros_init_successful = false; }}

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t publisher;
std_msgs__msg__String msg;

bool micro_ros_init_successful = false;
unsigned long prev_ms = 0;
int counter = 0;

// Funzione per inizializzare micro-ROS
bool init_microros() {
  allocator = rcl_get_default_allocator();

  // Inizializza il supporto
  rcl_ret_t rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK) return false;

  // Inizializza il nodo
  rc = rclc_node_init_default(&node, "esp32_node", "", &support);
  if (rc != RCL_RET_OK) return false;

  // Inizializza il publisher sul topic "/esp32_chatter"
  rc = rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "esp32_chatter"
  );
  if (rc != RCL_RET_OK) return false;

  // Alloca la memoria per la stringa del messaggio (massimo 20 caratteri)
  msg.data.data = (char * ) malloc(20 * sizeof(char));
  msg.data.size = 0;
  msg.data.capacity = 20;

  return true;
}

// Funzione per ripulire le risorse in caso di disconnessione
void destroy_microros() {
  free(msg.data.data);
  rcl_publisher_fini(&publisher, &node);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  // Configura la seriale a 1.000.000 di baud
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(1000);

  // Tenta la prima connessione
  micro_ros_init_successful = init_microros();
}

void loop() {
  // Se la connessione è attiva, pubblica il messaggio ogni 500ms
  if (micro_ros_init_successful) {
    unsigned long current_ms = millis();
    if (current_ms - prev_ms >= 500) {
      prev_ms = current_ms;

      // Crea la stringa "Hello World: [numero]"
      snprintf(msg.data.data, msg.data.capacity, "Hello World: %d", counter++);
      msg.data.size = strlen(msg.data.data);

      // Pubblica il messaggio sul topic
      RCCHECK(rcl_publish(&publisher, &msg, NULL));

      // Se il publish fallisce, pulisce tutto così al prossimo giro tenta la riconnessione
      if (!micro_ros_init_successful) {
        destroy_microros();
      }
    }
  } else {
    // Se NON è connesso, tenta il riavvio della connessione ogni 1000ms
    // Questo risolve il problema del tasto RESET mancante!
    unsigned long current_ms = millis();
    if (current_ms - prev_ms >= 1000) {
      prev_ms = current_ms;
      micro_ros_init_successful = init_microros();
    }
  }
}