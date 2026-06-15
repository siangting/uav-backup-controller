#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/empty.h>
#include <rmw_microros/rmw_microros.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <Servo.h>

// ================== Pin Definitions ==================
#define GREEN_LED_PIN 12
#define YELLOW_LED_PIN 9
#define RED_LED_PIN   10
#define ERROR_LED_PIN 15
#define NORMAL_SERVO_PIN 6
#define BACKUP_SERVO_PIN 2

// ================== Timing ==================
#define INIT_WAIT_MS 45000  // Wait 45 seconds for system initialization before starting heartbeat monitoring
#define INIT_SERVO_POSITION_MS 5000 // Time to move servos to initial position after startup
#define ERROR_THRESHOLD_MS     10000  // Connection lost for more than 50 seconds triggers blinking

Servo normal_servo;
Servo backup_servo;

// ================== micro-ROS Objects ==================
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t heartbeat_sub;
rclc_executor_t executor;
std_msgs__msg__Empty heartbeat_msg;

// ================== State Variables ==================
bool entities_created = false;
unsigned long disconnect_start_time = 0; 
bool is_disconnect_timer_running = false;

// ================== Entities Management ==================

bool create_entities() {
    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "pico_heartbeat_monitor", "", &support) != RCL_RET_OK) return false;
    
    if (rclc_subscription_init_default(
        &heartbeat_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "/heartbeat"
    ) != RCL_RET_OK) return false;

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
    
    rclc_executor_add_subscription(&executor, &heartbeat_sub, &heartbeat_msg, 
        [](const void *msgin){ Serial.println("Heartbeat OK"); }, ON_NEW_DATA);

    return true;
}

void destroy_entities() {
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rclc_executor_fini(&executor);
    rcl_subscription_fini(&heartbeat_sub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}

// ================== Setup ==================
void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);

    // pico onboard LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(ERROR_LED_PIN, OUTPUT);

    // Initial boot state: Red light on, others off
    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(ERROR_LED_PIN, HIGH); 

    normal_servo.attach(NORMAL_SERVO_PIN);
    backup_servo.attach(BACKUP_SERVO_PIN);

    normal_servo.writeMicroseconds(1166); // 1000us corresponds to full speed in one direction
    backup_servo.writeMicroseconds(1166); // 1000us corresponds to full speed in one direction
    delay(INIT_SERVO_POSITION_MS); // Allow servos to move to initial position
    normal_servo.writeMicroseconds(1500); 
    backup_servo.writeMicroseconds(1500);
    delay(INIT_WAIT_MS); // 等待系統INIT完成
}

// ================== Loop ==================
void loop() {
    // --- 1. Check Agent connection status ---
    bool ping_success = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK);

    if (!ping_success) {
        // If just started disconnecting, record time and clean up Entities
        if (!is_disconnect_timer_running) {
            disconnect_start_time = millis();
            is_disconnect_timer_running = true;
            if (entities_created) {
                destroy_entities();
                entities_created = false;
            }
        }

        // Determine if disconnected for more than 10 seconds
        if (millis() - disconnect_start_time > ERROR_THRESHOLD_MS) {
            // Enter error blinking mode
            Serial.println("FATAL ERROR: Agent offline > 10s");
            digitalWrite(GREEN_LED_PIN, LOW);
            digitalWrite(YELLOW_LED_PIN, LOW);
            digitalWrite(RED_LED_PIN, LOW);

            backup_servo.writeMicroseconds(1500); // Servo stop 

            digitalWrite(ERROR_LED_PIN, LOW);   // On
            vTaskDelay(pdMS_TO_TICKS(1000));
            digitalWrite(ERROR_LED_PIN, HIGH);  // Off
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            // Within 10 seconds of disconnection: keep red light on
            digitalWrite(GREEN_LED_PIN, LOW);
            digitalWrite(YELLOW_LED_PIN, LOW);
            digitalWrite(RED_LED_PIN, HIGH);
            normal_servo.writeMicroseconds(1500); // Servo stop 
            backup_servo.writeMicroseconds(1000); // 1000us corresponds to full speed in one direction
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        return; 
    }

    // --- 2. Connection recovery handling ---
    if (is_disconnect_timer_running) {
        is_disconnect_timer_running = false;
        Serial.println("Agent back online!");
    }
    
    digitalWrite(RED_LED_PIN, LOW);    // Turn off red light
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH); // Turn on green light
    backup_servo.writeMicroseconds(1500); // Servo stop 
    normal_servo.writeMicroseconds(1000); // 1000us corresponds to full speed in one direction

    if (!entities_created) {
        if (create_entities()) {
            entities_created = true;
        } else {
            destroy_entities();
            vTaskDelay(pdMS_TO_TICKS(500));
            return;
        }
    }

    // --- 3. Normal operation ---
    if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)) != RCL_RET_OK) {
        entities_created = false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}