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

#include "SEGGER_RTT.h"
#include <Servo.h>

// ================== Pin Definitions ==================
#define GREEN_LED_PIN 12
#define YELLOW_LED_PIN 11
#define RED_LED_PIN   10
#define ERROR_LED_PIN 15
#define NORMAL_SERVO_PIN 6
#define BACKUP_SERVO_PIN 2

// ================== Timing ==================
#define INIT_WAIT_MS 50000
#define INIT_SERVO_POSITION_MS 5000
#define ERROR_THRESHOLD_MS     10000

Servo normal_servo;
Servo backup_servo;

#define LOG(fmt, ...) \
    SEGGER_RTT_printf(0, fmt "\n", ##__VA_ARGS__)

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

// Edge detection for AGENT_CONNECTED / AGENT_DISCONNECTED markers
bool agent_was_connected = false;

// ================== Entities Management ==================
bool create_entities() {
    uint32_t t0 = millis();

    allocator = rcl_get_default_allocator();
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
    if (rclc_node_init_default(&node, "pico_heartbeat_monitor", "", &support) != RCL_RET_OK) return false;

    if (rclc_subscription_init_default(
        &heartbeat_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
        "/heartbeat"
    ) != RCL_RET_OK) return false;

    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;

    rclc_executor_add_subscription(
        &executor,
        &heartbeat_sub,
        &heartbeat_msg,
        [](const void *msgin){
            // LOG("[HEARTBEAT] received");  // 10Hz，太吵，預設關閉
        },
        ON_NEW_DATA
    );

    uint32_t t1 = millis();

    // ---- [Profiling] micro-ROS 初始化 time (Pico 自己算) ----
    LOG("[T_UROS_INIT] init_time = %lu ms", (unsigned long)(t1 - t0));

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
    // ---- [Profiling] Boot time 起點 (millis() 此時 ~= 0) ----
    uint32_t t_boot_start = millis();

    SEGGER_RTT_Init();
    SEGGER_RTT_SetFlagsUpBuffer(0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    set_microros_serial_transports(Serial);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(YELLOW_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(ERROR_LED_PIN, OUTPUT);

    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(ERROR_LED_PIN, HIGH);

    normal_servo.attach(NORMAL_SERVO_PIN);
    backup_servo.attach(BACKUP_SERVO_PIN);

    normal_servo.writeMicroseconds(1166);
    backup_servo.writeMicroseconds(1166);
    delay(INIT_SERVO_POSITION_MS);
    normal_servo.writeMicroseconds(1500);
    backup_servo.writeMicroseconds(1500);

    // ---- [Profiling] Boot time 終點 (Pico 自己算) ----
    uint32_t t_boot_end = millis();
    LOG("[T_BOOT] boot_time = %lu ms", (unsigned long)(t_boot_end - t_boot_start));
}

// ================== Loop ==================
void loop() {

    bool ping_success = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK);

    // ===============================
    // Disconnect path
    // ===============================
    if (!ping_success) {

        if (!is_disconnect_timer_running) {
            disconnect_start_time = millis();
            is_disconnect_timer_running = true;

            // ---- [Profiling] Backup time 標記 (邊緣觸發，每次斷線印一次) ----
            // backup_time = pico_AGENT_DISCONNECTED_時間 - host_"Stopping processes"_時間
            if (agent_was_connected) {
                LOG("[EVT] AGENT_DISCONNECTED");
                agent_was_connected = false;
            }

            if (entities_created) {
                destroy_entities();
                entities_created = false;
            }
        }

        if (millis() - disconnect_start_time > ERROR_THRESHOLD_MS) {
            // 延時模組階段
            digitalWrite(GREEN_LED_PIN, LOW);
            digitalWrite(YELLOW_LED_PIN, LOW);
            digitalWrite(RED_LED_PIN, LOW);

            backup_servo.writeMicroseconds(1500);

            digitalWrite(ERROR_LED_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(1000));
            digitalWrite(ERROR_LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(1000));

        } else {
            // 備援啟動階段
            digitalWrite(GREEN_LED_PIN, LOW);
            digitalWrite(YELLOW_LED_PIN, LOW);
            digitalWrite(RED_LED_PIN, HIGH);
            normal_servo.writeMicroseconds(1500);
            backup_servo.writeMicroseconds(1000);
            LOG("[EVT] Backup activation completed");
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        return;
    }

    // ===============================
    // Connected path
    // ===============================
    if (is_disconnect_timer_running) {
        is_disconnect_timer_running = false;
    }

    // ---- [Profiling] Connect latency 標記 (邊緣觸發，每次連上印一次) ----
    // connect_latency = pico_AGENT_CONNECTED_時間 - host_"Starting micro-ROS agent"_時間
    if (!agent_was_connected) {
        LOG("[EVT] AGENT_CONNECTED");
        agent_was_connected = true;
    }

    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
    backup_servo.writeMicroseconds(1500);
    normal_servo.writeMicroseconds(1000);

    // Create entities (第一次連上、或重連後)
    if (!entities_created) {
        if (create_entities()) {
            entities_created = true;
        } else {
            destroy_entities();
            vTaskDelay(pdMS_TO_TICKS(500));
            return;
        }
    }

    // Spin executor
    if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)) != RCL_RET_OK) {
        entities_created = false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));
}