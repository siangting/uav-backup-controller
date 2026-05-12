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

// ================== Pin Definitions ==================
#define GREEN_LED_PIN 12
#define RED_LED_PIN   10
#define ERROR_LED_PIN 15

// ================== Timing ==================
#define ERROR_THRESHOLD_MS 50000

#define TICK_TO_MS(t) ((t) * portTICK_PERIOD_MS)

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

uint32_t last_loop_time = 0;

// ================== Entities Management ==================

bool create_entities() {
    uint32_t t0 = xTaskGetTickCount();

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
            LOG("[HEARTBEAT] received");
        },
        ON_NEW_DATA
    );

    uint32_t t1 = xTaskGetTickCount();
    LOG("[CREATE] entities time = %lu ms", TICK_TO_MS(t1 - t0));

    return true;
}

void destroy_entities() {
    uint32_t t0 = xTaskGetTickCount();

    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rclc_executor_fini(&executor);
    rcl_subscription_fini(&heartbeat_sub, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    uint32_t t1 = xTaskGetTickCount();
    LOG("[DESTROY] time = %lu ms", TICK_TO_MS(t1 - t0));
}

// ================== Setup ==================
void setup() {
    SEGGER_RTT_Init();

    // 非阻塞模式（避免影響 timing）
    SEGGER_RTT_SetFlagsUpBuffer(0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    set_microros_serial_transports(Serial);

    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(ERROR_LED_PIN, OUTPUT);

    digitalWrite(GREEN_LED_PIN, LOW);
    digitalWrite(RED_LED_PIN, HIGH);
    digitalWrite(ERROR_LED_PIN, HIGH);

    LOG("[BOOT] system start");
}

// ================== Loop ==================
void loop() {

    uint32_t loop_start = xTaskGetTickCount();

    // ===============================
    // 1. Ping Agent latency
    // ===============================
    uint32_t t_ping0 = xTaskGetTickCount();
    bool ping_success = (rmw_uros_ping_agent(100, 1) == RMW_RET_OK);
    uint32_t t_ping1 = xTaskGetTickCount();

    // LOG("[PING] latency = %lu ms", TICK_TO_MS(t_ping1 - t_ping0));

    // ===============================
    // 2. Disconnect handling
    // ===============================
    if (!ping_success) {

        if (!is_disconnect_timer_running) {
            disconnect_start_time = millis();
            is_disconnect_timer_running = true;

            LOG("[STATE] disconnect detected");

            if (entities_created) {
                destroy_entities();
                entities_created = false;
            }
        }

        if (millis() - disconnect_start_time > ERROR_THRESHOLD_MS) {

            LOG("[STATE] Startup delay module");

            digitalWrite(GREEN_LED_PIN, LOW);
            digitalWrite(RED_LED_PIN, LOW);

            digitalWrite(ERROR_LED_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(1000));
            digitalWrite(ERROR_LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(1000));

        } else {

            digitalWrite(GREEN_LED_PIN, LOW);
            digitalWrite(RED_LED_PIN, HIGH);
            LOG("[STATE] Backup activation completed");
            vTaskDelay(pdMS_TO_TICKS(500));

        }

        return;
    }

    // ===============================
    // 3. Recovery
    // ===============================
    if (is_disconnect_timer_running) {
        is_disconnect_timer_running = false;

        LOG("[STATE] agent back online");

        // LOG("[RECOVERY] time = %lu ms", millis() - disconnect_start_time);
    }

    digitalWrite(RED_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
    LOG("[STATE] agent connected");

    // ===============================
    // 4. Create entities
    // ===============================
    if (!entities_created) {
        if (create_entities()) {
            entities_created = true;
        } else {
            // LOG("[ERROR] create_entities failed");
            destroy_entities();
            vTaskDelay(pdMS_TO_TICKS(500));
            return;
        }
    }

    // ===============================
    // 5. Executor latency
    // ===============================
    uint32_t t_exec0 = xTaskGetTickCount();

    if (rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)) != RCL_RET_OK) {
        // LOG("[ERROR] executor failed");
        entities_created = false;
    }

    uint32_t t_exec1 = xTaskGetTickCount();

    // LOG("[EXECUTOR] time = %lu ms", TICK_TO_MS(t_exec1 - t_exec0));

    // ===============================
    // 6. Loop period
    // ===============================
    uint32_t loop_end = xTaskGetTickCount();

    // LOG("[LOOP] period = %lu ms", TICK_TO_MS(loop_end - last_loop_time));

    last_loop_time = loop_end;

    vTaskDelay(pdMS_TO_TICKS(10));
}