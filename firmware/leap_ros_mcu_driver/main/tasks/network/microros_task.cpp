// ============================================================
// 硬件对应：micro-ROS 通信（ESP32-S3 Wi-Fi + UDP）
// 说明：基于 ESP32-S3 无线网络，与 ROS 2 Agent 建立 micro-ROS 节点、收发话题
// 文件：main/tasks/network/microros_task.cpp
// ============================================================

#include "system_globals.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "msg/battery_msg.h"
#include "msg/imu_msg.h"
#include "msg/lidar_msg.h"
#include "msg/motion_msg.h"
#include "msg/ultrasonic_msg.h"
#include "pid_config.h"

#include "geometry_msgs/msg/twist.h"
#include "leap_interfaces/srv/get_speed_pid.h"
#include "leap_interfaces/srv/set_speed_pid.h"
#include "nav_msgs/msg/odometry.h"
#include "rcl/rcl.h"
#include "rcl/node_options.h"
#include "rcl/time.h"
#include "rcl/timer.h"
#include "rcl/wait.h"
#include "rclc/executor.h"
#include "rmw/qos_profiles.h"
#include "rmw_microros/custom_transport.h"
#include "rmw_microros/ping.h"
#include "rmw_microros/rmw_microros.h"
#include "rmw_microros/time_sync.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string_functions.h"
#include "sensor_msgs/msg/battery_state.h"
#include "sensor_msgs/msg/imu.h"
#include "sensor_msgs/msg/laser_scan.h"
#include "sensor_msgs/msg/range.h"
#include "uxr/client/profile/transport/custom/custom_transport.h"

static const char *TAG = "MICROROS";
static constexpr size_t kLaserScanPointCount = 360;
static constexpr double kDegToRad = 0.017453292519943295;
static constexpr double kGravity = 9.80665;
static constexpr TickType_t kAgentCheckIntervalTicks = pdMS_TO_TICKS(1000);
static constexpr int kAgentPingTimeoutMs = 500;
static constexpr uint8_t kAgentPingAttempts = 2;

// 时间同步相关
static constexpr int kEpochSyncTimeoutMs = 1000;           // 建链首次同步的单次超时
static constexpr int kEpochPeriodicSyncTimeoutMs = 300;    // 周期重同步的单次超时（网络正常时首次采样即 break）
static constexpr TickType_t kEpochResyncIntervalTicks = pdMS_TO_TICKS(10000);  // 每 10s 重同步一轮
static constexpr int kEpochSyncSamples = 3;                // 每轮采样次数，取 RTT 最小者（3 次足够，减少主循环阻塞）
static constexpr int64_t kEpochGoodRttUs = 20000;          // RTT 小于 20ms 视为好样本，可提前结束本轮
static constexpr int64_t kEpochMinPlausibleRttUs = 1000;   // RTT < 1ms 视为可疑（很可能是陈旧回包），丢弃重采
static constexpr int64_t kEpochMaxOffsetJumpMs = 500;      // 相邻两轮 offset 跳变超过 500ms 判为离群，丢弃本轮

// SNTP（方案 A）：直接向上位机的 chrony 校时，彻底绕开 XRCE 时间同步
static constexpr char kNtpServerIp[] = "192.168.0.59";     // 上位机 chrony，监听 0.0.0.0:123
static constexpr int kNtpSyncTimeoutMs = 10000;
// 合理性下限：2020-01-01T00:00:00Z 对应的 epoch 毫秒，低于此值说明 SNTP 给的时间不可用
static constexpr int64_t kMinPlausibleEpochMs = 1577836800000LL;

#ifndef CONFIG_MICRO_ROS_LOCAL_PORT
#define CONFIG_MICRO_ROS_LOCAL_PORT "8888"
#endif

struct UdpTransportContext {
    int fd;
    sockaddr_in remote;
    uint16_t local_port;
};

static UdpTransportContext s_udp_ctx = {};
static geometry_msgs__msg__Twist s_cmd_vel_msg = {};
static nav_msgs__msg__Odometry s_odom_msg = {};
static sensor_msgs__msg__Imu s_imu_msg = {};
static sensor_msgs__msg__LaserScan s_scan_msg = {};
static sensor_msgs__msg__BatteryState s_battery_msg = {};
static sensor_msgs__msg__Range s_ultrasonic_msg = {};
static rcl_publisher_t s_odom_publisher = {};
static rcl_publisher_t s_imu_publisher = {};
static rcl_publisher_t s_scan_publisher = {};
static rcl_publisher_t s_battery_publisher = {};
static rcl_publisher_t s_ultrasonic_publisher = {};
static rcl_subscription_t s_cmd_vel_subscriber = {};
static rcl_service_t s_set_speed_pid_service = {};
static rcl_service_t s_get_speed_pid_service = {};
static rclc_executor_t s_service_executor = {};
static leap_interfaces__srv__SetSpeedPid_Request s_set_speed_pid_req = {};
static leap_interfaces__srv__SetSpeedPid_Response s_set_speed_pid_res = {};
static leap_interfaces__srv__GetSpeedPid_Request s_get_speed_pid_req = {};
static leap_interfaces__srv__GetSpeedPid_Response s_get_speed_pid_res = {};
static rcl_init_options_t s_init_options = {};
static rcl_context_t s_context = {};
static rcl_allocator_t s_allocator = {};
static rcl_node_t s_node = {};
static rcl_timer_t s_publish_timer = {};
static rcl_wait_set_t s_wait_set = {};
static rcl_clock_t s_clock = {};
static bool s_ros_created = false;
static bool s_strings_initialized = false;
static bool s_scan_ranges_initialized = false;
static bool s_init_options_initialized = false;
static bool s_context_initialized = false;
static bool s_node_initialized = false;
static bool s_odom_publisher_initialized = false;
static bool s_imu_publisher_initialized = false;
static bool s_scan_publisher_initialized = false;
static bool s_battery_msg_initialized = false;
static bool s_battery_publisher_initialized = false;
static bool s_ultrasonic_publisher_initialized = false;
static bool s_cmd_vel_subscriber_initialized = false;
static bool s_set_speed_pid_service_initialized = false;
static bool s_get_speed_pid_service_initialized = false;
static bool s_service_executor_initialized = false;
static bool s_clock_initialized = false;
static bool s_timer_initialized = false;
static bool s_wait_set_initialized = false;
static uint32_t s_publish_tick = 0;
static TickType_t s_last_agent_check_tick = 0;
static bool s_ros_session_error = false;

// 时间同步兜底：保存最近一次成功同步得到的「epoch 时间 - 本地单调时钟」偏移
static int64_t s_epoch_offset_ms = 0;
static bool s_epoch_offset_valid = false;
static TickType_t s_last_epoch_sync_tick = 0;
static bool s_sntp_synced = false;   // SNTP 是否已拿到可信墙钟（是则不再走 XRCE 时间同步）

#define RCCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGW(TAG, "rcl status line %d: %d", __LINE__, static_cast<int>(rc)); \
        return false; \
    } \
} while (0)

#define RCSOFTCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        ESP_LOGW(TAG, "rcl status line %d: %d", __LINE__, static_cast<int>(rc)); \
    } \
} while (0)

#define RCPUBLISHCHECK(fn) do { \
    rcl_ret_t rc = (fn); \
    if (rc != RCL_RET_OK) { \
        s_ros_session_error = true; \
        ESP_LOGW(TAG, "rcl publish failed line %d: %d", __LINE__, static_cast<int>(rc)); \
    } \
} while (0)

extern "C" bool transport_open_udp(uxrCustomTransport *transport) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    ctx->fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (ctx->fd < 0) {
        return false;
    }

    sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(ctx->local_port);
    bind(ctx->fd, reinterpret_cast<sockaddr *>(&local), sizeof(local));
    return true;
}

extern "C" bool transport_close_udp(uxrCustomTransport *transport) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    if (ctx->fd >= 0) {
        close(ctx->fd);
        ctx->fd = -1;
    }
    return true;
}

extern "C" size_t transport_write_udp(
    uxrCustomTransport *transport,
    const uint8_t *buf,
    size_t len,
    uint8_t *) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    const int sent = sendto(
        ctx->fd,
        buf,
        len,
        0,
        reinterpret_cast<sockaddr *>(&ctx->remote),
        sizeof(ctx->remote));
    return sent > 0 ? static_cast<size_t>(sent) : 0;
}

extern "C" size_t transport_read_udp(
    uxrCustomTransport *transport,
    uint8_t *buf,
    size_t len,
    int timeout,
    uint8_t *) {
    auto *ctx = static_cast<UdpTransportContext *>(transport->args);
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(ctx->fd, &readfds);

    timeval tv = {};
    tv.tv_sec = timeout / 1000;
    tv.tv_usec = (timeout % 1000) * 1000;
    const int ret = select(ctx->fd + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0) {
        return 0;
    }

    const int received = recv(ctx->fd, buf, len, 0);
    return received > 0 ? static_cast<size_t>(received) : 0;
}

static void cleanup_result(rcl_ret_t ret) {
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "cleanup status: %d", static_cast<int>(ret));
    }
}

static void reset_ros_handles() {
    s_odom_publisher = rcl_get_zero_initialized_publisher();
    s_imu_publisher = rcl_get_zero_initialized_publisher();
    s_scan_publisher = rcl_get_zero_initialized_publisher();
    s_battery_publisher = rcl_get_zero_initialized_publisher();
    s_ultrasonic_publisher = rcl_get_zero_initialized_publisher();
    s_cmd_vel_subscriber = rcl_get_zero_initialized_subscription();
    s_set_speed_pid_service = rcl_get_zero_initialized_service();
    s_get_speed_pid_service = rcl_get_zero_initialized_service();
    s_service_executor = rclc_executor_get_zero_initialized_executor();
    s_init_options = rcl_get_zero_initialized_init_options();
    s_context = rcl_get_zero_initialized_context();
    s_node = rcl_get_zero_initialized_node();
    s_publish_timer = rcl_get_zero_initialized_timer();
    s_wait_set = rcl_get_zero_initialized_wait_set();
    s_clock = {};
}

static void handle_cmd_vel(const geometry_msgs__msg__Twist *msg) {
    if (msg == nullptr || q_motion_cmd == nullptr) {
        return;
    }

    MotionMsg cmd = {};
    cmd.source = MOTION_SRC_MICROROS;
    cmd.control_mode = 0;
    cmd.target_vx = static_cast<float>(msg->linear.x * 1000.0);
    cmd.target_vy = static_cast<float>(msg->linear.y * 1000.0);
    cmd.target_wz = static_cast<float>(msg->angular.z);
    xQueueOverwrite(q_motion_cmd, &cmd);
    g_emergency_stop = false;
}

static void set_speed_pid_response(
    leap_interfaces__srv__SetSpeedPid_Response *res,
    bool success,
    const PidMsg &pid_msg) {
    if (res == nullptr) {
        return;
    }
    res->success = success;
    res->kp = pid_msg.kp;
    res->ki = pid_msg.ki;
    res->kd = pid_msg.kd;
}

static void get_speed_pid_response(
    leap_interfaces__srv__GetSpeedPid_Response *res,
    bool success,
    const PidMsg &pid_msg) {
    if (res == nullptr) {
        return;
    }
    res->success = success;
    res->kp = pid_msg.kp;
    res->ki = pid_msg.ki;
    res->kd = pid_msg.kd;
}

static void handle_set_speed_pid_service(const void *req, void *res) {
    auto *request = static_cast<const leap_interfaces__srv__SetSpeedPid_Request *>(req);
    auto *response = static_cast<leap_interfaces__srv__SetSpeedPid_Response *>(res);
    if (request == nullptr || response == nullptr || q_speedpid_cmd == nullptr ||
        !isfinite(request->kp) || !isfinite(request->ki) || !isfinite(request->kd)) {
        set_speed_pid_response(response, false, g_speed_pid_state);
        return;
    }

    PidMsg pid_msg = {};
    pid_msg.kp = request->kp;
    pid_msg.ki = request->ki;
    pid_msg.kd = request->kd;
    esp_err_t save_err = pid_save_speed_config(&pid_msg);
    if (save_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save speed PID config: %s", esp_err_to_name(save_err));
        set_speed_pid_response(response, false, g_speed_pid_state);
        return;
    }

    g_speed_pid_state = pid_msg;
    xQueueOverwrite(q_speedpid_cmd, &pid_msg);
    set_speed_pid_response(response, true, pid_msg);
    ESP_LOGI(TAG, "Speed PID service set and saved: kp=%.3f, ki=%.3f, kd=%.3f",
             pid_msg.kp, pid_msg.ki, pid_msg.kd);
}

static void handle_get_speed_pid_service(const void *, void *res) {
    auto *response = static_cast<leap_interfaces__srv__GetSpeedPid_Response *>(res);
    get_speed_pid_response(response, true, g_speed_pid_state);
}

static void spin_pid_services() {
    if (s_service_executor_initialized) {
        RCSOFTCHECK(rclc_executor_spin_some(&s_service_executor, RCL_MS_TO_NS(0)));
    }
}

// 记录「epoch 时间 - 本地单调时钟」的偏移，供同步失败时兜底。
// 注意：同步失败时 rmw_uros_epoch_millis() 并不返回 0，而是返回自启动毫秒数（uxr_nanos - 0），
// 所以是否可信必须用 rmw_uros_epoch_synchronized() 判断，不能只判 > 0。
static void update_epoch_offset() {
    const int64_t epoch_ms = rmw_uros_epoch_millis();
    if (epoch_ms <= 0) {
        return;
    }
    s_epoch_offset_ms = epoch_ms - static_cast<int64_t>(esp_timer_get_time() / 1000ULL);
    s_epoch_offset_valid = true;
}

// 同步前把 UDP 接收缓冲读空（P0-1）。
//
// 根因：TIMESTAMP 走 best-effort 流，上一轮（尤其是超时失败那次）的回包会滞留在
// socket 缓冲里。新一轮 rmw_uros_sync_session() 一调用就立刻读到这个旧包，
// process_timestamp_reply 用【旧的 t0/t1/t2】配【新的 t3】算 offset：
//     time_offset = ((t0 + t3) - (t1 + t2)) / 2   ->  误差 = 陈旧时长 / 2（可达秒级）
// 而此时从外部测到的"RTT"只有十几毫秒（回包是现成的），会骗过最小 RTT 优选。
// 因此每次采样前必须先清空接收缓冲，保证读到的回包一定是本次请求产生的。
static void drain_udp_rx() {
    if (s_udp_ctx.fd < 0) {
        return;
    }

    uint8_t scratch[256];
    while (true) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(s_udp_ctx.fd, &readfds);

        timeval tv = {};
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        const int ret = select(s_udp_ctx.fd + 1, &readfds, nullptr, nullptr, &tv);
        if (ret <= 0) {
            return;  // 无数据可读，缓冲已空
        }
        const int received = recv(s_udp_ctx.fd, scratch, sizeof(scratch), 0);
        if (received <= 0) {
            return;
        }
    }
}

// 多采样同步，取「调用耗时（RTT 代理）最小」的那一次作为 offset。
//
// 误差界：|误差| = |d1 - d2| / 2 <= RTT / 2，所以只要拿到的样本 RTT < 200ms，
// 偏差必然 < 100ms。前提是样本必须"新鲜"——由 drain_udp_rx() 保证。
static bool sync_epoch_best_sample(int timeout_ms) {
    int64_t best_epoch_ms = 0;
    int64_t best_local_ms = 0;   // 与 best_epoch_ms 同一时刻取的本地时钟（P0-2）
    int64_t best_rtt_us = -1;

    for (int i = 0; i < kEpochSyncSamples; ++i) {
        drain_udp_rx();   // P0-1：丢弃所有陈旧回包

        const int64_t start_us = esp_timer_get_time();
        const rmw_ret_t sync_ret = rmw_uros_sync_session(timeout_ms);
        const int64_t rtt_us = esp_timer_get_time() - start_us;

        // 【方案 B】sync 返回后再清一次接收缓冲。
        // 目的：本次 sync 期间可能连带到达了「上一次请求的迟到回包」之外的多余包，
        // 若不清掉，它们会在下一次采样时被当成回包读走，造成
        // 「旧 t0/t1/t2 + 新 t3」的错位（误差 = 陈旧时长 / 2，可达秒级）。
        // 注意：本次真正用到的回包已经被 process_timestamp_reply 消费掉了，清的是残留。
        drain_udp_rx();

        if (sync_ret != RMW_RET_OK || !rmw_uros_epoch_synchronized()) {
            continue;
        }
        const int64_t epoch_ms = rmw_uros_epoch_millis();
        if (epoch_ms <= 0) {
            continue;
        }
        // P0-2：与 epoch_ms 同一时刻取本地时钟，避免"循环内取 epoch、循环外取 local"的错位
        const int64_t local_ms = static_cast<int64_t>(esp_timer_get_time() / 1000ULL);

        // P1-2：RTT 小得不合理，多半是陈旧/异常回包，丢弃并强制再采一次
        if (rtt_us < kEpochMinPlausibleRttUs) {
            continue;
        }

        if (best_rtt_us < 0 || rtt_us < best_rtt_us) {
            best_rtt_us = rtt_us;
            best_epoch_ms = epoch_ms;
            best_local_ms = local_ms;
        }
        // 已经拿到很好的样本，无需继续采样（减少正常情况下的开销）
        if (best_rtt_us < kEpochGoodRttUs) {
            break;
        }
    }

    if (best_epoch_ms <= 0 || best_rtt_us < 0) {
        return false;
    }

    const int64_t new_offset_ms = best_epoch_ms - best_local_ms;

    // P1-1：离群剔除（首次无历史值，不剔除）
    if (s_epoch_offset_valid) {
        int64_t delta_ms = new_offset_ms - s_epoch_offset_ms;
        if (delta_ms < 0) {
            delta_ms = -delta_ms;
        }
        if (delta_ms > kEpochMaxOffsetJumpMs) {
            ESP_LOGW(TAG, "epoch offset outlier rejected: new=%lld old=%lld (delta=%lld ms, rtt=%lld us)",
                     static_cast<long long>(new_offset_ms),
                     static_cast<long long>(s_epoch_offset_ms),
                     static_cast<long long>(delta_ms),
                     static_cast<long long>(best_rtt_us));
            return false;
        }
    }

    s_epoch_offset_ms = new_offset_ms;
    s_epoch_offset_valid = true;
    ESP_LOGI(TAG, "epoch sync ok, rtt=%lld us, epoch=%lld, local=%lld, offset=%lld",
             static_cast<long long>(best_rtt_us),
             static_cast<long long>(best_epoch_ms),
             static_cast<long long>(best_local_ms),
             static_cast<long long>(s_epoch_offset_ms));
    return true;
}

// 【方案 A】通过 SNTP 向上位机的 chrony 校时，拿真正的墙钟。
//
// 背景：XRCE 的时间同步（rmw_uros_sync_session）算出的 session->time_offset 会被"迟到回包"
// 污染（旧 t0/t1/t2 + 新 t3），而 MCU 侧没有独立墙钟可以自证 offset 是否正确，
// 导致"offset 稳定自洽、但时间戳偏差 1-2 秒"且 MCU 侧无法发现。
//
// SNTP 成功后 ESP-IDF 会调用 settimeofday()，使 clock_gettime(CLOCK_REALTIME) 变成 Unix epoch；
// 而 esp_timer_get_time() 不受影响（仍是自启动单调时钟）。所以取两者之差作为 offset，
// 之后 current_epoch_millis()（= esp_timer + offset）就直接输出真墙钟，无需再改。
//
// 注意：本函数必须在 micro-ROS 会话建立之前调用 —— settimeofday 会让时钟跳变 ~1.7e9 秒，
// 若会话已存在，micro-ROS 内部的 uxr_millis() 心跳/超时逻辑会被这个跳变打乱。
// 本函数调用点位于 micro-ROS 首次建链之前，因此安全；阻塞等待也只发生在启动阶段。
static bool sntp_time_sync_once(void) {
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(kNtpServerIp);
    config.wait_for_sync = true;
    config.start = true;
    // 首次偏移约 1.7e9 秒，不能用平滑模式（adjtime 慢速逼近对这个量级不可能收敛）
    config.smooth_sync = false;

    if (esp_netif_sntp_init(&config) != ESP_OK) {
        ESP_LOGW(TAG, "SNTP init failed, will fall back to XRCE time sync");
        return false;
    }

    const esp_err_t sync_err = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(kNtpSyncTimeoutMs));
    if (sync_err != ESP_OK) {
        ESP_LOGW(TAG, "SNTP sync failed/timeout: %s, will fall back to XRCE time sync",
                 esp_err_to_name(sync_err));
        esp_netif_sntp_deinit();
        return false;
    }

    timeval tv = {};
    gettimeofday(&tv, nullptr);
    const int64_t epoch_ms = static_cast<int64_t>(tv.tv_sec) * 1000 + tv.tv_usec / 1000;
    if (epoch_ms < kMinPlausibleEpochMs) {
        ESP_LOGW(TAG, "SNTP epoch implausible: %lld, will fall back to XRCE time sync",
                 static_cast<long long>(epoch_ms));
        esp_netif_sntp_deinit();
        return false;
    }

    const int64_t local_ms = static_cast<int64_t>(esp_timer_get_time() / 1000ULL);
    s_epoch_offset_ms = epoch_ms - local_ms;
    s_epoch_offset_valid = true;
    s_sntp_synced = true;
    ESP_LOGI(TAG, "SNTP synced from %s: epoch=%lld, local=%lld, offset=%lld",
             kNtpServerIp,
             static_cast<long long>(epoch_ms),
             static_cast<long long>(local_ms),
             static_cast<long long>(s_epoch_offset_ms));

    // 停掉 SNTP 周期更新：后续每次更新都是 settimeofday 硬跳变，
    // 会打乱 micro-ROS 内部的 uxr_millis() 心跳/超时（我们的时间戳走 esp_timer，不受影响）。
    esp_netif_sntp_deinit();
    return true;
}

// 取当前 epoch 毫秒。
//
// 【P0-1】一律用「本地单调时钟 + 我们自己校验过的 offset」，不再使用 rmw_uros_epoch_millis()。
// 原因：rmw_uros_epoch_millis() 用的是 rmw 内部的 session->time_offset，而 process_timestamp_reply
// 只要收到回包就【无条件】写这个 offset（不判质量）。drain_udp_rx() 只能清掉已到达 socket 的旧包，
// 清不掉网络在途的迟到包，它仍会在某一轮 sync 中间抵达并污染 rmw 侧 offset；
// 而 P1-1 的离群剔除只保护我们自己的 s_epoch_offset_ms，保护不了 rmw 内部值。
// 结果就是"日志里 offset 稳定自洽，但发布出去的时间戳偏差数秒"。
// esp_timer_get_time() 与 uxr_nanos() 同源（同为自启动的硬件定时器、速率一致），
// 因此用自己校验过的 offset 外推完全等价，且可控。
static int64_t current_epoch_millis() {
    if (s_epoch_offset_valid) {
        return static_cast<int64_t>(esp_timer_get_time() / 1000ULL) + s_epoch_offset_ms;
    }
    // 还没有任何一次校验通过的 offset，退回 rmw 的值（行为与改动前一致）
    if (rmw_uros_epoch_synchronized()) {
        const int64_t epoch_ms = rmw_uros_epoch_millis();
        if (epoch_ms > 0) {
            return epoch_ms;
        }
    }
    return 0;  // 从未同步成功过：无墙钟可用，保持原行为
}

static void set_stamp(std_msgs__msg__Header *header, int64_t stamp_ms) {
    if (stamp_ms <= 0) {
        // 限频告警：50 Hz 下每 250 次（约 5 秒）打印一条，避免日志刷屏
        static uint32_t s_invalid_stamp_count = 0;
        if ((s_invalid_stamp_count++ % 250U) == 0U) {
            ESP_LOGW(TAG, "no valid epoch time, header.stamp will be 0 (sync never succeeded)");
        }
    }
    header->stamp.sec = static_cast<int32_t>(stamp_ms / 1000);
    header->stamp.nanosec = static_cast<uint32_t>((stamp_ms % 1000) * 1000000);
}

static void publish_odom(int64_t stamp_ms) {
    MotionMsg motion = {};
    if (q_motion_state == nullptr || xQueuePeek(q_motion_state, &motion, 0) != pdTRUE) {
        return;
    }

    set_stamp(&s_odom_msg.header, stamp_ms);
    s_odom_msg.pose.pose.position.x = motion.x / 1000.0;
    s_odom_msg.pose.pose.position.y = motion.y / 1000.0;
    s_odom_msg.pose.pose.position.z = 0.0;
    s_odom_msg.pose.pose.orientation.w = motion.qw;
    s_odom_msg.pose.pose.orientation.x = motion.qx;
    s_odom_msg.pose.pose.orientation.y = motion.qy;
    s_odom_msg.pose.pose.orientation.z = motion.qz;
    s_odom_msg.twist.twist.linear.x = motion.vx / 1000.0;
    s_odom_msg.twist.twist.linear.y = motion.vy / 1000.0;
    s_odom_msg.twist.twist.angular.z = motion.wz;
    RCPUBLISHCHECK(rcl_publish(&s_odom_publisher, &s_odom_msg, nullptr));
}

static void publish_imu(int64_t stamp_ms) {
    ImuMsg imu = {};
    if (q_imu_state == nullptr || xQueuePeek(q_imu_state, &imu, 0) != pdTRUE) {
        return;
    }

    set_stamp(&s_imu_msg.header, stamp_ms);
    s_imu_msg.orientation.w = imu.qw;
    s_imu_msg.orientation.x = imu.qx;
    s_imu_msg.orientation.y = imu.qy;
    s_imu_msg.orientation.z = imu.qz;
    s_imu_msg.angular_velocity.x = imu.gyro_x * kDegToRad;
    s_imu_msg.angular_velocity.y = imu.gyro_y * kDegToRad;
    s_imu_msg.angular_velocity.z = imu.gyro_z * kDegToRad;
    s_imu_msg.linear_acceleration.x = imu.acc_x * kGravity;
    s_imu_msg.linear_acceleration.y = imu.acc_y * kGravity;
    s_imu_msg.linear_acceleration.z = imu.acc_z * kGravity;
    RCPUBLISHCHECK(rcl_publish(&s_imu_publisher, &s_imu_msg, nullptr));
}

static void publish_scan(int64_t stamp_ms) {
    LidarMsg lidar = {};
    if (q_lidar_state == nullptr || xQueuePeek(q_lidar_state, &lidar, 0) != pdTRUE) {
        return;
    }

    set_stamp(&s_scan_msg.header, stamp_ms);
    for (size_t i = 0; i < kLaserScanPointCount; ++i) {
        s_scan_msg.ranges.data[i] = lidar.distances[i] > 0
            ? static_cast<float>(lidar.distances[i]) / 1000.0f
            : 0.0f;
    }
    RCPUBLISHCHECK(rcl_publish(&s_scan_publisher, &s_scan_msg, nullptr));
}

static void publish_battery(int64_t stamp_ms) {
    BatteryMsg battery = {};
    if (q_battery_state == nullptr || xQueuePeek(q_battery_state, &battery, 0) != pdTRUE ||
        !battery.valid) {
        return;
    }

    set_stamp(&s_battery_msg.header, stamp_ms);
    s_battery_msg.voltage = battery.voltage_v;
    s_battery_msg.temperature = NAN;
    s_battery_msg.current = NAN;
    s_battery_msg.charge = NAN;
    s_battery_msg.capacity = NAN;
    s_battery_msg.design_capacity = NAN;
    s_battery_msg.percentage = static_cast<float>(battery.percentage) / 100.0f;
    s_battery_msg.power_supply_status =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
    s_battery_msg.power_supply_health =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_HEALTH_GOOD;
    s_battery_msg.power_supply_technology =
        sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LIPO;
    s_battery_msg.present = true;
    RCPUBLISHCHECK(rcl_publish(&s_battery_publisher, &s_battery_msg, nullptr));
}

static void publish_ultrasonic(int64_t stamp_ms) {
    UltrasonicMsg ultrasonic = {};
    if (q_ultrasonic_state == nullptr ||
        xQueuePeek(q_ultrasonic_state, &ultrasonic, 0) != pdTRUE) {
        return;
    }

    set_stamp(&s_ultrasonic_msg.header, stamp_ms);
    s_ultrasonic_msg.range = ultrasonic.distance_cm > 0.0f
        ? ultrasonic.distance_cm / 100.0f
        : NAN;
    RCPUBLISHCHECK(rcl_publish(&s_ultrasonic_publisher, &s_ultrasonic_msg, nullptr));
}

static void publish_state_timer(rcl_timer_t *timer, int64_t) {
    if (timer == nullptr || g_wifi_comm_mode != WifiCommMode::kMicroRos) {
        return;
    }

    const int64_t stamp_ms = current_epoch_millis();
    publish_odom(stamp_ms);
    publish_imu(stamp_ms);

    if ((++s_publish_tick % 5) == 0) {
        publish_scan(stamp_ms);
        publish_battery(stamp_ms);
        publish_ultrasonic(stamp_ms);
    }
}

static bool setup_udp_transport() {
    memset(&s_udp_ctx, 0, sizeof(s_udp_ctx));
    s_udp_ctx.fd = -1;
    s_udp_ctx.local_port = static_cast<uint16_t>(atoi(CONFIG_MICRO_ROS_LOCAL_PORT));
    s_udp_ctx.remote.sin_family = AF_INET;
    s_udp_ctx.remote.sin_port = htons(g_microros_agent_port);

    if (inet_pton(AF_INET, g_microros_agent_ip, &s_udp_ctx.remote.sin_addr.s_addr) != 1) {
        ESP_LOGE(TAG, "invalid micro-ROS agent IP: %s", g_microros_agent_ip);
        return false;
    }

    rmw_uros_set_custom_transport(
        false,
        &s_udp_ctx,
        transport_open_udp,
        transport_close_udp,
        transport_write_udp,
        transport_read_udp);
    return true;
}

static bool create_ros_entities() {
    s_publish_tick = 0;
    s_last_agent_check_tick = xTaskGetTickCount();
    s_ros_session_error = false;
    reset_ros_handles();

    rosidl_runtime_c__String__init(&s_odom_msg.header.frame_id);
    rosidl_runtime_c__String__init(&s_odom_msg.child_frame_id);
    rosidl_runtime_c__String__init(&s_imu_msg.header.frame_id);
    rosidl_runtime_c__String__init(&s_scan_msg.header.frame_id);
    rosidl_runtime_c__String__init(&s_ultrasonic_msg.header.frame_id);
    s_strings_initialized = true;

    (void)rosidl_runtime_c__String__assign(&s_odom_msg.header.frame_id, "odom");
    (void)rosidl_runtime_c__String__assign(&s_odom_msg.child_frame_id, "base_link");
    (void)rosidl_runtime_c__String__assign(&s_imu_msg.header.frame_id, "imu_link");
    (void)rosidl_runtime_c__String__assign(&s_scan_msg.header.frame_id, "laser_frame");
    (void)rosidl_runtime_c__String__assign(&s_ultrasonic_msg.header.frame_id, "ultrasonic_link");

    if (!rosidl_runtime_c__float32__Sequence__init(&s_scan_msg.ranges, kLaserScanPointCount)) {
        ESP_LOGE(TAG, "failed to allocate LaserScan ranges");
        return false;
    }
    s_scan_ranges_initialized = true;

    if (!sensor_msgs__msg__BatteryState__init(&s_battery_msg)) {
        ESP_LOGE(TAG, "failed to initialize BatteryState message");
        return false;
    }
    s_battery_msg_initialized = true;
    (void)rosidl_runtime_c__String__assign(&s_battery_msg.header.frame_id, "battery");
    (void)rosidl_runtime_c__String__assign(&s_battery_msg.location, "main");

    s_scan_msg.angle_min = 0.0f;
    s_scan_msg.angle_max = static_cast<float>((kLaserScanPointCount - 1) * kDegToRad);
    s_scan_msg.angle_increment = static_cast<float>(kDegToRad);
    s_scan_msg.time_increment = 0.0f;
    s_scan_msg.scan_time = 0.1f;
    s_scan_msg.range_min = 0.02f;
    s_scan_msg.range_max = 12.0f;

    s_ultrasonic_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
    s_ultrasonic_msg.field_of_view = 0.26f;
    s_ultrasonic_msg.min_range = 0.02f;
    s_ultrasonic_msg.max_range = 4.0f;
    s_ultrasonic_msg.range = NAN;

    // IMU 协方差：必须「非 -1 且非 0」。
    // robot_localization 只要三个数组的首元素为 -1 就整段忽略对应分量；为 0 会被替换成极小值，等价于完全信任。
    // 对角顺序：x(roll), y(pitch), z(yaw)，索引 0 / 4 / 8。
    // orientation [rad^2]：roll/pitch 经加速度计校正的互补滤波(~1-2 deg)；
    //                      yaw 为纯陀螺积分、无磁力计，长时间漂移，给大方差。
    s_imu_msg.orientation_covariance[0] = 1.0e-3;   // roll
    s_imu_msg.orientation_covariance[4] = 1.0e-3;   // pitch
    s_imu_msg.orientation_covariance[8] = 1.0e-1;   // yaw（漂移大，且 EKF 的 imu0 不融 yaw）

    // angular_velocity [(rad/s)^2]：LSM6DS3 ±250dps、8.75mdps/LSB、0.2dps 死区、上电零偏校准。
    // 1e-4 -> std 0.01 rad/s(约 0.57 dps)，偏保守；A5/A6 实测后再调。
    s_imu_msg.angular_velocity_covariance[0] = 1.0e-4;
    s_imu_msg.angular_velocity_covariance[4] = 1.0e-4;
    s_imu_msg.angular_velocity_covariance[8] = 1.0e-6;   // z 轴：2D 下真正被融合的分量

    // linear_acceleration [(m/s^2)^2]：原始加速度未去重力，EKF 暂不融 ax/ay/az。
    s_imu_msg.linear_acceleration_covariance[0] = 1.0e-2;
    s_imu_msg.linear_acceleration_covariance[4] = 1.0e-2;
    s_imu_msg.linear_acceleration_covariance[8] = 1.0e-2;

    // /odom 协方差（36 元素行主序，对角索引 0/7/14/21/28/35 = x,y,z,roll,pitch,yaw）。
    // 未参与融合的分量给大方差，避免被滤波器当成可信观测；单位：位置 m^2、姿态 rad^2。
    constexpr double kOdomUnusedCov = 1.0e6;

    s_odom_msg.pose.covariance[0]  = 1.0e-3;          // x   (std 约 3.2 cm)
    s_odom_msg.pose.covariance[7]  = 1.0e-3;          // y
    s_odom_msg.pose.covariance[14] = kOdomUnusedCov;  // z    （无观测，EKF 已关 z）
    s_odom_msg.pose.covariance[21] = kOdomUnusedCov;  // roll
    s_odom_msg.pose.covariance[28] = kOdomUnusedCov;  // pitch
    s_odom_msg.pose.covariance[35] = 1.0e-3;          // yaw  (std 约 0.032 rad ≈ 1.8 deg)

    s_odom_msg.twist.covariance[0]  = 1.0e-3;         // vx
    s_odom_msg.twist.covariance[7]  = 1.0e-3;         // vy
    s_odom_msg.twist.covariance[14] = kOdomUnusedCov; // vz
    s_odom_msg.twist.covariance[21] = kOdomUnusedCov; // vroll
    s_odom_msg.twist.covariance[28] = kOdomUnusedCov; // vpitch
    s_odom_msg.twist.covariance[35] = kOdomUnusedCov; // vyaw （按方案让给陀螺）

    s_allocator = rcl_get_default_allocator();
    s_init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&s_init_options, s_allocator));
    s_init_options_initialized = true;

    const rmw_ret_t ping_ret = rmw_uros_ping_agent(300, 3);
    if (ping_ret != RMW_RET_OK) {
        // ESP_LOGW(TAG, "micro-ROS agent unavailable: %s:%u",
        //          g_microros_agent_ip, static_cast<unsigned>(g_microros_agent_port));
        return false;
    }

    s_context = rcl_get_zero_initialized_context();
    RCCHECK(rcl_init(0, nullptr, &s_init_options, &s_context));
    s_context_initialized = true;

    // SNTP 已拿到可信墙钟时不再走 XRCE 时间同步：
    // 避免被污染的 session->time_offset 覆盖掉正确的 s_epoch_offset_ms。
    if (!s_sntp_synced) {
        if (!sync_epoch_best_sample(kEpochSyncTimeoutMs)) {
            ESP_LOGW(TAG, "time sync failed at session create, falling back to last known epoch offset");
        }
    }
    s_last_epoch_sync_tick = xTaskGetTickCount();

    s_node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    node_options.enable_rosout = false;
    const rcl_ret_t node_ret = rcl_node_init(&s_node, "leap_low_driver", "", &s_context, &node_options);
    cleanup_result(rcl_node_options_fini(&node_options));
    if (node_ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "rcl status line %d: %d", __LINE__, static_cast<int>(node_ret));
        return false;
    }
    s_node_initialized = true;

    rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();
    rcl_publisher_options_t sensor_pub_options = rcl_publisher_get_default_options();
    sensor_pub_options.qos = rmw_qos_profile_sensor_data;
    s_odom_publisher = rcl_get_zero_initialized_publisher();
    // 【P0-2】/odom 改用 sensor_data QoS（best effort），与 /imu 一致。
    // 原因：默认 QoS 是 reliable，rmw_publish 会走 uxr_run_session_until_confirm_delivery()
    // 阻塞等待 Agent ACK，最多 RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT(1000ms) 一帧，
    // 既把 50Hz 主循环拖到 ~1Hz，又让消息在可靠流里滞留、stamp 变旧（实测 /odom 偏 0.87s、
    // /imu 偏 3.15s 的直接原因）。里程计是高频数据，本就应 best effort。
    RCCHECK(rcl_publisher_init(
        &s_odom_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom",
        &sensor_pub_options));
    s_odom_publisher_initialized = true;

    s_imu_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(
        &s_imu_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu",
        &sensor_pub_options));
    s_imu_publisher_initialized = true;

    s_scan_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(
        &s_scan_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
        "scan",
        &pub_options));
    s_scan_publisher_initialized = true;

    s_battery_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(
        &s_battery_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
        "battery_state",
        &sensor_pub_options));
    s_battery_publisher_initialized = true;

    s_ultrasonic_publisher = rcl_get_zero_initialized_publisher();
    RCCHECK(rcl_publisher_init(
        &s_ultrasonic_publisher,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "ultrasonic",
        &sensor_pub_options));
    s_ultrasonic_publisher_initialized = true;

    rcl_subscription_options_t sub_options = rcl_subscription_get_default_options();
    sub_options.qos = rmw_qos_profile_sensor_data;
    s_cmd_vel_subscriber = rcl_get_zero_initialized_subscription();
    RCCHECK(rcl_subscription_init(
        &s_cmd_vel_subscriber,
        &s_node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel",
        &sub_options));
    s_cmd_vel_subscriber_initialized = true;

    rcl_service_options_t service_options = rcl_service_get_default_options();
    service_options.qos = rmw_qos_profile_services_default;
    RCCHECK(rcl_service_init(
        &s_set_speed_pid_service,
        &s_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(leap_interfaces, srv, SetSpeedPid),
        "set_speed_pid",
        &service_options));
    s_set_speed_pid_service_initialized = true;

    RCCHECK(rcl_service_init(
        &s_get_speed_pid_service,
        &s_node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(leap_interfaces, srv, GetSpeedPid),
        "get_speed_pid",
        &service_options));
    s_get_speed_pid_service_initialized = true;

    RCCHECK(rclc_executor_init(&s_service_executor, &s_context, 2, &s_allocator));
    s_service_executor_initialized = true;
    RCCHECK(rclc_executor_add_service(
        &s_service_executor,
        &s_set_speed_pid_service,
        &s_set_speed_pid_req,
        &s_set_speed_pid_res,
        handle_set_speed_pid_service));
    RCCHECK(rclc_executor_add_service(
        &s_service_executor,
        &s_get_speed_pid_service,
        &s_get_speed_pid_req,
        &s_get_speed_pid_res,
        handle_get_speed_pid_service));

    s_clock = {};
    s_publish_timer = rcl_get_zero_initialized_timer();
    RCCHECK(rcl_steady_clock_init(&s_clock, &s_allocator));
    s_clock_initialized = true;
    RCCHECK(rcl_timer_init(
        &s_publish_timer,
        &s_clock,
        &s_context,
        RCL_MS_TO_NS(20),
        publish_state_timer,
        s_allocator));
    s_timer_initialized = true;

    s_wait_set = rcl_get_zero_initialized_wait_set();
    RCCHECK(rcl_wait_set_init(&s_wait_set, 1, 0, 1, 0, 0, 0, &s_context, s_allocator));
    s_wait_set_initialized = true;
    s_ros_created = true;
    return true;
}

static void destroy_ros_entities() {
    if (!s_ros_created &&
        !s_strings_initialized &&
        !s_scan_ranges_initialized &&
        !s_init_options_initialized &&
        !s_context_initialized) {
        return;
    }

    if (s_context_initialized) {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&s_context);
        if (rmw_context) {
            (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
        }
    }

    if (s_wait_set_initialized) {
        cleanup_result(rcl_wait_set_fini(&s_wait_set));
        s_wait_set_initialized = false;
    }
    if (s_timer_initialized) {
        cleanup_result(rcl_timer_fini(&s_publish_timer));
        s_timer_initialized = false;
    }
    if (s_clock_initialized) {
        cleanup_result(rcl_clock_fini(&s_clock));
        s_clock_initialized = false;
    }
    if (s_ultrasonic_publisher_initialized) {
        cleanup_result(rcl_publisher_fini(&s_ultrasonic_publisher, &s_node));
        s_ultrasonic_publisher_initialized = false;
    }
    if (s_scan_publisher_initialized) {
        cleanup_result(rcl_publisher_fini(&s_scan_publisher, &s_node));
        s_scan_publisher_initialized = false;
    }
    if (s_battery_publisher_initialized) {
        cleanup_result(rcl_publisher_fini(&s_battery_publisher, &s_node));
        s_battery_publisher_initialized = false;
    }
    if (s_imu_publisher_initialized) {
        cleanup_result(rcl_publisher_fini(&s_imu_publisher, &s_node));
        s_imu_publisher_initialized = false;
    }
    if (s_odom_publisher_initialized) {
        cleanup_result(rcl_publisher_fini(&s_odom_publisher, &s_node));
        s_odom_publisher_initialized = false;
    }
    if (s_cmd_vel_subscriber_initialized) {
        cleanup_result(rcl_subscription_fini(&s_cmd_vel_subscriber, &s_node));
        s_cmd_vel_subscriber_initialized = false;
    }
    if (s_service_executor_initialized) {
        cleanup_result(rclc_executor_fini(&s_service_executor));
        s_service_executor_initialized = false;
    }
    if (s_set_speed_pid_service_initialized) {
        cleanup_result(rcl_service_fini(&s_set_speed_pid_service, &s_node));
        s_set_speed_pid_service_initialized = false;
    }
    if (s_get_speed_pid_service_initialized) {
        cleanup_result(rcl_service_fini(&s_get_speed_pid_service, &s_node));
        s_get_speed_pid_service_initialized = false;
    }
    if (s_node_initialized) {
        cleanup_result(rcl_node_fini(&s_node));
        s_node_initialized = false;
    }
    if (s_context_initialized) {
        cleanup_result(rcl_shutdown(&s_context));
        cleanup_result(rcl_context_fini(&s_context));
        s_context_initialized = false;
    }
    if (s_init_options_initialized) {
        cleanup_result(rcl_init_options_fini(&s_init_options));
        s_init_options_initialized = false;
    }
    if (s_strings_initialized) {
        rosidl_runtime_c__String__fini(&s_odom_msg.header.frame_id);
        rosidl_runtime_c__String__fini(&s_odom_msg.child_frame_id);
        rosidl_runtime_c__String__fini(&s_imu_msg.header.frame_id);
        rosidl_runtime_c__String__fini(&s_scan_msg.header.frame_id);
        rosidl_runtime_c__String__fini(&s_ultrasonic_msg.header.frame_id);
        s_strings_initialized = false;
    }
    if (s_scan_ranges_initialized) {
        rosidl_runtime_c__float32__Sequence__fini(&s_scan_msg.ranges);
        s_scan_ranges_initialized = false;
    }
    if (s_battery_msg_initialized) {
        sensor_msgs__msg__BatteryState__fini(&s_battery_msg);
        s_battery_msg_initialized = false;
    }

    s_ros_created = false;
    s_ros_session_error = false;
    reset_ros_handles();
}

static bool spin_once(int timeout_ms) {
    if (!s_wait_set_initialized) {
        return false;
    }

    rcl_ret_t ret = rcl_wait_set_clear(&s_wait_set);
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "rcl_wait_set_clear failed: %d", static_cast<int>(ret));
        return false;
    }
    ret = rcl_wait_set_add_subscription(&s_wait_set, &s_cmd_vel_subscriber, nullptr);
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "rcl_wait_set_add_subscription failed: %d", static_cast<int>(ret));
        return false;
    }
    ret = rcl_wait_set_add_timer(&s_wait_set, &s_publish_timer, nullptr);
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "rcl_wait_set_add_timer failed: %d", static_cast<int>(ret));
        return false;
    }

    ret = rcl_wait(&s_wait_set, RCL_MS_TO_NS(timeout_ms));
    if (ret == RCL_RET_TIMEOUT) {
        return true;
    }
    if (ret != RCL_RET_OK) {
        ESP_LOGW(TAG, "rcl_wait failed: %d", static_cast<int>(ret));
        return false;
    }

    if (s_wait_set.subscriptions[0] != nullptr) {
        const rcl_ret_t take_ret = rcl_take(&s_cmd_vel_subscriber, &s_cmd_vel_msg, nullptr, nullptr);
        if (take_ret == RCL_RET_OK) {
            handle_cmd_vel(&s_cmd_vel_msg);
        } else if (take_ret != RCL_RET_SUBSCRIPTION_TAKE_FAILED) {
            ESP_LOGW(TAG, "rcl_take failed: %d", static_cast<int>(take_ret));
        }
    }

    if (s_wait_set.timers[0] != nullptr) {
        RCSOFTCHECK(rcl_timer_call(&s_publish_timer));
    }
    spin_pid_services();
    if (s_ros_session_error) {
        ESP_LOGW(TAG, "micro-ROS publish failed, recreating session");
        return false;
    }
    return true;
}

static bool agent_still_reachable(TickType_t now) {
    if ((now - s_last_agent_check_tick) < kAgentCheckIntervalTicks) {
        return true;
    }

    s_last_agent_check_tick = now;
    const rmw_ret_t ping_ret = rmw_uros_ping_agent(kAgentPingTimeoutMs, kAgentPingAttempts);
    if (ping_ret == RMW_RET_OK) {
        return true;
    }

    ESP_LOGW(TAG, "micro-ROS agent unreachable, recreating session");
    return false;
}

// 周期性重同步 epoch 时间，抑制 MCU 单调时钟与上位机墙钟之间的长期漂移。
// 放在主循环而非新增 rcl_timer：现有 wait_set 只注册了 1 个 timer，
// 加第二个需要同步改 rcl_wait_set_init 的容量，改动面更大。
// 失败不重试，等下一个周期；单轮做 kEpochSyncSamples 次采样并取最小 RTT 样本。
static void maybe_resync_epoch(TickType_t now) {
    // SNTP 已成为时间源：不再做周期 XRCE 同步。
    // 这样既避免污染正确的 offset，也让主循环不再被同步阻塞（/odom 恢复 50Hz）。
    if (s_sntp_synced) {
        return;
    }
    if (!s_ros_created) {
        return;
    }
    if ((now - s_last_epoch_sync_tick) < kEpochResyncIntervalTicks) {
        return;
    }
    s_last_epoch_sync_tick = now;

    if (!sync_epoch_best_sample(kEpochPeriodicSyncTimeoutMs)) {
        ESP_LOGW(TAG, "periodic time sync failed, keeping last epoch offset");
        return;
    }
}

void microros_task(void *p) {
    (void)p;

    // 【方案 A】进入主循环前先通过 SNTP 拿真墙钟。
    // 此时 Wi-Fi STA 已连上（本任务由 wifi_start_task 在 STA 成功后创建），
    // 而 micro-ROS 会话尚未建立 —— settimeofday 的时钟跳变不会影响 XRCE 的心跳/超时逻辑。
    // 失败则保持原有 XRCE 时间同步路径，零回归。
    if (!sntp_time_sync_once()) {
        ESP_LOGW(TAG, "SNTP unavailable, using XRCE time sync as fallback");
    }

    while (1) {
        if (g_wifi_comm_mode == WifiCommMode::kMicroRos) {
            if (!s_ros_created) {
                if (setup_udp_transport() && create_ros_entities()) {
                    ESP_LOGI(TAG, "micro-ROS active: agent=%s:%u, pub=/odom,/imu,/scan,/battery_state,/ultrasonic, sub=/cmd_vel, srv=/set_speed_pid,/get_speed_pid",
                             g_microros_agent_ip,
                             static_cast<unsigned>(g_microros_agent_port));
                } else {
                    destroy_ros_entities();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }
            }
            const TickType_t now_tick = xTaskGetTickCount();
            if (!agent_still_reachable(now_tick)) {
                destroy_ros_entities();
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            maybe_resync_epoch(now_tick);
            if (!spin_once(20)) {
                destroy_ros_entities();
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        } else {
            if (s_ros_created) {
                destroy_ros_entities();
                ESP_LOGI(TAG, "micro-ROS inactive");
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
