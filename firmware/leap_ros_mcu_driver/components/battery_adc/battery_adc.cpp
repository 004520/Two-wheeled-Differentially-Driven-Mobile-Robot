// ============================================================
// 硬件对应：电池电压（锂电池分压采样）
// 说明：通过 ADC 单次采样 + 校准，得到电池电压与电量百分比
// 文件：components/battery_adc/battery_adc.cpp
// ============================================================

#include "battery_adc.h"

#include <math.h>

// ESP-IDF 新版 ADC 驱动头文件
#include "esp_adc/adc_cali.h"        // ADC 校准相关接口
#include "esp_adc/adc_cali_scheme.h" // 校准方案定义（如曲线拟合）
#include "esp_adc/adc_oneshot.h"     // 单次触发模式驱动
#include "esp_log.h"                 // 日志系统

static const char *TAG = "BATTERY_ADC";

// --- 硬件与算法常量定义 ---

// ADC 衰减设置：12dB 衰减允许测量最高约 3.9V 的输入电压
// 对于分压后的电池信号，这提供了最大的动态范围和较好的线性度
static constexpr adc_atten_t kBatteryAdcAtten = ADC_ATTEN_DB_12;

// ADC 位宽：使用默认位宽（ESP32通常为12位，即0-4095 range）
static constexpr adc_bitwidth_t kBatteryAdcBitwidth = ADC_BITWIDTH_DEFAULT;

// 硬件分压比计算：
// 假设电池满电 8.4V (2串锂电)，经过电阻分压后在 GPIO 引脚测得 2.6857V
// 分压比 = 电池实际电压 / ADC引脚电压
// 注意：这里直接使用实测值，电阻的1%公差会直接转化为测量误差
static constexpr float kDividerRatio = 8.4f / 2.6857f;

// 电池电量估算边界电压（针对2串锂电池组，即7.4V标称）
static constexpr float kBatteryEmptyVoltage = 6.0f; // 截止电压，低于此值视为0%
static constexpr float kBatteryFullVoltage = 8.4f;  // 满电电压，高于此值视为100%

// 软件滤波参数：每次读取连续采样16次取平均，以抑制电源噪声和ADC量化抖动
static constexpr int kSampleCount = 16;

// --- 类实现 ---

BatteryAdc::BatteryAdc(adc_unit_t unit, adc_channel_t channel)
    : unit_(unit),          // ADC单元编号 (ADC_UNIT_1 或 ADC_UNIT_2)
      channel_(channel),    // ADC通道编号 (如 ADC_CHANNEL_6 对应 GPIO34等)
      unit_handle_(nullptr),// ADC单元操作句柄
      cali_handle_(nullptr),// ADC校准操作句柄
      initialized_(false),  // 初始化状态标志
      calibrated_(false) {} // 校准成功标志

/**
 * @brief 初始化 ADC 硬件及校准方案
 * @return esp_err_t ESP_OK 表示成功
 */
esp_err_t BatteryAdc::Init() {
  if (initialized_) {
    return ESP_OK; // 防止重复初始化
  }

  // 1. 创建 ADC 单次采样单元
  adc_oneshot_unit_init_cfg_t unit_config = {};
  unit_config.unit_id = unit_;
  esp_err_t err = adc_oneshot_new_unit(&unit_config, &unit_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to create ADC unit: %s", esp_err_to_name(err));
    return err;
  }

  // 2. 配置具体通道的参数（衰减、位宽）
  adc_oneshot_chan_cfg_t channel_config = {};
  channel_config.atten = kBatteryAdcAtten;   // 设置12dB衰减，扩大测量量程
  channel_config.bitwidth = kBatteryAdcBitwidth; // 设置分辨率
  err = adc_oneshot_config_channel(
      unit_handle_, channel_, &channel_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "failed to configure ADC channel: %s", esp_err_to_name(err));
    return err;
  }

  // 3. 配置并启用 ADC 校准方案（曲线拟合）
  // 曲线拟合校准能修正ADC在不同电压段的非线性误差，比简单线性映射更精准
  adc_cali_curve_fitting_config_t cali_config = {};
  cali_config.unit_id = unit_;
  cali_config.chan = channel_;
  cali_config.atten = kBatteryAdcAtten;
  cali_config.bitwidth = kBatteryAdcBitwidth;
  
  err = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle_);
  calibrated_ = (err == ESP_OK);
  if (!calibrated_) {
    // 如果校准失败（例如该芯片型号不支持或工厂数据缺失），后续将回退到线性计算
    ESP_LOGW(TAG, "ADC calibration unavailable: %s", esp_err_to_name(err));
  }

  initialized_ = true;
  ESP_LOGI(TAG, "battery ADC initialized on unit %d channel %d, ratio %.4f",
           static_cast<int>(unit_), static_cast<int>(channel_), kDividerRatio);
  return ESP_OK;
}

/**
 * @brief 读取电池电压并估算电量百分比
 * @return BatteryAdcReading 包含原始值、ADC电压、电池电压、电量百分比的结构体
 */
BatteryAdcReading BatteryAdc::Read() {
  BatteryAdcReading reading = {};
  if (!initialized_) {
    return reading; // 未初始化则返回空结构体
  }

  int raw_sum = 0;      // 原始ADC读数累加和
  int mv_sum = 0;       // 毫伏电压累加和
  int valid_samples = 0;// 有效采样计数

  // 循环采样进行软件滤波
  for (int i = 0; i < kSampleCount; ++i) {
    int raw = 0;
    // 执行一次ADC转换
    esp_err_t err = adc_oneshot_read(unit_handle_, channel_, &raw);
    if (err != ESP_OK) {
      continue; // 读取失败则跳过本次
    }

    int mv = 0;
    if (calibrated_) {
      // 【推荐路径】使用校准句柄将原始值转换为精确的毫伏值
      err = adc_cali_raw_to_voltage(cali_handle_, raw, &mv);
      if (err != ESP_OK) {
        continue;
      }
    } else {
      // 【降级路径】若无校准数据，使用理想线性公式换算
      // 假设参考电压为3300mV，12位ADC满量程为4095
      mv = static_cast<int>(lroundf((static_cast<float>(raw) / 4095.0f) * 3300.0f));
    }

    raw_sum += raw;
    mv_sum += mv;
    ++valid_samples;
  }

  // 如果没有有效样本，返回无效结果
  if (valid_samples == 0) {
    return reading;
  }

  // 计算平均值
  reading.raw = raw_sum / valid_samples;
  // 将平均毫伏值转换为伏特 (V)
  reading.adc_voltage_v = static_cast<float>(mv_sum) / static_cast<float>(valid_samples) / 1000.0f;
  
  // 【关键步骤】根据硬件分压比还原电池实际电压
  // V_battery = V_adc_pin * (R1+R2)/R2
  reading.battery_voltage_v = reading.adc_voltage_v * kDividerRatio;
  
  // 估算电量百分比
  reading.percentage = static_cast<uint8_t>(lroundf(EstimatePercentage(reading.battery_voltage_v)));
  reading.valid = true;
  
  return reading;
}

/**
 * @brief 根据电池电压线性估算剩余电量百分比
 * @param battery_voltage_v 电池实际电压 (V)
 * @return float 电量百分比 (0.0 - 100.0)
 */
float BatteryAdc::EstimatePercentage(float battery_voltage_v) const {
  // 线性插值公式：Pct = (V_curr - V_min) / (V_max - V_min) * 100%
  const float pct =
      ((battery_voltage_v - kBatteryEmptyVoltage) /
       (kBatteryFullVoltage - kBatteryEmptyVoltage)) *
      100.0f;
  
  // 边界保护：防止因噪声或负载压降导致计算出的电量超出物理范围
  if (pct < 0.0f) {
    return 0.0f;
  }
  if (pct > 100.0f) {
    return 100.0f;
  }
  return pct;
}
