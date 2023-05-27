// Copyright 2023 University of Leeds.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "sys/types.h"
#include "sys/sysinfo.h"

#include <filesystem>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include "rpi_monitor_node.hpp"

// Default node name and topic name.
static const char * kNodeName = "rpi_monitor";

static const float kTempWarnC = 65.0;
static const float kTempErrorC = 75.0;
static const std::string kCPUName("CPU All");
static const std::string kCPUTemperaturePath("/sys/class/thermal/thermal_zone0/temp");

static const std::string kCPUUsagePath("/proc/stat");
static const float kCPUUsageError = 90.0;
static const float kCPUUsageWarn = 80.0;

static const float kMemoryUsageError = 95.0;
static const float kMemoryUsageWarn = 80.0;


static std::vector<std::string> WordSplit(std::string in_str)
{
  std::vector<std::string> strings;
  std::stringstream ss(in_str);
  std::string word;
  while (ss >> word) {
    strings.push_back(word);
  }
  return strings;
}

RPiMonitorNode::RPiMonitorNode()
: Node(kNodeName)
{
  RCLCPP_INFO(get_logger(), "%s: Called", __func__);
  // Create updater.
  updater_ = new diagnostic_updater::Updater(this);
  updater_->setHardwareID("RPI Monitor");
  updater_->add("CPU Temperature", this, &RPiMonitorNode::CheckCPUTemperature);
  updater_->add("CPU Usage", this, &RPiMonitorNode::CheckCPUUsage);
  updater_->add("Memory Usage", this, &RPiMonitorNode::CheckMemoryUsage);
}

RPiMonitorNode::~RPiMonitorNode()
{
  delete updater_;
}

void RPiMonitorNode::CheckCPUTemperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string error_str = "";
  // Use temperature info to fill out the stat class.
  CPUTemperatureData temperature_data = ReadCPUTemperatureData();
  if (temperature_data.values_read_) {
    stat.addf("CPU", "%.1f DegC", temperature_data.cpu_temperature_c_);
    if (temperature_data.cpu_temperature_c_ >= kTempErrorC) {
      level = std::max(level, static_cast<int>(diagnostic_msgs::msg::DiagnosticStatus::ERROR));
    } else if (temperature_data.cpu_temperature_c_ >= kTempWarnC) {
      level = std::max(level, static_cast<int>(diagnostic_msgs::msg::DiagnosticStatus::WARN));
    }
  } else {
    stat.add("File open error", kCPUTemperaturePath);
  }
  stat.summary(level, temp_dict_.at(level));
}

RPiMonitorNode::CPUTemperatureData RPiMonitorNode::ReadCPUTemperatureData()
{
  CPUTemperatureData temperatures;
  // Read temperature from /sys...
  std::ifstream input_stream(kCPUTemperaturePath, std::ios::in);
  if (input_stream.is_open()) {
    input_stream >> temperatures.cpu_temperature_c_;
    input_stream.close();
    temperatures.cpu_temperature_c_ /= 1000;
    temperatures.values_read_ = true;
  }
  return temperatures;
}

void RPiMonitorNode::CheckCPUUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Use usage data to fill out the stat class.
  UsageData usage_data = ReadCPUUsageData();
  int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  if (usage_data.values_read_) {
    // Set level.
    if (usage_data.total_usage_ >= kCPUUsageError) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (usage_data.total_usage_ >= kCPUUsageWarn) {
      level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    // Add data to message.
    stat.addf("All CPUs", "%.2f%%", usage_data.total_usage_);
  } else {
    // Failed to open "file".
    stat.add("File open error", kCPUUsagePath);
  }
  stat.summary(level, load_dict_.at(level));
}

RPiMonitorNode::UsageData RPiMonitorNode::ReadCPUUsageData()
{
  // Read first line of output from /proc/stat.
  //       user    nice system  idle      iowait irq softirq steal guest guest_nice
  // "cpu  8293074 2617 2405913 598771170 210263 0   60505   0     0     0"
  // steal onwards can be ignored on a Raspberry Pi.
  // All values in Jiffies ()
  UsageData usage_data;
  // Open the file
  std::fstream file(kCPUUsagePath, std::fstream::in);
  if (file) {
    // Read first line of file only.
    std::string line;
    std::getline(file, line);
    file.close();
    // Split into words.
    std::vector<std::string> words = WordSplit(line);
    // Remove first word "cpu".
    words.erase(words.begin());
    // Calculate total.
    uint64_t total = 0;
    for (auto word : words) {
      total += std::stoull(word);
    }
    // Idle is 4th element.
    uint64_t idle = std::stoull(words[3]);
    // Total usage in percent is (total jiffies - idle) / total jiffies * 100.
    usage_data.total_usage_ = static_cast<double>(total - idle);
    usage_data.total_usage_ /= static_cast<double>(total);
    usage_data.total_usage_ *= 100.0;
    usage_data.values_read_ = true;
  }
  return usage_data;
}

void RPiMonitorNode::CheckMemoryUsage(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // Get data. Note that this never fails so no file error logic is needed.
  UsageData usage_data = ReadMemoryUsageData();
  // Fill out message
  int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  // Set level.
  if (usage_data.total_usage_ >= kMemoryUsageError) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
  } else if (usage_data.total_usage_ >= kMemoryUsageWarn) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }
  // Add data to message.
  stat.addf("RAM used", "%.2f%%", usage_data.total_usage_);
  stat.summary(level, load_dict_.at(level));
}

RPiMonitorNode::UsageData RPiMonitorNode::ReadMemoryUsageData()
{
  UsageData usage_data;
  // Get the data.
  struct sysinfo memory_info;
  sysinfo(&memory_info);
  // Total RAM
  uint64_t ram_total = memory_info.totalram;
  // Done on separate line so prevent overflow.
  ram_total *= memory_info.mem_unit;
  // RAM in use.
  uint64_t ram_in_use = memory_info.totalram - memory_info.freeram;
  ram_in_use *= memory_info.mem_unit;
  // Turn into percentage.
  usage_data.total_usage_ = static_cast<double>(ram_total - ram_in_use);
  usage_data.total_usage_ /= static_cast<double>(ram_total);
  usage_data.total_usage_ *= 100.0;
  usage_data.values_read_ = true;
  return usage_data;
}
