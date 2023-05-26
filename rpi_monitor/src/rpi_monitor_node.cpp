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

#include <filesystem>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include "rpi_monitor_node.hpp"

// Default node name and topic name.
static const char * kNodeName = "rpi_monitor_node";


RPiMonitorNode::RPiMonitorNode()
: Node(kNodeName), temps_(), temp_warn_(0.0), temp_error_(0.0), temp_dict_()
{
  RCLCPP_INFO(get_logger(), "%s: Called", __func__);
  updater_ = new diagnostic_updater::Updater(this);
  updater_->setHardwareID("RPI Monitor");
  updater_->add("CPU Temperature", this, &RPiMonitorNode::CheckTemperature);
}

RPiMonitorNode::~RPiMonitorNode() {
  delete updater_;
}

void RPiMonitorNode::CheckTemperature(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (temps_.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Temperature files not found");
    return;
  }

  int level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string error_str = "";

  for (auto itr = temps_.begin(); itr != temps_.end(); ++itr) {
    // Read temperature from /sys...
    const std::filesystem::path path(itr->path_);
    std::ifstream input_stream(path, std::ios::in);
    if (!input_stream.is_open()) {
      stat.add("file open error", itr->path_);
      error_str = "file open error";
      continue;
    }

    float temp;
    input_stream >> temp;
    input_stream.close();
    temp /= 1000;
    stat.addf(itr->label_, "%.1f DegC", temp);

    level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (temp >= temp_error_) {
      level = std::max(level, static_cast<int>(diagnostic_msgs::msg::DiagnosticStatus::ERROR));
    } else if (temp >= temp_warn_) {
      level = std::max(level, static_cast<int>(diagnostic_msgs::msg::DiagnosticStatus::WARN));
    }
  }

  if (!error_str.empty()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, error_str);
  } else {
    stat.summary(level, temp_dict_.at(level));
  }
}
