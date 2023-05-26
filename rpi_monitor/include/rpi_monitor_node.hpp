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

#ifndef RPI_MONITOR_NODE_HPP_
#define RPI_MONITOR_NODE_HPP_

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/publisher.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief Publish diagnostic messages.
 */
class RPiMonitorNode : public rclcpp::Node
{
public:
  RPiMonitorNode();
  ~RPiMonitorNode();

private:
  void CheckTemperature(diagnostic_updater::DiagnosticStatusWrapper & stat);

  diagnostic_updater::Updater * updater_;
  /// @brief CPU temperature status messages
  const std::map<int, const char *> temp_dict_ =
  {
    {diagnostic_msgs::msg::DiagnosticStatus::OK, "OK"},
    {diagnostic_msgs::msg::DiagnosticStatus::WARN, "warm"},
    {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "hot"}
  };

};

#endif  // RPI_MONITOR_NODE_HPP_
