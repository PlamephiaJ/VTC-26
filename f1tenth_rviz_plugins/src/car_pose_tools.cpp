// Copyright (c) 2026 Hongrui Zheng, Ahmad Amine, Cedric Hollande
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

#include "rviz_default_plugins/tools/goal_pose/goal_tool.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace f1tenth_rviz_plugins
{

class Car3PoseTool : public rviz_default_plugins::tools::GoalTool
{
public:
  Car3PoseTool()
  {
    shortcut_key_ = '3';
  }

  void onInitialize() override
  {
    rviz_default_plugins::tools::GoalTool::onInitialize();
    setName("Car 3");
  }
};

class Car4PoseTool : public rviz_default_plugins::tools::GoalTool
{
public:
  Car4PoseTool()
  {
    shortcut_key_ = '4';
  }

  void onInitialize() override
  {
    rviz_default_plugins::tools::GoalTool::onInitialize();
    setName("Car 4");
  }
};

class Car5PoseTool : public rviz_default_plugins::tools::GoalTool
{
public:
  Car5PoseTool()
  {
    shortcut_key_ = '5';
  }

  void onInitialize() override
  {
    rviz_default_plugins::tools::GoalTool::onInitialize();
    setName("Car 5");
  }
};

}  // namespace f1tenth_rviz_plugins

PLUGINLIB_EXPORT_CLASS(f1tenth_rviz_plugins::Car3PoseTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(f1tenth_rviz_plugins::Car4PoseTool, rviz_common::Tool)
PLUGINLIB_EXPORT_CLASS(f1tenth_rviz_plugins::Car5PoseTool, rviz_common::Tool)
