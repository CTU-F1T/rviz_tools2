/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "bool_tool.hpp"

#include <sstream>

#include <OgreVector.h>

#include "rclcpp/qos.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/view_picker_iface.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/view_controller.hpp"

namespace rviz_tools2
{
namespace tools
{

BoolTool::BoolTool()
: qos_profile_(1)
{
  shortcut_key_ = 'a';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/eStop",
    "The topic on which to publish messages.",
    getPropertyContainer(), SLOT(updateTopic()), this);

  value_property_ = new rviz_common::properties::BoolProperty(
    "Value", true,
    "Logic value to be published on click.",
    getPropertyContainer(), SLOT(updateAutoDeactivate()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
    topic_property_, qos_profile_);
}

void BoolTool::onInitialize()
{
  hit_cursor_ = cursor_;
  std_cursor_ = rviz_common::getDefaultCursor();
  qos_profile_property_->initialize(
    [this](rclcpp::QoS profile) {this->qos_profile_ = profile;});
  updateTopic();
}

void BoolTool::activate() {
  std_msgs::msg::Bool b;
  b.data = value;
  publisher_->publish(b);
  RVIZ_COMMON_LOG_INFO_STREAM(
    "Sending '" << value_property_->getBool() << "' to '" <<  topic_property_->getStdString() << "'"
  );
}

void BoolTool::deactivate() {}

void BoolTool::updateTopic()
{
  rclcpp::Node::SharedPtr raw_node =
    context_->getRosNodeAbstraction().lock()->get_raw_node();
  // TODO(anhosi, wjwwood): replace with abstraction for publishers once available
  publisher_ = raw_node->
    template create_publisher<std_msgs::msg::Bool>(
    topic_property_->getStdString(), qos_profile_);
  clock_ = raw_node->get_clock();
}

void BoolTool::updateValue()
{
  value = value_property_->getBool();
}

void BoolTool::updateAutoDeactivate() {}

int BoolTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  return Finished;
}
/*
void BoolTool::setStatusForPosition(const Ogre::Vector3 & position)
{
  std::ostringstream s;
  s << "<b>Left-Click:</b> Select this point.";
  s.precision(3);
  s << " [" << position.x << "," << position.y << "," << position.z << "]";
  setStatus(s.str().c_str());
}

void BoolTool::publishPosition(const Ogre::Vector3 & position) const
{
  auto point = rviz_common::pointOgreToMsg(position);
  geometry_msgs::msg::PointStamped point_stamped;
  point_stamped.point = point;
  point_stamped.header.frame_id = context_->getFixedFrame().toStdString();
  point_stamped.header.stamp = clock_->now();
  publisher_->publish(point_stamped);
}
*/
}  // namespace tools
}  // namespace rviz_tools2

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(rviz_tools2::tools::BoolTool, rviz_common::Tool)

