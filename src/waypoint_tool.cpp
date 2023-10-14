/**
 * @file waypoint_tool.cpp
 * @author KoKoLates (the21515@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "waypoint_tool.h"

#include <ros/console.h>
#include <rviz/geometry.h>
#include <rviz/mesh_loader.h>
#include <rviz/validate_floats.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>
#include <rviz/properties/vector_property.h>

#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <geometry_msgs/PoseStamped.h>

namespace waypoint_plugin {

WaypointTool::WaypointTool(): move_axis_node_(NULL), widget_dock_(NULL), 
    widget_(NULL), server_("waypoint_plugin", "", false), unique_idx_(0) {
    shortcut_key_ = '1';
}

WaypointTool::~WaypointTool() {
    str2nodeptr::iterator node_itr;
    for (node_itr = node_map_.begin(); node_itr != node_map_.end(); node_itr++) {
        scene_manager_->destroySceneNode(node_itr->second);
    }
    delete widget_;
    delete widget_dock_;
}

void WaypointTool::initialize() {
    axis_resource_ = "package://waypoint-plugin/media/axis.dae";

    if (rviz::loadMeshFromResource(axis_resource_).isNull()) {
        ROS_ERROR("Waypoint Tool: failed to load model resource '%s'.", axis_resource_.c_str());
        return;
    }

    move_axis_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity(axis_resource_);
    move_axis_node_->attachObject(entity);
    move_axis_node_->setVisible(false);

    rviz::WindowManagerInterface* window_context_ = context_->getWindowManager();
    widget_ = new WaypointWidget(context_, &node_map_, &server_, &unique_idx_, NULL, this);

    if (window_context_) {
        widget_dock_ = window_context_->addPane("Waypoint Plugin", widget_);
    }
    widget_->enable();

    // add delete option for interactive marker
    menu_handler_.insert("delete", boost::bind(&WaypointTool::processFeedBack, this, _1));
    menu_handler_.insert("set manual", boost::bind(&WaypointTool::processFeedBack, this, _1));
}

void WaypointTool::activate() {
    if (move_axis_node_) {
        move_axis_node_->setVisible(true);
    }
}

void WaypointTool::deactivate() {
    if (move_axis_node_) {
        move_axis_node_->setVisible(false);
    }
}

int WaypointTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
    if (!move_axis_node_) {
        return Render;
    }

    double height = 0; // default height
    Ogre::Vector3 intersection;
    Ogre::Quaternion quternion;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, height);
    if (rviz::getPointOnPlaneFromWindowXY(
        event.viewport, ground_plane, event.x,
        event.y, intersection
    )) {
        move_axis_node_->setVisible(true);
        move_axis_node_->setPosition(intersection);

        widget_->setWaypointLabel(intersection);

        // check if mouse pointer is near existing waypoint
        str2nodeptr::iterator node_itr;
        for (node_itr = node_map_.begin(); node_itr != node_map_.end(); node_itr++) {
            Ogre::Vector3 stored_pose = node_itr->second->getPosition();
            double distance = std::sqrt(
                pow(stored_pose.x - intersection.x, 2) + pow(stored_pose.y - intersection.y, 2)
            );

            if (distance < 0.4) {
                move_axis_node_->setVisible(false);

                // delete the waypoint if right clicked
                if (event.rightDown()) {
                    node_itr->second->detachAllObjects();
                    std::stringstream waypoint_name;
                    waypoint_name << waypoint_name_prefix << node_itr->first;
                    server_.erase(waypoint_name.str());
                    server_.applyChanges();
                    node_map_.erase(node_itr);

                    move_axis_node_->setVisible(true);
                    return Render | Finished;
                }
            }
        }

        // adding waypoint
        if (event.leftDown()) {
            makeItem(intersection, quternion);
            return Render | Finished;
        }
    }
    else {
        move_axis_node_->setVisible(false);
    }
    return Render;
}

void WaypointTool::makeItem(const Ogre::Vector3& position, const Ogre::Quaternion& quaternion) {
    unique_idx_++;

    std::stringstream waypoint_name;
    waypoint_name << waypoint_name_prefix << unique_idx_;
    std::string str_name(waypoint_name.str());

    if (rviz::loadMeshFromResource(axis_resource_).isNull()) {
        ROS_ERROR("WaypointTool: failed to load model resource '%s'.", axis_resource_.c_str());
        return;
    }

    // create a new scene node and save it in a std::map
    Ogre::SceneNode* node_ptr = scene_manager_->getRootSceneNode() -> createChildSceneNode();
    Ogre::Entity* entity = scene_manager_->createEntity(axis_resource_);
    node_ptr->attachObject(entity);
    node_ptr->setVisible(true);
    node_ptr->setPosition(position);
    node_ptr->setOrientation(quaternion);

    str2nodeptr::iterator node_entry = node_map_.find(unique_idx_);
    if (node_entry == node_map_.end()) {
        node_map_.insert(std::make_pair(unique_idx_, node_ptr));
    }
    else {
        ROS_WARN("%s already in map", str_name.c_str());
        return;
    }

    int waypoint_num = node_map_.size();
    widget_->setWaypointCount(waypoint_num);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = position.x;
    pose.pose.position.y = position.y;
    pose.pose.position.z = position.z;
    pose.pose.orientation.x = quaternion.x;
    pose.pose.orientation.y = quaternion.y;
    pose.pose.orientation.z = quaternion.z;
    pose.pose.orientation.w = quaternion.w;

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.stamp = ros::Time::now();
    int_marker.header.frame_id = widget_->getFrameId().toStdString();
    int_marker.pose = pose.pose;
    int_marker.scale = 2;
    int_marker.name = str_name;

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.scale.x = 2.0;
    marker.scale.y = 2.0;
    marker.scale.z = 2.0;
    marker.color.r = 2.0;
    marker.color.g = 2.0;
    marker.color.b = 2.0;
    marker.color.a = 2.0;

    visualization_msgs::InteractiveMarkerControl c_control;
    c_control.always_visible = true;
    c_control.markers.push_back(marker);
    int_marker.controls.push_back(c_control);

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 0.707106781;
    control.orientation.x = 0;
    control.orientation.y = 0.707106781;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
    int_marker.controls.push_back(control);
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    control.name = "menu_delete";
    control.description = str_name;
    int_marker.controls.push_back(control);

    server_.insert(int_marker);
    server_.setCallback(int_marker.name, boost::bind(&WaypointTool::processFeedBack, this, _1));
    menu_handler_.apply(server_, int_marker.name);

    // set the current marker as selected
    Ogre::Vector3 p = position;
    Ogre::Quaternion q = quaternion;
    widget_->setSelectedMarkerName(str_name);
    widget_->setWaypointLabel(p);
    widget_->setPose(p, q);
    server_.applyChanges();
}

void WaypointTool::processFeedBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    switch (feedback -> event_type) {
        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
        {
            str2nodeptr::iterator node_entry = node_map_.find(std::stoi(
                feedback->marker_name.substr(strlen(waypoint_name_prefix))));
            if (node_entry == node_map_.end()) {
                ROS_ERROR("%s not found in map", feedback->marker_name.c_str());
            }
            else {
                if (feedback->menu_entry_id == 1) {
                    std::stringstream waypoint_name;
                    waypoint_name << waypoint_name_prefix << node_entry->first;
                    server_.erase(waypoint_name.str());

                    menu_handler_.reApply(server_);
                    server_.applyChanges();
                    node_entry->second->detachAllObjects();
                    node_map_.erase(node_entry);

                    int waypoint_num = node_map_.size();
                    widget_->setWaypointCount(waypoint_num);
                }
                else {
                    //Set the pose manually from the line edits
                    Ogre::Vector3 position;
                    Ogre::Quaternion quaternion;

                    widget_->getPose(position, quaternion);

                    geometry_msgs::Pose pose;
                    pose.position.x = position.x;
                    pose.position.y = position.y;
                    pose.position.z = position.z;

                    pose.orientation.x = quaternion.x;
                    pose.orientation.y = quaternion.y;
                    pose.orientation.z = quaternion.z;
                    pose.orientation.w = quaternion.w;

                    node_entry->second->setPosition(position);
                    node_entry->second->setOrientation(quaternion);

                    widget_->setWaypointLabel(position);
                    server_.setPose(feedback->marker_name, pose);
                    server_.applyChanges();
                }
            }
        }
        break;
        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        {
            str2nodeptr::iterator node_entry = node_map_.find(std::stoi(
                feedback->marker_name.substr(strlen(waypoint_name_prefix))
            ));
            if (node_entry == node_map_.end()) {
                ROS_ERROR("%s not found in map", feedback->marker_name.c_str());
            }
            else {
                geometry_msgs::PoseStamped pose;
                pose.pose = feedback->pose;

                Ogre::Vector3 position;
                position.x = pose.pose.position.x;
                position.y = pose.pose.position.y;
                position.z = pose.pose.position.z;
                node_entry->second->setPosition(position);

                Ogre::Quaternion quternion;
                quternion.x = pose.pose.orientation.x;
                quternion.y = pose.pose.orientation.y;
                quternion.z = pose.pose.orientation.z;
                quternion.w = pose.pose.orientation.w;
                node_entry->second->setOrientation(quternion);

                widget_->setWaypointLabel(position);
                widget_->setPose(position, quternion);
                widget_->setSelectedMarkerName(feedback->marker_name);
            }
        }
        break;
    }
}

void WaypointTool::getMarkerPose() {
    str2nodeptr::iterator node_itr;
    for (node_itr = node_map_.begin(); node_itr != node_map_.end(); node_itr++) {
        visualization_msgs::InteractiveMarker int_marker;

        std::stringstream waypoint_name;
        waypoint_name << waypoint_name_prefix << node_itr->first;
        server_.get(waypoint_name.str(), int_marker);

        ROS_ERROR(
            "pose: %g %g %g", 
            int_marker.pose.position.x, 
            int_marker.pose.position.y, 
            int_marker.pose.position.z
        );
    }
}

void WaypointTool::clearAllMarker() {
    str2nodeptr::iterator node_itr;
    for (node_itr = node_map_.begin(); node_itr != node_map_.end(); node_itr++) {
        scene_manager_->destroySceneNode(node_itr->second);
    }
    node_map_.clear();
    unique_idx_ = 0;
}

void WaypointTool::save(rviz::Config config) const {
    config.mapSetValue("Class", getClassId());
    rviz::Config waypoint_config = config.mapMakeChild("WaypointTool");

    waypoint_config.mapSetValue("topic", widget_->getOutputTopic());
    waypoint_config.mapSetValue("frame_id", widget_->getFrameId());
}

void WaypointTool::load(const rviz::Config& config) {
    rviz::Config waypoint_config = config.mapGetChild("WaypointTool");
    QString topic, frame;
    if (!waypoint_config.mapGetString("topic", &topic)) {
        topic = "/waypoints";
    }
    if (!waypoint_config.mapGetString("frame_id", &frame)) {
        frame = "map";
    }

    widget_->setConfig(topic, frame);
}

} // namespace end

# include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(waypoint_plugin::WaypointTool, rviz::Tool);
