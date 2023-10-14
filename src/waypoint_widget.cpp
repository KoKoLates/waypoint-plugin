/**
 * @file waypoint_widget.cpp
 * @author KoKoLates (the21515@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tf.h>
#include <QFileDialog>
#include <boost/foreach.hpp>
#include <rviz/display_context.h>
#include <interactive_markers/interactive_marker_server.h>

#include "waypoint_tool.h"

#define foreach BOOST_FOREACH

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace waypoint_plugin {

struct MissionKeywords {
    inline static const std::string KPosition = "position";
};

WaypointWidget::WaypointWidget (rviz::DisplayContext* context, std::map<int, Ogre::SceneNode* >* map_ptr,
                                interactive_markers::InteractiveMarkerServer* server, int* unique_idx, 
                                QWidget* parent, WaypointTool* waypoint_tool)
    : QWidget(parent)
    , context_(context) 
    , ui_(new Ui::PluginWidget()) 
    , map_ptr_(map_ptr_)
    , unique_idx_(unique_idx) 
    , server_(server)
    , frame_id_("map") 
    , waypoint_tool_(waypoint_tool) 
    , selected_marker_name_(std::string(waypoint_name_prefix) + "1") 
{        
    scene_manager_ = context_->getSceneManager();
    ui_->setupUi(this);
    path_publisher_ = handler_.advertise<nav_msgs::Path>("waypath", 1);

    // Qt signaks and slots Managements
    connect(ui_->button_save, SIGNAL(clicked()), this, SLOT(saveHandler()));
    connect(ui_->button_load, SIGNAL(clicked()), this, SLOT(loadHandler()));
    connect(ui_->button_clear, SIGNAL(clicked()), this, SLOT(clearHandler()));
    connect(ui_->button_publish, SIGNAL(clicked()), this, SLOT(publishHandler()));

    // Position spin box interface
    connect(ui_->x_spinbox, SIGNAL(valueChanged(double)), this, SLOT(poseChange(double)));
    connect(ui_->y_spinbox, SIGNAL(valueChanged(double)), this, SLOT(poseChange(double)));
    connect(ui_->z_spinbox, SIGNAL(valueChanged(double)), this, SLOT(poseChange(double)));
    connect(ui_->yaw_spinbox, SIGNAL(valueChanged(double)), this, SLOT(poseChange(double)));
    
    // topic / frame input interface
    connect(ui_->topic_input, SIGNAL(editingFinished()), this, SLOT(topicChange()));
    connect(ui_->frame_input, SIGNAL(editingFinished()), this, SLOT(frameChange()));
}

WaypointWidget::~WaypointWidget() {
    delete ui_;
    map_ptr_ = NULL;
}

void WaypointWidget::enable() {
    show();
}

void WaypointWidget::disable() {
    path_publisher_.shutdown();
    hide();
}

void WaypointWidget::saveHandler() {
    // saving the waypoint that have been setup
    QString filename = QFileDialog::getSaveFileName(
        0, tr("Waypoints Save"), "waypoints", 
        tr("Save Files (*.bag *.yaml *.json)")
    );
    if (filename.isEmpty()) {
        ROS_ERROR("No saving file selected");
        return;
    }
    const std::string str_filename = filename.toStdString();
    ROS_INFO_STREAM("Saving the waypoints into " << str_filename);

    // based on the file type
    if (filename.endsWith(".bag")) {
        saveBag(str_filename);
    }
    else {
        ROS_INFO_STREAM("Invalid saving file format: " << str_filename);
    }
    // [TODO] the yaml and json save
}

void WaypointWidget::loadHandler() {
    const QString filename = QFileDialog::getOpenFileName(
        0, tr("Waypoint load"), "~/", 
        tr("Load Files (*.bag *.yaml *.json)")
    );
    if (filename.isEmpty()) {
        ROS_ERROR("No loading file selected");
        return;
    }
    const std::string str_filename = filename.toStdString();
    ROS_INFO("loading waypoints from %s", str_filename.c_str());
    if (filename.endsWith(".bag")) {
        loadBag(str_filename);
    } 
    else {
        ROS_INFO_STREAM("Invalid loading file format: " << str_filename);
    }
}

void WaypointWidget::clearHandler() {
    // clear scene node
    std::map<int, Ogre::SceneNode*>::iterator node_itr;
    for (node_itr = map_ptr_->begin(); node_itr != map_ptr_->end(); node_itr++) {
        scene_manager_->destroySceneNode(node_itr->second);
    }
    //clear waypoint and reset index
    map_ptr_->clear();
    *unique_idx_=0;

    // clear interactive marker
    server_->clear();
    server_->applyChanges();
}

void WaypointWidget::publishHandler() {
    nav_msgs::Path path;
    std::map<int, Ogre::SceneNode*>::iterator node_itr;

    for (node_itr = map_ptr_->begin(); node_itr != map_ptr_->end(); node_itr++) {
        Ogre::Vector3 position;
        position = node_itr->second->getPosition();

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = position.x;
        pose.pose.position.y = position.y;
        pose.pose.position.z = position.z;

        Ogre::Quaternion quat;
        quat = node_itr->second->getOrientation();
        pose.pose.orientation.x = quat.x;
        pose.pose.orientation.y = quat.y;
        pose.pose.orientation.z = quat.z;
        pose.pose.orientation.w = quat.w;

        path.poses.push_back(pose);
    }
    path.header.frame_id = frame_id_.toStdString();
    path_publisher_.publish(path);
}

void WaypointWidget::poseChange(double value) {
    auto node_entry = map_ptr_->end();
    try {
        const int selected_marker_idx = std::stoi(selected_marker_name_.substr(strlen(waypoint_name_prefix)));
        node_entry = map_ptr_->find(selected_marker_idx);
    }
    catch (const std::logic_error& e) {
        ROS_ERROR_STREAM(e.what());
        return;
    }

    if (node_entry == map_ptr_->end())
        ROS_ERROR("%s not found in map", selected_marker_name_.c_str());
    else
    {
        Ogre::Vector3 position;
        Ogre::Quaternion quaternion;
        getPose(position, quaternion);

        node_entry->second->setPosition(position);
        node_entry->second->setOrientation(quaternion);

        std::stringstream waypoint_name;
        waypoint_name << waypoint_name_prefix << node_entry->first;
        std::string waypoint_name_str(waypoint_name.str());

        visualization_msgs::InteractiveMarker int_marker;
        if(server_->get(waypoint_name_str, int_marker)) {
            int_marker.pose.position.x = position.x;
            int_marker.pose.position.y = position.y;
            int_marker.pose.position.z = position.z;

            int_marker.pose.orientation.x = quaternion.x;
            int_marker.pose.orientation.y = quaternion.y;
            int_marker.pose.orientation.z = quaternion.z;
            int_marker.pose.orientation.w = quaternion.w;

            server_->setPose(waypoint_name_str, int_marker.pose, int_marker.header);
        }
        server_->applyChanges();
    }
}

void WaypointWidget::frameChange() {
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
    QString new_frame = ui_->frame_input->text();

    // Only take action if the frame has changed.
    if ((new_frame != frame_id_)  && (new_frame != "")) {
        frame_id_ = new_frame;
        ROS_INFO("new frame: %s", frame_id_.toStdString().c_str());

        // update the frames for all interactive markers
        std::map<int, Ogre::SceneNode *>::iterator node_itr;
        for (node_itr = map_ptr_->begin(); node_itr != map_ptr_->end(); node_itr++) {
        std::stringstream waypoint_name;
        waypoint_name << "waypoint" << node_itr->first;
        std::string waypoint_name_str(waypoint_name.str());

        visualization_msgs::InteractiveMarker int_marker;
        if(server_->get(waypoint_name_str, int_marker)) {
            int_marker.header.frame_id = new_frame.toStdString();
            server_->setPose(waypoint_name_str, int_marker.pose, int_marker.header);
        }
    }
    server_->applyChanges();
  }
}

void WaypointWidget::topicChange() {
    QString new_topic = ui_->topic_input->text();

  // Only take action if the name has changed.
  if(new_topic != output_topic_) {
    path_publisher_.shutdown();
    output_topic_ = new_topic;
    if((output_topic_ != "") && (output_topic_ != "/")) {
      path_publisher_ = handler_.advertise<nav_msgs::Path>(output_topic_.toStdString(), 1);
    }
  }
}

void WaypointWidget::saveBag(const std::string& filename) {
    rosbag::Bag bag;
    try {
        bag.open(filename, rosbag::bagmode::Write);
    }
    catch (const rosbag::BagIOException& exception) {
        ROS_ERROR("could not open bag %s", filename.c_str());
        return;
    }

    nav_msgs::Path path;
    std::map<int, Ogre::SceneNode*>::iterator node_itr;
    for (node_itr = map_ptr_->begin(); node_itr != map_ptr_->end(); ++node_itr) {
        Ogre::Vector3 position;
        position = node_itr->second->getPosition();

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = position.x;
        pose.pose.position.y = position.y;
        pose.pose.position.z = position.z;

        Ogre::Quaternion quat;
        quat = node_itr->second->getOrientation();
        pose.pose.orientation.x = quat.x;
        pose.pose.orientation.y = quat.y;
        pose.pose.orientation.z = quat.z;
        pose.pose.orientation.w = quat.w;

        path.poses.push_back(pose);
    }
    path.header.frame_id = frame_id_.toStdString();

    bag.write("waypoints", ros::Time::now(), path);
    bag.close();
}

void WaypointWidget::loadBag(const std::string& filename) {
    rosbag::Bag bag;
    try {
        bag.open(filename, rosbag::bagmode::Read);
    } 
    catch (const rosbag::BagIOException& exception) {
        ROS_ERROR("could not open bag %s", filename.c_str());
        return;
    }

    std::vector<std::string> topics;
    topics.push_back(std::string("waypoints"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    BOOST_FOREACH (rosbag::MessageInstance const m, view) {
    nav_msgs::Path::ConstPtr p = m.instantiate<nav_msgs::Path>();
    if (p == nullptr) continue;
    ROS_INFO("n waypoints %zu", p->poses.size());

    for (size_t i = 0; i < p->poses.size(); i++) {
        geometry_msgs::PoseStamped pos = p->poses[i];
        Ogre::Vector3 position;
        position.x = pos.pose.position.x;
        position.y = pos.pose.position.y;
        position.z = pos.pose.position.z;

        Ogre::Quaternion quaternion;
        quaternion.x = pos.pose.orientation.x;
        quaternion.y = pos.pose.orientation.y;
        quaternion.z = pos.pose.orientation.z;
        quaternion.w = pos.pose.orientation.w;

        waypoint_tool_->makeItem(position, quaternion);
    }
  }
}

void WaypointWidget::setPose(const Ogre::Vector3& position, const Ogre::Quaternion& quaternion) {
    ui_->x_spinbox->blockSignals(true);
    ui_->y_spinbox->blockSignals(true);
    ui_->z_spinbox->blockSignals(true);
    ui_->yaw_spinbox->blockSignals(true);

    ui_->x_spinbox->setValue(position.x);
    ui_->y_spinbox->setValue(position.y);
    ui_->z_spinbox->setValue(position.z);

    tf::Quaternion qt(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    ui_->yaw_spinbox->setValue(tf::getYaw(qt));

    ui_->x_spinbox->blockSignals(false);
    ui_->y_spinbox->blockSignals(false);
    ui_->z_spinbox->blockSignals(false);
    ui_->yaw_spinbox->blockSignals(false);
}

void WaypointWidget::setConfig(QString topic, QString frame) {
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
    ui_->topic_input->blockSignals(true);
    ui_->frame_input->blockSignals(true);

    ui_->topic_input->setText(topic);
    ui_->frame_input->setText(frame);

    ui_->topic_input->blockSignals(false);
    ui_->frame_input->blockSignals(false);

    topicChange();
    frameChange();
}

void WaypointWidget::setWaypointLabel(Ogre::Vector3 position) {
    std::ostringstream string_stream;
    string_stream.precision(2);
    string_stream << selected_marker_name_;
    std::string label = string_stream.str();
    ui_->text_selected->setText(QString::fromStdString(label));
}

void WaypointWidget::setWaypointCount(int size) {
    std::ostringstream string_stream;
    string_stream << "Total waypoints: " << size;
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
    ui_->text_total->setText(
        QString::fromStdString(string_stream.str())
    );
}

void WaypointWidget::setSelectedMarkerName(std::string name) {
    selected_marker_name_ = name;
}

void WaypointWidget::getPose(Ogre::Vector3& position, Ogre::Quaternion& quaternion) {
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
    position.x = ui_->x_spinbox->value();
    position.y = ui_->y_spinbox->value();
    position.z = ui_->z_spinbox->value();
    double yaw = ui_->yaw_spinbox->value();

    tf::Quaternion qt = tf::createQuaternionFromYaw(yaw);
    quaternion.x = qt.x();
    quaternion.y = qt.y();
    quaternion.z = qt.z();
    quaternion.w = qt.w();
}

QString WaypointWidget::getFrameId() {
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
    return frame_id_;
}

QString WaypointWidget::getOutputTopic() {
    boost::mutex::scoped_lock lock(frame_updates_mutex_);
    return output_topic_;
}

}
