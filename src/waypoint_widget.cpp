/**
 * @file waypoint_widget.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-18
 * 
 * @copyright Copyright (c) 2023
 * 
 */

# include <rosbag/bag.h>
# include <rosbag/view.h>
# include <rviz/display_context.h>
# include <interactive_markers/interactive_marker_server.h>

# include "waypoint_tool.h"
# include <tf/tf.h>
# include <QFileDialog>
# include <boost/foreach.hpp>
# define foreach BOOST_FOREACH

# include <OGRE/OgreSceneNode.h>
# include <OGRE/OgreSceneManager.h>

namespace waypoint_plugin {

struct MissionKeywords {
    inline static const std::string KPosition = "position";
};

WaypointWidget::WaypointWidget (
    rviz::DisplayContext* context, 
    std::map<int, Ogre::SceneNode* >* map_ptr, 
    interactive_markers::InteractiveMarkerServer* server, 
    int* unique_idx, 
    QWidget* parent=0, 
    WaypointTool* waypoint_tool=0): QWidget(parent), 
    context_(context), 
    ui_(new Ui::PluginWidget()), 
    map_ptr_(map_ptr_), 
    unique_idx_(unique_idx), 
    server_(server), 
    frame_id_("map"), 
    waypoint_tool_(waypoint_tool), 
    selected_marker_name_(std::string(waypoint_name_prefix) + "1") 
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

/**
 * @brief 
 * 
 */
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

/**
 * @brief 
 * 
 */
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

/**
 * @brief 
 * 
 */
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

/**
 * @brief 
 * 
 */
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

        path.poses.push_back(pos);
    }
    path.header.frame_id = frame_id_.toStdString();
    path_publisher_.publish(path);
}

/**
 * @brief 
 * 
 * @param value 
 */
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
        ndoe_entry->second->setOrientation(quaternion);

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

}

void WaypointWidget::topicChange() {

}

/**
 * @brief 
 * 
 * @param filename 
 */
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

/**
 * @brief 
 * 
 * @param filename 
 */
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

}
