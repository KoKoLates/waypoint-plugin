/**
 * @file waypoint_widget.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-06-17
 * 
 * @copyright Copyright (c) 2023
 * 
 */

# ifndef RVIZ_PLUGIN_WAYPOINT_WIDGET
# define RVIZ_PLUGIN_WAYPOINT_WIDGET

# ifndef Q_MOC_RUN
# include <boost/thread/mutex.hpp>
# endif

# include <QWidget>

# include "ros/ros.h"
# include <nav_msgs/Path.h>

# include "ui_waypoint_plugin.h"

namespace Ogre {
    class Vector3;
    class Quaternion;
    class SceneNode;
    class SceneManager;
}

namespace rviz {
    class DisplayContext;
}

namespace interactive_markers {
    class InteractiveMarkerServer;
}

namespace Ui {
    // the Qt widget class
    class PluginWidget;
}

namespace waypoint_plugin {
    class WaypointTool;
}

namespace waypoint_plugin {

constexpr char waypoint_name_prefix[] = "waypoint#";

class WaypointWidget: public QWidget {
    friend class WaypointTool;
    Q_OBJECT

    public: 
    WaypointWidget (
        rviz::DisplayContext* context, 
        std::map<int, Ogre::SceneNode*>* map_ptr, 
        interactive_markers::InteractiveMarkerServer* server, 
        int* unique_idx, 
        QWidget* parent=0,
        WaypointTool* waypoint_tool=0
    );
    ~WaypointWidget();

    void enable();
    void disable();
    
    // setter
    void setPose(const Ogre::Vector3& position, const Ogre::Quaternion& quaternion);
    void setConfig(QString topic, QString frame, float height);
    void setWaypointLabel(Ogre::Vector3 position);
    void setWaypointCount(int size);
    void setSelectedMarkerName(std::string name);

    // getter
    void getPose(Ogre::Vector3& position, Ogre::Quaternion& quaternion);
    QString getFrameId();
    QString getOutputTopic();

    protected:
    Ui::PluginWidget* ui_;
    rviz::DisplayContext* context_;

    private:
    ros::NodeHandle handler_;
    ros::Publisher path_publisher_;

    WaypointTool* waypoint_tool_;
    std::map<int, Ogre::SceneNode* >* map_ptr_;
    Ogre::SceneManager* scene_manager_;
    interactive_markers::InteractiveMarkerServer* server_;
    int* unique_idx_;

    // the current name of topic and frame
    QString output_topic_;
    QString frame_id_;

    //mutex for changes of variables
    boost::mutex frame_updates_mutex_;

    std::string selected_marker_name_;

    // save and load private function
    void saveBag(const std::string& filename);
    // void saveYaml(const std::string& filename);
    // void saveJson(const std::string& filename);

    void loadBag(const std::string& filename);
    // void loadYaml(const std::string& filename);
    // void loadJson(const std::string& filename);

    private Q_SLOTS:
    void saveHandler();
    void loadHandler();
    void clearHandler();
    void publishHandler();
    void frameChange();
    void topicChange();
    void poseChange(double value);
};
}

# endif // RVIZ_PLUGIN_WAYPOINT_WIDGET
