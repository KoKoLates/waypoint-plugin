/**
 * @file waypoint_tool.h
 * @author KoKoLates (the21515@gmail.com)
 * @version 0.1
 * @date 2023-06-19
 */

#pragma once

#include <rviz/tool.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/menu_handler.h>
#include <interactive_markers/interactive_marker_server.h>

#include "waypoint_widget.h"

namespace rviz {
    class VectorProperty;
    class ViewportMouseEvent;
    class VisualizationManager;
    class PanelDockWidget;
} // namespace rviz

namespace Ogre {
    class Vector3;
    class SceneNode;
} // namespace Ogre


namespace waypoint_plugin {

class WaypointTool: public rviz::Tool {
    Q_OBJECT
public:
    WaypointTool();
    ~WaypointTool();

    // virtual function
    virtual void activate();
    virtual void deactivate();
    virtual void onInitialize();
    virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

    void makeItem(const Ogre::Vector3&, const Ogre::Quaternion&);

    virtual void load(const rviz::Config& config);
    virtual void save(rviz::Config config) const;

private:
    void getMarkerPose();
    void clearAllMarker();
    void processFeedBack(const visualization_msgs::InteractiveMarkerFeedbackConstPtr&);

    Ogre::SceneNode *move_axis_node_;
    std::string axis_resource_;

    // waypoint plugin Qt widget
    WaypointWidget *widget_;
    rviz::PanelDockWidget *widget_dock_;

    // interactive marker
    interactive_markers::InteractiveMarkerServer server_;
    interactive_markers::MenuHandler menu_handler_;

    // stores waypoints based on unique name
    typedef std::map<int, Ogre::SceneNode*> str2nodeptr;
    str2nodeptr node_map_;

    // index
    int unique_idx_; 
};
}
