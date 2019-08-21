#ifndef CNOID_ROS_PLUGIN_BODY_ROS_DIFF_DRIVE_CONTROLLER_ITEM_H_INCLUDED
#define CNOID_ROS_PLUGIN_BODY_ROS_DIFF_DRIVE_CONTROLLER_ITEM_H_INCLUDED

#include <cnoid/ControllerItem>
#include <cnoid/Body>
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/Archive>
#include <cnoid/ItemManager>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include "exportdecl.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

#include <vector>
#include <fstream>
#include <mutex>


using namespace std;

namespace cnoid {

class CNOID_EXPORT BodyRosDiffDriveControllerItem : public ControllerItem
{
public:
    static void initialize(ExtensionManager* ext);

    BodyRosDiffDriveControllerItem();
    BodyRosDiffDriveControllerItem(const BodyRosDiffDriveControllerItem& org);
    virtual ~BodyRosDiffDriveControllerItem();
    
    virtual bool initialize(Target* target);
    virtual double timeStep() const { return timeStep_; };
    virtual bool start();
    virtual void stop();
    virtual void input() { };
    virtual void output() { };
    virtual bool control();

    const BodyPtr& body() const { return simulationBody_; };

    double controlTime() const { return controlTime_; };

    void setModuleName(const string& name);

protected:
    virtual Item* doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
    void doPutProperties(PutPropertyFunction& putProperty);
    
private:
    BodyPtr simulationBody_;
    double timeStep_;

    const Target* controllerTarget_;
    double controlTime_;
    ostream& os_;

    double tread_;
    vector<string> wheel_name_;
    vector<double> wheel_radius_;
    vector<Link*> track_;

    shared_ptr<ros::NodeHandle> rosnode_;
    shared_ptr<ros::AsyncSpinner> async_ros_spin_;

    ros::Subscriber twist_subscriber_;
    geometry_msgs::Twist cmd_twist_;

    mutex twistMutex_;
    

    void twistCallback(const geometry_msgs::Twist& msg);

    vector<double> calcCMDVel(const double linear, const double angular);

}; // class BodyRosDiffDriveControllerItem

typedef ref_ptr<BodyRosDiffDriveControllerItem> BodyRosDiffDriveControllerItemPtr;

} // namespace cnoid

#endif
