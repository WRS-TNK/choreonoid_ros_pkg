#include "BodyRosDiffDriveControllerItem.h"
#include <cnoid/BodyItem>
#include <cnoid/Link>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

using namespace cnoid;
using namespace std;

void BodyRosDiffDriveControllerItem::initialize(ExtensionManager* ext)
{
    static bool initialized = false;
    int argc = 0;
    char** argv;
    if (!ros::isInitialized())
        ros::init(argc, argv, "choreonoid");
    if(!initialized) {
        ext->itemManager().registerClass<BodyRosDiffDriveControllerItem>("BodyRosDiffDriveControllerItem");
        ext->itemManager().addCreationPanel<BodyRosDiffDriveControllerItem>();
        initialized = true;
    }
}

BodyRosDiffDriveControllerItem::BodyRosDiffDriveControllerItem() :
    os_(MessageView::instance()->cout())
{
    ROS_INFO("YOU ARE HERE");
    wheel_name_.resize(2);
    track_.resize(2);
    tread_ = 0.0;
    controllerTarget_ = NULL;
    ROS_INFO("YOU ARE END HERE");
}

BodyRosDiffDriveControllerItem::BodyRosDiffDriveControllerItem(const BodyRosDiffDriveControllerItem& org) :
    ControllerItem(org),
    os_(MessageView::instance()->cout())
{
    wheel_name_.resize(2);
    track_.resize(2);
    tread_ = org.tread_;
    wheel_name_ = org.wheel_name_;
    controllerTarget_ = NULL;
}

BodyRosDiffDriveControllerItem::~BodyRosDiffDriveControllerItem()
{
    stop();
}

Item* BodyRosDiffDriveControllerItem::doDuplicate() const
{
    return new BodyRosDiffDriveControllerItem(*this);
}

bool BodyRosDiffDriveControllerItem::store(Archive& archive)
{
    archive.write("Tread", tread_);
    archive.write("LeftWheelLinkName", wheel_name_[0]);
    archive.write("RightWheelLinkName", wheel_name_[1]);

    return true;
}

bool BodyRosDiffDriveControllerItem::restore(const Archive& archive)
{
    archive.read("Tread", tread_);
    archive.read("LeftWheelLinkName", wheel_name_[0]);
    archive.read("RightWheelLinkName", wheel_name_[1]);

    return true;
}

void BodyRosDiffDriveControllerItem::doPutProperties(PutPropertyFunction& putProperty)
{
    putProperty.min(0.0)("Wheel tread", tread_, changeProperty(tread_));
    putProperty("Left Wheel's Link name", wheel_name_[0], changeProperty(wheel_name_[0]));
    putProperty("Right Wheel's Link name", wheel_name_[1], changeProperty(wheel_name_[1]));
}

bool BodyRosDiffDriveControllerItem::initialize(Target* target)
{

    ROS_INFO("Initialize start");
    if(!target) {
        MessageView::instance()->putln(MessageView::ERROR, "Target not found");
        return false;
    } else if(!target->body()) {
        MessageView::instance()->putln(MessageView::ERROR, "BodyItem not found");
        return false;
    }
    
    controllerTarget_ = target;
    simulationBody_ = target->body();
    timeStep_ = target->worldTimeStep();
    controlTime_ = target->currentTime();

    ROS_INFO("Initialize end");
    
    return true;
}

bool BodyRosDiffDriveControllerItem::start()
{

    ROS_INFO("start start");
    // check if the parameter is initialized
    if(tread_==0.0) {
        MessageView::instance()->putln(MessageView::ERROR, "Put tread value !");
        return false;
    } else if(wheel_name_[0]=="" || wheel_name_[1]=="") {
        MessageView::instance()->putln(MessageView::ERROR, "Put Wheel Link names !");
        return false;
    }
    for(size_t i=0; i<track_.size(); i++){
        track_[i] = simulationBody_->link(wheel_name_[i].c_str());
        track_[i]->setActuationMode(track_[i]->actuationMode());
    }

    
    
    string name = simulationBody_->name();
    replace(name.begin(), name.end(), '-', '_');
    rosnode_ = shared_ptr<ros::NodeHandle>(new ros::NodeHandle(name));

    twist_subscriber_ = rosnode_->subscribe("/command/velocity", 1, &BodyRosDiffDriveControllerItem::twistCallback, this);
    
    async_ros_spin_.reset(new ros::AsyncSpinner(0));
    async_ros_spin_->start();

    return true;
}

void BodyRosDiffDriveControllerItem::stop()
{
    if(ros::ok()){
        if(async_ros_spin_) 
            async_ros_spin_->stop();

        if(rosnode_)
            rosnode_->shutdown();
    }
}

void BodyRosDiffDriveControllerItem::twistCallback(const geometry_msgs::Twist& msg)
{
    lock_guard<mutex> lock(twistMutex_);
    cmd_twist_ = msg;
}

vector<double> BodyRosDiffDriveControllerItem::calcCMDVel(const double linear, const double angular)
{
    vector<double> result(2,0);
    result[0] = linear - (tread_/2.0)*angular;
    result[1] = linear + (tread_/2.0)*angular;

    return result;            
}

bool BodyRosDiffDriveControllerItem::control()
{
    vector<double> cmd(2);
    {
        lock_guard<mutex> lock(twistMutex_);
        cmd = calcCMDVel(cmd_twist_.linear.x, cmd_twist_.angular.z);
    }
    
    for(size_t i=0; i<track_.size(); i++)
        track_[i]->dq_target() = cmd[i];

    return true;
}

void BodyRosDiffDriveControllerItem::input()
{
    
}

void BodyRosDiffDriveControllerItem::output()
{
    
}
