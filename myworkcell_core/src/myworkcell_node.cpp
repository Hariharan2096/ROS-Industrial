#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>

class ScanNPlan
{
private:
    ros::ServiceClient vision_client_;
    
public:
    ScanNPlan(ros::NodeHandle& nh)
    {
        vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
    }

    void start()
    {
        ROS_INFO("Attempting to localize part");

        myworkcell_core::LocalizePart srv;
        if (!vision_client_.call(srv))
        {
            ROS_ERROR("Could not localize part");
            return;
        }

        ROS_INFO_STREAM("part localized: "<< srv.response);
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "myworkcell_node");
    ros::NodeHandle nh;
    
    ROS_INFO("ScanNPlan has been initialized");

    ScanNPlan app(nh);

    ros::Duration(.5).sleep();
    app.start();

    ros::spin();
}