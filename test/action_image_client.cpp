#include <actionlib/client/simple_action_client.h>
#include <camera_control_msgs/GrabImagesAction.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<camera_control_msgs::GrabImagesAction>
    ActionImageActionClient;
typedef boost::movelib::unique_ptr<ActionImageActionClient> ActionImageActionClientPtr;

class ActionImageClient {
 public:
  ActionImageClient();
  void sendGoal();

 private:
  ros::NodeHandle nh_;
  ActionImageActionClientPtr action_client_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_publisher_;
};

ActionImageClient::ActionImageClient() : it_(nh_) {
  image_publisher_ = it_.advertise("debug_images", 1);

  action_client_ = boost::movelib::make_unique<ActionImageActionClient>(
      "rgb/grab_images", true);
  action_client_->waitForServer();
}

void ActionImageClient::sendGoal() {
  camera_control_msgs::GrabImagesGoal goal;
  goal.exposure_auto = true;
  // goal.exposure_given = true;
  goal.exposure_times.push_back(10);
  goal.exposure_times.push_back(50);
  goal.exposure_times.push_back(100);
  goal.exposure_times.push_back(150);
  goal.exposure_times.push_back(200);
  goal.exposure_times.push_back(300);

  action_client_->sendGoal(goal);
  action_client_->waitForResult(ros::Duration(5));

  if (action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO_STREAM("ActionImageClient: Goal succeeded");
  else
    ROS_WARN_STREAM("ActionImageClient: Goal failed ("
                    << action_client_->getState().state_
                    << "): " << action_client_->getState().getText());

  camera_control_msgs::GrabImagesResultConstPtr result =
      action_client_->getResult();
  for (size_t i = 0; i < result->images.size(); ++i) {
    ros::Duration(1).sleep();
    ROS_INFO_STREAM("Publishing image " << i << " with an exposure of "
                                        << result->reached_exposure_times[i]);
    image_publisher_.publish(result->images[i]);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "action_image_client");
  ActionImageClient client;
  client.sendGoal();
  return 0;
}
