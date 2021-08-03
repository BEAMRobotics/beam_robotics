
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <boost/foreach.hpp>
#include <gflags/gflags.h>

#include <beam_utils/gflags.h>
#include <beam_utils/math.h>

DEFINE_string(bag_in, "", "Full path to input bag file (Required)");
DEFINE_validator(bag_in, &beam::gflags::ValidateBagFileMustExist);
DEFINE_string(bag_out, "", "Full path to output bag file (Required)");
DEFINE_validator(bag_out, &beam::gflags::ValidateCannotBeEmpty);
DEFINE_string(topic_in, "", "Topic to convert (Required)");
DEFINE_validator(topic_in, &beam::gflags::ValidateCannotBeEmpty);
DEFINE_string(topic_out, "", "Output topic (Required)");
DEFINE_validator(topic_out, &beam::gflags::ValidateCannotBeEmpty);
DEFINE_string(child_frame_id, "",
              "child frame id for odometry message. PoseStamped only has "
              "frame_id which is the baselink frame. The child frame id should "
              "be the fixed frame (map or world or odom).(Required)");
DEFINE_validator(child_frame_id, &beam::gflags::ValidateCannotBeEmpty);
DEFINE_bool(invert_pose, false,
            "Set to true to invert the pose before saving to the new topic. "
            "(Optional - default: false)");

struct Params {
  std::string bag_in;
  std::string bag_out;
  std::string topic_in;
  std::string topic_out;
  std::string child_frame_id;
  bool invert_pose;
};

void ConvertMessages(const Params& params) {
  rosbag::Bag bag_in;
  rosbag::Bag bag_out;
  BEAM_INFO("Opening input bag: {}", params.bag_in);
  bag_in.open(params.bag_in, rosbag::bagmode::Read);
  BEAM_INFO("Opening output bag: {}", params.bag_out);
  bag_out.open(params.bag_out, rosbag::bagmode::Write);

  BEAM_INFO("Looking for topic: {}", params.topic_in);
  BEAM_INFO("Saving conversion to topic: {}", params.topic_out);

  rosbag::View view(bag_in);
  int counter = 0;
  BOOST_FOREACH (rosbag::MessageInstance const msg, view) {
    bag_out.write(msg.getTopic(), msg.getTime(), msg);

    if (msg.getTopic() == params.topic_in) {
      auto pose_stamped_msg = msg.instantiate<geometry_msgs::PoseStamped>();

      if (pose_stamped_msg == NULL) {
        BEAM_ERROR(
            "Cannot instantiate geometry_msgs::PoseStamped message. Check that "
            "the input topic is of correct type.");
        continue;
      }

      counter++;
      nav_msgs::Odometry odometry_msg;
      odometry_msg.header = pose_stamped_msg->header;
      odometry_msg.child_frame_id = params.child_frame_id;
      if (params.invert_pose) {
        Eigen::Quaterniond q;
        q.x() = pose_stamped_msg->pose.orientation.x;
        q.y() = pose_stamped_msg->pose.orientation.y;
        q.z() = pose_stamped_msg->pose.orientation.z;
        q.w() = pose_stamped_msg->pose.orientation.w;
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d R(q);
        T.block(0, 0, 3, 3) = R;
        T(0, 3) = pose_stamped_msg->pose.position.x;
        T(1, 3) = pose_stamped_msg->pose.position.y;
        T(2, 3) = pose_stamped_msg->pose.position.z;
        Eigen::Matrix4d T_inv = beam::InvertTransform(T);
        Eigen::Matrix3d R_inv = T_inv.block(0, 0, 3, 3);
        Eigen::Quaterniond q_inv(R_inv);
        odometry_msg.pose.pose.position.x = T_inv(0, 3);
        odometry_msg.pose.pose.position.y = T_inv(1, 3);
        odometry_msg.pose.pose.position.x = T_inv(2, 3);

        odometry_msg.pose.pose.orientation.x = q_inv.x();
        odometry_msg.pose.pose.orientation.y = q_inv.y();
        odometry_msg.pose.pose.orientation.z = q_inv.z();
        odometry_msg.pose.pose.orientation.w = q_inv.w();
      } else {
        odometry_msg.pose.pose = pose_stamped_msg->pose;
      }
      bag_out.write(params.topic_out, msg.getTime(),
                    boost::make_shared<nav_msgs::Odometry>(odometry_msg));
    }
  }

  BEAM_INFO(
      "Done converting messages. Total converted messages: {}. Results have "
      "been written to: {}",
      counter, bag_out.getFileName());
  bag_in.close();
  bag_out.close();
}

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  Params params;
  params.bag_in = FLAGS_bag_in;
  params.bag_out = FLAGS_bag_out;
  params.topic_in = FLAGS_topic_in;
  params.topic_out = FLAGS_topic_out;
  params.child_frame_id = FLAGS_child_frame_id;
  params.invert_pose = FLAGS_invert_pose;

  ConvertMessages(params);

  return 0;
}
