#include <boost/foreach.hpp>
#include <gflags/gflags.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>

#include <beam_utils/gflags.h>
#include <beam_utils/log.h>

DEFINE_string(bag_file_path, "", "Full path to bag file (Required).");
DEFINE_validator(bag_file_path, &beam::gflags::ValidateBagFileMustExist);
DEFINE_string(output, "",
              "Full path to output bag file.(e.g., /path/to/bag_name.bag)");
DEFINE_validator(output, &beam::gflags::ValidateCannotBeEmpty);
DEFINE_string(topic, "", "Topic whose messages you want to edit.");
DEFINE_validator(topic, &beam::gflags::ValidateCannotBeEmpty);
DEFINE_double(offset, 0, "Time offset to add in seconds.");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  rosbag::Bag bag_in;
  rosbag::Bag bag_out;
  ros::Duration d(FLAGS_offset);

  // load bag file
  try {
    BEAM_INFO("Opening ROS bag: {}", FLAGS_bag_file_path);
    bag_in.open(FLAGS_bag_file_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException& ex) {
    BEAM_ERROR("Bag exception : %s", ex.what());
    throw std::runtime_error{"Unable to open bag."};
  }

  // open output bag
  try {
    bag_out.open(FLAGS_output, rosbag::bagmode::Write);
  } catch (rosbag::BagException& ex) {
    BEAM_ERROR("Bag exception : %s", ex.what());
    throw std::runtime_error{"Unable to open output bag."};
  }

  rosbag::View view(bag_in);
  BEAM_INFO("Copying bag and saving new timestamps...");
  BOOST_FOREACH (rosbag::MessageInstance const msg, view) {
    if (msg.getTopic() == FLAGS_topic) {
      ros::Time outtime = msg.getTime() + d;
      auto image_msg = msg.instantiate<sensor_msgs::Image>();
      if (image_msg == NULL) {
        BEAM_WARN("Cannot instantiate image message.");
        continue;
      }
      image_msg->header.stamp = outtime;
      bag_out.write(msg.getTopic(), image_msg->header.stamp, image_msg);
    } else {
      bag_out.write(msg.getTopic(), msg.getTime(), msg);
    }
  }

  bag_in.close();
  bag_out.close();
  BEAM_INFO("Done.");

  return 0;
}
