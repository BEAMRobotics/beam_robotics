#include <rosbag_tools/RosBagIO.h>
#include <rosbag_tools/VelodyneTools.h>

#include <gflags/gflags.h>

#include <beam_utils/filesystem.h>
#include <beam_utils/gflags.h>

DEFINE_string(input, "", "Full path to input bag file (Required)");
DEFINE_validator(input, &beam::gflags::ValidateBagFileMustExist);
DEFINE_string(output_postfix, "_unpacked",
              "postfix to apply to new lidar topic, and new bag (Optional)");
DEFINE_string(lidar_model, "VLP16",
              "Options: VLP16, 32C, 32E, VLS128. (Optional)");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  using namespace rosbag_tools;

  VelodyneTools velodyne_tools(FLAGS_lidar_model);

  boost::filesystem::path p(FLAGS_input);

  std::string parent_path = p.parent_path().string();
  std::string filename = p.stem().string();
  std::string output_path =
      beam::CombinePaths(parent_path, filename + FLAGS_output_postfix + ".bag");

  RosBagReader reader(FLAGS_input);
  RosBagWriter writer(output_path);

  while (true) {
    rosbag::View::iterator iter;
    if (!reader.GetNextMsg(iter)) { break; }
    velodyne_msgs::VelodyneScan::ConstPtr msg =
        iter->instantiate<velodyne_msgs::VelodyneScan>();
    if (msg) {
      sensor_msgs::PointCloud2 cloud = velodyne_tools.UnpackScan(msg);
      cloud.header = msg->header;
      writer.AddMsg(iter->getTopic() + FLAGS_output_postfix, iter->getTime(),
                    cloud);
    } else {
      writer.AddMsg(*iter);
    }
  }
  writer.CloseBag();
  return 0;
}
