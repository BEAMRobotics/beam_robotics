#include <rosbag_tools/RosBagIO.h>

#include <beam_utils/log.h>

namespace rosbag_tools {

RosBagReader::RosBagReader(const std::string& bag_file_path,
                           const std::vector<std::string>& topics,
                           const ros::Time& start_time,
                           const ros::Time& end_time)
    : bag_file_path_(bag_file_path) {
  // open bag
  BEAM_INFO("Opening bag file: {}", bag_file_path_);
  bag_.open(bag_file_path_, rosbag::bagmode::Read);

  // create view
  if (!topics.empty()) {
    view_ = std::make_unique<rosbag::View>(bag_, rosbag::TopicQuery(topics),
                                           start_time, end_time, true);
  } else {
    view_ = std::make_unique<rosbag::View>(bag_, start_time, end_time, true);
  }

  if (view_->size() == 0) {
    BEAM_CRITICAL(
        "No messages read. Check your input topics, and start & end times.");
    throw std::runtime_error{"Empty rosbag view"};
  }

  // set iterators
  iter_ = view_->begin();
  iter_velodyne_ = view_->begin();
  iter_image_ = view_->begin();
}

bool RosBagReader::GetNextMsg(rosbag::View::iterator& iter) {
  if (iter_ == view_->end()) { return false; }
  iter = iter_;

  // increment for next time
  iter_++;

  return true;
}

boost::shared_ptr<sensor_msgs::Image> RosBagReader::GetNextImageMsg() {
  while (true) {
    if (iter_image_ == view_->end()) { return nullptr; }
    boost::shared_ptr<sensor_msgs::Image> msg =
        iter_image_->instantiate<sensor_msgs::Image>();
    if (msg != nullptr) {
      // increment for next time
      iter_image_++;
      return msg;
    }

    // else, wrong message type so keep iterating
    iter_image_++;
  }
}

boost::shared_ptr<velodyne_msgs::VelodyneScan>
    RosBagReader::GetNextVelodyneMsg() {
  while (true) {
    if (iter_velodyne_ == view_->end()) { return nullptr; }
    boost::shared_ptr<velodyne_msgs::VelodyneScan> msg =
        iter_velodyne_->instantiate<velodyne_msgs::VelodyneScan>();
    if (msg != nullptr) {
      // increment for next time
      iter_velodyne_++;
      return msg;
    }

    // else, wrong message type so keep iterating
    iter_velodyne_++;
  }
}

RosBagWriter::RosBagWriter(const std::string& bag_file_path)
    : bag_file_path_(bag_file_path) {
  BEAM_INFO("Saving new bag to: {}", bag_file_path_);
  bag_.open(bag_file_path_, rosbag::bagmode::Write);
}

void RosBagWriter::AddMsg(const rosbag::MessageInstance& msg) {
  bag_.write(msg.getTopic(), msg.getTime(), msg);
}

void RosBagWriter::AddMsg(const std::string& topic, const ros::Time& stamp,
                          const sensor_msgs::PointCloud2& cloud) {
  bag_.write(topic, stamp, cloud);
}

void RosBagWriter::AddMsg(const std::string& topic, const ros::Time& stamp,
                          const sensor_msgs::Image& image) {
  bag_.write(topic, stamp, image);
}

void RosBagWriter::AddMsg(const std::string& topic, const ros::Time& stamp,
                          const sensor_msgs::CompressedImage& image) {
  bag_.write(topic, stamp, image);
}

void RosBagWriter::CloseBag() {
  bag_.close();
  BEAM_INFO("Finished writing bag to: {}", bag_file_path_);
}

} // namespace rosbag_tools
