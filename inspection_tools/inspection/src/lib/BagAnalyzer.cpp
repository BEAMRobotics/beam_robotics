#include "inspection/BagAnalyzer.h"

// Load all tf messages into a queryable TF object

namespace inspection {

BagAnalyzer::BagAnalyzer(std::string& bag_file_path)
    : bag_file_(bag_file_path) {}

void BagAnalyzer::LoadBag(const std::string& bag_file) {
  rosbag::Bag bag;

  bag.open("/home/steve/roben_scan_2019-04-11-20-09-55.bag",
           rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.emplace_back("/odometry/filtered2");
  topics.emplace_back(std::string("numbers"));

  rosbag::View view = {bag, rosbag::TopicQuery(topics)};

  //  view.addQuery(bag, func2);
  view.addQuery(bag, [](rosbag::ConnectionInfo const* ptr) {
    return ptr->topic == "/odometry/filtered";
  });

  /*  std::cout << "View size = " << view.size() << std::endl;
    std::cout << "View" << view.getBeginTime() << std::endl;*/

  // Instantiate every message in the view
  for (const auto& m : view) {
    //    std::cout << m.getTime() << std::endl;

    auto message = m.instantiate<nav_msgs::Odometry>();
    if (message->twist.twist.linear.x < 0.0001) { messages_.emplace_back(m); }
  }

  // Check size of new vector
  /*  std::cout << "Size of filtered view = " << messages_.size() << std::endl;
    std::cout << "Sizeof() filtered view = " << sizeof(messages_) <<
    std::endl;*/

  for (size_t i = 0; i < 10; i++) {
    auto header_message = messages_[i].instantiate<std_msgs::Header>();
    auto odom_message = messages_[i].instantiate<nav_msgs::Odometry>();
    /*
        if (header_message != nullptr)
          std::cout << "Header stamp = " << header_message->stamp << std::endl;
        std::cout << "Odom Header stamp = " << odom_message->header.stamp
                  << std::endl;*/
  }
  bag.close();
}

std::unique_ptr<tf2::BufferCore> BagAnalyzer::GetTF2Buffer() {
  //  rosbag::Bag bag_file_{"/home/steve/roben_scan_2019-04-11-20-09-55.bag"};

  //  bag.open("/home/steve/roben_scan_2019-04-11-20-09-55.bag",
  //           rosbag::bagmode::Read);

  std::vector<std::string> topics;
  //  topics.emplace_back("/tf");
  topics.emplace_back("/tf_static");

  rosbag::View tf_view = {bag_file_, rosbag::TopicQuery("/tf")};

  /*  std::cout << "View size = " << tf_view.size() << std::endl;
    std::cout << "View begin time = " << tf_view.getBeginTime() << std::endl;
    std::cout << "View end time = " << tf_view.getEndTime() << std::endl;*/

  ros::Duration t = {tf_view.getEndTime() - tf_view.getBeginTime()};
  //    std::cout << "Duration = " << t.sec << std::endl;

  auto buffer = std::make_unique<tf2::BufferCore>(t);

  // Store "tf" transforms in buffer
  for (const auto& msg_instance : tf_view) {
    auto tf_message = msg_instance.instantiate<tf2_msgs::TFMessage>();
    if (tf_message != nullptr) {
      for (const auto& tf : tf_message->transforms) {
        if (!buffer->setTransform(tf, "Auth"))
          std::cout << "Adding message failed" << std::endl;
      }
    }
  }

  rosbag::View tf_static_view = {bag_file_, rosbag::TopicQuery("/tf_static")};
  // Store "tf_static" transforms in buffer
  for (const auto& msg_instance : tf_static_view) {
    auto tf_message = msg_instance.instantiate<tf2_msgs::TFMessage>();
    if (tf_message != nullptr) {
      for (const auto& tf : tf_message->transforms) {
        if (!buffer->setTransform(tf, "Auth", true))
          std::cout << "Adding message failed" << std::endl;
      }
    }
  }

  /*
     if (buffer->canTransform("rot_laser_link", "rot_laser_optical",
                              tf_view.getEndTime() - ros::Duration(100))) {
       auto tf = buffer->lookupTransform("rot_laser_link", "rot_laser_optical",
                                         tf_view.getEndTime() -
 ros::Duration(100)); std::cout << "Tf = " << tf << std::endl; } else std::cout
 << "Can't transform 1" << std::endl;

     if (buffer->canTransform("base_link", "odom",
                              tf_view.getEndTime() - ros::Duration(100))) {
       auto tf = buffer->lookupTransform("base_link", "odom",
                                         tf_view.getEndTime() -
 ros::Duration(100)); std::cout << "Tf = " << tf << std::endl; } else std::cout
 << "Can't transform 2" << std::endl;


   std::vector<std::string> frames = {};
   buffer->_getFrameStrings(frames);
   for (const auto& frame : frames) {
 //    std::cout << "Frame = " << frame << std::endl;
   }*/

  return buffer;
}

std::vector<std::vector<rosbag::MessageInstance>> BagAnalyzer::GetPointClouds(
    std::vector<std::pair<ros::Time, ros::Time>> regions_of_interest) {
  std::vector<std::string> topics;
  topics.emplace_back("/m3d/velodyne_points");
  rosbag::View pc_view = {bag_file_, rosbag::TopicQuery(topics)};

  ROS_INFO("Size of pc_view: %u", pc_view.size());

  std::vector<std::vector<rosbag::MessageInstance>> pc_msgs{
      regions_of_interest.size()};

  for (const auto& msg_instance : pc_view) {
    ros::Time msg_time = msg_instance.getTime();
    for (size_t i = 0; i < regions_of_interest.size(); i++) {
      auto roi = regions_of_interest[i];
      if (msg_time >= roi.first && msg_time <= roi.second) {
        pc_msgs[i].push_back(msg_instance);
        break;
      }
    }
  }

  //  for (const auto& pc : pc_msgs) { std::cout << pc.size() << std::endl; }
  return pc_msgs;
}

std::vector<std::pair<ros::Time, ros::Time>>
    BagAnalyzer::GetROIs(float threshold) {
  rosbag::View odom_view = {bag_file_,
                            rosbag::TopicQuery("/odometry/filtered")};

  /*  std::cout << "Odom view size = " << odom_view.size() << std::endl;
    std::cout << "Odom view start = " << odom_view.getBeginTime() <<
    std::endl;*/

  std::vector<std_msgs::Header> odom_msg_headers = {};
  std::queue<std_msgs::Header> odom_msg_headers_q = {};

  HighResolutionTimer timer;
  // Instantiate every message in the view
  for (const auto& odom_msg_inst : odom_view) {
    auto odom_msg = odom_msg_inst.instantiate<nav_msgs::Odometry>();
    if (odom_msg->twist.twist.linear.x < threshold) {
      odom_msg_headers.push_back(odom_msg->header);
      odom_msg_headers_q.push(odom_msg->header);
    }
  }
  /*  std::cout << "Checking messages took: " << timer.elapsed() << " seconds."
              << std::endl;*/

  timer.restart();
  // Now break vector into sections
  std::pair<ros::Time, ros::Time> current_roi = {};
  std::pair<int, int> current_roi_seq = {};
  for (size_t i = 0; i < odom_msg_headers.size(); i++) {
    if (i == 0) {
      current_roi.first = odom_msg_headers[i].stamp;
      current_roi_seq.first = odom_msg_headers[i].seq;
      continue;
    }

    if (odom_msg_headers[i].seq == (odom_msg_headers[i - 1].seq + 1)) {
      current_roi.second = odom_msg_headers[i].stamp;
      current_roi_seq.second = odom_msg_headers[i].seq;
    }
  }

  /*
    std::cout << "Current ROI: Start Time = " << current_roi.first
              << ", Sequence ID = " << current_roi_seq.first << std::endl;
    std::cout << "Current ROI: End Time = " << current_roi.second
              << ", Sequence ID = " << current_roi_seq.second << std::endl;
    std::cout << "Getting ROIS took: " << timer.elapsed()
              << " seconds with vector method." << std::endl;
  */

  timer.restart();

  std::vector<std::pair<std_msgs::Header, std_msgs::Header>> roi_headers = {};

  std::pair<std_msgs::Header, std_msgs::Header> current_roi_header = {};
  current_roi_header.first = odom_msg_headers_q.front();
  current_roi_header.second = odom_msg_headers_q.front();
  odom_msg_headers_q.pop();
  while (!odom_msg_headers_q.empty()) {
    auto msg = odom_msg_headers_q.front();
    //    if (msg.seq == (current_roi_header.second.seq + 1))
    if ((msg.stamp.sec - current_roi_header.second.stamp.sec) < 2) {
      current_roi_header.second = msg;
    } else {
      roi_headers.push_back(current_roi_header);
      current_roi_header.first = msg;
      current_roi_header.second = msg;
    }
    odom_msg_headers_q.pop();
  }

  /*  std::cout << "Current ROI: Start Time = " <<
    current_roi_header.first.stamp
              << ", Sequence ID = " << current_roi_header.first.seq <<
    std::endl; std::cout << "Current ROI: End Time = " <<
    current_roi_header.second.stamp
              << ", Sequence ID = " << current_roi_header.second.seq <<
    std::endl; std::cout << "Getting ROIS took: " << timer.elapsed()
              << " seconds with queue method." << std::endl;

    std::cout << "Size of ROI vector = " << roi_headers.size() << std::endl;
    for (const auto& roi : roi_headers) {
      std::cout << "Number of headers in region: "
                << roi.second.seq - roi.first.seq << std::endl;
    }*/

  roi_headers.erase(
      std::remove_if(roi_headers.begin(), roi_headers.end(),
                     [](std::pair<std_msgs::Header, std_msgs::Header> roi) {
                       return (roi.second.seq - roi.first.seq) < 5;
                     }),
      roi_headers.end());

  /*  std::cout << "Size of ROI vector = " << roi_headers.size() << std::endl;
    for (const auto& roi : roi_headers) {
      std::cout << "Number of headers in region: "
                << roi.second.seq - roi.first.seq << std::endl;
    }*/

  roi_headers.erase(
      std::remove_if(roi_headers.begin(), roi_headers.end(),
                     [](std::pair<std_msgs::Header, std_msgs::Header> roi) {
                       return (roi.second.stamp.sec - roi.first.stamp.sec) < 10;
                     }),
      roi_headers.end());

  /*
    std::cout << "Size of ROI vector = " << roi_headers.size() << std::endl;
    for (const auto& roi : roi_headers) {
      std::cout << "Number of headers in region: "
                << roi.second.seq - roi.first.seq << std::endl;
    }
  */

  std::vector<std::pair<ros::Time, ros::Time>> rois = {};
  for (const auto& roi : roi_headers) {
    rois.emplace_back(roi.first.stamp, roi.second.stamp);
  }
  return rois;
}

} // namespace inspection

/*

*//**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 *//*
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M> {
public:
  void newMessage(const boost::shared_ptr<M const>& msg) {
    //    signalMessage(msg);
  }
};

class BagSubscriber2
    : public message_filters::SimpleFilter<nav_msgs::Odometry> {
public:
  void newMessage(const boost::shared_ptr<nav_msgs::Odometry const>& msg) {
    signalMessage(msg);
  }
};

class PCSubscriber
    : public message_filters::SimpleFilter<sensor_msgs::PointCloud2> {
public:
  void
  newMessage(const boost::shared_ptr<sensor_msgs::PointCloud2 const>& msg) {
    signalMessage(msg);
  }
};

bool func(rosbag::ConnectionInfo const* ptr) {
  return ptr->topic == "/odometry/filtered2";
}

void OdomCallback(const nav_msgs::OdometryConstPtr& odom,
                  const nav_msgs::OdometryConstPtr& odom2) {
  std::cout << "Odom callback" << std::endl;
}

void PC(const sensor_msgs::PointCloud2ConstPtr& pc,
        const nav_msgs::OdometryConstPtr& odom2) {
  std::cout << "PC callback" << std::endl;
}

void BagAnalyzer::LoadBag(const std::string& bag_file) {
  rosbag::Bag bag;

  bag.open("/home/steve/roben_scan_2019-04-11-20-09-55.bag",
           rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.emplace_back("/odometry/filtered2");
  topics.emplace_back(std::string("numbers"));

  rosbag::View view = {bag, rosbag::TopicQuery(topics)};

  //  view.addQuery(bag, func2);
  view.addQuery(bag, [](rosbag::ConnectionInfo const* ptr) {
    return ptr->topic == "/odometry/filtered";
  });

  std::cout << "View size = " << view.size() << std::endl;
  std::cout << "View" << view.getBeginTime() << std::endl;

  // Set up fake subscribers to capture odometry
  BagSubscriber<nav_msgs::Odometry> odom_sub;
  BagSubscriber<nav_msgs::Odometry> odom_sub2;

  BagSubscriber2 odom_sub3;
  BagSubscriber2 odom_sub4;

  PCSubscriber pc_sub;

  //  message_filters::TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry>
  //  sync{pc_sub, odom_sub4, 0};

  //    sync.registerCallback(boost::bind(&OdomCallback, _1, _2));

  // Instantiate every message in the view
  HighResolutionTimer timer;
  for (const auto& m : view) {
    //    std::cout << m.getTime() << std::endl;

    auto message = m.instantiate<nav_msgs::Odometry>();
    if (message->twist.twist.linear.x < 0.0001) { messages_.emplace_back(m); }

    if (message != nullptr) { odom_sub4.newMessage(message); }

    *//*    if (m.getTopic() == "/m3d/velodyne_pointcloud"){
          auto pc_msg = m.instantiate<sensor_msgs::PointCloud2>();
          if (pc_msg != NULL){
            pc_sub.newMessage(pc_msg);
          }
        }
        else {
          auto message = m.instantiate<nav_msgs::Odometry>();
          if (message != NULL){
            odom_sub4.newMessage(message);
          }
        }*//*
    //    std::cout << *message << '\n';
  }
  std::cout << "Timer elapsed time = " << timer.elapsed() << std::endl;

  // Check size of new vector
  std::cout << "Size of filtered view = " << messages_.size() << std::endl;
  std::cout << "Sizeof() filtered view = " << sizeof(messages_) << std::endl;

  for (size_t i = 0; i < 10; i++) {
    auto header_message = messages_[i].instantiate<std_msgs::Header>();
    auto odom_message = messages_[i].instantiate<nav_msgs::Odometry>();

    if (header_message != nullptr)
      std::cout << "Header stamp = " << header_message->stamp << std::endl;
    std::cout << "Odom Header stamp = " << odom_message->header.stamp
              << std::endl;
  }
  bag.close();
}*/
