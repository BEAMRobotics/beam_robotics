#include <ros/ros.h>
#include <ros/package.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_msgs/TFMessage.h>
#include "velodyne_h_v_calib/VelodyneTF.h"

#include <cstdlib>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <ctime>
#include <iomanip>

std::string vvlpFrameId;
std::string hvlpFrameId;

tf::Transform transform;
tf::Vector3 pos1;
tf::Quaternion rot1;
Eigen::Matrix4d transformation_matrix;

std::string T_BASELINK_HVLP, T_HVLP_GPS, T_XIMEA_VVLP, T_FLIR_XIMEA, T_IMU_XIMEA;

bool prev_save_data = false;

void saveTFData() {
	Eigen::Affine3d a;
    Eigen::Matrix4d M;
    tf::transformTFToEigen(transform, a);
    M = a.matrix();

    // Get current date
    std::time_t now = time(0);
    std::tm *localtm = localtime(&now);
    std::string year = std::to_string(localtm->tm_year + 1900);
    std::string month = localtm->tm_mon + 1 <= 9 ? "0" + std::to_string(localtm->tm_mon+1) : std::to_string(localtm->tm_mon+1);
    std::string day = std::to_string(localtm->tm_mday);
    std::ostringstream os;
    os << ros::package::getPath("ig_calibration_publisher") << "/calibrations/" << year
       << "_" << month << "_" << day << ".yaml";

    std::string path =  os.str();

    // Build T_VVLP_HVLP string from transformation matrix
    os.str("");
    os << std::fixed << std::setprecision(5)
       << M(0,0) << "   " << M(0,1) << "   " << M(0,2) << "   " << M(0,3) << "   "
       << M(1,0) << "   " << M(1,1) << "   " << M(1,2) << "   " << M(1,3) << "   "
       << M(2,0) << "   " << M(2,1) << "   " << M(2,2) << "   " << M(2,3) << "   "
       << M(3,0) << "   " << M(3,1) << "   " << M(3,2) << "   " << M(3,3) << "   ";
    std::string T_VVLP_HVLP = os.str();

    std::ofstream fout;
    fout.open(path);

    fout << "T_BASELINK_HVLP:    " << T_BASELINK_HVLP
    	 << "\nT_HVLP_GPS:    " << T_HVLP_GPS
    	 << "\nT_VVLP_HVLP:    " << T_VVLP_HVLP
    	 << "\nT_XIMEA_VVLP:    " << T_XIMEA_VVLP
    	 << "\nT_FLIR_XIMEA:    " << T_FLIR_XIMEA
    	 << "\nT_IMU_XIMEA:    " << T_IMU_XIMEA;

    fout.close();
    ROS_INFO("Saved data to: %s", path.c_str());
}

void tfCallback(const velodyne_h_v_calib::VelodyneTFConstPtr &velodyne_tf_msg)
{
	// Modify the transform relative to the original transform
	transform.setOrigin(pos1 + tf::Vector3(velodyne_tf_msg->x, velodyne_tf_msg->y, velodyne_tf_msg->z));
	transform.setRotation(rot1 * tf::createQuaternionFromRPY(velodyne_tf_msg->roll,velodyne_tf_msg->pitch,velodyne_tf_msg->yaw));

	// Check if data should be saved
	if (!prev_save_data && velodyne_tf_msg->save_data) {
		saveTFData();
	}
	prev_save_data = velodyne_tf_msg->save_data;
}

void loadParams() {
	std::string transform_matrix_string;
	ros::param::get("T_BASELINK_HVLP", T_BASELINK_HVLP);
	ros::param::get("T_HVLP_GPS", T_HVLP_GPS);
	ros::param::get("T_VVLP_HVLP", transform_matrix_string);
	ros::param::get("T_XIMEA_VVLP", T_XIMEA_VVLP);
	ros::param::get("T_FLIR_XIMEA", T_FLIR_XIMEA);
	ros::param::get("T_IMU_XIMEA", T_IMU_XIMEA);
	ros::param::get("vvlp_frame_id", vvlpFrameId);
	ros::param::get("hvlp_frame_id", hvlpFrameId);
	std::vector<double> transform_vector;

	// Convert string for vvlp_hvlp transform to float values
	for (int i = 0; i < 16; ++i) {
		std::size_t space_pos = transform_matrix_string.find(" ");
		if (space_pos != std::string::npos) {
			transform_vector.push_back(std::stod(transform_matrix_string.substr(0, space_pos)));
			transform_matrix_string = transform_matrix_string.substr(space_pos);
			transform_matrix_string = transform_matrix_string.substr(transform_matrix_string.find_first_not_of(" "));
		} else {
			transform_vector.push_back(std::stod(transform_matrix_string));
		}
	}

	transformation_matrix = (Eigen::Matrix4d() <<
		transform_vector[0], transform_vector[1], transform_vector[2], transform_vector[3],
		transform_vector[4], transform_vector[5], transform_vector[6], transform_vector[7],
		transform_vector[8], transform_vector[9], transform_vector[10], transform_vector[11],
		transform_vector[12], transform_vector[13], transform_vector[14], transform_vector[15]).finished();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "velodyne_h_v_calib");
	loadParams();
	ros::NodeHandle nh;
	ros::Time::init();

	static tf::TransformBroadcaster br;

	// Initialize transform
	transform.setOrigin(tf::Vector3(0,0,0));
	transform.setRotation(tf::createQuaternionFromRPY(0,0,0));

	Eigen::Affine3d affine_3d;
	affine_3d.matrix() = transformation_matrix;

	tf::transformEigenToTF(affine_3d, transform);

	// Store position and rotation of original transform
	pos1 = transform.getOrigin();
	rot1 = transform.getRotation();

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), hvlpFrameId, vvlpFrameId));

	ros::Subscriber velodyne_tf_sub = nh.subscribe("velodyne_calib_tf", 100, tfCallback);
	ros::Rate subscribeRate(10);

	while (true) {
		ros::spinOnce();
		try
		{
	    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), vvlpFrameId, hvlpFrameId));
		}
		catch (tf::TransformException ex)
		{
			ROS_WARN("VELODYNE HV TRANSFORM: Could not broadcast transform from %s to %s.", hvlpFrameId.c_str(), vvlpFrameId.c_str());
		}
		subscribeRate.sleep();
	}
}
