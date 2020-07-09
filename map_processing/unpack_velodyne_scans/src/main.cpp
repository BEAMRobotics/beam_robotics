#include "unpack_velodyne_scans/UnpackVelodyneScans.h"
#include <iostream>

int main(int argc, char* argv[]) {
	if (argc != 3) {
    std::cerr << "----------------------------------------------------------\n"
    					<< "**USAGE SUMMARY**\n"
              << "Enter the full file path to bag file as well as the velodyne"
							<< "calibration file as follows: \n"
              << "USAGE: " << argv[0] << " /path/to/bag/file.bag"
							<< "VLP16_hires_db.yaml \n"
              << "----------------------------------------------------------\n";
    return -1;
  } else {
		std::string bag_file_path;
		std::string calibration_file;
		bag_file_path = argv[1];
		calibration_file = argv[2];

    unpack_velodyne_scans::UnpackVelodyneScans unpack_velodyne_scans(
			bag_file_path, calibration_file);
    unpack_velodyne_scans.Unpack();
  }
  return 0;
}
