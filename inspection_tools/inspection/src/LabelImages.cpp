#pragma once

#include <stdio.h>
#include <Python.h>
#include <gflags/gflags.h>
#include <beam_utils/gflags.h>
#include <inspection/InspectionUtils.h>


DEFINE_string(images, "", "Full path to image directory (Required).");
DEFINE_validator(images, &beam::gflags::ValidateDirMustExist);
DEFINE_string(config, "", 
              "Full path to json config file. If none provided, it will use "
              "the file inspection/config/ImageLablerConfig.json");
DEFINE_bool(segment_cracks, true, "Segment cracks from images");
DEFINE_bool(segment_delams, true, "Segment delaminations from images");
DEFINE_bool(segment_corrosions, true, "Segment corrosions from images");
DEFINE_bool(segment_spalls, true, "Segment spalls from images");
DEFINE_bool(visualize, false, "Visualize segmentation results");
DEFINE_bool(verbose, false, "Print filepath of image being processed");


int main(int argc, char* argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  
  // load config path
  std::string config_path;
  if (FLAGS_config.empty()) {
    config_path = inspection::utils::GetConfigFilePath("ImageLabelerConfig.json");
  } else {
    config_path = FLAGS_config;
  }

  // count number of model flags
  int model_flags = 0;
  if (!FLAGS_segment_cracks) {
    model_flags++;
  }
  if (!FLAGS_segment_delams) {
    model_flags++;
  }
  if (!FLAGS_segment_corrosions) {
    model_flags++;
  }
  if (!FLAGS_segment_spalls) {
    model_flags++;
  }
  if (FLAGS_visualize) {
    model_flags++;
  }
  if (FLAGS_verbose) {
    model_flags++;
  }

  int argc_py = 3 + model_flags;
  const char* argv_py[argc_py]; 

  // get current path
  std::string file_name = "LabelImages.cpp";
  std::string curr_path = __FILE__;
  curr_path.erase(curr_path.end() - file_name.length(), curr_path.end());

  // reqired flags
  std::string py_ex_path = curr_path + "lib/ImageLabeler.py";
  argv_py[0] = static_cast<const char*>(py_ex_path.c_str());
  argv_py[1] = static_cast<const char*>(FLAGS_images.c_str());
  argv_py[2] = static_cast<const char*>(config_path.c_str());

  // optional flags
  int idx = 3;
  if (!FLAGS_segment_cracks) {
    argv_py[idx] = static_cast<const char*>("--crack");
    idx++;
  }
  if (!FLAGS_segment_delams) {
    argv_py[idx] = static_cast<const char*>("--delam");
    idx++;
  }
  if (!FLAGS_segment_corrosions) {
    argv_py[idx] = static_cast<const char*>("--corrosion");
    idx++;
  }
  if (!FLAGS_segment_spalls) {
    argv_py[idx] = static_cast<const char*>("--spall");
    idx++;
  }
  if (FLAGS_visualize) {
    argv_py[idx] = static_cast<const char*>("--vis");
    idx++;
  }
  if (FLAGS_verbose)idx++;{
    argv_py[idx] = static_cast<const char*>("--verbose");
  }

  //run python code
  Py_Initialize();
  PySys_SetArgvEx(argc_py, const_cast<char**>(argv_py), 1);

  FILE* fp;
  fp = fopen(const_cast<char*>((curr_path + "lib/ImageLabeler.py").c_str()), "rb");
  if (fp == 0) {
    printf("Open File Failed: ImageLabeler.py has been moved from src/lib/\n");

  } else {
    PyRun_SimpleFile(fp, const_cast<char*>((curr_path + "lib/ImageLabeler.py").c_str()));

    Py_Finalize();
    fclose(fp);
  }
  return 0;
}
