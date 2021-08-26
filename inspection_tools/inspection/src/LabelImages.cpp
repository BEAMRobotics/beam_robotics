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
DEFINE_bool(crack, true, "Segment cracks from images");
DEFINE_bool(delam, true, "Segment delaminations from images");
DEFINE_bool(corrosion, true, "Segment corrosions from images");
DEFINE_bool(spall, true, "Segment spalls from images");
DEFINE_bool(vis, false, "Visualize segmentation results");


int main(int argc, char* argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  
  //get current path
  std::string config_path;
  std::string file_name = "LabelImages.cpp";
  std::string curr_path = __FILE__;
  curr_path.erase(curr_path.end() - file_name.length(), curr_path.end());


  //load config path
  if (FLAGS_config.empty()){
    config_path = inspection::utils::GetConfigFilePath("ImageLabelerConfig.json");
  }else{
    config_path = FLAGS_config;
  }

  //count number of model flags
  int model_flags = 0;
  if (!FLAGS_crack){
    model_flags = model_flags + 1;
  }
  if (!FLAGS_delam){
    model_flags = model_flags + 1;
  }
  if (!FLAGS_corrosion){
    model_flags = model_flags + 1;
  }
  if (!FLAGS_spall){
    model_flags = model_flags + 1;
  }
  if (FLAGS_vis){
    model_flags = model_flags + 1;
  }

  FILE* fp;
  int argc_py = 3 + model_flags;
  const char* argv_py[argc_py]; 

  //reqired flags
  std::string py_ex_path = curr_path + "lib/ImageLabeler.py";
  argv_py[0] = static_cast<const char*>(py_ex_path.c_str());
  argv_py[1] = static_cast<const char*>(FLAGS_images.c_str());
  argv_py[2] = static_cast<const char*>(config_path.c_str());

  //optional flags
  int idx = 3;
  if (!FLAGS_crack){
    argv_py[idx] = static_cast<const char*>("--crack");
    idx = idx + 1;
  }
  if (!FLAGS_delam){
    argv_py[idx] = static_cast<const char*>("--delam");
    idx = idx + 1;
  }
  if (!FLAGS_corrosion){
    argv_py[idx] = static_cast<const char*>("--corrosion");
    idx = idx + 1;
  }
  if (!FLAGS_spall){
    argv_py[idx] = static_cast<const char*>("--spall");
    idx = idx +1;
  }
  if (FLAGS_vis){
    argv_py[idx] = static_cast<const char*>("--vis");
  }

  //run python code
  Py_Initialize();
  PySys_SetArgvEx(argc_py, const_cast<char**>(argv_py), 1);

  fp = fopen(const_cast<char*>((curr_path + "lib/ImageLabeler.py").c_str()), "rb");
  if (fp == 0){
    printf("Open File Failed: ImageLabeler.py has been moved from src/lib/\n");

  }else{
    PyRun_SimpleFile(fp, const_cast<char*>((curr_path + "lib/ImageLabeler.py").c_str()));

    Py_Finalize();
    fclose(fp);
  }
  return 0;
}
