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


int main(int argc, char* argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  
  std::string config_path;
  std::string file_name = "LabelImages.cpp";
  std::string curr_path = __FILE__;
  curr_path.erase(curr_path.end() - file_name.length(), curr_path.end());


  if (FLAGS_config.empty()){
    config_path = inspection::utils::GetConfigFilePath("ImageLabelerConfig.json");
  }else{
    config_path = FLAGS_config;
  }
  
  printf("\n\n");
  printf((curr_path + "lib/ImageLabeler.py").c_str());
  printf("\n");
  printf("/home/alice/projects/beam_robotics/inspection_tools/inspection/src/lib/ImageLabeler.py\n\n");

  if ("/home/alice/projects/beam_robotics/inspection_tools/inspection/src/lib/ImageLabeler.py" == (curr_path + "lib/ImageLabeler.py").c_str()){
     printf("why??? is it not working then!!!");
}

  FILE* fp;
  int argc_py = 3;
  const char* argv_py[3]; 
  argv_py[0] = curr_path + "lib/ImageLabeler.py";
  argv_py[1] = const_cast<char*>(FLAGS_images.c_str());
  argv_py[2] = const_cast<char*>(config_path.c_str());

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
