add_library(inspection
            GenerateReport.cpp
            ImageExtractor.cpp
            ImageLabeler.cpp
            LoadHandLabels.cpp
            MapLabeler.cpp
            QuantifyDefects.cpp
            SaveImages.cpp)
target_link_libraries(inspection ${catkin_LIBRARIES} ${PCL_LIBRARIES})
