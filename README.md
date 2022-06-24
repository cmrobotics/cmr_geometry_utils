<div align="center">

# CMR Geometry Utils

A header only library that provides geometry helpers.

</div>

<div align="center">
  
  [![CodeFactor](https://www.codefactor.io/repository/github/cmrobotics/cmr_geometry_utils/badge)](https://www.codefactor.io/repository/github/cmrobotics/cmr_geometry_utils)
  
</div>
  
<div align="center">

[![Percentage of issues still open](http://isitmaintained.com/badge/open/cmrobotics/cmr_geometry_utils.svg)](http://isitmaintained.com/project/cmrobotics/cmr_geometry_utils "Percentage of issues still open")
[![GitHub license](https://img.shields.io/github/license/cmrobotics/cmr_geometry_utils.svg)](https://github.com/cmrobotics/cmr_geometry_utils/blob/galactic-devel/LICENSE)
[![GitHub contributors](https://img.shields.io/github/contributors/cmrobotics/cmr_geometry_utils.svg)](https://GitHub.com/cmrobotics/cmr_geometry_utils/graphs/contributors/)
[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://GitHub.com/cmrobotics/cmr_geometry_utils/graphs/commit-activity)
[![GitHub pull-requests](https://img.shields.io/github/issues-pr/cmrobotics/cmr_geometry_utils.svg)](https://GitHub.com/cmrobotics/cmr_geometry_utils/pull/)

</div>


## Table of Contents
* [General Info](#general-information)
* [Features](#features)
* [Setup](#setup)
* [Usage](#usage)
* [Room for Improvement](#room-for-improvement)
* [Contact](#contact)
* [License](#license)

## General Information

The library provides helpers functions for basic geometry functions that prevents redeclaration across a stack.

## Features

### Distances

- Normalized angle distance for multiple objects (`geometry_msgs::msg::PoseStamped`, `geometry_msgs::msg::Pose`, `geometry_msgs::msg::Point`)
- Point to Point distance for multiple objects (`geometry_msgs::msg::PoseStamped`, `geometry_msgs::msg::Pose`, `geometry_msgs::msg::Point`)

## Setup

The project requires set(CMAKE_CXX_STANDARD 20) in your CMakeLists.txt to be built. To setup the library in your ROS2 project:

- Create your test folder in your project root.
- Add the following lines to your project CMakeLists.txt:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  add_subdirectory(test)
endif()
```

- Create your gtest cpp files in your test folder.
- Create a CMakeLists.txt in your test folder as following:

```cmake
set(dependencies
  ${dependencies}
  ${cmr_geometry_utils}
)

find_package(cmr_geometry_utils REQUIRED)

ament_add_gtest(name_of_your_test name_of_your_test.cpp)
target_link_libraries(name_of_your_test
  name_of_the_libs_you_want_to_link
)
ament_target_dependencies(name_of_your_test ${dependencies})
target_include_directories(name_of_your_test PRIVATE "../include" "include" ${cmr_geometry_utils_INCLUDE_DIRS})
```

## License
This project is open source and available under the [Apache License, Version 2.0](https://www.apache.org/licenses/LICENSE-2.0).
