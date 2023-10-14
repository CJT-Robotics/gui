# CJT-Robotics GUI-Software

[![GitHub Package - under construction](https://img.shields.io/badge/GitHub_Package-under_construction-ff0000?style=for-the-badge&logo=!&logoColor=fff)](https://)

[![Version - BETA](https://img.shields.io/badge/Version-BETA-ffff00)](https://) [![Customizable - Semi (in progress)](https://img.shields.io/badge/Customizable-Semi_(in_progress)-ffa500)](https://) [![Other Software - required](https://img.shields.io/badge/Other_Software-required-FF0000)](https://)

## Overview
* [Installation](#installation)
* [Getting Started](#getting-started)
* [Documetation](#documentation)
    - [Usfull Information](#usfull-information)
    - [Required Software](#required-software)
    - [Good to Know](#good-to-know)
    - [Wiki](#wiki)
* [Issue Reporting](#issue-reporting)
* [Credits]()
* [License]()

## Installation
To use this software for your own projects or your robot, we will give you an exact tutorial to get our software to work. If you have any kind of problems, feel free to create an Issue Reoprt on GitHub.
1. Direct to your catkin-workspace folder.

    ```bash
    cd <your_workspace_folder_name>/
    cd src/
    ```
    
2. Clone this repository in your src folder.

    ```bash
    git clone https://github.com/CJT-Robotics/gui.git
    ```

3. Make sure that the required third-party software is installed. You can find the list here: [Required Software](#required-software)
    
4. Navigate to your workspace folder and build your it, to use our software.

   ```bash
   cd ..
   catkin_make
   ```
   
5. Run our code. Please note that roscore should be started.
   
   - Driver GUI
  
     ```bash
     rosrun cjtrobotics_gui driver_gui
     ```
     
   - Robotic-Arm Operator GUI

     ```bash
     rosrun cjtrobotics_gui operator_gui
     ```
     
## Getting Started
This section is used to generally start our software. Here you will find general tips.
1. Build your catkin workspace. To do so, please ensure, that you are in the right folder (your catkin workspace). Otherwise the build command won't be working.

   ```bash
   cd <your_workspace_folder_name>/
   catkin_make
   ```
   
2. Now your workspace is up to date, if there were no errors. After starting roscore you can start our software.
   
    - Driver GUI
  
      ```bash
      rosrun cjtrobotics_gui driver_gui
      ```
     
   - Robotic-Arm Operator GUI
   
     ```bash
     rosrun cjtrobotics_gui operator_gui
     ```
     
3. To stop and shutdown our gui, you just have to close the window of the gui. After this just wait a second, then the threads are closed.

4. If you need any kind of help, there are tree options, to get it.
   
    - For the most frequently asked questions there are some solutions in the gui program.
    - We are happy, if you report an issue on GitHub. So other people with similar problems could solve there problems as well. Please use the recommend format you can find here [Issue Reporting]()
    - As the last option, you can submit us an email (contact@cjtrobotics.de) with your problem, then we hope, that we could help you.

## Documentation

Here you can find some information about the program it self. For example the required information, some other usfull information or the link to the wiki for a detailed documantation.

### Usfull Information

* This software is currently a BETA-Version.
* This GUI Software does not require any pre manipulated images. We have built some preset filters and viewing options. For example movement detection in different approaches, line for the postion of the wheels, detection of hazment signs, detection of landolt C and even more.
* This GUI Software also includs an emulator for your key inputs. So you can subscribe in your ros-network this inputs for maneuver your robot.
* This software can display multible cameras.
* This software displays you information of the robot if the robot provides this information.
* This programm can also display a temporary LIDAR image.
* If you dont use all this technologies or dont want to display them in your GUI, you can also disable this features or dont connect them. If the source is not availible, you can also use this program.

### Required Software

* Due to the use of the ImGui library this software is rquired. But if you download this package the imgui folder should be included as well.
* You have installed the roscpp package. If you havent, you get directions if you start the programm.
* You have installed the image_transport package. If you havent, you get directions if you start the programm.
* You have installed the sensor_msgs package. If you havent, you get directions if you start the programm.
* You have installed the cv_bridge package. If you havent, you get directions if you start the programm.

### Good to Know

* This program is still under development. So this means, that this program isnt finished developed at all.
* If the package folder is not named "cjtrobotics_gui", the software will crash. In later fixes we will solve this error.

### Wiki

We use the wiki for an exact documentation of this program. We explain the code in all details and we hope that if you have any problems with customizing our software or questions to the code, that this helps you. Here is the link to our wiki: [Wiki](https://github.com/CJT-Robotics/gui/wiki)

## Issue Reporting

> To make our lives easier and to maintain a clear structure, we request that the issue report follows a predefined structure.

Title:

* [Label] - "The topic of the report"

Content:

* [Date of the report] - Version: "Version of the Prgramm" Could be find under package.xml
* Describe your issue
* (If you send any files, name them as the topic of the file) Contentdescription - Link

Please also add your label at the labels section.

> Here you can find a test issue with this format: [Issue](https://github.com/CJT-Robotics/gui/issues/4)
