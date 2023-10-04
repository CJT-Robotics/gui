# CJT-Robotics GUI-Software

> [![Version - BETA](https://img.shields.io/badge/Version-BETA-ffff00)](https://) [![Customizable - Semi (in progress)](https://img.shields.io/badge/Customizable-Semi_(in_progress)-ffa500)](https://) [![Other Software - required](https://img.shields.io/badge/Other_Software-required-FF0000)](https://)

## Overview
* [Installation](#installation)
* [Getting Started](#getting-started)
* [Documetation]()
    - [Usfull Information]()
    - [Required Software]()
    - [Good to Know]()
    - [Wiki]()
* [Issue Reporting]()
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
    
3. Navigate to your workspace folder and build your it, to use our software.

   ```bash
   cd ..
   catkin_make
   ```
   
4. Run our code. Please note that roscore should be started.
   
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
     rosrun cjtrobotics_gui driver_gui
     ```
     
3. To stop and shutdown our gui, you just have to close the window of the gui. After this just wait a second, then the threads are closed.

4. If you need any kind of help, there are tree options, to get it.
   
    - For the most frequently questions there are some solutions in the gui program.
    - We are happy, if you report an issue on GitHub. So other people with similar problems could solve there problems as well.
    - As the last option, you can submit us an email (contact@cjtrobotics.de) with your problem, then we hope, that we could help you.
