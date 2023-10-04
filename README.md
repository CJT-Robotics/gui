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
1.
