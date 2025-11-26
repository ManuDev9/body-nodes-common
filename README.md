# body-nodes-common
Bodynodes repository for the BnCommon module

This repository will contain all the modules in charge of:
- basic datatypes in use in all other projects
- utility functions to do data manipulation for different use cases
- higher level computation for our custom motion capture

There will be packages for python, java, csharp, and cpp, along with data to test them.

Check for the language supported versioning in each folder. If detailed info is not provided,
feel free to test the code in your environment and add your versions in the supported_version.txt
file. Raise an issue if you find any problems.



Modules:
  - BnConstants: It contains all the main constants values in use by all the projects. Version dependent
  - BnAxisConfig: This is an utility object to easily reorient the axis of bodynodes data to the main
                  application in cases where the axis references are different 
  - BnUtils: It contains various utility functions
  - MotionTracking:
    - BnMotionTracking_2Nodes: Motion Tracking object for two bodynodes. It returns the endpoint given arms data
  - RobotIK:
    - BnRobotIK_ArmZYY: Simple Inverse Kinematic object for a basic 3DoF robotic arm.
                        It returns three angles along local Z, Y, and Y given an endpoint
  - RobotMT: End to End Motion Tracking and Inverse Kinematic object. It is generic and can be constructed with any Tracking and IK object
             Given arms data returns the angles for the robot
  - BnTestCommon: It contains all the tests for the library

