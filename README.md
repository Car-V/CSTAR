# CSTAR - Concrete Sounding Tool Automated Robot

Concrete Sounding Tool Automated Robot (C-STAR) is a project aimed at automating the process of locating concrete delamination in parking garages. Concrete delamination causes concrete to break and results in damage ranging from potholes to structural collapse, posing dangers and costs. Civil and structural engineers are tasked with finding delamination spots through various nondestructive testing methods. One technique is to drag a metal chain across the garage and mark spots where hollow sounds are heard. This method is effective but involves manual labor, excessive time, and human error. As there is no automated testing method available on the market, C-STAR aims to fill a notable void in making these outdated techniques more efficient and accessible. 

This repository includes the functioning code used for C-STAR at Northeastern University Electrical and Computer Engineering Capstone Competition Spring 2025, where the project placed 2nd. The Wiki describes various design decisions, dependencies, how to operate the robot, and future work to improve the project.

An original iteration of C-STAR was inherited from Generate: A Sherman Center Program. At its acquisition, C-STAR included its mechanical body and some hardware features, requiring electrical and computer engineering upgrades to achieve the intended purpose. For the project’s continuation and as a capstone deliverable, the chassis, 24-V battery, and wheels were reused. The remainder of C-STAR was overhauled and became integrated with new software, which is included in this repository. 

*control* - Python scripts for manual drive of C-STAR

*extra* - Sample audio files, captures of ROS2 frames, saved SLAM maps, and photos of the working software

*src* - ROS2 Humble workspace

*tests* - Python test scripts for team's debugging

*host_program.py* - Host server program for receiving C-STAR data and creating delamination map
