# BP_Project_EPFL

This project contains all software related to the identification and control of a ball and plate system project

--------------------------------------------------------------------------------
## Required Softwares
- **NI LabVIEW 2021 SP1 32bits** (2021 or newer)
    - Real time hardware toolboxes
    - NI Vision Aquisition Software (optional to generate new vision software)
    - NI IMAQ
    - NI MAX
- **FlyCapture2 Full SDK** (`FlyCap`) 
    - 32bits and 64 bits versions
- **MATLAB** (2019b or newer)
    - Control System Toolbox
    - Signal Processing Toolbox
    - Statistics and Machine Learning Toolbox
- **Toolbox manager** (`tbxmanager`) for MATLAB
    - yalmip
    - sedumi
- **MOSEK** version 9.2 (not necessary but recommended)

## Installation Instructions

#### Toolbox manager
- Run script [`install_tbxmanager.m`](install_tbxmanager.m) to install Toolbox manager with all required modules.

#### Mosek
- Download appropiate installer from [MOSEK](https://www.mosek.com/downloads/) website and install the software.
- For academic uses, a [free license](https://www.mosek.com/products/academic-licenses/) could be requested. For commercial purposes, a 30-day [trial license](https://www.mosek.com/products/trial/) is also available.

#### FlyCapture2 Full SDK
- Run installation files in the following [link:](https://www.flir.com/support-center/iis/machine-vision/downloads/spinnaker-sdk-flycapture-and-firmware-download/)
- Follow installation guide in this [link](https://www.flir.com/support-center/iis/machine-vision/application-note/getting-started-with-ni-max-and-labview/)
- Check if camera is working properly with NI MAX and `test_vision.vi`

--------------------------------------------------------------------------------
## Basic Usage Instructions

### Test ball and plate device
- Plug in components (camera, motor, encoder, computer, amplifier, myRIO, circuit).
- Place the plate in upright position before powering up the myRIO.
- Put the ball on the plate and run `Vision.vi`. Check that it shows the camera feed and coordinates are updated, ball coordinates are different than 0.
- Run `main.vi` with Identify mode and check that graphs are showing reasonable values. `Vision.vi` and `main.vi` are run simultaneously.

### Get a frequency response model
- Perform steps described above
- Change input characteristics in the option tab and run `main.vi` 
- Stop after the first excitation signal is finished
- Use the binary folder generated and run `BP_identify.m` to get a frequency response model. Modify variable names before saving
- Run `BP_merge` to combine different tests and obtain the MIMO transfer function

### Design and implement a data driven controller
- Run `BP_control` with a SISO frequency response data. Change controller order and weighting filters.
- Repeat for X and Y
- Run `BP_saveController` to convert controllers into implemented format in LabView and save the binary file on the microcontroller
- Run `main.vi` with Control mode to test the data driven controller

--------------------------------------------------------------------------------
## Solving Common Errors
- It is not possible to change options of reference trajectory while `main.vi` is running. Remember to stop the program before applying any change.
- If the shared variable is not found, either check `measure_outer.vi` and make sure that it is reading the variable in the microcontroller, or check that `Vision.vi` is running and sending ball coordinates in the microcontroller variable
- If motors are not moving after running `main.vi`, check that `Vision.vi` is running, make sure the shared variable is correctly transmitted and read, that the amplifier is on, or simply restart the program
- If `Vision.vi` is giving a transformation or color match error, make sure that the space above and around the plate is well lit. If problem persists, change exposure settings of the camera from the front panel or from NI MAX
- If the motors seem to only rotate in one direction, check that no other connections on the amplifier are plugged (Only use two channels and unplug any other wire)

--------------------------------------------------------------------------------
## Authors
- [Omar Meebed]