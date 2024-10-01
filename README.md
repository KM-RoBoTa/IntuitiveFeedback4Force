# Intuitive force sensors reading
This repository contains all the necessary code to read the force sensors. <br /> 
You can find here:
- the C++ code for the Linux OS that reads the data sent by the sensors, found in the ```readForces_cppCode``` folder
- an example of how to use the code, found in the ```example``` folder

## Axes
todo: insert picture here

## C++ code to read the data
The sensors are continuously read in a second thread, with an interface function allowing an easy grab of those values from the main thread. A calibration function is also included in the code. <br /> 

To use the code, you simply need to add the 2 C++ files in your project, and include the ```force_sensors.hpp``` header. 

By default, the number of sensors is set to 4, but you can edit the value ```NBR_SENSORS``` in the ```force_sensors.hpp``` file. <br /> 
You need to create an instance of the ```ForceSensors``` class, which automatically opens the port, creates a thread and starts continuously reading the incoming data. <br /> 
You can calibrate the sensors by using the ```ForceSensors::calibrate()``` method. <br /> 
Read data can be fetched by the main thread using the ```ForceSensors::getForces()``` method. This method also returns the sensors' frequency.

The force values are in ```N```.

## Example
The example is very basic to showcase the use of the code: after calibrating, the main thread fetches and prints 3 times the force values from the second thread every 10ms. 

To use the example, first plug in the force sensors and check the assigned port. If the port is different than ```"/dev/ttyACM0"```, edit the ```main.cpp``` file. <br /> 
To build the example: 
```bash
cd <path-to-the-repo-folder>/example
mkdir build && cd build 
cmake ../
make sensors
```
To run the code: ```./sensors``` from the build folder.

## About
KM-RoBoTa sarl's code for reading Intuitive's force sensors.

### Authors
- Katarina Lichardova: katarina.lichardova@km-robota.com

### Copyright
Copyright 2021-2024, Laura Paez Coy and Kamilo Melo. <br /> 
This code is under MIT licence: https://opensource.org/licenses/MIT