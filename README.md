# ur5_vrep_python
Use python and Vrep to do the path planning for ur5 robot

# Installation
Install vrep, see the website at http://www.coppeliarobotics.com/downloads.html for installation instructions.

# Requirements
* Python
* Numpy

# Example
Make sure you have following files in your directory, in order to run the various examples:

1. vrep.py
2. vrepConst.py
3. the appropriate remote API library: "remoteApi.dll" (Windows), "remoteApi.dylib" (Mac) or "remoteApi.so" (Linux)

* Open `ur5_pickup.ttt` in vrep and start the simulation. If everything goes well, you will see the scene as below.
![demo-0](https://github.com/Junzhuodu/ur5_vrep_python/blob/master/Images/002.png)

* Run `gotoTarget.py` / `gotoPosition.py`.
Then you can get the path planning for robot to move.
![demo-1](https://github.com/Junzhuodu/ur5_vrep_python/blob/master/Images/001.png)
