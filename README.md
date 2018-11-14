# robot-simulator

robot-simulator is an attempt to help the design/simulation of robotic platforms from the mechanical design (using openscad) to the geometric direct and inverse kinematic modeling simulation using OpenGL 3D visualization in Pyton3

# dependencies
##pyglet and OpenGL python libraries
```
  sudo apt-get install python3-pip
  sudo pip3 install pyglet
  sudo pip3 install PyOpenGL
```

##openscad software
```
  sudo apt-get install openscad
```
###For Ubuntu 18.04
```
  gdebi will manage .deb dependencies
  sudo apt-get install gdebi-core
  wget http://ftp.us.debian.org/debian/pool/main/o/openscad/openscad_2015.03-2+dfsg-2+b3_amd64.deb
  sudo gdebi openscad_2015.03-2+dfsg-2+b3_amd64.deb
```

#usage
robot-simulator is provided with a simple robot arm model.
Just launch `robot-simulator.py` and the program will (at the first time only)  generate the stl models for the different parts of the robot.
The camera is moved using the keys ZQSD and CTRL,SPACE. It is rotated using the mouse, but it requires the user to left click on the window to capture and release the mouse.
Faster motions are achieved by pressing the SHIFT key.
The different degrees of freedom of the robot are controlled through the keys TYUIO and GHJKL. 

