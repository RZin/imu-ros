# IMU project

## Dependencies
* C++'11
* serial

## Configure this project

### Instal serial
"choose /path/to/serial"

```
cd /path/to/serial
git clone https://github.com/wjwwood/serial.git
make
make test
make install
```

### Get IMU Project
Clone the project from repository:

`git clone https://github.com/RZin/imu-ros.git`

For compiling on catkin, first, we create a symbolic link at the ROS workspace:
```
roscd && cd ../src
```
then
```
ln -s /path/to/serial/directory /path/to/your/project/serial
```
in our case:
```
ln -s /home/username/serial /home/username/catkin_ws/src/serial 
```
next
```
cd ..
catkin_make 
```

Make changes
```
git add <files/dirs changed>
git commit -m  "<commit message>"
git push -u origin master
```

## Launch files
* imu.launch: record imu data only
* imu_rs.launch: record imu plus realsense 435 data 

## todo 
* Add udev rules for imu data sctreaming device (currently PORT = "/dev/ttyACM0")
* Think what value should be used if "-" or "@" or " " is recieved from imu (currently 0.0, but that msg should be disregarded at all)