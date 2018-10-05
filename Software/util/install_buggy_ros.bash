# script to install the dependencies for the buggy system

# update apt
sudo apt-get update

# install ros-kinetic-serial
echo "Installing ros-kinetic-serial"
ros_kinetic_exists=$(dpkg -l | grep ros-kinetic-serial)
if [ -z "$ros_kinetic_exists" ]
then
    sudo apt-get install ros-kinetic-serial
fi

# install ros-kinetic-geodesy
echo "Installing ros-kinetic-geodesy"
ros_geodesy_path=$(dpkg -l | grep ros-kinetic-geodesy)
if [ -z "$ros_geodesy_path" ]
then
    sudo apt-get install ros-kinetic-geodesy
fi

# install jsoncpp
echo "Installing jsoncpp"
git clone https://github.com/open-source-parsers/jsoncpp.git
cd jsoncpp
python amalgamate.py
cd dist
mv jsoncpp.cpp json
mv json ../../../real_time/ROS_RoboBuggy/src/robobuggy/include/
cd ../..
rm -rf jsoncpp

# install libfreespace
echo "Installing libfreespace"
git clone https://github.com/hcrest/libfreespace.git
cd libfreespace
cmake .
make
sudo make install
cd ..
rm -rf libfreespace

# run a catkin_make
echo "Running catkin_make"
cd ../real_time/ROS_RoboBuggy
catkin_make

# add the package to the bashrc
echo "Adding robobuggy package to the ros package path"
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
