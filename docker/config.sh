# hpp-planning Docker environment
# robotpkg provides base HPP C++ deps at /opt/openrobots
# hpp-manipulation + hpp-python built from source in ~/devel/install (takes priority)

export INSTALL_HPP_DIR=$DEVEL_HPP_DIR/install
export ROBOTPKG=/opt/openrobots

export PATH=$INSTALL_HPP_DIR/sbin:$INSTALL_HPP_DIR/bin:$ROBOTPKG/bin:$ROBOTPKG/sbin:$PATH
export PKG_CONFIG_PATH=$INSTALL_HPP_DIR/lib/pkgconfig/:$ROBOTPKG/lib/pkgconfig

export PYTHONPATH=$INSTALL_HPP_DIR/lib/python3.12/site-packages:$ROBOTPKG/lib/python3.12/site-packages:/usr/lib/python3/dist-packages:$PYTHONPATH

export LD_LIBRARY_PATH=$INSTALL_HPP_DIR/lib:$ROBOTPKG/lib:$INSTALL_HPP_DIR/lib64:$LD_LIBRARY_PATH

export CMAKE_PREFIX_PATH=$INSTALL_HPP_DIR:$ROBOTPKG:/usr

# ROS2
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
fi

# hpp-planning ROS2 workspace (if built)
if [ -f $DEVEL_HPP_DIR/ros2_ws/install/setup.bash ]; then
    source $DEVEL_HPP_DIR/ros2_ws/install/setup.bash
fi

# hpp-planning on PYTHONPATH
if [ -d $DEVEL_HPP_DIR/hpp-planning ]; then
    export PYTHONPATH=$DEVEL_HPP_DIR/hpp-planning:$PYTHONPATH
fi

export ROS_PACKAGE_PATH=$INSTALL_HPP_DIR/share:$ROBOTPKG/share:${ROS_PACKAGE_PATH:-}
