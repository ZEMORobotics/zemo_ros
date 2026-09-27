START_MARKER="# ===== ZEMO ROS 2 CONFIG START ====="
END_MARKER="# ===== ZEMO ROS 2 CONFIG END ====="

sed -i "\|$START_MARKER|,\|$END_MARKER|d" "$HOME/.bashrc"

cat << 'EOF' >> "$HOME/.bashrc"

# ===== ZEMO ROS 2 CONFIG START =====

test -f /opt/ros/jazzy/setup.bash && source /opt/ros/jazzy/setup.bash
test -f "$HOME/ros2_ws/install/local_setup.bash" && source "$HOME/ros2_ws/install/local_setup.bash"

export ROS_DOMAIN_ID=10

cb() {
    cd "$HOME/ros2_ws" || return

    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        "$@" && source install/local_setup.bash
}

temp() {
    sudo watch -n 1 vcgencmd measure_temp
}

canup() {
    sudo ip link set can0 up type can bitrate 500000 restart-ms 100 sjw 3
    sudo ip link set can0 txqueuelen 100
}

candown() {
    sudo ip link set can0 down
}

DDS_ENV_FILE="$HOME/.ros_dds_env"

if [ -f "$DDS_ENV_FILE" ]; then
    source "$DDS_ENV_FILE"
fi

local_dds() {
    echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' > "$DDS_ENV_FILE"
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
}

subnet_dds() {
    echo 'export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET' > "$DDS_ENV_FILE"
    export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
}


# ===== ZEMO ROS 2 CONFIG END =====
EOF

source "$HOME/.bashrc"