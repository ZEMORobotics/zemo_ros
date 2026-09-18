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

# ===== ZEMO ROS 2 CONFIG END =====
EOF