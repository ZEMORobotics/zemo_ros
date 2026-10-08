# This is for ros2_control, in order to maintain real time scheculing, ros2_control needs higher priority which is not normally allowed for user space.
# This script contains all commands to enable that, and it should not be run again.

set -e

USER_NAME="${SUDO_USER:-$USER}"

# Create realtime group if it doesn't already exist
getent group realtime > /dev/null || sudo groupadd realtime

# Add current user to realtime group
sudo usermod -aG realtime "$USER_NAME"

# Configure realtime limits
sudo tee /etc/security/limits.d/99-realtime.conf > /dev/null <<EOF
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock unlimited
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock unlimited
EOF

echo "Realtime permissions configured."
echo "Reboot required."

# After reboot
# groups
# ulimit -r
# ulimit -l