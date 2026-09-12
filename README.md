# ackermann
## Luu map
```bash
# Lưu ảnh occupancy grid (để xem bằng map_server/RViz)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/rosdev/ros2_ws/maps/my_map'}}"

# Lưu pose graph (bắt buộc để dùng cho localization sau này)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/rosdev/ros2_ws/maps/my_map'}"
```