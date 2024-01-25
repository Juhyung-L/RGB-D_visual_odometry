# RGB-D_visual_odometry
This ROS node takes in image frame and point cloud from a RGB-D camera to estimate the 3D odometry of the moving robot.

How it works:
- Extract SIFT features from each image frame and match it to the features from the previous frame.
- Get the 3D points corresponding to the feature points (both current-frame feature and previous-frame feature).
- Use Singular Value Decomposition (SVD) to find the rigid body transformation between two point clouds.

Detailed explanation at: https://juhyungsprojects.blogspot.com/2024/01/rgb-d-visual-odometry.html

https://github.com/Juhyung-L/RGB-D_visual_odometry/assets/102873080/37268c18-0aa7-4dfe-9798-ea1f87e46147

[![Watch the video](https://img.youtube.com/vi/NPGlUDOf-rE/maxresdefault.jpg)](https://youtu.be/NPGlUDOf-rE)

![odometry_diagram_spin](https://github.com/Juhyung-L/RGB-D_visual_odometry/assets/102873080/47bb46aa-2449-400b-946c-279278bb2dae)
