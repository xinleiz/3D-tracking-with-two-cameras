3D-tracking-with-two-cameras
============================
Using two calibrated cameras to track one point's 3D position in real world.
Key features for tracking:

1. Tracking using findBlob

2. Transfer matrix between World Coordinate and Camera Coordiante is calculated using SolvePnP based on 4 point pairs.

3. Calculation for Matrix that transfer Points in Camera coordinate to World Coordinate mainly based on this link:http://stackoverflow.com/questions/13957150/opencv-computing-camera-position-rotation

4. 3D postion in world coordinate is calculated by the shortest line between two 3d lines.
http://paulbourke.net/geometry/pointlineplane/
