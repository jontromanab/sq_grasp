# Superquadics Fitting
============================
![fitting](https://user-images.githubusercontent.com/3790876/31579784-003e5e78-b0fb-11e7-8acc-b652655fbdbf.jpg)


### Real time superquadric fitting on objects in dense clutter

A superquadric can be defined a generalized quadric in
which the exponents of the implicit representation of the sur-
face are arbitrary real numbers, allowing for a more flexible
set of shapes while keeping the symmetry characteristics of
the regular quadric. A superquadric can be obtained as the spherical
product of two superellipses *S1* and *S2* , to obtain the
parametric equation


![screenshot from 2017-03-24 03 42 51](https://cloud.githubusercontent.com/assets/3790876/24288733/210b1a90-1044-11e7-829a-a70ffa63d904.png)

with *n* is between *-PI/2* and *PI/2* and *w* is between *-PI* and *PI*, where *a1, a2, a3* are the scaling factors of the three principal axes, the exponent *e1* controls the shape of the superellipsoid's cross-section in the planes orthogonal to *(x,y)* plane and *e2* controls the shape of the superellipsoid's cross-section parallel to the *(x,y)* plane. The pose of the super quadric with respect to a world frame is specified by the six parameters that define a rigid motion, *Px, Py, Pz* for the position and *Qx, Qy, Qz, Qw* for the orientation. The superquadric can also be expressed using an implicit equation in normal form as


![eq](https://cloud.githubusercontent.com/assets/3790876/24290189/ab40168e-1049-11e7-9fad-9948dc7a9920.png)


To fit the superquadric model to a point cloud data, distance *d* from the points to the function *f(a,x,y,z)* must be minimized  in terms of radial distance *||OP||*

![eq2](https://cloud.githubusercontent.com/assets/3790876/24290336/3526a5d4-104a-11e7-91fe-19322148df95.png)


### How to run??

To fit superquadrics on objects in real time, first setup the launch file. 
1. Setup the cloud_topic
* for kinect1: **/camera/depth_registered/points**
* for kinect2: **/kinect2/sd/points**
2. Setup the workspace


The workspace takes 6 parameters -x, +x, -y, +y, -z, +z which are defined in the camera frame ( Special attention should be provided while using the package where the camera is connected to the robot. ~~The parameters are in world frame.~~ **Parameters are in camera frame**)

3. Setup segmentation parameters
The pacakge relies on LCCP (Local Convexity connected pathes) segmentation for segmenting objects in dense clutter. After the table plane is removed, the objects on the table are clustered into individual objects. Most of the parameters are for supervoxel and lccp segmentation. The extra parameters are **zmin**(minimum distance from the z-plane), **zmax**(minimum distance from the z-plane) and **th_points**(Number of points to be considered as an object). The bool parameter **remove_nan** decides to remove the nan points from the online cloud.

Start the kinect: (for kinect1)

**roslaunch openni_launch openni.launch**

run the superquadrics node

**roslaunch sq_fitting sq_fit.launch**

#### Published Topics:
* **superq/filtered_cloud/** (point cloud) publishes the filtered cloud
* **superq/table/** (point cloud) publishes the segmented table only
* **superq/tabletop_objects/** (point cloud) publishes the objects on the table
* **superq/segmented_objects/** (point cloud) publishes the segmented individual objects with different colors
* **superq/superquadrics/** (point cloud) publises sampled superquadrics

![qjmhk1490359827](https://cloud.githubusercontent.com/assets/3790876/24294828/c4fc270c-105d-11e7-9248-cb32d9a5ea37.jpg)

To fit superquadrics on a pcd file, run

**rosrun sq_fitting seg_and_fit_test_pcd /your pcd file name**

It will generated a new pcd file called objects_superquadrics.pcd
