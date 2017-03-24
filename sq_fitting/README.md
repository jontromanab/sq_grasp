# Superquadics Fitting
============================

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
