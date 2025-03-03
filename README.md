# SET UP #

## AutoPep8 ##
# TODO

When you run the `SetUpDevelopmentEnvironment.sh` script, the up-to-date clang formatting file should be moved into your workspace.

## VARIABLE NAMING CONVENTION ##

Variables follow the camel case naming convention with the following format.

`<name>_<frame>_<unit>`

Note: If one or more of the tags are not required then it can be omitted from the variable. For example, if a vector is independent of a frame, a frame
tag should not be included. (The obvious exception is the 'name' tag)

Note: These conventions were introduced after the start of development and hence not all the code may follow it. It is advised that the variable naming conventions be changed at any available opportunity.

### name ###

This is the name of the variable. Generally, it should contain at least 2 descriptive words, although there may be exceptions to having 1. The purpose is to make it easy to identify what the variable represents.


### frame ###

This is to indicate the frame which the vector is oriented against. It does not necessarily mean that it is the frame the variable is in as during transitions you can have a vector that is within an intermediary frame. If a vector is in the orientation of frame1 but has its origin in frame 2 then the 'frame' is frame1 but the vector is relative to frame2. Hence the term 'RelTo\<Frame2\>' should be used in the name tag.

For variables that represent a rotation, this should clearly indicate the starting frame and which frame the rotation will result in. For example, say a quaternion variable represents a rotation from frame1 to frame 2, the frame tag will say '\<frame1\>To\<Frame2\>'

### unit ###

This part of the tag represents the unit of the variable. All units should be in lowercase, except for places where it may be easier to read if camel case was used. Numbers represent the power to raise the unit to the left.

'p' is reserved for per.

For example: kg m per second squared (Newton) is kgmps2

# FRAMES #

Unless in very specific situations, all frames can be assumed to follow a right-handed orthogonal axis system. For more information about rotations, read the ** ROTATION SEQUENCES** section.

|       Frame Name      | Abbreviation |                 Description                  |     Origin    |  Definition of x-axis  |   Definition of y-axis   |     Definition of z-axis     |                                     Note                                                           |
|:---------------------:|:------------:|:--------------------------------------------:|:-------------:|:----------------------:|:------------------------:|:----------------------------:|:--------------------------------------------------------------------------------------------------:|
|       Fixed Frame     |     Fix      | Origin which all frames are based around.    |    [0,0,0]    |       [1,0,0]          |          [0,1,0]         |            [0,0,1]           |                                                                                                    |
|       Body Frame      |     Bod      | Frame used to define the body of rigid body. |      COG      |   Longitudinal Axis    |       Lateral Axis       |    Cross Product of x & y    | Note that definition may vary for some bodies.                                                     |
|     Sensor Frame      |     Sen      | Frame which the sensor is orientated.        | Sensor Origin |     Sensor x-axis      |       Sensor y-axis      |        Sensor z-axis         | This frame is dependent on the sensor it is representing. Also used for describing hardware frames |

# VECTORS #

# Cartesian Vectors #

All cartesian vectors shall be broken down into the form:

cartesianVector = [

  X_COMPONENTS,

  Y_COMPONENT,

  Z_COMPONENT

]

# Spherical Vectors #

All spherical vectors shall be broken down into the form:

sphericalVector = [

  RADIUS,

  AZIMUTH,

  ELEVATION

]

NOTE: azimuth and elevation can be referred to as longitude and latitude.

**Azimuth**: Defined as the angle between the projection of the radius vector on the xy plane and x-axis, with positive rotation being applied as the right-hand rule around the z-axis.

**Elevation**: Defined as the angle between the projection of the radius vector on the xy plane and the radius vector, with positive rotation applied as the right-hand rule on the x-axis.

### Units for a Spherical Vector ###

There is a problem with tagging the units for spherical vector variables. This is because there is a distance and an angle in the same vector. To get around this, the angles will **ALWAYS** be in radians. Hence the unit tag is there to describe the units of the radius element.
# ROTATIONS #
## QUATERNIONS ##

The standard for quaternions in this project is:

quaternion = [

  X_QUATERNION_COMPONENT,

  Y_QUATERNION_COMPONENT,

  Z_QUATERNION_COMPONENT,

  S_QUATERNION_COMPONENT

]

where, s represents the scalar component.

If applicable, a quaternion can also be broken down into its scalar and vector components:

quaternion_gibs_vector = [

  X_QUATERNION_COMPONENT,

  Y_QUATERNION_COMPONENT,

  Z_QUATERNION_COMPONENT

]

quaternion_scaler = S_QUATERNION_COMPONENT

## EULER ANGLES ##

The standard for Euler Angle vectors is 123 (XYZ).

eulerAngle = [

  ROLL,

  PITCH,

  YAW
  
]

## ROTATION SEQUENCES ##

Whenever Euler angles are involved, the rotation sequence is 321 (ZYX).

Positive rotation is defined with respect to the right-hand rule.

Yaw is defined as rotation around the z-axis
Pitch is defined as rotation around the y-axis
Roll is defined as rotation around the x-axis
