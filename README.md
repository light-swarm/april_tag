april_tag
=========

Detects 2D fiducial markers (april tags) from ros image stream and produces id, location and orientation of the tags. This ros node wraps the C++ April Tag library written by Michael Kaess and Hordur Johannson. April tags were developed by Edwin Olson. 

More on april tags here:
http://april.eecs.umich.edu/wiki/index.php/AprilTags

April Tags C++ library:
http://people.csail.mit.edu/kaess/apriltags/

Input:

`/camera/image_raw`

Output:

AprilTagList which is a list of AprilTag:

```
uint32 	id
uint32 	hamming_distance
float64 distance
float64 x
float64 y
float64 z
float64 yaw
float64 pitch
float64 roll
```

distance,x,y,z are in cms. z is depth away from camera. x is horizontal with camera right as positive. 

Depends on: libeigen3-dev

-- palash







