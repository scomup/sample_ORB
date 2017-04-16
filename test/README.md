### Lesson00: 00_test_orbextractor
This lesson introduces the orbextractor module for ORB_SLAM2(feature/keypoint extractor) and how does it Work.

### Lesson01: 01_test_frame
This lesson introduces the Frame module. A Frame contained all keypoints, keypoints are saved in differnet grids for high-speed access. The camera information also be saved in Frame.

### Lesson02: 02_test_matcher
This lesson introduces the Matcher module. The Matcher module responsible for find matched keypoints form 2 frame use different method. The original ORB_SLAM2 uses matching method including: Searching Nearby, Searching by Projection and Searching by Bag of words. at here we only implement Searching Nearby.

### Lesson03: 03_test_initializer
This lesson introduces the initializer module. ORB_SLAM2 uses RANSAC to find homography and fundamental matrix separately. and chooses better one to reconstituted 3d position of the keypoints.

### Lesson04: 04_test_keyframe
We will use 3 lesson to build completed map(Lesson04~06). basicly a completed map module is composed of 3 sub-modules: map, keyframe and mappoint. at lesson4, we talk about the keyframe module first. A keyframe is choiced by certain rule from frames. ORB_SLAM2 access mappoint through keyframe, instead of access them directly. 

### Lesson05: 05_test_mappoint
At lesson5, we talk about the mappoint. mappoint is the basic element for the map of ORB_SLAM2. MapPoint is a landmark in real environment, help us compute the pose of camera. Mappoint must have a 3d coordinate to image projection, and a descriptor in order to find matched keypoints in image.

### Lesson06: 06_test_map
Finally we will build completed map at lesson6.　A module for Visualization also be implemented in this lesson.

### Lesson07: 07_test_poseOptimizer
Once ORB_SLAM2 create map, The Optimizer is used to minimized the projection error to find current camera pose. 

### Lesson08: 08_test_tracking
This lesson introduces the tracking module. we can locate the camera pose continuously in this range of the initial map by tracking module. 

### Lesson09: 09_test_newKeyframe
This lesson, we insert new keyframe into the environment map. SLAM try to buiding a map of an unknown environment and locating the robot simultaneously. With the above steps, we can locate the camera pose continuously in this range of the initial map(lesson08) and build environment map. In fact, the basic SLAM has been implemented. 






