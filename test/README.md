###Lesson00: 00_test_orbextractor
This lesson introduces the orbextractor module for ORB_SLAM2(feature/keypoint extractor) and How does it Work.

###Lesson01: 01_test_frame
This lesson introduces the Frame module. A Frame contained all keypoints, All keypoints are saved in differnet grids for high-speed access. The camera information also saved for each Frame.

###Lesson02: 02_test_matcher
This lesson introduces the Matcher module. The Matcher module responsible for find matched keypoints for 2 frame by different method. The original ORB_SLAM2 uses matching method including: Searching Nearby, Projection and Bag of words. at here we only implement Searching Nearby.

###Lesson03: 03_test_initializer
This lesson introduces the initializer module. ORB_SLAM2 RANSAC finds homography and fundamental matrix separately. and chooses better one to reconstituted the 3d position for keypoints.

###Lesson04: 04_test_keyframe
We will use 3 lesson to build completed map(Lesson04~06). basicly a completed map include map, keyframe and mappoint. at lesson4, we talk about the keyframe module first. A keyframe is choiced by certain rule from frames. ORB_SLAM2 access mappoint through keyframe, instead access them directly. 

###Lesson05: 05_test_mappoint
At lesson5, we talk about the mappoint. mappoint is the basic element for the map of ORB_SLAM2. MapPoint is a landmark in real environment, help us make sure the pose of camera. Mappoint must have a 3d coordinate for image projection, and a descriptor for find a matching keypoint in image.

###Lesson06: 06_test_map
Finally we will build a completed map at lesson6




