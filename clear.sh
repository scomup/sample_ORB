echo "Cleaning test 00 ..."
cd test/00_test_orbextractor
rm -rf build
cd ../../

echo "Cleaning test 01 ..."
cd test/01_test_frame
rm -rf build
cd ../../

echo "Cleaning test 02 ..."
cd test/02_test_matcher
rm -rf build
cd ../../

echo "Cleaning test 03 ..."
cd test/03_test_initializer
rm -rf build
cd ../../

echo "Cleaning test 04 ..."
cd test/04_test_keyframe
rm -rf build
cd ../../

echo "Cleaning test 05 ..."
cd test/05_test_mappoint
rm -rf build
cd ../../

echo "Cleaning test 06 ..."
cd test/06_test_map
rm -rf build
cd ../../

echo "Cleaning test 07 ..."
cd test/07_test_poseOptimizer
rm -rf build
cd ../../

echo "Cleaning test 08 ..."
cd test/08_test_tracking
rm -rf build
cd ../../

echo "Cleaning test 09 ..."
cd test/09_test_newKeyframe
rm -rf build
cd ../../

echo "Cleaning test 10 ..."
cd test/10_test_trackLocalmap
rm -rf build
cd ../../

echo "Cleaning test 11 ..."
cd test/11_test_bundleAdjustment
rm -rf build
cd ../../

echo "Cleaning g2o ..."
cd Thirdparty/g2o
rm -rf build
cd ../../
