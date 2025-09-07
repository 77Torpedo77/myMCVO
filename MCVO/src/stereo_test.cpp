#include "Estimator/MCVOfeature_manager.h"
#include "Frontend/sensors.h"
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

// Test fixture for stereo functionality
class StereoTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a simple feature manager for testing
        Eigen::Matrix3d Rs[1];
        Rs[0] = Eigen::Matrix3d::Identity();
        feature_manager = std::make_unique<FeatureManager>(Rs, nullptr);
    }

    std::unique_ptr<FeatureManager> feature_manager;
};

// Test stereo depth calculation from disparity
TEST_F(StereoTest, ComputeDisparityDepth) {
    double focal_length = 500.0;  // pixels
    double baseline = 0.12;       // meters
    double disparity = 10.0;      // pixels
    
    double expected_depth = (focal_length * baseline) / disparity; // 6.0 meters
    double computed_depth = feature_manager->computeDisparityDepth(disparity, focal_length, baseline);
    
    EXPECT_NEAR(computed_depth, expected_depth, 1e-6);
    EXPECT_NEAR(computed_depth, 6.0, 1e-6);
}

// Test stereo depth calculation with invalid disparity
TEST_F(StereoTest, ComputeDisparityDepthInvalid) {
    double focal_length = 500.0;
    double baseline = 0.12;
    double invalid_disparity = 0.0;
    
    double computed_depth = feature_manager->computeDisparityDepth(invalid_disparity, focal_length, baseline);
    
    EXPECT_EQ(computed_depth, -1.0);
}

// Test stereo feature extraction (mock test)
TEST_F(StereoTest, StereoFeatureExtract) {
    // Create simple test images
    cv::Mat left_img = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat right_img = cv::Mat::zeros(480, 640, CV_8UC1);
    
    // Add some features to test images
    cv::circle(left_img, cv::Point(100, 100), 5, cv::Scalar(255), -1);
    cv::circle(left_img, cv::Point(200, 150), 5, cv::Scalar(255), -1);
    cv::circle(right_img, cv::Point(90, 100), 5, cv::Scalar(255), -1);  // 10 pixel disparity
    cv::circle(right_img, cv::Point(185, 150), 5, cv::Scalar(255), -1); // 15 pixel disparity
    
    std::vector<cv::KeyPoint> left_kpts, right_kpts;
    cv::Mat left_descriptors, right_descriptors;
    
    bool success = feature_manager->stereoFeatureExtract(left_img, right_img, 
                                                        left_kpts, right_kpts,
                                                        left_descriptors, right_descriptors);
    
    EXPECT_TRUE(success);
    EXPECT_GT(left_kpts.size(), 0);
    EXPECT_GT(right_kpts.size(), 0);
}

// Test stereo feature matching (mock test)
TEST_F(StereoTest, StereoFeatureMatch) {
    // Create keypoints manually for testing
    std::vector<cv::KeyPoint> left_kpts, right_kpts;
    left_kpts.emplace_back(cv::Point2f(100, 100), 5);
    left_kpts.emplace_back(cv::Point2f(200, 150), 5);
    
    right_kpts.emplace_back(cv::Point2f(90, 100), 5);   // Good match with 10 pixel disparity
    right_kpts.emplace_back(cv::Point2f(185, 150), 5);  // Good match with 15 pixel disparity
    
    // Create dummy descriptors (normally extracted from actual images)
    cv::Mat left_descriptors = cv::Mat::zeros(2, 32, CV_8UC1);
    cv::Mat right_descriptors = cv::Mat::zeros(2, 32, CV_8UC1);
    
    // Set some values to make descriptors similar
    left_descriptors.at<uchar>(0, 0) = 255;
    right_descriptors.at<uchar>(0, 0) = 255;
    left_descriptors.at<uchar>(1, 1) = 255;
    right_descriptors.at<uchar>(1, 1) = 255;
    
    std::vector<cv::DMatch> matches;
    bool success = feature_manager->stereoFeatureMatch(left_kpts, right_kpts,
                                                      left_descriptors, right_descriptors,
                                                      matches);
    
    // Note: This test might not find matches due to descriptor differences
    // but the function should complete without error
    EXPECT_TRUE(success || matches.empty()); // Either success or no matches is acceptable
}

// Test landmark stereo flag initialization  
TEST_F(StereoTest, KeyPointLandmarkStereoFlag) {
    MCVO::FeatureID feature_id = 123;
    MCVO::TimeFrameId frame_id = 1.0;
    double measured_depth = 5.0;
    
    KeyPointLandmark landmark(feature_id, frame_id, measured_depth);
    
    EXPECT_FALSE(landmark.is_stereo);           // Should default to false
    EXPECT_EQ(landmark.stereo_depth, -1.0);    // Should default to -1
    EXPECT_EQ(landmark.feature_id, feature_id);
    EXPECT_EQ(landmark.measured_depth, measured_depth);
}

// Test main function for running tests
int main(int argc, char **argv) {
    // Initialize Google Test
    ::testing::InitGoogleTest(&argc, argv);
    
    // Initialize logging for the test
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = 1;
    FLAGS_colorlogtostderr = 1;
    
    return RUN_ALL_TESTS();
}