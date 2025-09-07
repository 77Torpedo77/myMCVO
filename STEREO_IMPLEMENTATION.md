# Multi-Stereo Scale Estimation Implementation

This document describes the implementation of multi-stereo camera scale estimation functionality in MCVO.

## Overview

The system has been extended to support stereo-based scale estimation as an alternative to the existing monocular triangulation approach. When stereo cameras are available, the system uses disparity-based depth calculation for more accurate and robust scale estimation.

## Key Components

### 1. Feature Structure Extensions (`MCVOfeature_manager.h`)

- **KeyPointLandmark::is_stereo**: Boolean flag to distinguish stereo vs monocular features
- **KeyPointLandmark::stereo_depth**: Stores depth calculated from stereo disparity
- **EstimateFlag::STEREO_DISPARITY**: New estimation type for stereo-derived depths

### 2. Stereo Depth Calculation (`MCVOfeature_manager.cpp`)

- **calculateStereoDepth()**: Main function for computing depth from stereo disparity
- **computeDisparityDepth()**: Utility function implementing depth = (f×B)/disparity
- **stereoFeatureExtract()**: ORB-based feature extraction for stereo image pairs
- **stereoFeatureMatch()**: Robust stereo matching with epipolar constraints

### 3. Optimization Framework (`stereo_disparity_factor.*`)

- **StereoDisparityFactor**: Ceres cost function constraining estimated vs stereo depth
- **MultiStereoScaleFactor**: Ensures scale consistency across multiple stereo cameras
- Integrated into main optimization loop with weighted residuals

### 4. Mixed Configuration Support

- **propagateStereoScaleToMono()**: Transfers stereo scale to co-visible monocular features
- **hasStereoCamera()**: Utility to detect stereo camera availability
- **countStereoLandmarks()**: Statistics for stereo landmark coverage

## Usage Flow

1. **Initialization**: System detects stereo cameras and prioritizes stereo depth in initialization
2. **Feature Processing**: For each frame:
   - Extract features from left/right images if stereo config unavailable
   - Match features with epipolar and ratio constraints
   - Calculate disparity and convert to depth
3. **Scale Estimation**: 
   - Use stereo depth directly for stereo features
   - Propagate scale to monocular features through co-visibility
4. **Optimization**: Add stereo disparity constraints alongside reprojection errors

## Configuration Requirements

### For Existing Stereo Setup:
- Ensure `sensor_type: STEREO` in camera configuration
- Provide baseline and focal length parameters
- Stereo depth will be computed automatically

### For Manual Stereo Matching:
- System falls back to feature extraction and matching
- Requires left/right image topics
- May have reduced accuracy compared to calibrated stereo

## Performance Considerations

- Stereo constraints add computational cost but improve accuracy
- Scale propagation helps in mixed mono/stereo scenarios
- Epipolar filtering reduces mismatches in stereo matching

## Testing

Basic tests are provided in `stereo_test.cpp` covering:
- Disparity to depth conversion
- Feature extraction and matching
- Landmark initialization with stereo flags

## Future Enhancements

- Adaptive stereo constraint weighting based on disparity quality
- More sophisticated scale propagation algorithms
- Integration with visual-inertial initialization
- Support for rectified vs unrectified stereo images