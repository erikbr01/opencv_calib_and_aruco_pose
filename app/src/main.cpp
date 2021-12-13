#include "include_helper.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

const float arucoSquareDimension = 0.162f;

// Simple function to detect markers
void detectMarker(ArucoFunctions detector) {
  Ptr<cv::aruco::Dictionary> dictionary =
      aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

  // cv::Ptr<cv::aruco::Dictionary> dictionary =
  //     cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  // detector.createArucoMarker(dictionary);

  Mat image;

  // Create window
  namedWindow("Display window");

  // Get first webcam capture
  VideoCapture cap(0);

  if (!cap.isOpened()) {
    cout << "cannot open camera";
  }

  while (true) {
    // update the image
    cap >> image;

    // detect markers
    image = detector.DetectArucoMarker(image, dictionary);

    // show image
    imshow("Display window", image);

    // wait 25 milliseconds for key interrupt
    waitKey(25);
  }
}

// Monitors webcam and draws axis of any detected marker
int startWebcamMonitoring(const Mat &cameraMatrix,
                          const Mat &distanceCoefficients,
                          float arucoSquareDimension) {
  Mat frame;

  vector<int> markerIds;
  vector<vector<Point2f>> markerCorners, rejectedCandidates;
  aruco::DetectorParameters parameters;

  Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(
      aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_250);

  VideoCapture vid(2);

  if (!vid.isOpened()) {
    return -1;
  }
  namedWindow("Webcam", WINDOW_AUTOSIZE);

  vector<Vec3d> rotationVectors, translationVectors;

  while (true) {
    if (!vid.read(frame))
      break;

    aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
    aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension,
                                     cameraMatrix, distanceCoefficients,
                                     rotationVectors, translationVectors);

    for (int i = 0; i < markerIds.size(); i++) {
      aruco::drawAxis(frame, cameraMatrix, distanceCoefficients,
                      rotationVectors[i], translationVectors[i], 0.1f);
    }

    imshow("Webcam", frame);

    if (waitKey(30) >= 0)
      break;
  }
  return 1;
}

int main() {

  Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
  Mat distanceCoefficients;

  ArucoFunctions detector;
  Calibrator calib;
  // calib.camera_calib(cameraMatrix, distanceCoefficients);
  calib.loadCamerCalibration("camCalibration", cameraMatrix,
                             distanceCoefficients);
  cout << "Starting webcam monitoring" << endl;
  startWebcamMonitoring(cameraMatrix, distanceCoefficients,
                        arucoSquareDimension);
  return 0;
}
