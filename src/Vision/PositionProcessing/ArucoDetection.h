#ifndef ARUCO_DETECTION_H
#define ARUCO_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <array>
#include <mutex>
#include <string>
#include "Entity/Entity.h"
#include "Utils/EnumsAndConstants.h"

#define ARUCO_CONFIG_FILE "Config/ArucoConfig.json"
#define ARUCO_DETECTOR_LABEL "ARUCO"

class ArucoDetection {
public:
    static constexpr int kEntityCount = 6;
    static constexpr int kInvalidId = -1;

    ArucoDetection();
    ~ArucoDetection() = default;

    void init();

    void runFromFrame(const cv::Mat &bgrFrame);

    void getDebugFrame(cv::Mat &frame);

    void setMarkerId(int entityIndex, int markerId);
    int  getMarkerId(int entityIndex) const;

    void setBallHsvRange(const cv::Scalar &lower, const cv::Scalar &upper, int minArea);

    void setTeamColor(int teamColor);
    int  getTeamColor() const;

    bool loadFromFile(const std::string &path = ARUCO_CONFIG_FILE);
    bool saveToFile  (const std::string &path = ARUCO_CONFIG_FILE) const;

private:
    cv::Ptr<cv::aruco::Dictionary>          _dictionary;
    cv::Ptr<cv::aruco::DetectorParameters>  _parameters;

    std::array<int, kEntityCount> _markerForEntity;

    cv::Scalar _ballLowerHsv;
    cv::Scalar _ballUpperHsv;
    int        _ballMinArea;

    int        _myTeamColor;

    cv::Mat _debugFrame;
    mutable std::mutex _locker;

    void detectBall(const cv::Mat &bgrFrame, Entity &ball);
};

#endif
