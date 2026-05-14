#include "ArucoDetection.h"

#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <vector>

#include "GameInfo/GameInfo.h"
#include "Logging/logging.h"
#include "Utils/Utils.h"

ArucoDetection::ArucoDetection()
    : _ballLowerHsv(6, 80, 100),
      _ballUpperHsv(30, 255, 255),
      _ballMinArea(80),
      _myTeamColor(Color::BLUE)
{
    _markerForEntity.fill(kInvalidId);
    init();
}

void ArucoDetection::setTeamColor(int teamColor)
{
    std::lock_guard<std::mutex> guard(_locker);
    _myTeamColor = teamColor;
}

int ArucoDetection::getTeamColor() const
{
    std::lock_guard<std::mutex> guard(_locker);
    return _myTeamColor;
}

void ArucoDetection::init()
{
    _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    _parameters = cv::aruco::DetectorParameters::create();

    if (!loadFromFile()) {
        spdlog::get("General")->info(
            "ArucoDetection: no config file found at {}, using defaults", ARUCO_CONFIG_FILE);
    }
}

void ArucoDetection::setMarkerId(int entityIndex, int markerId)
{
    std::lock_guard<std::mutex> guard(_locker);
    if (entityIndex < 0 || entityIndex >= kEntityCount) return;
    _markerForEntity[entityIndex] = markerId;
}

int ArucoDetection::getMarkerId(int entityIndex) const
{
    std::lock_guard<std::mutex> guard(_locker);
    if (entityIndex < 0 || entityIndex >= kEntityCount) return kInvalidId;
    return _markerForEntity[entityIndex];
}

void ArucoDetection::setBallHsvRange(const cv::Scalar &lower, const cv::Scalar &upper, int minArea)
{
    std::lock_guard<std::mutex> guard(_locker);
    _ballLowerHsv = lower;
    _ballUpperHsv = upper;
    _ballMinArea  = minArea;
}

void ArucoDetection::detectBall(const cv::Mat &bgrFrame, Entity &ball)
{
    if (bgrFrame.empty()) return;

    cv::Mat hsv;
    cv::cvtColor(bgrFrame, hsv, cv::COLOR_BGR2HSV);

    cv::Mat mask;
    cv::inRange(hsv, _ballLowerHsv, _ballUpperHsv, mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return;

    // Pick the most ball-like contour: area >= min_area AND highest circularity score.
    // circularity = 4*pi*area / perimeter^2  ∈ (0, 1]; perfect circle ~ 1.0
    const double kMinCircularity = 0.55;

    int    bestIdx     = -1;
    double bestScore   = -1.0;
    double bestArea    = 0.0;

    for (size_t i = 0; i < contours.size(); ++i) {
        double area = cv::contourArea(contours[i]);
        if (area < _ballMinArea) continue;

        double perim = cv::arcLength(contours[i], true);
        if (perim <= 0.0) continue;

        double circularity = (4.0 * CV_PI * area) / (perim * perim);
        if (circularity < kMinCircularity) continue;

        // Score favors round-and-reasonably-sized blobs. Tweak if needed.
        double score = circularity * std::log(area + 1.0);
        if (score > bestScore) {
            bestScore = score;
            bestIdx   = static_cast<int>(i);
            bestArea  = area;
        }
    }

    if (bestIdx < 0) {
        // Fall back: nothing passed circularity test — use biggest blob ≥ min_area
        // (helps when motion blur deforms the contour).
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area >= _ballMinArea && area > bestArea) {
                bestArea = area;
                bestIdx  = static_cast<int>(i);
            }
        }
        if (bestIdx < 0) return;
    }

    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contours[bestIdx], center, radius);

    Point cmPos = Utils::convertPositionPixelToCm(Point(center.x, center.y));
    ball.update(cmPos, 0.0);
    ball.id(0);

    cv::circle(_debugFrame, center, static_cast<int>(radius), cv::Scalar(0, 165, 255), 2);
    cv::putText(_debugFrame, "ball",
                cv::Point(static_cast<int>(center.x + radius + 4), static_cast<int>(center.y)),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 165, 255), 1);
}

void ArucoDetection::runFromFrame(const cv::Mat &bgrFrame)
{
    if (bgrFrame.empty()) return;

    std::lock_guard<std::mutex> guard(_locker);

    _debugFrame = bgrFrame.clone();

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(bgrFrame, _dictionary, corners, ids, _parameters);

    int myTeam    = _myTeamColor;
    int enemyTeam = (myTeam == Color::YELLOW) ? Color::BLUE : Color::YELLOW;

    Players allPlayers(kEntityCount, Entity());
    for (int i = 0; i < kEntityCount; ++i) {
        int teamColor = (i < 3) ? myTeam    : enemyTeam;
        int localId   = (i < 3) ? i         : (i - 3);
        allPlayers[i].id(static_cast<uint>((teamColor - 1) * 100 + localId));
        allPlayers[i].team(static_cast<uint>(teamColor));
        allPlayers[i].outdate();
    }

    if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(_debugFrame, corners, ids);

        for (size_t i = 0; i < ids.size(); ++i) {
            int markerId = ids[i];

            int entityIndex = -1;
            for (int e = 0; e < kEntityCount; ++e) {
                if (_markerForEntity[e] == markerId) {
                    entityIndex = e;
                    break;
                }
            }
            if (entityIndex < 0) continue;

            const auto &mc = corners[i];
            float cx = (mc[0].x + mc[1].x + mc[2].x + mc[3].x) * 0.25f;
            float cy = (mc[0].y + mc[1].y + mc[2].y + mc[3].y) * 0.25f;

            float dx = mc[1].x - mc[0].x;
            float dy = mc[1].y - mc[0].y;
            double angle = std::atan2(dy, dx);

            Point cmPos = Utils::convertPositionPixelToCm(Point(cx, cy));
            allPlayers[entityIndex].update(cmPos, angle);

            int len = 30;
            cv::Point p1(static_cast<int>(cx), static_cast<int>(cy));
            cv::Point p2(static_cast<int>(cx + len * std::cos(angle)),
                         static_cast<int>(cy + len * std::sin(angle)));
            cv::line(_debugFrame, p1, p2, cv::Scalar(0, 255, 0), 2);
        }
    }

    Entity ball(0);
    ball.outdate();
    detectBall(bgrFrame, ball);

    vss.setEntities(ball, allPlayers);
}

void ArucoDetection::getDebugFrame(cv::Mat &frame)
{
    std::lock_guard<std::mutex> guard(_locker);
    if (!_debugFrame.empty()) frame = _debugFrame.clone();
}

bool ArucoDetection::loadFromFile(const std::string &path)
{
    std::ifstream f(path);
    if (!f.is_open()) return false;

    std::stringstream ss;
    ss << f.rdbuf();
    std::string content = ss.str();

    auto findInt = [&](const std::string &key, int &out) -> bool {
        std::string needle = "\"" + key + "\"";
        size_t k = content.find(needle);
        if (k == std::string::npos) return false;
        size_t colon = content.find(':', k);
        if (colon == std::string::npos) return false;
        size_t start = colon + 1;
        while (start < content.size() && (content[start] == ' ' || content[start] == '\t')) start++;
        size_t end = start;
        while (end < content.size() && (std::isdigit(content[end]) || content[end] == '-')) end++;
        if (end == start) return false;
        try {
            out = std::stoi(content.substr(start, end - start));
            return true;
        } catch (...) { return false; }
    };

    std::lock_guard<std::mutex> guard(_locker);
    static const std::array<std::string, kEntityCount> keys = {
        "ROBOT1", "ROBOT2", "ROBOT3", "ADV1", "ADV2", "ADV3"
    };
    for (int i = 0; i < kEntityCount; ++i) {
        int v;
        if (findInt(keys[i], v)) _markerForEntity[i] = v;
    }
    int v;
    if (findInt("ballMinArea", v))    _ballMinArea = v;
    int hLo, sLo, vLo, hHi, sHi, vHi;
    if (findInt("ballHueLow",  hLo) && findInt("ballSatLow",  sLo) && findInt("ballValLow",  vLo) &&
        findInt("ballHueHigh", hHi) && findInt("ballSatHigh", sHi) && findInt("ballValHigh", vHi)) {
        _ballLowerHsv = cv::Scalar(hLo, sLo, vLo);
        _ballUpperHsv = cv::Scalar(hHi, sHi, vHi);
    }
    return true;
}

bool ArucoDetection::saveToFile(const std::string &path) const
{
    std::ofstream f(path);
    if (!f.is_open()) return false;

    std::lock_guard<std::mutex> guard(_locker);

    f << "{\n";
    f << "  \"ROBOT1\": " << _markerForEntity[0] << ",\n";
    f << "  \"ROBOT2\": " << _markerForEntity[1] << ",\n";
    f << "  \"ROBOT3\": " << _markerForEntity[2] << ",\n";
    f << "  \"ADV1\":   " << _markerForEntity[3] << ",\n";
    f << "  \"ADV2\":   " << _markerForEntity[4] << ",\n";
    f << "  \"ADV3\":   " << _markerForEntity[5] << ",\n";
    f << "  \"ballHueLow\":  " << static_cast<int>(_ballLowerHsv[0]) << ",\n";
    f << "  \"ballSatLow\":  " << static_cast<int>(_ballLowerHsv[1]) << ",\n";
    f << "  \"ballValLow\":  " << static_cast<int>(_ballLowerHsv[2]) << ",\n";
    f << "  \"ballHueHigh\": " << static_cast<int>(_ballUpperHsv[0]) << ",\n";
    f << "  \"ballSatHigh\": " << static_cast<int>(_ballUpperHsv[1]) << ",\n";
    f << "  \"ballValHigh\": " << static_cast<int>(_ballUpperHsv[2]) << ",\n";
    f << "  \"ballMinArea\": " << _ballMinArea << "\n";
    f << "}\n";
    return true;
}
