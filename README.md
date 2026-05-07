## VSS-Vision (Sysmic Robotics fork)

Vision system for Very Small Size Soccer (VSSS). Captures from USB or FLIR Spinnaker
cameras, detects robots and ball, and broadcasts positions over multicast UDP using
the SSL-Vision protobuf schema.

This fork extends the original RoboCIn vss-vision with:
- **FLIR Spinnaker camera support** (alongside USB cameras and video files)
- **ArUco-based robot detector** (alternative to the original LUT/blob pipeline)

## Dependencies

```
Qt5 (Core, Widgets, Gui, Network)
OpenCV 4.5+ with contrib (for cv::aruco)
SFML (network, system)
TBB
Protobuf
spdlog (vendored under include/)
FLIR Spinnaker SDK (optional but currently required by CMake — see Spinnaker section)
```

Install on Ubuntu 22.04:
```bash
./InstallDependencies
```

The Spinnaker SDK is **not** in the apt repos and must be installed manually at
`/opt/spinnaker/`. Until that's optionalised in the build, even USB-only setups
need the SDK present for linking. See `docs/ARUCO_PENDING.md`.

## Build

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
cd ../src
./VSS-VISION
```

## Camera sources

The "Source" panel at the bottom-left of the main window has three tabs:

1. **Camera** — USB camera. Lists `/dev/video*` devices. Configurable from
   `Cam Config` button (uses `v4l2-ctl` under the hood).
2. **Video** — playback from a file (XML path persisted in `Config/Video.xml`).
3. **Spinnaker** — FLIR cameras detected via the Spinnaker SDK. Camera
   parameters (gain/exposure/etc) are not yet exposed in the UI for this source.

Click `Capture` after choosing a tab and source.

## Detection: BlobDetection vs ArUco

Two robot detection algorithms ship in this build:

| Algorithm | When | What it needs | Pipeline cost |
|-----------|------|---------------|---------------|
| **BlobDetection** (legacy) | Default | LUT segmentation + RLE compression | Heavier — runs full segmentation pipeline |
| **ArUco** (new) | Toggle from menu | ArUco markers on robots (DICT_ARUCO_ORIGINAL) | Lighter — skips segmentation/RLE |

**Toggle:** `Configure → Use ArUco Detector` (checkbox in the menu bar).

When ArUco is active, `Vision::update()` skips `LUTSegmentation` and
`RunLengthEncoding`, and runs `ArucoDetection::runFromFrame()` directly on the
BGR frame. The ball is detected via HSV in the same pass.

### ArUco marker mapping

Each robot slot (3 per team) maps to a single ArUco marker ID. To assign IDs:

- **Per-robot:** click the gear (⚙) icon next to each robot in the right panel
  → input dialog asks for the ID.
- **Bulk:** edit `src/Config/ArucoConfig.json` directly. Format:

```json
{
  "ROBOT1": 256, "ROBOT2": 272, "ROBOT3": 273,
  "ADV1":   771, "ADV2":   939, "ADV3":   955,
  "ballHueLow":  6,  "ballSatLow":  80,  "ballValLow":  100,
  "ballHueHigh": 30, "ballSatHigh": 255, "ballValHigh": 255,
  "ballMinArea": 250
}
```

Use `-1` to disable a slot.

### Network output (downstream compatibility)

ArUco and BlobDetection produce **identical** UDP packets — the consumer (vsss
software) does not need any change. Both write to `vss.setEntities(ball,
players)` with:

- `Entity::team()` set to `Color::BLUE` (2) or `Color::YELLOW` (3)
- `Entity::id()` set to `(team-1)*100 + localId` → 100..102 (BLUE) or 200..202 (YELLOW)
- `Entity::position()` in cm (converted via `Utils::convertPositionPixelToCm`)
- `Entity::angle()` in radians

`VisionServer::send()` then serialises into `SSL_DetectionRobot` /
`SSL_DetectionBall` and broadcasts on `224.5.23.2:10015`.

## Architecture

```
                                    +------------------+
                  USB / Spinnaker → | CameraManager    | (singleton; picks USB
                  / Video           |                  |  or Spinnaker per tab)
                                    +--------+---------+
                                             |
                                  cv::Mat    | (TBB graph: camera node)
                                             ↓
                                    +------------------+
                                    | Vision::update() |
                                    +--------+---------+
                                             |
                       _useAruco?  ← toggle from Configure menu
                          /     \
                         no      yes
                          |       |
       +------------------+       +-----------------------+
       | LUTSegmentation  |       | ArucoDetection        |
       | + RLE compression|       | (cv::aruco + HSV ball)|
       | + BlobDetection  |       |                       |
       +--------+---------+       +-----------+-----------+
                |                             |
                +-------------+---------------+
                              ↓
                       vss.setEntities(ball, players)
                              ↓
                       VisionServer::send(...)
                              ↓
                       multicast 224.5.23.2:10015
```

Singletons in play: `Vision`, `CameraManager`, `GameInfo` (aliased as `vss`).
Threading uses an Intel TBB flow graph driven by `TBBThreadManager`.

## Repository layout

```
.
├── include/spdlog/                   # vendored logging
├── src/
│   ├── main.cpp                      # entry point
│   ├── CameraManager/                # USB / video / Spinnaker abstraction
│   ├── Vision/
│   │   ├── Vision.{cpp,h}            # facade + algorithm selector
│   │   ├── ImageProcessing/          # WarpCorrection, LUTSegmentation, MaggicSegmentation
│   │   └── PositionProcessing/
│   │       ├── BlobDetection.cpp     # legacy LUT/blob detector
│   │       ├── ArucoDetection.cpp    # new ArUco-based detector
│   │       ├── PositionProcessing.cpp# shared base for legacy detectors
│   │       └── runlengthencoding.cpp # RLE compression for blob path
│   ├── Network/
│   │   └── visionServer/             # protobuf + multicast UDP
│   ├── Windows/
│   │   ├── MainVSSWindow.{cpp,h,ui}  # main window, source tabs, menu
│   │   └── RobotWidget.{cpp,h,ui}    # per-robot info widget (gear button → ArUco ID)
│   ├── Entity/                       # Entity (id, position, angle, team)
│   ├── GameInfo/                     # global game state (vss singleton)
│   ├── Utils/                        # geometry, kalman, types
│   ├── TBBThreadManager.{cpp,h}      # camera/vision flow graph
│   └── Config/
│       ├── ArucoConfig.json          # ArUco marker IDs + ball HSV
│       ├── LUTVideo.xml              # LUT segmentation table
│       ├── FieldLimits.xml           # perspective correction points
│       ├── CameraConfigD/L.json      # USB camera defaults / last-used
│       └── ...
└── docs/
    └── ARUCO_PENDING.md              # outstanding work for ArUco integration
```

## Run from Docker

```shell
./docker_build              # build image
./docker_run [video id]     # run; pass /dev/video<id> for USB camera passthrough
```

Note: the Docker image **does not** include the Spinnaker SDK, so Spinnaker
cameras work only on bare-metal builds at the moment.

## Adding a new detector

If you want to add a third detection algorithm:

1. Create `src/Vision/PositionProcessing/MyDetector.{cpp,h}`. It does **not**
   need to inherit from `PositionProcessing` if your algorithm doesn't operate
   on RLE'd segmented data — see `ArucoDetection` for a standalone example.
2. Add the files to `CMakeLists.txt` under `SOURCES` and `HEADERS`.
3. In `Vision.h`, add a new pointer member and a flag, mirroring `_arucoDetector`
   / `_useAruco`.
4. In `Vision::update()`, branch on the flag and route to your detector.
5. Your detector must call `vss.setEntities(ball, players)` with `Entity::id()`
   and `Entity::team()` matching the convention above (see "Network output"),
   so the wire format stays compatible.
6. Add a UI toggle in `MainVSSWindow` constructor (look for the
   `Use ArUco Detector` action — same pattern).

## Pending work

See [`docs/ARUCO_PENDING.md`](docs/ARUCO_PENDING.md) for the running list.
