/****************************************************************************
** Meta object code from reading C++ file 'MainWindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../../../../src/rtabmap/guilib/include/rtabmap/gui/MainWindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MainWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_rtabmap__MainWindow_t {
    QByteArrayData data[157];
    char stringdata0[2523];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_rtabmap__MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_rtabmap__MainWindow_t qt_meta_stringdata_rtabmap__MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 19), // "rtabmap::MainWindow"
QT_MOC_LITERAL(1, 20, 13), // "statsReceived"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 19), // "rtabmap::Statistics"
QT_MOC_LITERAL(4, 55, 14), // "statsProcessed"
QT_MOC_LITERAL(5, 70, 18), // "cameraInfoReceived"
QT_MOC_LITERAL(6, 89, 19), // "rtabmap::CameraInfo"
QT_MOC_LITERAL(7, 109, 19), // "cameraInfoProcessed"
QT_MOC_LITERAL(8, 129, 16), // "odometryReceived"
QT_MOC_LITERAL(9, 146, 22), // "rtabmap::OdometryEvent"
QT_MOC_LITERAL(10, 169, 17), // "odometryProcessed"
QT_MOC_LITERAL(11, 187, 17), // "thresholdsChanged"
QT_MOC_LITERAL(12, 205, 12), // "stateChanged"
QT_MOC_LITERAL(13, 218, 17), // "MainWindow::State"
QT_MOC_LITERAL(14, 236, 24), // "rtabmapEventInitReceived"
QT_MOC_LITERAL(15, 261, 6), // "status"
QT_MOC_LITERAL(16, 268, 4), // "info"
QT_MOC_LITERAL(17, 273, 25), // "rtabmapEvent3DMapReceived"
QT_MOC_LITERAL(18, 299, 26), // "rtabmap::RtabmapEvent3DMap"
QT_MOC_LITERAL(19, 326, 5), // "event"
QT_MOC_LITERAL(20, 332, 26), // "rtabmapEvent3DMapProcessed"
QT_MOC_LITERAL(21, 359, 30), // "rtabmapGlobalPathEventReceived"
QT_MOC_LITERAL(22, 390, 31), // "rtabmap::RtabmapGlobalPathEvent"
QT_MOC_LITERAL(23, 422, 25), // "rtabmapLabelErrorReceived"
QT_MOC_LITERAL(24, 448, 2), // "id"
QT_MOC_LITERAL(25, 451, 5), // "label"
QT_MOC_LITERAL(26, 457, 30), // "rtabmapGoalStatusEventReceived"
QT_MOC_LITERAL(27, 488, 14), // "imgRateChanged"
QT_MOC_LITERAL(28, 503, 20), // "detectionRateChanged"
QT_MOC_LITERAL(29, 524, 16), // "timeLimitChanged"
QT_MOC_LITERAL(30, 541, 18), // "mappingModeChanged"
QT_MOC_LITERAL(31, 560, 20), // "noMoreImagesReceived"
QT_MOC_LITERAL(32, 581, 21), // "loopClosureThrChanged"
QT_MOC_LITERAL(33, 603, 13), // "twistReceived"
QT_MOC_LITERAL(34, 617, 1), // "x"
QT_MOC_LITERAL(35, 619, 1), // "y"
QT_MOC_LITERAL(36, 621, 1), // "z"
QT_MOC_LITERAL(37, 623, 4), // "roll"
QT_MOC_LITERAL(38, 628, 5), // "pitch"
QT_MOC_LITERAL(39, 634, 3), // "yaw"
QT_MOC_LITERAL(40, 638, 3), // "row"
QT_MOC_LITERAL(41, 642, 3), // "col"
QT_MOC_LITERAL(42, 646, 12), // "processStats"
QT_MOC_LITERAL(43, 659, 4), // "stat"
QT_MOC_LITERAL(44, 664, 23), // "updateCacheFromDatabase"
QT_MOC_LITERAL(45, 688, 4), // "path"
QT_MOC_LITERAL(46, 693, 12), // "openDatabase"
QT_MOC_LITERAL(47, 706, 22), // "rtabmap::ParametersMap"
QT_MOC_LITERAL(48, 729, 19), // "overridedParameters"
QT_MOC_LITERAL(49, 749, 16), // "updateParameters"
QT_MOC_LITERAL(50, 766, 10), // "parameters"
QT_MOC_LITERAL(51, 777, 11), // "changeState"
QT_MOC_LITERAL(52, 789, 5), // "state"
QT_MOC_LITERAL(53, 795, 11), // "newDatabase"
QT_MOC_LITERAL(54, 807, 13), // "closeDatabase"
QT_MOC_LITERAL(55, 821, 14), // "startDetection"
QT_MOC_LITERAL(56, 836, 14), // "pauseDetection"
QT_MOC_LITERAL(57, 851, 13), // "stopDetection"
QT_MOC_LITERAL(58, 865, 13), // "saveConfigGUI"
QT_MOC_LITERAL(59, 879, 17), // "downloadAllClouds"
QT_MOC_LITERAL(60, 897, 17), // "downloadPoseGraph"
QT_MOC_LITERAL(61, 915, 13), // "clearTheCache"
QT_MOC_LITERAL(62, 929, 8), // "openHelp"
QT_MOC_LITERAL(63, 938, 15), // "openPreferences"
QT_MOC_LITERAL(64, 954, 21), // "openPreferencesSource"
QT_MOC_LITERAL(65, 976, 15), // "setDefaultViews"
QT_MOC_LITERAL(66, 992, 13), // "resetOdometry"
QT_MOC_LITERAL(67, 1006, 13), // "triggerNewMap"
QT_MOC_LITERAL(68, 1020, 12), // "deleteMemory"
QT_MOC_LITERAL(69, 1033, 4), // "beep"
QT_MOC_LITERAL(70, 1038, 14), // "cancelProgress"
QT_MOC_LITERAL(71, 1053, 17), // "configGUIModified"
QT_MOC_LITERAL(72, 1071, 12), // "editDatabase"
QT_MOC_LITERAL(73, 1084, 18), // "notifyNoMoreImages"
QT_MOC_LITERAL(74, 1103, 19), // "printLoopClosureIds"
QT_MOC_LITERAL(75, 1123, 16), // "generateGraphDOT"
QT_MOC_LITERAL(76, 1140, 14), // "exportPosesRaw"
QT_MOC_LITERAL(77, 1155, 19), // "exportPosesRGBDSLAM"
QT_MOC_LITERAL(78, 1175, 32), // "exportPosesRGBDSLAMMotionCapture"
QT_MOC_LITERAL(79, 1208, 16), // "exportPosesKITTI"
QT_MOC_LITERAL(80, 1225, 15), // "exportPosesTORO"
QT_MOC_LITERAL(81, 1241, 14), // "exportPosesG2O"
QT_MOC_LITERAL(82, 1256, 12), // "exportImages"
QT_MOC_LITERAL(83, 1269, 13), // "exportOctomap"
QT_MOC_LITERAL(84, 1283, 14), // "postProcessing"
QT_MOC_LITERAL(85, 1298, 16), // "depthCalibration"
QT_MOC_LITERAL(86, 1315, 20), // "openWorkingDirectory"
QT_MOC_LITERAL(87, 1336, 14), // "updateEditMenu"
QT_MOC_LITERAL(88, 1351, 12), // "selectStream"
QT_MOC_LITERAL(89, 1364, 12), // "selectOpenni"
QT_MOC_LITERAL(90, 1377, 14), // "selectFreenect"
QT_MOC_LITERAL(91, 1392, 14), // "selectOpenniCv"
QT_MOC_LITERAL(92, 1407, 18), // "selectOpenniCvAsus"
QT_MOC_LITERAL(93, 1426, 13), // "selectOpenni2"
QT_MOC_LITERAL(94, 1440, 15), // "selectFreenect2"
QT_MOC_LITERAL(95, 1456, 10), // "selectK4W2"
QT_MOC_LITERAL(96, 1467, 9), // "selectK4A"
QT_MOC_LITERAL(97, 1477, 15), // "selectRealSense"
QT_MOC_LITERAL(98, 1493, 16), // "selectRealSense2"
QT_MOC_LITERAL(99, 1510, 20), // "selectRealSense2L515"
QT_MOC_LITERAL(100, 1531, 22), // "selectRealSense2Stereo"
QT_MOC_LITERAL(101, 1554, 18), // "selectStereoDC1394"
QT_MOC_LITERAL(102, 1573, 23), // "selectStereoFlyCapture2"
QT_MOC_LITERAL(103, 1597, 15), // "selectStereoZed"
QT_MOC_LITERAL(104, 1613, 17), // "selectStereoZedOC"
QT_MOC_LITERAL(105, 1631, 16), // "selectStereoTara"
QT_MOC_LITERAL(106, 1648, 15), // "selectStereoUsb"
QT_MOC_LITERAL(107, 1664, 14), // "selectMyntEyeS"
QT_MOC_LITERAL(108, 1679, 13), // "selectDepthAI"
QT_MOC_LITERAL(109, 1693, 13), // "dumpTheMemory"
QT_MOC_LITERAL(110, 1707, 17), // "dumpThePrediction"
QT_MOC_LITERAL(111, 1725, 8), // "sendGoal"
QT_MOC_LITERAL(112, 1734, 13), // "sendWaypoints"
QT_MOC_LITERAL(113, 1748, 8), // "postGoal"
QT_MOC_LITERAL(114, 1757, 4), // "goal"
QT_MOC_LITERAL(115, 1762, 10), // "cancelGoal"
QT_MOC_LITERAL(116, 1773, 25), // "anchorCloudsToGroundTruth"
QT_MOC_LITERAL(117, 1799, 25), // "selectScreenCaptureFormat"
QT_MOC_LITERAL(118, 1825, 7), // "checked"
QT_MOC_LITERAL(119, 1833, 14), // "takeScreenshot"
QT_MOC_LITERAL(120, 1848, 17), // "updateElapsedTime"
QT_MOC_LITERAL(121, 1866, 17), // "processCameraInfo"
QT_MOC_LITERAL(122, 1884, 15), // "processOdometry"
QT_MOC_LITERAL(123, 1900, 4), // "odom"
QT_MOC_LITERAL(124, 1905, 11), // "dataIgnored"
QT_MOC_LITERAL(125, 1917, 17), // "applyPrefSettings"
QT_MOC_LITERAL(126, 1935, 30), // "PreferencesDialog::PANEL_FLAGS"
QT_MOC_LITERAL(127, 1966, 5), // "flags"
QT_MOC_LITERAL(128, 1972, 23), // "processRtabmapEventInit"
QT_MOC_LITERAL(129, 1996, 24), // "processRtabmapEvent3DMap"
QT_MOC_LITERAL(130, 2021, 29), // "processRtabmapGlobalPathEvent"
QT_MOC_LITERAL(131, 2051, 29), // "processRtabmapLabelErrorEvent"
QT_MOC_LITERAL(132, 2081, 29), // "processRtabmapGoalStatusEvent"
QT_MOC_LITERAL(133, 2111, 20), // "changeImgRateSetting"
QT_MOC_LITERAL(134, 2132, 26), // "changeDetectionRateSetting"
QT_MOC_LITERAL(135, 2159, 22), // "changeTimeLimitSetting"
QT_MOC_LITERAL(136, 2182, 17), // "changeMappingMode"
QT_MOC_LITERAL(137, 2200, 14), // "setAspectRatio"
QT_MOC_LITERAL(138, 2215, 1), // "w"
QT_MOC_LITERAL(139, 2217, 1), // "h"
QT_MOC_LITERAL(140, 2219, 18), // "setAspectRatio16_9"
QT_MOC_LITERAL(141, 2238, 19), // "setAspectRatio16_10"
QT_MOC_LITERAL(142, 2258, 17), // "setAspectRatio4_3"
QT_MOC_LITERAL(143, 2276, 18), // "setAspectRatio240p"
QT_MOC_LITERAL(144, 2295, 18), // "setAspectRatio360p"
QT_MOC_LITERAL(145, 2314, 18), // "setAspectRatio480p"
QT_MOC_LITERAL(146, 2333, 18), // "setAspectRatio720p"
QT_MOC_LITERAL(147, 2352, 19), // "setAspectRatio1080p"
QT_MOC_LITERAL(148, 2372, 20), // "setAspectRatioCustom"
QT_MOC_LITERAL(149, 2393, 13), // "exportGridMap"
QT_MOC_LITERAL(150, 2407, 12), // "exportClouds"
QT_MOC_LITERAL(151, 2420, 19), // "exportBundlerFormat"
QT_MOC_LITERAL(152, 2440, 10), // "viewClouds"
QT_MOC_LITERAL(153, 2451, 12), // "dataRecorder"
QT_MOC_LITERAL(154, 2464, 21), // "dataRecorderDestroyed"
QT_MOC_LITERAL(155, 2486, 20), // "updateNodeVisibility"
QT_MOC_LITERAL(156, 2507, 15) // "updateGraphView"

    },
    "rtabmap::MainWindow\0statsReceived\0\0"
    "rtabmap::Statistics\0statsProcessed\0"
    "cameraInfoReceived\0rtabmap::CameraInfo\0"
    "cameraInfoProcessed\0odometryReceived\0"
    "rtabmap::OdometryEvent\0odometryProcessed\0"
    "thresholdsChanged\0stateChanged\0"
    "MainWindow::State\0rtabmapEventInitReceived\0"
    "status\0info\0rtabmapEvent3DMapReceived\0"
    "rtabmap::RtabmapEvent3DMap\0event\0"
    "rtabmapEvent3DMapProcessed\0"
    "rtabmapGlobalPathEventReceived\0"
    "rtabmap::RtabmapGlobalPathEvent\0"
    "rtabmapLabelErrorReceived\0id\0label\0"
    "rtabmapGoalStatusEventReceived\0"
    "imgRateChanged\0detectionRateChanged\0"
    "timeLimitChanged\0mappingModeChanged\0"
    "noMoreImagesReceived\0loopClosureThrChanged\0"
    "twistReceived\0x\0y\0z\0roll\0pitch\0yaw\0"
    "row\0col\0processStats\0stat\0"
    "updateCacheFromDatabase\0path\0openDatabase\0"
    "rtabmap::ParametersMap\0overridedParameters\0"
    "updateParameters\0parameters\0changeState\0"
    "state\0newDatabase\0closeDatabase\0"
    "startDetection\0pauseDetection\0"
    "stopDetection\0saveConfigGUI\0"
    "downloadAllClouds\0downloadPoseGraph\0"
    "clearTheCache\0openHelp\0openPreferences\0"
    "openPreferencesSource\0setDefaultViews\0"
    "resetOdometry\0triggerNewMap\0deleteMemory\0"
    "beep\0cancelProgress\0configGUIModified\0"
    "editDatabase\0notifyNoMoreImages\0"
    "printLoopClosureIds\0generateGraphDOT\0"
    "exportPosesRaw\0exportPosesRGBDSLAM\0"
    "exportPosesRGBDSLAMMotionCapture\0"
    "exportPosesKITTI\0exportPosesTORO\0"
    "exportPosesG2O\0exportImages\0exportOctomap\0"
    "postProcessing\0depthCalibration\0"
    "openWorkingDirectory\0updateEditMenu\0"
    "selectStream\0selectOpenni\0selectFreenect\0"
    "selectOpenniCv\0selectOpenniCvAsus\0"
    "selectOpenni2\0selectFreenect2\0selectK4W2\0"
    "selectK4A\0selectRealSense\0selectRealSense2\0"
    "selectRealSense2L515\0selectRealSense2Stereo\0"
    "selectStereoDC1394\0selectStereoFlyCapture2\0"
    "selectStereoZed\0selectStereoZedOC\0"
    "selectStereoTara\0selectStereoUsb\0"
    "selectMyntEyeS\0selectDepthAI\0dumpTheMemory\0"
    "dumpThePrediction\0sendGoal\0sendWaypoints\0"
    "postGoal\0goal\0cancelGoal\0"
    "anchorCloudsToGroundTruth\0"
    "selectScreenCaptureFormat\0checked\0"
    "takeScreenshot\0updateElapsedTime\0"
    "processCameraInfo\0processOdometry\0"
    "odom\0dataIgnored\0applyPrefSettings\0"
    "PreferencesDialog::PANEL_FLAGS\0flags\0"
    "processRtabmapEventInit\0"
    "processRtabmapEvent3DMap\0"
    "processRtabmapGlobalPathEvent\0"
    "processRtabmapLabelErrorEvent\0"
    "processRtabmapGoalStatusEvent\0"
    "changeImgRateSetting\0changeDetectionRateSetting\0"
    "changeTimeLimitSetting\0changeMappingMode\0"
    "setAspectRatio\0w\0h\0setAspectRatio16_9\0"
    "setAspectRatio16_10\0setAspectRatio4_3\0"
    "setAspectRatio240p\0setAspectRatio360p\0"
    "setAspectRatio480p\0setAspectRatio720p\0"
    "setAspectRatio1080p\0setAspectRatioCustom\0"
    "exportGridMap\0exportClouds\0"
    "exportBundlerFormat\0viewClouds\0"
    "dataRecorder\0dataRecorderDestroyed\0"
    "updateNodeVisibility\0updateGraphView"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_rtabmap__MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
     127,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
      21,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,  649,    2, 0x06 /* Public */,
       4,    0,  652,    2, 0x06 /* Public */,
       5,    1,  653,    2, 0x06 /* Public */,
       7,    0,  656,    2, 0x06 /* Public */,
       8,    2,  657,    2, 0x06 /* Public */,
      10,    0,  662,    2, 0x06 /* Public */,
      11,    2,  663,    2, 0x06 /* Public */,
      12,    1,  668,    2, 0x06 /* Public */,
      14,    2,  671,    2, 0x06 /* Public */,
      17,    1,  676,    2, 0x06 /* Public */,
      20,    0,  679,    2, 0x06 /* Public */,
      21,    1,  680,    2, 0x06 /* Public */,
      23,    2,  683,    2, 0x06 /* Public */,
      26,    1,  688,    2, 0x06 /* Public */,
      27,    1,  691,    2, 0x06 /* Public */,
      28,    1,  694,    2, 0x06 /* Public */,
      29,    1,  697,    2, 0x06 /* Public */,
      30,    1,  700,    2, 0x06 /* Public */,
      31,    0,  703,    2, 0x06 /* Public */,
      32,    1,  704,    2, 0x06 /* Public */,
      33,    8,  707,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      42,    1,  724,    2, 0x0a /* Public */,
      44,    1,  727,    2, 0x0a /* Public */,
      46,    2,  730,    2, 0x0a /* Public */,
      46,    1,  735,    2, 0x2a /* Public | MethodCloned */,
      49,    1,  738,    2, 0x0a /* Public */,
      51,    1,  741,    2, 0x09 /* Protected */,
      53,    0,  744,    2, 0x09 /* Protected */,
      46,    0,  745,    2, 0x09 /* Protected */,
      54,    0,  746,    2, 0x09 /* Protected */,
      55,    0,  747,    2, 0x09 /* Protected */,
      56,    0,  748,    2, 0x09 /* Protected */,
      57,    0,  749,    2, 0x09 /* Protected */,
      58,    0,  750,    2, 0x09 /* Protected */,
      59,    0,  751,    2, 0x09 /* Protected */,
      60,    0,  752,    2, 0x09 /* Protected */,
      61,    0,  753,    2, 0x09 /* Protected */,
      62,    0,  754,    2, 0x09 /* Protected */,
      63,    0,  755,    2, 0x09 /* Protected */,
      64,    0,  756,    2, 0x09 /* Protected */,
      65,    0,  757,    2, 0x09 /* Protected */,
      66,    0,  758,    2, 0x09 /* Protected */,
      67,    0,  759,    2, 0x09 /* Protected */,
      68,    0,  760,    2, 0x09 /* Protected */,
      69,    0,  761,    2, 0x09 /* Protected */,
      70,    0,  762,    2, 0x09 /* Protected */,
      71,    0,  763,    2, 0x09 /* Protected */,
      72,    0,  764,    2, 0x09 /* Protected */,
      73,    0,  765,    2, 0x09 /* Protected */,
      74,    0,  766,    2, 0x09 /* Protected */,
      75,    0,  767,    2, 0x09 /* Protected */,
      76,    0,  768,    2, 0x09 /* Protected */,
      77,    0,  769,    2, 0x09 /* Protected */,
      78,    0,  770,    2, 0x09 /* Protected */,
      79,    0,  771,    2, 0x09 /* Protected */,
      80,    0,  772,    2, 0x09 /* Protected */,
      81,    0,  773,    2, 0x09 /* Protected */,
      82,    0,  774,    2, 0x09 /* Protected */,
      83,    0,  775,    2, 0x09 /* Protected */,
      84,    0,  776,    2, 0x09 /* Protected */,
      85,    0,  777,    2, 0x09 /* Protected */,
      86,    0,  778,    2, 0x09 /* Protected */,
      87,    0,  779,    2, 0x09 /* Protected */,
      88,    0,  780,    2, 0x09 /* Protected */,
      89,    0,  781,    2, 0x09 /* Protected */,
      90,    0,  782,    2, 0x09 /* Protected */,
      91,    0,  783,    2, 0x09 /* Protected */,
      92,    0,  784,    2, 0x09 /* Protected */,
      93,    0,  785,    2, 0x09 /* Protected */,
      94,    0,  786,    2, 0x09 /* Protected */,
      95,    0,  787,    2, 0x09 /* Protected */,
      96,    0,  788,    2, 0x09 /* Protected */,
      97,    0,  789,    2, 0x09 /* Protected */,
      98,    0,  790,    2, 0x09 /* Protected */,
      99,    0,  791,    2, 0x09 /* Protected */,
     100,    0,  792,    2, 0x09 /* Protected */,
     101,    0,  793,    2, 0x09 /* Protected */,
     102,    0,  794,    2, 0x09 /* Protected */,
     103,    0,  795,    2, 0x09 /* Protected */,
     104,    0,  796,    2, 0x09 /* Protected */,
     105,    0,  797,    2, 0x09 /* Protected */,
     106,    0,  798,    2, 0x09 /* Protected */,
     107,    0,  799,    2, 0x09 /* Protected */,
     108,    0,  800,    2, 0x09 /* Protected */,
     109,    0,  801,    2, 0x09 /* Protected */,
     110,    0,  802,    2, 0x09 /* Protected */,
     111,    0,  803,    2, 0x09 /* Protected */,
     112,    0,  804,    2, 0x09 /* Protected */,
     113,    1,  805,    2, 0x09 /* Protected */,
     115,    0,  808,    2, 0x09 /* Protected */,
      25,    0,  809,    2, 0x09 /* Protected */,
      44,    0,  810,    2, 0x09 /* Protected */,
     116,    0,  811,    2, 0x09 /* Protected */,
     117,    1,  812,    2, 0x09 /* Protected */,
     119,    0,  815,    2, 0x09 /* Protected */,
     120,    0,  816,    2, 0x09 /* Protected */,
     121,    1,  817,    2, 0x09 /* Protected */,
     122,    2,  820,    2, 0x09 /* Protected */,
     125,    1,  825,    2, 0x09 /* Protected */,
     125,    1,  828,    2, 0x09 /* Protected */,
     128,    2,  831,    2, 0x09 /* Protected */,
     129,    1,  836,    2, 0x09 /* Protected */,
     130,    1,  839,    2, 0x09 /* Protected */,
     131,    2,  842,    2, 0x09 /* Protected */,
     132,    1,  847,    2, 0x09 /* Protected */,
     133,    0,  850,    2, 0x09 /* Protected */,
     134,    0,  851,    2, 0x09 /* Protected */,
     135,    0,  852,    2, 0x09 /* Protected */,
     136,    0,  853,    2, 0x09 /* Protected */,
     137,    2,  854,    2, 0x09 /* Protected */,
     140,    0,  859,    2, 0x09 /* Protected */,
     141,    0,  860,    2, 0x09 /* Protected */,
     142,    0,  861,    2, 0x09 /* Protected */,
     143,    0,  862,    2, 0x09 /* Protected */,
     144,    0,  863,    2, 0x09 /* Protected */,
     145,    0,  864,    2, 0x09 /* Protected */,
     146,    0,  865,    2, 0x09 /* Protected */,
     147,    0,  866,    2, 0x09 /* Protected */,
     148,    0,  867,    2, 0x09 /* Protected */,
     149,    0,  868,    2, 0x09 /* Protected */,
     150,    0,  869,    2, 0x09 /* Protected */,
     151,    0,  870,    2, 0x09 /* Protected */,
     152,    0,  871,    2, 0x09 /* Protected */,
     153,    0,  872,    2, 0x09 /* Protected */,
     154,    0,  873,    2, 0x09 /* Protected */,
     155,    2,  874,    2, 0x09 /* Protected */,
     156,    0,  879,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    2,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 6,    2,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9, QMetaType::Bool,    2,    2,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,    2,    2,
    QMetaType::Void, 0x80000000 | 13,    2,
    QMetaType::Void, QMetaType::Int, QMetaType::QString,   15,   16,
    QMetaType::Void, 0x80000000 | 18,   19,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 22,   19,
    QMetaType::Void, QMetaType::Int, QMetaType::QString,   24,   25,
    QMetaType::Void, QMetaType::Int,   15,
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::Double,    2,
    QMetaType::Void, QMetaType::Float,    2,
    QMetaType::Void, QMetaType::Bool,    2,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QReal,    2,
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Int, QMetaType::Int,   34,   35,   36,   37,   38,   39,   40,   41,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,   43,
    QMetaType::Void, QMetaType::QString,   45,
    QMetaType::Void, QMetaType::QString, 0x80000000 | 47,   45,   48,
    QMetaType::Void, QMetaType::QString,   45,
    QMetaType::Void, 0x80000000 | 47,   50,
    QMetaType::Void, 0x80000000 | 13,   52,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Bool,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,  114,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,  118,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 6,   16,
    QMetaType::Void, 0x80000000 | 9, QMetaType::Bool,  123,  124,
    QMetaType::Void, 0x80000000 | 126,  127,
    QMetaType::Void, 0x80000000 | 47,   50,
    QMetaType::Void, QMetaType::Int, QMetaType::QString,   15,   16,
    QMetaType::Void, 0x80000000 | 18,   19,
    QMetaType::Void, 0x80000000 | 22,   19,
    QMetaType::Void, QMetaType::Int, QMetaType::QString,   24,   25,
    QMetaType::Void, QMetaType::Int,   15,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,  138,  139,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Bool,    2,    2,
    QMetaType::Void,

       0        // eod
};

void rtabmap::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->statsReceived((*reinterpret_cast< const rtabmap::Statistics(*)>(_a[1]))); break;
        case 1: _t->statsProcessed(); break;
        case 2: _t->cameraInfoReceived((*reinterpret_cast< const rtabmap::CameraInfo(*)>(_a[1]))); break;
        case 3: _t->cameraInfoProcessed(); break;
        case 4: _t->odometryReceived((*reinterpret_cast< const rtabmap::OdometryEvent(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 5: _t->odometryProcessed(); break;
        case 6: _t->thresholdsChanged((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 7: _t->stateChanged((*reinterpret_cast< MainWindow::State(*)>(_a[1]))); break;
        case 8: _t->rtabmapEventInitReceived((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        case 9: _t->rtabmapEvent3DMapReceived((*reinterpret_cast< const rtabmap::RtabmapEvent3DMap(*)>(_a[1]))); break;
        case 10: _t->rtabmapEvent3DMapProcessed(); break;
        case 11: _t->rtabmapGlobalPathEventReceived((*reinterpret_cast< const rtabmap::RtabmapGlobalPathEvent(*)>(_a[1]))); break;
        case 12: _t->rtabmapLabelErrorReceived((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        case 13: _t->rtabmapGoalStatusEventReceived((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->imgRateChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 15: _t->detectionRateChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 16: _t->timeLimitChanged((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 17: _t->mappingModeChanged((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 18: _t->noMoreImagesReceived(); break;
        case 19: _t->loopClosureThrChanged((*reinterpret_cast< qreal(*)>(_a[1]))); break;
        case 20: _t->twistReceived((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2])),(*reinterpret_cast< float(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4])),(*reinterpret_cast< float(*)>(_a[5])),(*reinterpret_cast< float(*)>(_a[6])),(*reinterpret_cast< int(*)>(_a[7])),(*reinterpret_cast< int(*)>(_a[8]))); break;
        case 21: _t->processStats((*reinterpret_cast< const rtabmap::Statistics(*)>(_a[1]))); break;
        case 22: _t->updateCacheFromDatabase((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 23: _t->openDatabase((*reinterpret_cast< const QString(*)>(_a[1])),(*reinterpret_cast< const rtabmap::ParametersMap(*)>(_a[2]))); break;
        case 24: _t->openDatabase((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 25: _t->updateParameters((*reinterpret_cast< const rtabmap::ParametersMap(*)>(_a[1]))); break;
        case 26: _t->changeState((*reinterpret_cast< MainWindow::State(*)>(_a[1]))); break;
        case 27: _t->newDatabase(); break;
        case 28: _t->openDatabase(); break;
        case 29: { bool _r = _t->closeDatabase();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = std::move(_r); }  break;
        case 30: _t->startDetection(); break;
        case 31: _t->pauseDetection(); break;
        case 32: _t->stopDetection(); break;
        case 33: _t->saveConfigGUI(); break;
        case 34: _t->downloadAllClouds(); break;
        case 35: _t->downloadPoseGraph(); break;
        case 36: _t->clearTheCache(); break;
        case 37: _t->openHelp(); break;
        case 38: _t->openPreferences(); break;
        case 39: _t->openPreferencesSource(); break;
        case 40: _t->setDefaultViews(); break;
        case 41: _t->resetOdometry(); break;
        case 42: _t->triggerNewMap(); break;
        case 43: _t->deleteMemory(); break;
        case 44: _t->beep(); break;
        case 45: _t->cancelProgress(); break;
        case 46: _t->configGUIModified(); break;
        case 47: _t->editDatabase(); break;
        case 48: _t->notifyNoMoreImages(); break;
        case 49: _t->printLoopClosureIds(); break;
        case 50: _t->generateGraphDOT(); break;
        case 51: _t->exportPosesRaw(); break;
        case 52: _t->exportPosesRGBDSLAM(); break;
        case 53: _t->exportPosesRGBDSLAMMotionCapture(); break;
        case 54: _t->exportPosesKITTI(); break;
        case 55: _t->exportPosesTORO(); break;
        case 56: _t->exportPosesG2O(); break;
        case 57: _t->exportImages(); break;
        case 58: _t->exportOctomap(); break;
        case 59: _t->postProcessing(); break;
        case 60: _t->depthCalibration(); break;
        case 61: _t->openWorkingDirectory(); break;
        case 62: _t->updateEditMenu(); break;
        case 63: _t->selectStream(); break;
        case 64: _t->selectOpenni(); break;
        case 65: _t->selectFreenect(); break;
        case 66: _t->selectOpenniCv(); break;
        case 67: _t->selectOpenniCvAsus(); break;
        case 68: _t->selectOpenni2(); break;
        case 69: _t->selectFreenect2(); break;
        case 70: _t->selectK4W2(); break;
        case 71: _t->selectK4A(); break;
        case 72: _t->selectRealSense(); break;
        case 73: _t->selectRealSense2(); break;
        case 74: _t->selectRealSense2L515(); break;
        case 75: _t->selectRealSense2Stereo(); break;
        case 76: _t->selectStereoDC1394(); break;
        case 77: _t->selectStereoFlyCapture2(); break;
        case 78: _t->selectStereoZed(); break;
        case 79: _t->selectStereoZedOC(); break;
        case 80: _t->selectStereoTara(); break;
        case 81: _t->selectStereoUsb(); break;
        case 82: _t->selectMyntEyeS(); break;
        case 83: _t->selectDepthAI(); break;
        case 84: _t->dumpTheMemory(); break;
        case 85: _t->dumpThePrediction(); break;
        case 86: _t->sendGoal(); break;
        case 87: _t->sendWaypoints(); break;
        case 88: _t->postGoal((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 89: _t->cancelGoal(); break;
        case 90: _t->label(); break;
        case 91: _t->updateCacheFromDatabase(); break;
        case 92: _t->anchorCloudsToGroundTruth(); break;
        case 93: _t->selectScreenCaptureFormat((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 94: _t->takeScreenshot(); break;
        case 95: _t->updateElapsedTime(); break;
        case 96: _t->processCameraInfo((*reinterpret_cast< const rtabmap::CameraInfo(*)>(_a[1]))); break;
        case 97: _t->processOdometry((*reinterpret_cast< const rtabmap::OdometryEvent(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 98: _t->applyPrefSettings((*reinterpret_cast< PreferencesDialog::PANEL_FLAGS(*)>(_a[1]))); break;
        case 99: _t->applyPrefSettings((*reinterpret_cast< const rtabmap::ParametersMap(*)>(_a[1]))); break;
        case 100: _t->processRtabmapEventInit((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        case 101: _t->processRtabmapEvent3DMap((*reinterpret_cast< const rtabmap::RtabmapEvent3DMap(*)>(_a[1]))); break;
        case 102: _t->processRtabmapGlobalPathEvent((*reinterpret_cast< const rtabmap::RtabmapGlobalPathEvent(*)>(_a[1]))); break;
        case 103: _t->processRtabmapLabelErrorEvent((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< const QString(*)>(_a[2]))); break;
        case 104: _t->processRtabmapGoalStatusEvent((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 105: _t->changeImgRateSetting(); break;
        case 106: _t->changeDetectionRateSetting(); break;
        case 107: _t->changeTimeLimitSetting(); break;
        case 108: _t->changeMappingMode(); break;
        case 109: _t->setAspectRatio((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 110: _t->setAspectRatio16_9(); break;
        case 111: _t->setAspectRatio16_10(); break;
        case 112: _t->setAspectRatio4_3(); break;
        case 113: _t->setAspectRatio240p(); break;
        case 114: _t->setAspectRatio360p(); break;
        case 115: _t->setAspectRatio480p(); break;
        case 116: _t->setAspectRatio720p(); break;
        case 117: _t->setAspectRatio1080p(); break;
        case 118: _t->setAspectRatioCustom(); break;
        case 119: _t->exportGridMap(); break;
        case 120: _t->exportClouds(); break;
        case 121: _t->exportBundlerFormat(); break;
        case 122: _t->viewClouds(); break;
        case 123: _t->dataRecorder(); break;
        case 124: _t->dataRecorderDestroyed(); break;
        case 125: _t->updateNodeVisibility((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 126: _t->updateGraphView(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            typedef void (MainWindow::*_t)(const rtabmap::Statistics & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::statsReceived)) {
                *result = 0;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::statsProcessed)) {
                *result = 1;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(const rtabmap::CameraInfo & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::cameraInfoReceived)) {
                *result = 2;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::cameraInfoProcessed)) {
                *result = 3;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(const rtabmap::OdometryEvent & , bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::odometryReceived)) {
                *result = 4;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::odometryProcessed)) {
                *result = 5;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(int , int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::thresholdsChanged)) {
                *result = 6;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(MainWindow::State );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::stateChanged)) {
                *result = 7;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(int , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::rtabmapEventInitReceived)) {
                *result = 8;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(const rtabmap::RtabmapEvent3DMap & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::rtabmapEvent3DMapReceived)) {
                *result = 9;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::rtabmapEvent3DMapProcessed)) {
                *result = 10;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(const rtabmap::RtabmapGlobalPathEvent & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::rtabmapGlobalPathEventReceived)) {
                *result = 11;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(int , const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::rtabmapLabelErrorReceived)) {
                *result = 12;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::rtabmapGoalStatusEventReceived)) {
                *result = 13;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::imgRateChanged)) {
                *result = 14;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::detectionRateChanged)) {
                *result = 15;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::timeLimitChanged)) {
                *result = 16;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::mappingModeChanged)) {
                *result = 17;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::noMoreImagesReceived)) {
                *result = 18;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(qreal );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::loopClosureThrChanged)) {
                *result = 19;
                return;
            }
        }
        {
            typedef void (MainWindow::*_t)(float , float , float , float , float , float , int , int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::twistReceived)) {
                *result = 20;
                return;
            }
        }
    }
}

const QMetaObject rtabmap::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_rtabmap__MainWindow.data,
      qt_meta_data_rtabmap__MainWindow,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *rtabmap::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *rtabmap::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_rtabmap__MainWindow.stringdata0))
        return static_cast<void*>(this);
    if (!strcmp(_clname, "UEventsHandler"))
        return static_cast< UEventsHandler*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int rtabmap::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 127)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 127;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 127)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 127;
    }
    return _id;
}

// SIGNAL 0
void rtabmap::MainWindow::statsReceived(const rtabmap::Statistics & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void rtabmap::MainWindow::statsProcessed()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void rtabmap::MainWindow::cameraInfoReceived(const rtabmap::CameraInfo & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void rtabmap::MainWindow::cameraInfoProcessed()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}

// SIGNAL 4
void rtabmap::MainWindow::odometryReceived(const rtabmap::OdometryEvent & _t1, bool _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void rtabmap::MainWindow::odometryProcessed()
{
    QMetaObject::activate(this, &staticMetaObject, 5, nullptr);
}

// SIGNAL 6
void rtabmap::MainWindow::thresholdsChanged(int _t1, int _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void rtabmap::MainWindow::stateChanged(MainWindow::State _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void rtabmap::MainWindow::rtabmapEventInitReceived(int _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}

// SIGNAL 9
void rtabmap::MainWindow::rtabmapEvent3DMapReceived(const rtabmap::RtabmapEvent3DMap & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 9, _a);
}

// SIGNAL 10
void rtabmap::MainWindow::rtabmapEvent3DMapProcessed()
{
    QMetaObject::activate(this, &staticMetaObject, 10, nullptr);
}

// SIGNAL 11
void rtabmap::MainWindow::rtabmapGlobalPathEventReceived(const rtabmap::RtabmapGlobalPathEvent & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 11, _a);
}

// SIGNAL 12
void rtabmap::MainWindow::rtabmapLabelErrorReceived(int _t1, const QString & _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 12, _a);
}

// SIGNAL 13
void rtabmap::MainWindow::rtabmapGoalStatusEventReceived(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 13, _a);
}

// SIGNAL 14
void rtabmap::MainWindow::imgRateChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 14, _a);
}

// SIGNAL 15
void rtabmap::MainWindow::detectionRateChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 15, _a);
}

// SIGNAL 16
void rtabmap::MainWindow::timeLimitChanged(float _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 16, _a);
}

// SIGNAL 17
void rtabmap::MainWindow::mappingModeChanged(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 17, _a);
}

// SIGNAL 18
void rtabmap::MainWindow::noMoreImagesReceived()
{
    QMetaObject::activate(this, &staticMetaObject, 18, nullptr);
}

// SIGNAL 19
void rtabmap::MainWindow::loopClosureThrChanged(qreal _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 19, _a);
}

// SIGNAL 20
void rtabmap::MainWindow::twistReceived(float _t1, float _t2, float _t3, float _t4, float _t5, float _t6, int _t7, int _t8)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)), const_cast<void*>(reinterpret_cast<const void*>(&_t5)), const_cast<void*>(reinterpret_cast<const void*>(&_t6)), const_cast<void*>(reinterpret_cast<const void*>(&_t7)), const_cast<void*>(reinterpret_cast<const void*>(&_t8)) };
    QMetaObject::activate(this, &staticMetaObject, 20, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
