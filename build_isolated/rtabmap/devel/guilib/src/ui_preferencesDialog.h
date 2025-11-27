/********************************************************************************
** Form generated from reading UI file 'preferencesDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PREFERENCESDIALOG_H
#define UI_PREFERENCESDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QTreeView>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "../include/rtabmap/utilite/UPlot.h"
#include "3rdParty/QMultiComboBox.h"

QT_BEGIN_NAMESPACE

class Ui_preferencesDialog
{
public:
    QGridLayout *gridLayout_49;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout_18;
    QTreeView *treeView;
    QVBoxLayout *verticalLayout_17;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_16;
    QVBoxLayout *verticalLayout_24;
    QStackedWidget *stackedWidget;
    QWidget *page_22;
    QVBoxLayout *verticalLayout_29;
    QGroupBox *groupBox_generalSettingsGui0;
    QVBoxLayout *verticalLayout_28;
    QGridLayout *gridLayout_39;
    QCheckBox *checkBox_beep;
    QCheckBox *checkBox_notifyWhenNewGlobalPathIsReceived;
    QLabel *label_247;
    QLabel *label_313;
    QLabel *label_102;
    QCheckBox *general_checkBox_imagesKept;
    QCheckBox *general_checkBox_cloudsKept;
    QCheckBox *checkBox_stamps;
    QLabel *label_342;
    QLabel *label_88;
    QLabel *label_59;
    QCheckBox *checkbox_odomDisabled;
    QLabel *label_239;
    QCheckBox *checkBox_cacheStatistics;
    QLabel *label_347;
    QCheckBox *checkbox_groundTruthAlign;
    QGroupBox *groupBox_2;
    QHBoxLayout *horizontalLayout_9;
    QPushButton *pushButton_loadConfig;
    QPushButton *pushButton_saveConfig;
    QPushButton *pushButton_resetConfig;
    QSpacerItem *horizontalSpacer_4;
    QGroupBox *groupBox_5;
    QGridLayout *gridLayout_31;
    QCheckBox *checkBox_imageRejectedShown;
    QLabel *label_105;
    QCheckBox *checkBox_imageHighestHypShown;
    QLabel *label_73;
    QLabel *label_70;
    QCheckBox *checkBox_verticalLayoutUsed;
    QGroupBox *groupBox_29;
    QGridLayout *gridLayout_104;
    QLabel *label_222;
    QRadioButton *radioButton_wordsGraphView;
    QRadioButton *radioButton_posteriorGraphView;
    QLabel *label_478;
    QRadioButton *radioButton_nochangeGraphView;
    QLabel *label_479;
    QLabel *label_481;
    QRadioButton *radioButton_localizationsGraphView;
    QGroupBox *groupBox_21;
    QGridLayout *gridLayout_85;
    QLabel *label_369;
    QCheckBox *checkBox_odom_onlyInliersShown;
    QLabel *label_159;
    QSpinBox *spinBox_odomQualityWarnThr;
    QSpacerItem *verticalSpacer_24;
    QWidget *page_2;
    QVBoxLayout *verticalLayout_42;
    QGroupBox *groupBox_cloudRendering1;
    QVBoxLayout *verticalLayout_98;
    QGroupBox *groupBox_12;
    QVBoxLayout *verticalLayout_112;
    QGridLayout *gridLayout_2;
    QLineEdit *lineEdit_roiRatios;
    QSpinBox *spinBox_ptsize;
    QLabel *label_355;
    QLabel *label_169;
    QDoubleSpinBox *doubleSpinBox_normalRadiusSearch;
    QLabel *label_459;
    QSpinBox *spinBox_colorScheme_odom;
    QSpinBox *spinBox_colorScheme;
    QDoubleSpinBox *doubleSpinBox_maxDepth_odom;
    QLineEdit *lineEdit_roiRatios_odom;
    QLabel *label_154;
    QLabel *label_137;
    QCheckBox *checkBox_showClouds;
    QCheckBox *checkBox_showOdomClouds;
    QLabel *label_119;
    QSpinBox *spinBox_decimation_odom;
    QLabel *label_108;
    QDoubleSpinBox *doubleSpinBox_opacity;
    QLabel *label_155;
    QLabel *label_157;
    QDoubleSpinBox *doubleSpinBox_noiseRadius;
    QSpinBox *spinBox_decimation;
    QDoubleSpinBox *doubleSpinBox_maxDepth;
    QLabel *label_132;
    QDoubleSpinBox *doubleSpinBox_minDepth;
    QDoubleSpinBox *doubleSpinBox_minDepth_odom;
    QLabel *label_150;
    QLabel *label_353;
    QDoubleSpinBox *doubleSpinBox_floorFilterHeight;
    QDoubleSpinBox *doubleSpinBox_voxel;
    QSpinBox *spinBox_ptsize_odom;
    QLabel *label_168;
    QSpinBox *spinBox_noiseMinNeighbors;
    QLabel *label_338;
    QLabel *label_171;
    QDoubleSpinBox *doubleSpinBox_ceilingFilterHeight;
    QSpinBox *spinBox_normalKSearch;
    QLabel *label_210;
    QLabel *label_427;
    QDoubleSpinBox *doubleSpinBox_opacity_odom;
    QGroupBox *groupBox_organized;
    QVBoxLayout *verticalLayout_31;
    QGridLayout *gridLayout_64;
    QLabel *label_85;
    QLabel *label_quad;
    QSpinBox *spinBox_mesh_triangleSize;
    QLabel *label_87;
    QCheckBox *checkBox_mesh_quad;
    QDoubleSpinBox *doubleSpinBox_mesh_angleTolerance;
    QLabel *label_337;
    QCheckBox *checkBox_mesh_texture;
    QGroupBox *groupBox_17;
    QGridLayout *gridLayout_79;
    QDoubleSpinBox *doubleSpinBox_opacity_odom_scan;
    QLabel *label_273;
    QSpinBox *spinBox_ptsize_odom_scan;
    QLabel *label_110;
    QLabel *label_271;
    QLabel *label_356;
    QLabel *label_359;
    QDoubleSpinBox *doubleSpinBox_ceilingFilterHeight_scan;
    QLabel *label_241;
    QLabel *label_458;
    QLabel *label_460;
    QDoubleSpinBox *doubleSpinBox_maxRange;
    QLabel *label_428;
    QLabel *label_158;
    QDoubleSpinBox *doubleSpinBox_voxelSizeScan_odom;
    QCheckBox *checkBox_showScans;
    QLabel *label_156;
    QSpinBox *spinBox_ptsize_scan;
    QSpinBox *spinBox_downsamplingScan_odom;
    QCheckBox *checkBox_showOdomScans;
    QSpinBox *spinBox_downsamplingScan;
    QDoubleSpinBox *doubleSpinBox_opacity_scan;
    QDoubleSpinBox *doubleSpinBox_voxelSizeScan;
    QLabel *label_367;
    QLabel *label_368;
    QDoubleSpinBox *doubleSpinBox_floorFilterHeight_scan;
    QSpinBox *spinBox_normalKSearch_scan;
    QDoubleSpinBox *doubleSpinBox_normalRadiusSearch_scan;
    QSpinBox *spinBox_colorSchemeScan;
    QSpinBox *spinBox_colorSchemeScan_odom;
    QLabel *label_461;
    QDoubleSpinBox *doubleSpinBox_minRange;
    QDoubleSpinBox *doubleSpinBox_maxRange_odom;
    QDoubleSpinBox *doubleSpinBox_minRange_odom;
    QGroupBox *groupBox_18;
    QGridLayout *gridLayout_82;
    QLabel *label_213;
    QCheckBox *checkBox_showFeatures;
    QCheckBox *checkBox_showGraphs;
    QCheckBox *checkBox_showLabels;
    QSpinBox *spinBox_ptsize_odom_features;
    QCheckBox *checkBox_showOdomFeatures;
    QLabel *label_123;
    QLabel *label_243;
    QLabel *label_360;
    QLabel *label_361;
    QSpinBox *spinBox_ptsize_features;
    QLabel *label_166;
    QLabel *label_362;
    QCheckBox *checkBox_showFrustums;
    QCheckBox *checkBox_showOdomFrustums;
    QLabel *label_301;
    QCheckBox *checkBox_showLandmarks;
    QLabel *label_528;
    QDoubleSpinBox *doubleSpinBox_landmarkSize;
    QLabel *label_567;
    QLabel *label_568;
    QCheckBox *checkBox_showIMUAcc;
    QCheckBox *checkBox_showIMUGravity;
    QCheckBox *checkBox_showIMUGravity_odom;
    QDoubleSpinBox *doubleSpinBox_lengthIMUGravity;
    QDoubleSpinBox *doubleSpinBox_lengthIMUGravity_odom;
    QLabel *label_569;
    QLabel *label_572;
    QCheckBox *checkBox_showFrames;
    QSpacerItem *verticalSpacer;
    QWidget *page_61;
    QVBoxLayout *verticalLayout_104;
    QGroupBox *groupBox_filtering2;
    QVBoxLayout *verticalLayout_37;
    QLabel *label_160;
    QGridLayout *gridLayout_65;
    QLabel *label_71;
    QRadioButton *radioButton_noFiltering;
    QRadioButton *radioButton_subtractFiltering;
    QGridLayout *gridLayout_17;
    QDoubleSpinBox *doubleSpinBox_cloudFilterRadius;
    QLabel *label_37;
    QDoubleSpinBox *doubleSpinBox_cloudFilterAngle;
    QLabel *label_48;
    QLabel *label_161;
    QLabel *label_33;
    QRadioButton *radioButton_nodeFiltering;
    QGridLayout *gridLayout_69;
    QDoubleSpinBox *doubleSpinBox_subtractFilteringRadius;
    QLabel *label_192;
    QDoubleSpinBox *doubleSpinBox_subtractFilteringAngle;
    QLabel *label_193;
    QSpinBox *spinBox_subtractFilteringMinPts;
    QLabel *label_174;
    QSpacerItem *verticalSpacer_55;
    QWidget *page_62;
    QVBoxLayout *verticalLayout_105;
    QGroupBox *groupBox_gridMap2;
    QVBoxLayout *verticalLayout_45;
    QLabel *label_167;
    QGridLayout *gridLayout_20;
    QCheckBox *checkBox_map_shown;
    QLabel *label_map_shown;
    QLabel *label_170;
    QDoubleSpinBox *doubleSpinBox_map_opacity;
    QGroupBox *groupBox_octomap;
    QGridLayout *gridLayout_72;
    QCheckBox *checkBox_octomap_show3dMap;
    QLabel *label_octomap_treeDepth;
    QSpinBox *spinBox_octomap_treeDepth;
    QSpinBox *spinBox_octomap_pointSize;
    QLabel *label_octomap_treeDepth_5;
    QCheckBox *checkBox_octomap_2dgrid;
    QLabel *label_octomap_treeDepth_3;
    QLabel *label_octomap_treeDepth_6;
    QLabel *label_octomap_treeDepth_8;
    QComboBox *comboBox_octomap_renderingType;
    QSpacerItem *verticalSpacer_56;
    QWidget *page_8;
    QVBoxLayout *verticalLayout_43;
    QGroupBox *groupBox_logging1;
    QGridLayout *gridLayout_40;
    QLabel *label_60;
    QComboBox *comboBox_loggerLevel;
    QComboBox *comboBox_loggerEventLevel;
    QLabel *label_61;
    QComboBox *comboBox_loggerPauseLevel;
    QLabel *label_62;
    QCheckBox *checkBox_logger_printTime;
    QLabel *label_67;
    QComboBox *comboBox_loggerType;
    QLabel *label_68;
    QLabel *label_118;
    QCheckBox *checkBox_logger_printThreadId;
    QMultiComboBox *comboBox_loggerFilter;
    QLabel *label_164;
    QSpacerItem *verticalSpacer_2;
    QWidget *page_4;
    QVBoxLayout *verticalLayout_27;
    QGroupBox *groupBox_source0;
    QVBoxLayout *verticalLayout_7;
    QGridLayout *gridLayout_28;
    QLabel *label_3;
    QLabel *label_42;
    QLabel *label_19;
    QLabel *label_39;
    QLabel *label_244;
    QHBoxLayout *horizontalLayout_5;
    QToolButton *toolButton_source_path_calibration;
    QLineEdit *lineEdit_calibrationFile;
    QDoubleSpinBox *general_doubleSpinBox_imgRate;
    QLabel *label_17;
    QCheckBox *source_mirroring;
    QLineEdit *lineEdit_sourceLocalTransform;
    QComboBox *comboBox_sourceType;
    QPushButton *pushButton_test_camera;
    QPushButton *pushButton_calibrate;
    QLabel *label_24;
    QLineEdit *lineEdit_sourceDevice;
    QPushButton *pushButton_calibrate_simple;
    QLabel *label_18;
    QLabel *label_36;
    QSpinBox *spinBox_source_imageDecimation;
    QLabel *label_229;
    QCheckBox *checkbox_rgbd_colorOnly;
    QStackedWidget *stackedWidget_src;
    QWidget *page_41;
    QVBoxLayout *verticalLayout_64;
    QGroupBox *groupBox_sourceRGBD;
    QVBoxLayout *verticalLayout_11;
    QLabel *label_40;
    QGridLayout *gridLayout_55;
    QComboBox *comboBox_cameraRGBD;
    QLabel *label_228;
    QStackedWidget *stackedWidget_rgbd;
    QWidget *page_32;
    QVBoxLayout *verticalLayout_63;
    QGroupBox *groupBox_openni;
    QGridLayout *gridLayout_57;
    QLineEdit *lineEdit_openniOniPath;
    QLabel *label_245;
    QSpacerItem *verticalSpacer_34;
    QToolButton *toolButton_openniOniPath;
    QWidget *page_33;
    QVBoxLayout *verticalLayout_80;
    QSpacerItem *verticalSpacer_3;
    QWidget *page_34;
    QVBoxLayout *verticalLayout_89;
    QSpacerItem *verticalSpacer_27;
    QWidget *page_35;
    QVBoxLayout *verticalLayout_90;
    QSpacerItem *verticalSpacer_28;
    QWidget *page_36;
    QVBoxLayout *verticalLayout_51;
    QGroupBox *groupBox_openni2;
    QGridLayout *gridLayout_54;
    QLabel *label_435;
    QSpinBox *openni2_hshift;
    QCheckBox *openni2_autoWhiteBalance;
    QSpinBox *openni2_exposure;
    QLabel *label_264;
    QCheckBox *openni2_stampsIdsUsed;
    QLabel *label_231;
    QLabel *label_220;
    QLineEdit *lineEdit_openni2OniPath;
    QToolButton *toolButton_openni2OniPath;
    QLabel *label_217;
    QLabel *label_223;
    QCheckBox *openni2_mirroring;
    QSpinBox *openni2_gain;
    QSpacerItem *verticalSpacer_33;
    QCheckBox *openni2_autoExposure;
    QLabel *label_218;
    QLabel *label_219;
    QLabel *label_436;
    QSpinBox *openni2_vshift;
    QWidget *page_37;
    QVBoxLayout *verticalLayout_62;
    QGroupBox *groupBox_freenect2;
    QGridLayout *gridLayout_56;
    QLabel *label_269;
    QDoubleSpinBox *doubleSpinBox_freenect2MaxDepth;
    QComboBox *comboBox_freenect2Format;
    QCheckBox *checkBox_freenect2BilateralFiltering;
    QLabel *label_268;
    QCheckBox *checkBox_freenect2EdgeAwareFiltering;
    QLabel *label_266;
    QLabel *label_230;
    QDoubleSpinBox *doubleSpinBox_freenect2MinDepth;
    QLabel *label_270;
    QLabel *label_267;
    QCheckBox *checkBox_freenect2NoiseFiltering;
    QLabel *label_434;
    QLineEdit *lineEdit_freenect2Pipeline;
    QWidget *page_63;
    QVBoxLayout *verticalLayout_106;
    QGroupBox *groupBox_realsense;
    QGridLayout *gridLayout_73;
    QComboBox *comboBox_realsensePresetRGB;
    QLabel *label_318;
    QLabel *label_317;
    QComboBox *comboBox_realsensePresetDepth;
    QLabel *label_realsenseOdom;
    QCheckBox *checkbox_realsenseOdom;
    QCheckBox *checkbox_realsenseDepthScaledToRGBSize;
    QLabel *label_realsenseOdom_2;
    QComboBox *comboBox_realsenseRGBSource;
    QLabel *label_realsenseOdom_3;
    QSpacerItem *verticalSpacer_32;
    QWidget *page_47;
    QVBoxLayout *verticalLayout_79;
    QGroupBox *groupBox_cameraRGBDImages;
    QGridLayout *gridLayout_35;
    QLabel *label_252;
    QToolButton *toolButton_cameraRGBDImages_path_rgb;
    QLabel *label_257;
    QToolButton *toolButton_cameraRGBDImages_path_depth;
    QLineEdit *lineEdit_cameraRGBDImages_path_rgb;
    QLineEdit *lineEdit_cameraRGBDImages_path_depth;
    QLabel *label_254;
    QDoubleSpinBox *doubleSpinBox_cameraRGBDImages_scale;
    QSpacerItem *verticalSpacer_40;
    QLabel *label_441;
    QSpinBox *spinBox_cameraRGBDImages_startIndex;
    QLabel *label_540;
    QSpinBox *spinBox_cameraRGBDImages_maxFrames;
    QWidget *page_72;
    QVBoxLayout *verticalLayout_127;
    QGroupBox *groupBox_k4w2;
    QGridLayout *gridLayout_96;
    QComboBox *comboBox_k4w2Format;
    QLabel *label_440;
    QSpacerItem *verticalSpacer_68;
    QSpacerItem *verticalSpacer_67;
    QWidget *page_55;
    QVBoxLayout *verticalLayout_103;
    QGroupBox *groupBox_realsense2;
    QGridLayout *gridLayout_100;
    QSpinBox *spinBox_rs2_width_depth;
    QSpinBox *spinBox_rs2_height_depth;
    QLineEdit *lineEdit_rs2_jsonFile;
    QLabel *label_565;
    QLabel *label_549;
    QSpinBox *spinBox_rs2_height;
    QSpinBox *spinBox_rs2_rate;
    QCheckBox *checkbox_rs2_emitter;
    QLabel *label_468;
    QCheckBox *checkbox_rs2_irMode;
    QLabel *label_471;
    QSpinBox *spinBox_rs2_width;
    QLabel *label_550;
    QSpacerItem *verticalSpacer_71;
    QLabel *label_551;
    QLabel *label_562;
    QCheckBox *checkbox_rs2_irDepth;
    QCheckBox *checkbox_rs2_dualMode;
    QLineEdit *lineEdit_rs2_dualModeExtrinsics;
    QLabel *label_571;
    QToolButton *toolButton_rs2_jsonFile;
    QLabel *label_606;
    QCheckBox *checkbox_rs2_globalTimeStync;
    QLabel *label_611;
    QSpinBox *spinBox_rs2_rate_depth;
    QLabel *label_612;
    QLabel *label_613;
    QLabel *label_566;
    QWidget *page_84;
    QVBoxLayout *verticalLayout_145;
    QGroupBox *groupBox_k4a;
    QGridLayout *gridLayout_113;
    QCheckBox *checkbox_k4a_irDepth;
    QCheckBox *source_checkBox_useMKVStamps;
    QLabel *label_604;
    QLabel *label_601;
    QToolButton *toolButton_k4a_mkv;
    QLineEdit *lineEdit_k4a_mkv;
    QComboBox *comboBox_k4a_framerate;
    QComboBox *comboBox_k4a_rgb_resolution;
    QLabel *label_603;
    QLabel *label_558;
    QComboBox *comboBox_k4a_depth_resolution;
    QLabel *label_602;
    QLabel *label_600;
    QSpacerItem *verticalSpacer_42;
    QSpacerItem *verticalSpacer_83;
    QWidget *page_44;
    QVBoxLayout *verticalLayout_71;
    QGroupBox *groupBox_sourceStereo;
    QVBoxLayout *verticalLayout_67;
    QLabel *label_46;
    QGridLayout *gridLayout_58;
    QComboBox *comboBox_cameraStereo;
    QLabel *label_240;
    QCheckBox *checkbox_stereo_depthGenerated;
    QLabel *label_stereo_depthGenerated;
    QLabel *label_stereo_rectify;
    QCheckBox *checkBox_stereo_rectify;
    QLabel *label_stereo_rectify_2;
    QCheckBox *checkBox_stereo_exposureCompensation;
    QStackedWidget *stackedWidget_stereo;
    QWidget *page_49;
    QVBoxLayout *verticalLayout_91;
    QSpacerItem *verticalSpacer_30;
    QWidget *page_48;
    QVBoxLayout *verticalLayout_92;
    QSpacerItem *verticalSpacer_47;
    QWidget *page_53;
    QVBoxLayout *verticalLayout_70;
    QGroupBox *groupBox_cameraStereoImages;
    QGridLayout *gridLayout_61;
    QToolButton *toolButton_cameraStereoImages_path_left;
    QSpacerItem *verticalSpacer_38;
    QLabel *label_249;
    QLineEdit *lineEdit_cameraStereoImages_path_left;
    QLabel *label_253;
    QLineEdit *lineEdit_cameraStereoImages_path_right;
    QToolButton *toolButton_cameraStereoImages_path_right;
    QLabel *label_442;
    QSpinBox *spinBox_cameraStereoImages_startIndex;
    QLabel *label_530;
    QSpinBox *spinBox_cameraStereoImages_maxFrames;
    QWidget *page_50;
    QVBoxLayout *verticalLayout_78;
    QGroupBox *groupBox_cameraStereoVideo;
    QGridLayout *gridLayout_66;
    QToolButton *toolButton_cameraStereoVideo_path;
    QLineEdit *lineEdit_cameraStereoVideo_path;
    QSpacerItem *verticalSpacer_39;
    QLabel *label_339;
    QLabel *label_340;
    QLineEdit *lineEdit_cameraStereoVideo_path_2;
    QToolButton *toolButton_cameraStereoVideo_path_2;
    QWidget *page_59;
    QVBoxLayout *verticalLayout_101;
    QGroupBox *groupBox_cameraStereoZed;
    QGridLayout *gridLayout_37;
    QSpacerItem *verticalSpacer_54;
    QLabel *label_305;
    QComboBox *comboBox_stereoZed_resolution;
    QLabel *label_291;
    QComboBox *comboBox_stereoZed_sensingMode;
    QComboBox *comboBox_stereoZed_quality;
    QLabel *label_306;
    QSpinBox *spinBox_stereoZed_confidenceThr;
    QLabel *label_274;
    QHBoxLayout *horizontalLayout_8;
    QToolButton *toolButton_zedSvoPath;
    QLineEdit *lineEdit_zedSvoPath;
    QLabel *label_290;
    QCheckBox *checkbox_stereoZed_odom;
    QLabel *label_316;
    QLabel *label_343;
    QCheckBox *checkbox_stereoZed_selfCalibration;
    QLabel *label_570;
    QSpinBox *spinBox_stereoZed_texturenessConfidenceThr;
    QWidget *page_60;
    QVBoxLayout *verticalLayout_86;
    QGroupBox *groupBox_cameraStereoUsb;
    QGridLayout *gridLayout_99;
    QLabel *label_470;
    QSpinBox *spinBox_stereousbcam_streamHeight;
    QLabel *label_620;
    QLabel *label_619;
    QSpinBox *spinBox_stereo_right_device;
    QSpinBox *spinBox_stereousbcam_streamWidth;
    QSpacerItem *verticalSpacer_51;
    QWidget *page_77;
    QVBoxLayout *verticalLayout_135;
    QWidget *page_80;
    QVBoxLayout *verticalLayout_140;
    QGroupBox *groupBox_cameraStereoRealSense2;
    QGridLayout *gridLayout_108;
    QCheckBox *checkbox_stereoRealSense2_odom;
    QLabel *label_546;
    QSpacerItem *verticalSpacer_78;
    QWidget *page_86;
    QVBoxLayout *verticalLayout_148;
    QGroupBox *groupBox_cameraMyntEye;
    QGridLayout *gridLayout_115;
    QCheckBox *checkbox_stereoMyntEye_depth;
    QLabel *label_576;
    QLabel *label_581;
    QCheckBox *checkbox_stereoMyntEye_rectify;
    QSpinBox *spinBox_stereoMyntEye_brightness;
    QSpinBox *spinBox_stereoMyntEye_gain;
    QCheckBox *checkbox_stereoMyntEye_autoExposure;
    QLabel *label_595;
    QLabel *label_596;
    QLabel *label_594;
    QLabel *label_597;
    QSpinBox *spinBox_stereoMyntEye_contrast;
    QLabel *label_599;
    QSpinBox *spinBox_stereoMyntEye_irControl;
    QSpacerItem *verticalSpacer_85;
    QWidget *page_88;
    QVBoxLayout *verticalLayout_154;
    QGroupBox *groupBox_cameraStereoZedOC;
    QGridLayout *gridLayout_119;
    QComboBox *comboBox_stereoZedOC_resolution;
    QLabel *label_624;
    QSpacerItem *verticalSpacer_70;
    QWidget *page_89;
    QVBoxLayout *verticalLayout_155;
    QGroupBox *groupBox_cameraDepthAI;
    QGridLayout *gridLayout_120;
    QLabel *label_626;
    QLabel *label_625;
    QCheckBox *checkBox_depthai_depth;
    QComboBox *comboBox_depthai_resolution;
    QLabel *label_627;
    QSpinBox *spinBox_depthai_confidence;
    QSpacerItem *verticalSpacer_72;
    QWidget *page_42;
    QVBoxLayout *verticalLayout_65;
    QGroupBox *groupBox_sourceRGB;
    QVBoxLayout *verticalLayout_5;
    QGridLayout *gridLayout_5;
    QLabel *label_20;
    QComboBox *source_comboBox_image_type;
    QLabel *label_242;
    QCheckBox *checkBox_rgb_rectify;
    QStackedWidget *stackedWidget_image;
    QWidget *page_7;
    QVBoxLayout *verticalLayout_30;
    QGroupBox *source_groupBox_usbcam;
    QGridLayout *gridLayout_112;
    QSpinBox *spinBox_usbcam_streamHeight;
    QLabel *label_553;
    QLabel *label_552;
    QSpinBox *spinBox_usbcam_streamWidth;
    QSpacerItem *verticalSpacer_8;
    QWidget *page_5;
    QVBoxLayout *verticalLayout_6;
    QGroupBox *source_groupBox_images;
    QGridLayout *gridLayout_7;
    QLineEdit *source_images_lineEdit_path;
    QLabel *label_21;
    QSpinBox *source_images_spinBox_startPos;
    QToolButton *source_images_toolButton_selectSource;
    QLabel *label_34;
    QSpinBox *source_images_spinBox_maxFrames;
    QSpacerItem *verticalSpacer_9;
    QWidget *page_6;
    QVBoxLayout *verticalLayout_8;
    QGroupBox *source_groupBox_video;
    QGridLayout *gridLayout_8;
    QLineEdit *source_video_lineEdit_path;
    QToolButton *source_video_toolButton_selectSource;
    QSpacerItem *verticalSpacer_10;
    QWidget *page_43;
    QVBoxLayout *verticalLayout_66;
    QGroupBox *groupBox_sourceDatabase;
    QGridLayout *gridLayout_9;
    QLabel *label_263;
    QCheckBox *source_checkBox_ignoreGoals;
    QCheckBox *source_checkBox_ignoreOdometry;
    QToolButton *toolButton_dbViewer;
    QToolButton *source_database_toolButton_selectSource;
    QLineEdit *source_database_lineEdit_path;
    QSpacerItem *verticalSpacer_35;
    QLabel *label_58;
    QSpinBox *source_spinBox_databaseStartId;
    QLabel *label_72;
    QLabel *label_80;
    QCheckBox *source_checkBox_ignoreGoalDelay;
    QSpinBox *source_spinBox_database_cameraIndex;
    QLabel *label_90;
    QCheckBox *source_checkBox_useDbStamps;
    QLabel *label_315;
    QLabel *label_529;
    QSpinBox *source_spinBox_databaseStopId;
    QLabel *label_557;
    QCheckBox *source_checkBox_stereoToDepthDB;
    QGroupBox *groupBox_sourceImages_optional;
    QVBoxLayout *verticalLayout_93;
    QGridLayout *gridLayout_68;
    QLabel *label_288;
    QLabel *label_289;
    QLabel *label_265;
    QToolButton *toolButton_cameraImages_gt;
    QLineEdit *lineEdit_cameraImages_odom;
    QLabel *label_293;
    QLabel *label_348;
    QLabel *label_255;
    QToolButton *toolButton_cameraImages_timestamps;
    QLineEdit *lineEdit_cameraImages_timestamps;
    QLineEdit *lineEdit_cameraImages_gt;
    QCheckBox *checkBox_cameraImages_syncTimeStamps;
    QCheckBox *checkBox_cameraImages_timestamps;
    QLabel *label_292;
    QLabel *label_256;
    QLineEdit *lineEdit_cameraImages_path_scans;
    QLabel *label_251;
    QSpinBox *spinBox_cameraImages_max_scan_pts;
    QComboBox *comboBox_cameraImages_gtFormat;
    QToolButton *toolButton_cameraImages_path_scans;
    QLineEdit *lineEdit_cameraImages_laser_transform;
    QLabel *label_294;
    QComboBox *comboBox_cameraImages_bayerMode;
    QLabel *label_349;
    QComboBox *comboBox_cameraImages_odomFormat;
    QToolButton *toolButton_cameraImages_odom;
    QLabel *label_443;
    QDoubleSpinBox *doubleSpinBox_maxPoseTimeDiff;
    QLabel *label_464;
    QLabel *label_463;
    QToolButton *toolButton_cameraImages_path_imu;
    QLineEdit *lineEdit_cameraImages_path_imu;
    QLineEdit *lineEdit_cameraImages_imu_transform;
    QLabel *label_465;
    QSpinBox *spinBox_cameraImages_max_imu_rate;
    QLabel *label_605;
    QCheckBox *checkBox_cameraImages_configForEachFrame;
    QGroupBox *groupBox_depthFromScan;
    QVBoxLayout *verticalLayout_100;
    QGroupBox *groupBox_depthFromScan_fillHoles;
    QVBoxLayout *verticalLayout_99;
    QRadioButton *radioButton_depthFromScan_vertical;
    QRadioButton *radioButton_depthFromScan_horizontal;
    QCheckBox *checkBox_depthFromScan_fillBorders;
    QGroupBox *groupBox_depthImageFiltering;
    QVBoxLayout *verticalLayout_94;
    QHBoxLayout *horizontalLayout_10;
    QToolButton *toolButton_source_distortionModel;
    QLineEdit *lineEdit_source_distortionModel;
    QToolButton *toolButton_distortionModel;
    QLabel *label_341;
    QGroupBox *groupBox_bilateral;
    QGridLayout *gridLayout_78;
    QDoubleSpinBox *doubleSpinBox_bilateral_sigmaR;
    QLabel *label_344;
    QDoubleSpinBox *doubleSpinBox_bilateral_sigmaS;
    QLabel *label_345;
    QGroupBox *groupBox_imuFiltering;
    QVBoxLayout *verticalLayout_141;
    QGridLayout *gridLayout_109;
    QComboBox *comboBox_imuFilter_strategy;
    QLabel *label_541;
    QCheckBox *checkbox_publishInterIMU;
    QLabel *label_563;
    QLabel *label_621;
    QCheckBox *checkBox_imuFilter_baseFrameConversion;
    QStackedWidget *stackedWidget_imuFilter;
    QWidget *page_83;
    QVBoxLayout *verticalLayout_144;
    QSpacerItem *verticalSpacer_80;
    QWidget *page_81;
    QVBoxLayout *verticalLayout_142;
    QGroupBox *groupBox_madgwickfilter;
    QGridLayout *gridLayout_110;
    QDoubleSpinBox *doubleSpinBox_imuFilterMadgwickGain;
    QLabel *label_542;
    QDoubleSpinBox *doubleSpinBox_imuFilterMadgwickZeta;
    QLabel *label_543;
    QSpacerItem *verticalSpacer_79;
    QWidget *page_82;
    QVBoxLayout *verticalLayout_143;
    QGroupBox *groupBox_complementaryfilter;
    QGridLayout *gridLayout_111;
    QLabel *label_547;
    QLabel *label_544;
    QCheckBox *checkBox_imuFilterComplementaryDoBiasEstimation;
    QDoubleSpinBox *doubleSpinBox_imuFilterComplementaryGainAcc;
    QLabel *label_545;
    QDoubleSpinBox *doubleSpinBox_imuFilterComplementaryBiasAlpha;
    QCheckBox *checkBox_imuFilterComplementaryDoAdaptiveGain;
    QLabel *label_548;
    QSpacerItem *verticalSpacer_81;
    QGroupBox *groupBox_scan;
    QVBoxLayout *verticalLayout_13;
    QLabel *label_16;
    QGridLayout *gridLayout_62;
    QDoubleSpinBox *doubleSpinBox_source_scanRangeMin;
    QSpinBox *spinBox_source_scanNormalsK;
    QLabel *label_300;
    QDoubleSpinBox *doubleSpinBox_source_scanVoxelSize;
    QSpinBox *spinBox_source_scanDownsampleStep;
    QLabel *label_302;
    QLabel *label_304;
    QLabel *label_425;
    QDoubleSpinBox *doubleSpinBox_source_scanNormalsRadius;
    QLabel *label_453;
    QDoubleSpinBox *doubleSpinBox_source_scanRangeMax;
    QLabel *label_522;
    QLabel *label_523;
    QCheckBox *checkBox_source_scanFromDepth;
    QLabel *label_source_scanFromDepth;
    QDoubleSpinBox *doubleSpinBox_source_scanNormalsForceGroundUp;
    QSpacerItem *verticalSpacer_7;
    QWidget *page_20;
    QVBoxLayout *verticalLayout_23;
    QGroupBox *groupBox_rtabmap_basic0;
    QGridLayout *gridLayout_3;
    QDoubleSpinBox *general_doubleSpinBox_detectionRate_2;
    QCheckBox *general_checkBox_SLAM_mode_2;
    QLabel *label_79;
    QCheckBox *general_checkBox_activateRGBD_2;
    QLabel *label_activateRGBD_2;
    QCheckBox *general_checkBox_publishStats_2;
    QLabel *label_publishStat;
    QLabel *label_134;
    QSpinBox *general_spinBox_imagesBufferSize_2;
    QLabel *label_112;
    QDoubleSpinBox *general_doubleSpinBox_timeThr_2;
    QLabel *label_timeLimit_2;
    QDoubleSpinBox *general_doubleSpinBox_hardThr_2;
    QLabel *label_121;
    QDoubleSpinBox *doubleSpinBox_similarityThreshold_2;
    QLabel *label_similarity_2;
    QSpinBox *general_spinBox_maxStMemSize_2;
    QLabel *label_126;
    QSpacerItem *verticalSpacer_5;
    QWidget *page;
    QVBoxLayout *verticalLayout_44;
    QGroupBox *groupBox_rtabmap_advanced0;
    QVBoxLayout *verticalLayout_15;
    QGridLayout *gridLayout_43;
    QDoubleSpinBox *general_doubleSpinBox_detectionRate;
    QCheckBox *general_checkBox_SLAM_mode;
    QLabel *label_92;
    QCheckBox *general_checkBox_startNewMapOnLoopClosure;
    QLabel *label_135;
    QSpinBox *general_spinBox_imagesBufferSize;
    QCheckBox *general_checkBox_createIntermediateNodes;
    QLabel *label_185;
    QLabel *label_84;
    QLabel *label_165;
    QLabel *label_467;
    QCheckBox *general_checkBox_imagesAlreadyRectified;
    QLabel *label_329;
    QCheckBox *general_checkBox_startNewMapOnGoodSignature;
    QLabel *label_498;
    QCheckBox *general_checkBox_rectifyOnlyFeatures;
    QLabel *label_573;
    QCheckBox *general_checkBox_saveLocalizationData;
    QGroupBox *groupBox_6;
    QGridLayout *gridLayout_44;
    QDoubleSpinBox *general_doubleSpinBox_timeThr;
    QLabel *label_timeLimit;
    QSpinBox *general_spinBox_memoryThr;
    QLabel *label_maxWmSize;
    QDoubleSpinBox *general_doubleSpinBox_hardThr;
    QLabel *label_93;
    QDoubleSpinBox *general_doubleSpinBox_loopRatio;
    QLabel *label_96;
    QGroupBox *groupBox_publishing;
    QGridLayout *gridLayout_45;
    QCheckBox *general_checkBox_publishPdf;
    QLabel *label_106;
    QCheckBox *general_checkBox_publishLikelihood;
    QLabel *label_462;
    QCheckBox *general_checkBox_publishRawData;
    QLabel *label_91;
    QLabel *label_116;
    QLabel *label_452;
    QCheckBox *general_checkBox_publishRAM;
    QLabel *label_437;
    QCheckBox *general_checkBox_publishRMSE;
    QCheckBox *general_checkBox_saveWMState;
    QGroupBox *groupBox_statistics;
    QGridLayout *gridLayout_46;
    QCheckBox *general_checkBox_statisticLogsBufferedInRAM;
    QLabel *label_143;
    QCheckBox *general_checkBox_statisticLoggedHeaders;
    QLabel *label_209;
    QGridLayout *gridLayout_4;
    QLabel *label_workingDirectory;
    QToolButton *toolButton_workingDirectory;
    QLineEdit *lineEdit_workingDirectory;
    QSpacerItem *verticalSpacer_4;
    QWidget *page_3;
    QVBoxLayout *verticalLayout_4;
    QGroupBox *groupBox_memory1;
    QVBoxLayout *verticalLayout_10;
    QGridLayout *gridLayout_42;
    QSpinBox *spinBox_imagePreDecimation;
    QSpinBox *spinBox_imagePostDecimation;
    QSpinBox *general_spinBox_maxStMemSize;
    QLabel *label_retrieved_10;
    QDoubleSpinBox *general_doubleSpinBox_recentWmRatio;
    QLabel *label_retrieved_4;
    QCheckBox *general_checkBox_initWMWithAllNodes;
    QSpinBox *general_spinBox_maxRetrieved;
    QCheckBox *general_checkBox_keepRawData;
    QCheckBox *general_checkBox_reduceGraph;
    QLabel *label_retrieved_3;
    QCheckBox *general_checkBox_createMapLabels;
    QLabel *label_retrieved_5;
    QLabel *label_retrieved_11;
    QCheckBox *general_checkBox_keepDescriptors;
    QLabel *label_retrieved_12;
    QLabel *label_retrieved_13;
    QLabel *label_retrieved_2;
    QLabel *label_retrieved;
    QCheckBox *general_checkBox_generateIds;
    QCheckBox *general_checkBox_badSignaturesIgnored;
    QCheckBox *general_checkBox_transferSortingByWeightId;
    QLabel *label_retrieved_9;
    QLabel *label_29;
    QCheckBox *general_checkBox_saveDepth16bits;
    QLabel *label_ratioRecent;
    QLabel *label_retrieved_6;
    QLabel *label_retrieved_7;
    QCheckBox *general_checkBox_compressionParallelized;
    QCheckBox *general_checkBox_saveIntermediateNodeData;
    QLabel *label_retrieved_15;
    QLabel *label_retrieved_16;
    QGroupBox *groupBox_27;
    QGridLayout *gridLayout_10;
    QSpinBox *general_spinBox_laserScanDownsample;
    QLabel *label_retrieved_8;
    QDoubleSpinBox *general_doubleSpinBox_laserScanNormalRadius;
    QSpinBox *general_spinBox_laserScanNormalK;
    QLabel *label_retrieved_14;
    QLabel *label_retrieved_17;
    QLabel *label_retrieved_18;
    QDoubleSpinBox *general_doubleSpinBox_laserScanVoxelSize;
    QGroupBox *groupBox_rehearsal2;
    QGridLayout *gridLayout_12;
    QCheckBox *general_checkBox_RehearsalIdUpdatedToNewOne;
    QLabel *label_similarity;
    QDoubleSpinBox *doubleSpinBox_similarityThreshold;
    QLabel *label_rehearsalIdUpdate;
    QLabel *label_rehearsalIdUpdate_2;
    QCheckBox *rgdb_rehearsalWeightIgnoredWhileMoving;
    QSpacerItem *verticalSpacer_6;
    QWidget *page_9;
    QVBoxLayout *verticalLayout_58;
    QGroupBox *groupBox_signature_keypoint2;
    QVBoxLayout *verticalLayout_12;
    QHBoxLayout *horizontalLayout_7;
    QCheckBox *checkBox_useOdomFeatures;
    QLabel *label_145;
    QGroupBox *groupBox_8;
    QVBoxLayout *verticalLayout_60;
    QGridLayout *gridLayout_32;
    QLabel *label_22;
    QLabel *label_81;
    QLabel *label_99;
    QDoubleSpinBox *surf_doubleSpinBox_ratioBadSign;
    QLabel *label_57;
    QComboBox *comboBox_detector_strategy;
    QLineEdit *lineEdit_kp_roi;
    QLabel *label_101;
    QLabel *label_53;
    QDoubleSpinBox *doubleSpinBox_kp_roi1;
    QLabel *label_117;
    QDoubleSpinBox *surf_doubleSpinBox_maxDepth;
    QSpinBox *surf_spinBox_wordsPerImageTarget;
    QDoubleSpinBox *doubleSpinBox_kp_roi0;
    QLabel *label_97;
    QLabel *label_98;
    QDoubleSpinBox *doubleSpinBox_kp_roi2;
    QDoubleSpinBox *doubleSpinBox_kp_roi3;
    QLabel *label_100;
    QDoubleSpinBox *surf_doubleSpinBox_minDepth;
    QLabel *label_50;
    QSpinBox *spinBox_KPGridRows;
    QSpinBox *spinBox_KPGridCols;
    QLabel *label_86;
    QLabel *label_262;
    QCheckBox *checkBox_memDepthAsMask;
    QLabel *label_582;
    QCheckBox *checkBox_memStereoFromMotion;
    QGroupBox *groupBox_15;
    QVBoxLayout *verticalLayout_76;
    QLabel *label_44;
    QGridLayout *gridLayout_41;
    QSpinBox *subpix_winSize_kp;
    QLabel *label_202;
    QSpinBox *subpix_iterations_kp;
    QLabel *label_225;
    QDoubleSpinBox *subpix_eps_kp;
    QLabel *label_227;
    QGroupBox *groupBox_dictionary_strategy2;
    QGridLayout *gridLayout_18;
    QToolButton *toolButton_dictionaryPath;
    QLabel *label_78;
    QDoubleSpinBox *surf_doubleSpinBox_nndrRatio;
    QLineEdit *lineEdit_dictionaryPath;
    QLabel *label_dictionaryPath;
    QCheckBox *checkBox_dictionary_incremental;
    QLabel *label_12;
    QComboBox *comboBox_dictionary_strategy;
    QLabel *label_incrementalDict;
    QLabel *label_94;
    QCheckBox *checkBox_kp_parallelized;
    QCheckBox *checkBox_kp_newWordsComparedTogether;
    QLabel *label_186;
    QLabel *label_260;
    QCheckBox *checkBox_kp_incrementalFlann;
    QLabel *label_451;
    QDoubleSpinBox *surf_doubleSpinBox_rebalancingFactor;
    QLabel *label_556;
    QCheckBox *checkBox_kp_byteToFloat;
    QSpacerItem *verticalSpacer_15;
    QWidget *page_24;
    QVBoxLayout *verticalLayout_33;
    QGroupBox *groupBox_database2;
    QGridLayout *gridLayout_11;
    QCheckBox *general_checkBox_keepBinaryData;
    QLabel *label_keepRawData;
    QCheckBox *checkBox_dbInMemory;
    QComboBox *comboBox_dbJournalMode;
    QLabel *label_74;
    QLabel *label_120;
    QComboBox *comboBox_dbTempStore;
    QCheckBox *general_checkBox_keepNotLinkedNodes;
    QSpinBox *spinBox_dbCacheSize;
    QLabel *label_75;
    QLabel *label_77;
    QLabel *label_89;
    QComboBox *comboBox_dbSynchronous;
    QLabel *label_keepRehearsed;
    QLabel *label_299;
    QLineEdit *lineEdit_rgbCompressionFormat;
    QLabel *label_610;
    QLineEdit *lineEdit_targetDatabaseVersion;
    QSpacerItem *verticalSpacer_12;
    QWidget *page_23;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_bayes1;
    QVBoxLayout *verticalLayout_22;
    QLabel *label;
    QLabel *label_76;
    QGridLayout *gridLayout_24;
    QLabel *label_65;
    QLabel *label_66;
    QLabel *label_prediction_sum;
    QLineEdit *lineEdit_bayes_predictionLC;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_63;
    UPlot *predictionPlot;
    QGridLayout *gridLayout_22;
    QCheckBox *checkBox_bayes_fullPredictionUpdate;
    QDoubleSpinBox *general_doubleSpinBox_vp;
    QLabel *label_113;
    QLabel *label_142;
    QLabel *label_82;
    QCheckBox *checkBox_kp_tfIdfLikelihoodUsed;
    QLabel *label_502;
    QCheckBox *checkBox_rtabmap_loopGPS;
    QGroupBox *groupBox_vh_epipolar2;
    QGridLayout *gridLayout;
    QSpinBox *surf_spinBox_matchCountMinAccepted;
    QLabel *label_25;
    QDoubleSpinBox *surf_doubleSpinBox_ransacParam1;
    QLabel *label_27;
    QDoubleSpinBox *surf_doubleSpinBox_ransacParam2;
    QLabel *label_28;
    QSpacerItem *verticalSpacer_23;
    QWidget *page_10;
    QVBoxLayout *verticalLayout_19;
    QGroupBox *groupBox_slam1;
    QVBoxLayout *verticalLayout_25;
    QHBoxLayout *horizontalLayout_4;
    QCheckBox *general_checkBox_activateRGBD;
    QLabel *label_activateRGBD;
    QLabel *label_83;
    QGroupBox *groupBox_slam_update;
    QGridLayout *gridLayout_47;
    QLabel *label_432;
    QLabel *label_scanMatching;
    QLabel *label_152;
    QSpinBox *spinBox_maxLocalLocationsRetrieved;
    QCheckBox *loopClosure_reextract;
    QDoubleSpinBox *rgdb_newMapOdomChange;
    QLabel *label_scanMatching_5;
    QCheckBox *rgbd_savedLocalizationIgnored;
    QLabel *label_scanMatching_9;
    QDoubleSpinBox *rgdb_linearSpeedUpdate;
    QCheckBox *odomScanHistory;
    QDoubleSpinBox *localDetection_radius;
    QDoubleSpinBox *rgdb_localImmunizationRatio;
    QCheckBox *memCovOffDiagIgnored;
    QLabel *label_163;
    QLabel *label_433;
    QCheckBox *loopClosure_bunlde;
    QLabel *label_scanMatching_10;
    QLabel *label_scanMatching_11;
    QDoubleSpinBox *rgdb_linearUpdate;
    QLabel *label_153;
    QCheckBox *rgbd_loopCovLimited;
    QDoubleSpinBox *rgdb_angularUpdate;
    QLabel *label_scanMatching_2;
    QLabel *label_space2;
    QLabel *label_scanMatching_3;
    QSpinBox *spinBox_maxOdomCacheSize;
    QDoubleSpinBox *rgdb_angularSpeedUpdate;
    QLabel *label_scanMatching_12;
    QCheckBox *odomGravity;
    QLabel *label_space2_5;
    QLabel *label_scanMatching_7;
    QLabel *label_space2_12;
    QDoubleSpinBox *maxLocalizationDistance;
    QLabel *label_scanMatching_14;
    QCheckBox *loopClosure_identityGuess;
    QSpacerItem *verticalSpacer_11;
    QWidget *page_28;
    QVBoxLayout *verticalLayout_95;
    QGroupBox *groupBox_proximityDetection2;
    QVBoxLayout *verticalLayout_96;
    QGroupBox *groupBox_localDetection_time;
    QVBoxLayout *verticalLayout_21;
    QLabel *label_54;
    QGroupBox *groupBox_localDetection_space;
    QVBoxLayout *verticalLayout_48;
    QLabel *label_space1_2;
    QGridLayout *gridLayout_50;
    QDoubleSpinBox *localDetection_pathFilteringRadius;
    QLabel *label_scanMatching_4;
    QLabel *label_space3_3;
    QCheckBox *checkBox_localSpacePathOdomPosesUsed;
    QSpinBox *localDetection_maxDiffID;
    QLabel *label_space3_2;
    QLabel *label_scanMatching_6;
    QCheckBox *checkBox_localSpaceScanMatchingIDsSaved;
    QLabel *label_scanMatching_8;
    QDoubleSpinBox *localDetection_angle;
    QLabel *label_space3_4;
    QSpinBox *localDetection_maxPaths;
    QLabel *label_space3_8;
    QSpinBox *localDetection_maxNeighbors;
    QLabel *label_scanMatching_13;
    QCheckBox *checkBox_localSpaceOdomGuess;
    QSpacerItem *verticalSpacer_49;
    QWidget *page_19;
    QVBoxLayout *verticalLayout_26;
    QGroupBox *groupBox_graphOptimization2;
    QGridLayout *gridLayout_81;
    QComboBox *graphOptimization_type;
    QLabel *label_45;
    QCheckBox *graphOptimization_robust;
    QLabel *label_258;
    QDoubleSpinBox *graphOptimization_maxError;
    QLabel *label_259;
    QCheckBox *graphOptimization_fromGraphEnd;
    QFrame *line;
    QLabel *label_151;
    QLabel *label_211;
    QLabel *label_183;
    QLabel *label_431;
    QCheckBox *graphOptimization_priorsIgnored;
    QCheckBox *graphOptimization_landmarksIgnored;
    QLabel *label_521;
    QLabel *label_527;
    QDoubleSpinBox *graphOptimization_gravitySigma;
    QSpacerItem *verticalSpacer_48;
    QWidget *page_64;
    QVBoxLayout *verticalLayout_109;
    QGroupBox *groupBox_occupancy2;
    QVBoxLayout *verticalLayout_110;
    QGridLayout *gridLayout_75;
    QDoubleSpinBox *doubleSpinBox_grid_noiseRadius;
    QDoubleSpinBox *doubleSpinBox_grid_minGroundHeight;
    QLabel *label_335;
    QDoubleSpinBox *doubleSpinBox_grid_maxObstacleHeight;
    QDoubleSpinBox *doubleSpinBox_grid_maxGroundHeight;
    QLabel *label_321;
    QLabel *label_310;
    QLabel *label_309;
    QSpinBox *spinBox_grid_noiseMinNeighbors;
    QDoubleSpinBox *doubleSpinBox_grid_resolution;
    QLabel *label_331;
    QSpinBox *spinBox_grid_scanDecimation;
    QLabel *label_333;
    QLabel *label_334;
    QDoubleSpinBox *doubleSpinBox_grid_footprintWidth;
    QDoubleSpinBox *doubleSpinBox_grid_footprintHeight;
    QLabel *label_327;
    QLabel *label_326;
    QCheckBox *checkbox_rgbd_createOccupancyGrid;
    QLabel *label_325;
    QDoubleSpinBox *doubleSpinBox_grid_footprintLength;
    QCheckBox *checkBox_grid_projMapFrame;
    QLabel *label_365;
    QLabel *label_311;
    QLabel *label_323;
    QDoubleSpinBox *doubleSpinBox_grid_minDepth;
    QLabel *label_320;
    QDoubleSpinBox *doubleSpinBox_grid_maxDepth;
    QLabel *label_456;
    QCheckBox *checkBox_grid_preVoxelFiltering;
    QGroupBox *groupBox_grid_fromDepthImage;
    QVBoxLayout *verticalLayout_108;
    QLabel *label_octomap_groundObstacle_2;
    QGridLayout *gridLayout_76;
    QSpinBox *spinBox_grid_decimation;
    QLabel *label_322;
    QLineEdit *lineEdit_grid_roi;
    QLabel *label_324;
    QLabel *label_363;
    QCheckBox *checkBox_grid_projRayTracing;
    QGroupBox *groupBox_grid_normalsSegmentation;
    QVBoxLayout *verticalLayout_107;
    QLabel *label_312;
    QGridLayout *gridLayout_74;
    QDoubleSpinBox *doubleSpinBox_grid_maxGroundAngle;
    QSpinBox *spinBox_grid_minClusterSize;
    QLabel *label_307;
    QLabel *label_308;
    QLabel *label_272;
    QCheckBox *checkBox_grid_flatObstaclesDetected;
    QSpinBox *spinBox_grid_normalK;
    QLabel *label_330;
    QLabel *label_336;
    QDoubleSpinBox *doubleSpinBox_grid_clusterRadius;
    QGroupBox *groupBox_grid_3d;
    QVBoxLayout *verticalLayout_102;
    QLabel *label_octomap_groundObstacle_3;
    QGridLayout *gridLayout_67;
    QLabel *label_octomap_groundObstacle;
    QCheckBox *checkBox_grid_groundObstacle;
    QGroupBox *groupBox_9;
    QVBoxLayout *verticalLayout_111;
    QLabel *label_octomap_groundObstacle_5;
    QGridLayout *gridLayout_77;
    QCheckBox *checkBox_grid_unknownSpaceFilled;
    QLabel *label_328;
    QSpacerItem *verticalSpacer_57;
    QWidget *page_66;
    QVBoxLayout *verticalLayout_116;
    QGroupBox *groupBox_occupancyGlobal2;
    QVBoxLayout *verticalLayout_115;
    QGridLayout *gridLayout_88;
    QLabel *label_484;
    QCheckBox *checkBox_grid_fullUpdate;
    QLabel *label_332;
    QLabel *label_224;
    QLabel *label_482;
    QLabel *label_319;
    QCheckBox *checkBox_grid_erode;
    QSpinBox *spinBox_grid_maxNodes;
    QDoubleSpinBox *doubleSpinBox_grid_footprintRadius;
    QLabel *label_366;
    QDoubleSpinBox *doubleSpinBox_grid_minMapSize;
    QLabel *label_455;
    QLabel *label_454;
    QDoubleSpinBox *doubleSpinBox_grid_updateError;
    QLabel *label_457;
    QDoubleSpinBox *doubleSpinBox_grid_occThr;
    QLabel *label_483;
    QDoubleSpinBox *doubleSpinBox_grid_probHit;
    QDoubleSpinBox *doubleSpinBox_grid_probMiss;
    QLabel *label_485;
    QDoubleSpinBox *doubleSpinBox_grid_clampingMin;
    QDoubleSpinBox *doubleSpinBox_grid_clampingMax;
    QLabel *label_616;
    QDoubleSpinBox *doubleSpinBox_grid_altitudeDelta;
    QSpacerItem *verticalSpacer_59;
    QWidget *page_30;
    QVBoxLayout *verticalLayout_97;
    QGroupBox *groupBox_pathPlanning2;
    QVBoxLayout *verticalLayout_61;
    QLabel *label_space2_3;
    QGridLayout *gridLayout_51;
    QDoubleSpinBox *graphPlan_goalReachedRadius;
    QLabel *label_space2_2;
    QLabel *label_space3_5;
    QCheckBox *graphPlan_goalsSavedInUserData;
    QLabel *label_space2_4;
    QSpinBox *graphPlan_stuckIterations;
    QDoubleSpinBox *graphPlan_linearVelocity;
    QDoubleSpinBox *graphPlan_angularVelocity;
    QLabel *label_space3_6;
    QLabel *label_space3_7;
    QSpacerItem *verticalSpacer_50;
    QWidget *page_78;
    QVBoxLayout *verticalLayout_137;
    QGroupBox *groupBox_markerDetection2;
    QVBoxLayout *verticalLayout_136;
    QGridLayout *gridLayout_63;
    QLabel *label_space2_9;
    QDoubleSpinBox *ArucoVarianceLinear;
    QLabel *label_markerDetection;
    QLabel *label_space2_6;
    QLabel *label_space2_7;
    QCheckBox *RGBDMarkerDetection;
    QDoubleSpinBox *ArucoVarianceAngular;
    QDoubleSpinBox *ArucoMarkerLength;
    QLabel *label_space2_8;
    QDoubleSpinBox *ArucoMaxDepthError;
    QLabel *label_space2_13;
    QLabel *label_space2_14;
    QDoubleSpinBox *ArucoMarkerRangeMin;
    QDoubleSpinBox *ArucoMarkerRangeMax;
    QGroupBox *groupBox_31;
    QGridLayout *gridLayout_106;
    QComboBox *ArucoDictionary;
    QLabel *label_space2_10;
    QComboBox *ArucoCornerRefinementMethod;
    QLabel *label_space2_11;
    QSpacerItem *verticalSpacer_76;
    QWidget *page_12;
    QVBoxLayout *verticalLayout_32;
    QGroupBox *groupBox_odometry1;
    QVBoxLayout *verticalLayout_34;
    QLabel *label_114;
    QGridLayout *gridLayout_27;
    QLabel *label_198;
    QLabel *label_196;
    QLabel *label_246;
    QLabel *label_103;
    QComboBox *odom_strategy;
    QLabel *label_314;
    QCheckBox *odom_alignWithGround;
    QLabel *label_238;
    QCheckBox *odom_flow_guessMotion;
    QSpinBox *odom_dataBufferSize;
    QPushButton *pushButton_testOdometry;
    QLabel *label_221;
    QLabel *label_232;
    QSpinBox *odom_imageDecimation;
    QLabel *label_47;
    QLabel *label_248;
    QComboBox *odom_filteringStrategy;
    QCheckBox *odom_fillInfoData;
    QDoubleSpinBox *odom_flow_keyframeThr;
    QCheckBox *odom_holonomic;
    QLabel *label_233;
    QSpinBox *odom_countdown;
    QDoubleSpinBox *odom_flow_scanKeyframeThr;
    QLabel *label_351;
    QComboBox *odom_registration;
    QLabel *label_354;
    QSpinBox *odom_VisKeyFrameThr;
    QLabel *label_520;
    QDoubleSpinBox *odom_guess_smoothing_delay;
    QStackedWidget *stackedWidget_odometryType;
    QWidget *page_52;
    QVBoxLayout *verticalLayout_77;
    QGroupBox *groupBox_odomF2M;
    QVBoxLayout *verticalLayout_54;
    QLabel *label_226;
    QGridLayout *gridLayout_29;
    QSpinBox *spinBox_odom_f2m_maxNewFeatures;
    QDoubleSpinBox *doubleSpinBox_odom_f2m_scanRadius;
    QLabel *label_215;
    QLabel *label_195;
    QLabel *label_190;
    QSpinBox *odom_localHistory;
    QSpinBox *spinBox_odom_f2m_scanMaxSize;
    QLabel *label_194;
    QLabel *label_357;
    QSpinBox *odom_f2m_bundleMaxFrames;
    QLabel *label_358;
    QComboBox *odom_f2m_bundleStrategy;
    QLabel *label_430;
    QDoubleSpinBox *doubleSpinBox_odom_f2m_scanAngle;
    QLabel *label_524;
    QDoubleSpinBox *odom_f2m_validDepthRatio;
    QLabel *label_564;
    QDoubleSpinBox *doubleSpinBox_odom_f2m_scanRange;
    QLabel *label_598;
    QDoubleSpinBox *odom_f2m_gravitySigma;
    QSpacerItem *verticalSpacer_46;
    QWidget *page_58;
    QVBoxLayout *verticalLayout_55;
    QGroupBox *groupBox_odomF2F;
    QVBoxLayout *verticalLayout_53;
    QLabel *label_199;
    QHBoxLayout *horizontalLayout_12;
    QComboBox *comboBox_odomf2f_corType;
    QLabel *label_149;
    QGroupBox *groupBox_odomf2f_corType;
    QGridLayout *gridLayout_52;
    QSpinBox *odom_flow_winSize_2;
    QLabel *label_201;
    QSpinBox *odom_flow_maxLevel_2;
    QLabel *label_295;
    QLabel *label_296;
    QSpinBox *odom_flow_iterations_2;
    QLabel *label_297;
    QDoubleSpinBox *odom_flow_eps_2;
    QSpacerItem *verticalSpacer_45;
    QSpacerItem *verticalSpacer_43;
    QWidget *page_67;
    QVBoxLayout *verticalLayout_119;
    QGroupBox *groupBox_odomFovis;
    QVBoxLayout *verticalLayout_118;
    QLabel *label_234;
    QGroupBox *groupBox_23;
    QGridLayout *gridLayout_90;
    QSpinBox *spinBox_OdomFovisFeatureSearchWindow;
    QSpinBox *spinBox_OdomFovisMinFeaturesForEstimate;
    QLabel *label_371;
    QLabel *label_377;
    QCheckBox *checkBox_OdomFovisUpdateTargetFeaturesWithRefined;
    QLabel *label_374;
    QDoubleSpinBox *doubleSpinBox_OdomFovisInlierMaxReprojectionError;
    QCheckBox *checkBox_OdomFovisUseSubpixelRefinement;
    QLabel *label_375;
    QDoubleSpinBox *doubleSpinBox_OdomFovisCliqueInlierThreshold;
    QDoubleSpinBox *doubleSpinBox_OdomFovisMaxMeanReprojectionError;
    QLabel *label_250;
    QLabel *label_376;
    QLabel *label_379;
    QGroupBox *groupBox_24;
    QGridLayout *gridLayout_38;
    QSpinBox *spinBox_OdomFovisFastThreshold;
    QCheckBox *checkBox_OdomFovisUseAdaptiveThreshold;
    QDoubleSpinBox *doubleSpinBox_OdomFovisFastThresholdAdaptiveGain;
    QLabel *label_382;
    QSpinBox *spinBox_OdomFovisMinPyramidLevel;
    QSpinBox *spinBox_OdomFovisMaxPyramidLevel;
    QLabel *label_373;
    QSpinBox *spinBox_OdomFovisTargetPixelsPerFeature;
    QLabel *label_381;
    QSpinBox *spinBox_OdomFovisFeatureWindowSize;
    QLabel *label_372;
    QLabel *label_378;
    QCheckBox *checkBox_OdomFovisUseHomographyInitialization;
    QLabel *label_383;
    QLabel *label_400;
    QLabel *label_401;
    QSpacerItem *verticalSpacer_60;
    QGroupBox *groupBox_25;
    QGridLayout *gridLayout_91;
    QCheckBox *checkBox_OdomFovisUseBucketing;
    QLabel *label_402;
    QSpinBox *spinBox_OdomFovisBucketHeight;
    QSpinBox *spinBox_OdomFovisMaxKeypointsPerBucket;
    QSpinBox *spinBox_OdomFovisBucketWidth;
    QCheckBox *checkBox_OdomFovisUseImageNormalization;
    QLabel *label_403;
    QLabel *label_404;
    QLabel *label_405;
    QLabel *label_406;
    QGroupBox *groupBox_26;
    QGridLayout *gridLayout_92;
    QDoubleSpinBox *doubleSpinBox_OdomFovisStereoMaxDistEpipolarLine;
    QDoubleSpinBox *doubleSpinBox_OdomFovisStereoMaxRefinementDisplacement;
    QCheckBox *checkBox_OdomFovisStereoRequireMutualMatch;
    QLabel *label_407;
    QSpinBox *spinBox_OdomFovisStereoMaxDisparity;
    QLabel *label_408;
    QLabel *label_409;
    QLabel *label_410;
    QWidget *page_68;
    QVBoxLayout *verticalLayout_121;
    QGroupBox *groupBox_odomViso2;
    QVBoxLayout *verticalLayout_120;
    QLabel *label_380;
    QGridLayout *gridLayout_86;
    QLabel *label_386;
    QLabel *label_384;
    QLabel *label_385;
    QSpinBox *spinBox_OdomViso2RansacIters;
    QDoubleSpinBox *doubleSpinBox_OdomViso2InlierThreshold;
    QCheckBox *checkBox_OdomViso2Reweighting;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_87;
    QLabel *label_389;
    QLabel *label_388;
    QLabel *label_391;
    QSpinBox *spinBox_OdomViso2MatchNmsN;
    QLabel *label_387;
    QLabel *label_392;
    QLabel *label_395;
    QLabel *label_394;
    QLabel *label_393;
    QLabel *label_390;
    QLabel *label_396;
    QSpinBox *spinBox_OdomViso2MatchNmsTau;
    QSpinBox *spinBox_OdomViso2MatchBinsize;
    QSpinBox *spinBox_OdomViso2MatchRadius;
    QSpinBox *spinBox_OdomViso2MatchDispTolerance;
    QSpinBox *spinBox_OdomViso2MatchOutlierDispTolerance;
    QSpinBox *spinBox_OdomViso2MatchOutlierFlowTolerance;
    QSpinBox *spinBox_OdomViso2MatchRefinement;
    QCheckBox *checkBox_OdomViso2MatchMultiStage;
    QCheckBox *checkBox_OdomViso2MatchHalfResolution;
    QGroupBox *groupBox_22;
    QGridLayout *gridLayout_89;
    QLabel *label_398;
    QLabel *label_397;
    QLabel *label_399;
    QDoubleSpinBox *doubleSpinBox_OdomViso2BucketWidth;
    QDoubleSpinBox *doubleSpinBox_OdomViso2BucketHeight;
    QSpinBox *spinBox_OdomViso2BucketMaxFeatures;
    QSpacerItem *verticalSpacer_61;
    QWidget *page_69;
    QVBoxLayout *verticalLayout_122;
    QGroupBox *groupBox_odomDVO;
    QVBoxLayout *verticalLayout_123;
    QLabel *label_411;
    QSpacerItem *verticalSpacer_63;
    QWidget *page_70;
    QVBoxLayout *verticalLayout_125;
    QGroupBox *groupBox_odomORBSLAM;
    QVBoxLayout *verticalLayout_124;
    QLabel *label_412;
    QGridLayout *gridLayout_93;
    QDoubleSpinBox *doubleSpinBox_OdomORBSLAMBf;
    QLabel *label_414;
    QDoubleSpinBox *doubleSpinBox_OdomORBSLAMThDepth;
    QLabel *label_413;
    QLineEdit *lineEdit_OdomORBSLAMVocPath;
    QLabel *label_415;
    QToolButton *toolButton_OdomORBSLAMVocPath;
    QLabel *label_444;
    QDoubleSpinBox *doubleSpinBox_OdomORBSLAMFps;
    QLabel *label_445;
    QSpinBox *spinBox_OdomORBSLAMMaxFeatures;
    QLabel *label_449;
    QSpinBox *spinBox_OdomORBSLAMMapSize;
    QSpacerItem *verticalSpacer_64;
    QWidget *page_54;
    QVBoxLayout *verticalLayout_85;
    QGroupBox *groupBox_odomOKVIS;
    QVBoxLayout *verticalLayout_128;
    QLabel *label_466;
    QGridLayout *gridLayout_98;
    QLineEdit *lineEdit_OdomOkvisPath;
    QLabel *label_469;
    QToolButton *toolButton_OdomOkvisPath;
    QSpacerItem *verticalSpacer_69;
    QWidget *page_73;
    QVBoxLayout *verticalLayout_130;
    QGroupBox *groupBox_odomLOAM;
    QVBoxLayout *verticalLayout_129;
    QLabel *label_472;
    QGridLayout *gridLayout_101;
    QLabel *label_475;
    QLabel *label_473;
    QLabel *label_474;
    QDoubleSpinBox *odom_loam_scan_period;
    QComboBox *odom_loam_sensor;
    QLabel *label_476;
    QDoubleSpinBox *odom_loam_linvar;
    QDoubleSpinBox *odom_loam_angvar;
    QCheckBox *odom_loam_localMapping;
    QLabel *label_477;
    QSpacerItem *verticalSpacer_73;
    QWidget *page_74;
    QVBoxLayout *verticalLayout_132;
    QGroupBox *groupBox_odomMSCKF;
    QVBoxLayout *verticalLayout_131;
    QLabel *label_480;
    QGroupBox *groupBox_7;
    QGridLayout *gridLayout_103;
    QSpinBox *OdomMSCKFGridRow;
    QLabel *label_538;
    QSpinBox *OdomMSCKFGridCol;
    QLabel *label_537;
    QSpinBox *OdomMSCKFGridMinFeatureNum;
    QLabel *label_536;
    QSpinBox *OdomMSCKFGridMaxFeatureNum;
    QLabel *label_535;
    QSpinBox *OdomMSCKFPyramidLevels;
    QLabel *label_534;
    QSpinBox *OdomMSCKFPatchSize;
    QLabel *label_533;
    QSpinBox *OdomMSCKFFastThreshold;
    QLabel *label_532;
    QSpinBox *OdomMSCKFMaxIteration;
    QLabel *label_531;
    QDoubleSpinBox *OdomMSCKFTrackPrecision;
    QLabel *label_495;
    QDoubleSpinBox *OdomMSCKFRansacThreshold;
    QLabel *label_496;
    QDoubleSpinBox *OdomMSCKFStereoThreshold;
    QLabel *label_497;
    QGroupBox *groupBox_28;
    QGridLayout *gridLayout_102;
    QSpinBox *OdomMSCKFMaxCamStateSize;
    QLabel *label_539;
    QDoubleSpinBox *OdomMSCKFPositionStdThreshold;
    QLabel *label_486;
    QDoubleSpinBox *OdomMSCKFRotationThreshold;
    QLabel *label_487;
    QDoubleSpinBox *OdomMSCKFTranslationThreshold;
    QLabel *label_488;
    QDoubleSpinBox *OdomMSCKFTrackingRateThreshold;
    QLabel *label_489;
    QDoubleSpinBox *OdomMSCKFOptTranslationThreshold;
    QLabel *label_490;
    QDoubleSpinBox *OdomMSCKFNoiseGyro;
    QLabel *label_491;
    QDoubleSpinBox *OdomMSCKFNoiseAcc;
    QLabel *label_492;
    QDoubleSpinBox *OdomMSCKFNoiseGyroBias;
    QLabel *label_493;
    QDoubleSpinBox *OdomMSCKFNoiseAccBias;
    QLabel *label_494;
    QDoubleSpinBox *OdomMSCKFNoiseFeature;
    QLabel *label_507;
    QDoubleSpinBox *OdomMSCKFInitCovVel;
    QLabel *label_508;
    QDoubleSpinBox *OdomMSCKFInitCovGyroBias;
    QLabel *label_509;
    QDoubleSpinBox *OdomMSCKFInitCovAccBias;
    QLabel *label_510;
    QDoubleSpinBox *OdomMSCKFInitCovExRot;
    QLabel *label_511;
    QDoubleSpinBox *OdomMSCKFInitCovExTrans;
    QLabel *label_513;
    QSpacerItem *verticalSpacer_74;
    QWidget *page_79;
    QVBoxLayout *verticalLayout_139;
    QGroupBox *groupBox_odomVINS;
    QVBoxLayout *verticalLayout_138;
    QLabel *label_525;
    QGridLayout *gridLayout_107;
    QLineEdit *lineEdit_OdomVinsPath;
    QLabel *label_526;
    QToolButton *toolButton_OdomVinsPath;
    QSpacerItem *verticalSpacer_77;
    QWidget *page_26;
    QVBoxLayout *verticalLayout_88;
    QGroupBox *groupBox_odomMono;
    QVBoxLayout *verticalLayout_168;
    QLabel *label_672;
    QSpacerItem *verticalSpacer_62;
    QLabel *label_661;
    QGridLayout *gridLayout_166;
    QDoubleSpinBox *doubleSpinBox_minFlow;
    QLabel *label_659;
    QDoubleSpinBox *doubleSpinBox_minTranslation;
    QLabel *label_660;
    QLabel *label_662;
    QDoubleSpinBox *doubleSpinBox_minInitTranslation;
    QLabel *label_663;
    QDoubleSpinBox *doubleSpinBox_maxVariance;
    QStackedWidget *stackedWidget_odometryFiltering;
    QWidget *page_31;
    QVBoxLayout *verticalLayout_84;
    QWidget *page_57;
    QVBoxLayout *verticalLayout_49;
    QGroupBox *groupBox_odometryKalmanFilter2;
    QVBoxLayout *verticalLayout_171;
    QLabel *label_673;
    QGridLayout *gridLayout_168;
    QDoubleSpinBox *doubleSpinBox_kalmanMeasurementNoise;
    QDoubleSpinBox *doubleSpinBox_kalmanProcessNoise;
    QLabel *label_674;
    QLabel *label_675;
    QSpacerItem *verticalSpacer_31;
    QWidget *page_56;
    QVBoxLayout *verticalLayout_87;
    QGroupBox *groupBox_odometryParticleFilter2;
    QVBoxLayout *verticalLayout_170;
    QLabel *label_665;
    QGridLayout *gridLayout_167;
    QDoubleSpinBox *doubleSpinBox_particleLambdaT;
    QDoubleSpinBox *doubleSpinBox_particleNoiseR;
    QDoubleSpinBox *doubleSpinBox_particleNoiseT;
    QLabel *label_666;
    QLabel *label_667;
    QLabel *label_668;
    QLabel *label_669;
    QDoubleSpinBox *doubleSpinBox_particleLambdaR;
    QLabel *label_670;
    QSpinBox *spinBox_particleSize;
    QLabel *label_162;
    QSpacerItem *verticalSpacer_14;
    QWidget *page_46;
    QVBoxLayout *verticalLayout_81;
    QGroupBox *groupBox_motionEst1;
    QGridLayout *gridLayout_53;
    QComboBox *comboBox_registrationStrategy;
    QLabel *label_11;
    QCheckBox *loopClosure_bowForce2D;
    QLabel *label_184;
    QLabel *label_370;
    QCheckBox *reg_repeatOnce;
    QSpacerItem *verticalSpacer_41;
    QWidget *page_21;
    QVBoxLayout *verticalLayout_50;
    QGroupBox *groupBox_visualTransform2;
    QVBoxLayout *verticalLayout_41;
    QGridLayout *gridLayout_23;
    QComboBox *loopClosure_estimationType;
    QSpinBox *loopClosure_bowIterations;
    QLabel *label_15;
    QLabel *label_298;
    QCheckBox *loopClosure_forwardEst;
    QSpinBox *loopClosure_bowMinInliers;
    QLabel *label_146;
    QLabel *label_2;
    QComboBox *loopClosure_bundle;
    QLabel *label_346;
    QLabel *label_554;
    QLabel *label_555;
    QDoubleSpinBox *visMinDistribution;
    QDoubleSpinBox *visMeanDistance;
    QGroupBox *groupBox_4;
    QVBoxLayout *verticalLayout_150;
    QGridLayout *gridLayout_30;
    QComboBox *reextract_nn;
    QLabel *label_179;
    QDoubleSpinBox *reextract_nndrRatio;
    QLabel *label_181;
    QSpinBox *spinBox_visCorGuessWinSize;
    QLabel *label_303;
    QCheckBox *checkBox__visCorGuessMatchToProjection;
    QLabel *label_448;
    QSpacerItem *verticalSpacer_44;
    QGroupBox *groupBox_pymatcher;
    QVBoxLayout *verticalLayout_149;
    QLabel *label_586;
    QGridLayout *gridLayout_116;
    QToolButton *toolButton_pymatcher_path;
    QLineEdit *lineEdit_pymatcher_path;
    QLabel *label_585;
    QDoubleSpinBox *pymatcher_matchThreshold;
    QLabel *label_584;
    QLabel *label_583;
    QSpinBox *pymatcher_iterations;
    QLabel *label_587;
    QCheckBox *checkBox_pymatcher_cuda;
    QLabel *label_590;
    QLineEdit *lineEdit_pymatcher_model;
    QToolButton *toolButton_pymatcher_model;
    QSpacerItem *verticalSpacer_86;
    QGroupBox *groupBox_gms;
    QVBoxLayout *verticalLayout_151;
    QLabel *label_588;
    QGridLayout *gridLayout_117;
    QLabel *label_592;
    QCheckBox *checkBox_gms_withRotation;
    QLabel *label_589;
    QDoubleSpinBox *gms_thresholdFactor;
    QLabel *label_593;
    QCheckBox *checkBox_gms_withScale;
    QSpacerItem *verticalSpacer_87;
    QStackedWidget *stackedWidget_loopClosureEstimation;
    QWidget *page_38;
    QVBoxLayout *verticalLayout_68;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout_6;
    QDoubleSpinBox *loopClosure_bowInlierDistance;
    QLabel *label_14;
    QSpinBox *loopClosure_bowRefineIterations;
    QLabel *label_38;
    QWidget *page_39;
    QVBoxLayout *verticalLayout_69;
    QGroupBox *groupBox_13;
    QGridLayout *gridLayout_59;
    QDoubleSpinBox *loopClosure_pnpReprojError;
    QLabel *label_236;
    QComboBox *loopClosure_pnpFlags;
    QLabel *label_235;
    QLabel *label_loopClosure_pnpOpenCV2;
    QSpinBox *loopClosure_pnpRefineIterations;
    QWidget *page_40;
    QVBoxLayout *verticalLayout_72;
    QGroupBox *groupBox_14;
    QVBoxLayout *verticalLayout_73;
    QLabel *label_671;
    QGridLayout *gridLayout_60;
    QDoubleSpinBox *loopClosure_bowEpipolarGeometryVar;
    QLabel *label_664;
    QSpacerItem *verticalSpacer_52;
    QGroupBox *groupBox_VisExtract;
    QVBoxLayout *verticalLayout_46;
    QLabel *label_182;
    QGridLayout *gridLayout_25;
    QLabel *label_237;
    QLabel *label_261;
    QLineEdit *loopClosure_roi;
    QSpinBox *reextract_maxFeatures;
    QLabel *label_180;
    QLabel *label_275;
    QComboBox *vis_feature_detector;
    QDoubleSpinBox *loopClosure_bowMaxDepth;
    QLabel *label_178;
    QDoubleSpinBox *loopClosure_bowMinDepth;
    QSpinBox *reextract_gridrows;
    QLabel *label_438;
    QLabel *label_439;
    QSpinBox *reextract_gridcols;
    QLabel *label_450;
    QCheckBox *checkBox_visDepthAsMask;
    QGroupBox *groupBox_11;
    QVBoxLayout *verticalLayout_59;
    QLabel *label_35;
    QGridLayout *gridLayout_36;
    QSpinBox *subpix_winSize;
    QLabel *label_200;
    QSpinBox *subpix_iterations;
    QLabel *label_203;
    QDoubleSpinBox *subpix_eps;
    QLabel *label_204;
    QSpacerItem *verticalSpacer_37;
    QSpacerItem *verticalSpacer_66;
    QSpacerItem *verticalSpacer_20;
    QWidget *page_27;
    QVBoxLayout *verticalLayout_52;
    QGroupBox *groupBox_icp2;
    QVBoxLayout *verticalLayout_20;
    QLabel *label_7;
    QGridLayout *gridLayout_48;
    QLabel *label_139;
    QDoubleSpinBox *loopClosure_icpRangeMax;
    QSpinBox *loopClosure_icpDownsamplingStep;
    QLabel *label_55;
    QDoubleSpinBox *loopClosure_icpRatio;
    QLabel *label_136;
    QSpinBox *loopClosure_icpIterations;
    QDoubleSpinBox *loopClosure_icpRangeMin;
    QComboBox *comboBox_icpStrategy;
    QDoubleSpinBox *loopClosure_icpMaxCorrespondenceDistance;
    QCheckBox *loopClosure_icpPointToPlane;
    QCheckBox *loopClosure_icpForce4DoF;
    QComboBox *loopClosure_icpPointToPlaneLowComplexityStrategy;
    QLabel *label_51;
    QLabel *label_559;
    QDoubleSpinBox *loopClosure_icpVoxelSize;
    QLabel *label_426;
    QLabel *label_133;
    QDoubleSpinBox *loopClosure_icpPointToPlaneNormalsRadius;
    QLabel *label_124;
    QLabel *label_212;
    QLabel *label_622;
    QLabel *label_122;
    QLabel *label_144;
    QLabel *label_608;
    QDoubleSpinBox *globalDetection_icpMaxRotation;
    QSpinBox *loopClosure_icpPointToPlaneNormals;
    QDoubleSpinBox *globalDetection_icpMaxTranslation;
    QDoubleSpinBox *loopClosure_icpEpsilon;
    QLabel *label_617;
    QLabel *label_125;
    QLabel *label_607;
    QLabel *label_560;
    QLabel *label_429;
    QDoubleSpinBox *loopClosure_icpPointToPlaneGroundNormalsUp;
    QDoubleSpinBox *loopClosure_icpPointToPlaneNormalsMinComplexity;
    QLabel *label_446;
    QDoubleSpinBox *doubleSpinBox_icpOutlierRatio;
    QStackedWidget *stackedWidget_icpStrategy;
    QWidget *page_90;
    QVBoxLayout *verticalLayout_156;
    QGroupBox *groupBox_33;
    QVBoxLayout *verticalLayout_160;
    QLabel *label_628;
    QSpacerItem *verticalSpacer_82;
    QWidget *page_91;
    QVBoxLayout *verticalLayout_157;
    QGroupBox *groupBox_libpointmatcher;
    QVBoxLayout *verticalLayout_126;
    QLabel *label_422;
    QHBoxLayout *horizontalLayout_11;
    QToolButton *toolButton_IcpConfigPath;
    QLineEdit *lineEdit_IcpPMConfigPath;
    QLabel *label_423;
    QGridLayout *gridLayout_97;
    QSpinBox *spinBox_icpPMMatcherKnn;
    QLabel *label_424;
    QCheckBox *loopClosure_icpPMMatcherIntensity;
    QDoubleSpinBox *doubleSpinBox_icpPMMatcherEpsilon;
    QLabel *label_447;
    QLabel *label_609;
    QWidget *page_92;
    QVBoxLayout *verticalLayout_158;
    QGroupBox *groupBox_32;
    QVBoxLayout *verticalLayout_159;
    QLabel *label_623;
    QGridLayout *gridLayout_121;
    QDoubleSpinBox *doubleSpinBox_icpCCMaxFinalRMS;
    QLabel *label_630;
    QSpinBox *spinBox_icpCCSamplingLimit;
    QLabel *label_631;
    QLabel *label_629;
    QCheckBox *checkBox_icpCCFilterOutFarthestPoints;
    QSpacerItem *verticalSpacer_75;
    QSpacerItem *verticalSpacer_21;
    QWidget *page_65;
    QVBoxLayout *verticalLayout_114;
    QGroupBox *groupBox_optimizer1;
    QVBoxLayout *verticalLayout_113;
    QGridLayout *gridLayout_80;
    QSpinBox *graphOptimization_iterations;
    QDoubleSpinBox *graphOptimization_stopEpsilon;
    QLabel *label_141;
    QLabel *label_64;
    QCheckBox *graphOptimization_covarianceIgnored;
    QLabel *label_31;
    QGroupBox *groupBoxx_g2o;
    QVBoxLayout *verticalLayout_117;
    QGridLayout *gridLayout_70;
    QComboBox *comboBox_g2o_solver;
    QLabel *label_107;
    QComboBox *comboBox_g2o_optimizer;
    QLabel *label_138;
    QGroupBox *groupBox_20;
    QGridLayout *gridLayout_84;
    QDoubleSpinBox *doubleSpinBox_g2o_pixelVariance;
    QLabel *label_148;
    QDoubleSpinBox *doubleSpinBox_g2o_robustKernelDelta;
    QLabel *label_350;
    QDoubleSpinBox *doubleSpinBox_g2o_baseline;
    QLabel *label_352;
    QGroupBox *groupBox_19;
    QGridLayout *gridLayout_83;
    QComboBox *comboBox_gtsam_optimizer;
    QLabel *label_364;
    QSpacerItem *verticalSpacer_58;
    QWidget *page_29;
    QVBoxLayout *verticalLayout_57;
    QGroupBox *groupBox_stereo1;
    QVBoxLayout *verticalLayout_56;
    QLabel *label_187;
    QGroupBox *groupBox_10;
    QGridLayout *gridLayout_33;
    QLabel *label_284;
    QLabel *label_285;
    QSpinBox *stereo_winWidth;
    QLabel *label_205;
    QLabel *label_206;
    QCheckBox *stereo_ssd;
    QSpinBox *stereo_winHeight;
    QSpinBox *stereo_iterations;
    QDoubleSpinBox *stereo_flow_eps;
    QLabel *label_207;
    QLabel *label_208;
    QSpinBox *stereo_maxLevel;
    QLabel *label_287;
    QLabel *label_214;
    QLabel *label_286;
    QCheckBox *stereo_opticalFlow;
    QDoubleSpinBox *stereo_minDisparity;
    QDoubleSpinBox *stereo_maxDisparity;
    QSpacerItem *verticalSpacer_29;
    QWidget *page_51;
    QVBoxLayout *verticalLayout_83;
    QGroupBox *groupBox_stereoDense2;
    QVBoxLayout *verticalLayout_82;
    QHBoxLayout *horizontalLayout_13;
    QComboBox *comboBox_stereoDense_strategy;
    QLabel *label_515;
    QStackedWidget *stackedWidget_stereoDense;
    QWidget *page_75;
    QVBoxLayout *verticalLayout_133;
    QLabel *label_197;
    QGroupBox *groupBox_16;
    QGridLayout *gridLayout_34;
    QSpinBox *stereobm_uniquessRatio;
    QSpinBox *stereobm_numDisparities;
    QSpinBox *stereobm_preFilterCap;
    QLabel *label_278;
    QLabel *label_276;
    QLabel *label_281;
    QLabel *label_279;
    QSpinBox *stereobm_speckleWinSize;
    QLabel *label_277;
    QLabel *label_216;
    QSpinBox *stereobm_minDisparity;
    QSpinBox *stereobm_blockSize;
    QSpinBox *stereobm_preFilterSize;
    QLabel *label_280;
    QSpinBox *stereobm_tetureThreshold;
    QSpinBox *stereobm_speckleRange;
    QLabel *label_282;
    QLabel *label_283;
    QLabel *label_501;
    QSpinBox *stereobm_disp12MaxDiff;
    QWidget *page_76;
    QVBoxLayout *verticalLayout_134;
    QLabel *label_514;
    QGroupBox *groupBox_30;
    QGridLayout *gridLayout_105;
    QLabel *label_517;
    QSpinBox *stereosgbm_disp12MaxDiff;
    QLabel *label_516;
    QSpinBox *stereosgbm_numDisparities;
    QSpinBox *stereosgbm_uniquessRatio;
    QLabel *label_499;
    QLabel *label_500;
    QSpinBox *stereosgbm_speckleWinSize;
    QLabel *label_504;
    QLabel *label_503;
    QSpinBox *stereosgbm_minDisparity;
    QSpinBox *stereosgbm_preFilterCap;
    QSpinBox *stereosgbm_blockSize;
    QLabel *label_505;
    QLabel *label_506;
    QLabel *label_512;
    QSpinBox *stereosgbm_speckleRange;
    QSpinBox *stereosgbm_p2;
    QLabel *label_518;
    QSpinBox *stereosgbm_p1;
    QLabel *label_519;
    QComboBox *stereosgbm_mode;
    QSpacerItem *verticalSpacer_53;
    QWidget *page_45;
    QVBoxLayout *verticalLayout_74;
    QGroupBox *groupBox_feature1;
    QVBoxLayout *verticalLayout_75;
    QLabel *label_10;
    QSpacerItem *verticalSpacer_36;
    QWidget *page_13;
    QVBoxLayout *verticalLayout_38;
    QGroupBox *groupBox_detector_surf2;
    QGridLayout *gridLayout_15;
    QCheckBox *surf_checkBox_upright;
    QLabel *label_41;
    QLabel *label_8;
    QLabel *label_surf_checkBox_gpuKeypointsRatio;
    QLabel *label_surf_checkBox_gpuVersion;
    QSpinBox *surf_spinBox_octaves;
    QCheckBox *surf_checkBox_gpuVersion;
    QLabel *label_49;
    QDoubleSpinBox *surf_doubleSpinBox_gpuKeypointsRatio;
    QDoubleSpinBox *surf_doubleSpinBox_hessianThr;
    QCheckBox *checkBox_surfExtended;
    QSpinBox *surf_spinBox_octaveLayers;
    QLabel *label_26;
    QLabel *label_4;
    QSpacerItem *verticalSpacer_17;
    QWidget *page_14;
    QVBoxLayout *verticalLayout_39;
    QGroupBox *groupBox_detector_sift2;
    QGridLayout *gridLayout_19;
    QSpinBox *sift_spinBox_nFeatures;
    QLabel *label_129;
    QDoubleSpinBox *sift_doubleSpinBox_contrastThr;
    QLabel *label_5;
    QLabel *label_30;
    QDoubleSpinBox *sift_doubleSpinBox_edgeThr;
    QDoubleSpinBox *sift_doubleSpinBox_sigma;
    QSpinBox *sift_spinBox_nOctaveLayers;
    QLabel *label_130;
    QLabel *label_131;
    QLabel *label_591;
    QCheckBox *sift_checkBox_rootsift;
    QSpacerItem *verticalSpacer_19;
    QWidget *page_17;
    QVBoxLayout *verticalLayout_14;
    QGroupBox *groupBox_detector_fast2;
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout_94;
    QSpinBox *fastThreshold;
    QLabel *label_56;
    QSpinBox *fastThresholdMin;
    QLabel *label_111;
    QCheckBox *fastSuppressNonMax;
    QLabel *label_69;
    QLabel *label_561;
    QSpinBox *fastCV;
    QGroupBox *groupBox_fast_opencv2;
    QGridLayout *gridLayout_71;
    QSpinBox *fastThresholdMax;
    QLabel *label_115;
    QSpinBox *fastGridRows;
    QLabel *label_104;
    QSpinBox *fastGridCols;
    QLabel *label_109;
    QCheckBox *fastGpu;
    QLabel *label_fastGPU;
    QDoubleSpinBox *fastKeypointRatio;
    QLabel *label_fastGPUKptRatio;
    QSpacerItem *verticalSpacer_16;
    QWidget *page_15;
    QVBoxLayout *verticalLayout_40;
    QGroupBox *groupBox_detector_brief2;
    QGridLayout *gridLayout_16;
    QSpinBox *briefBytes;
    QLabel *label_52;
    QSpacerItem *verticalSpacer_22;
    QWidget *page_18;
    QVBoxLayout *verticalLayout_35;
    QGroupBox *groupBox_detector_orb2;
    QGridLayout *gridLayout_14;
    QDoubleSpinBox *doubleSpinBox_ORBScaleFactor;
    QLabel *label_32;
    QSpinBox *spinBox_ORBNLevels;
    QLabel *label_43;
    QSpinBox *spinBox_ORBEdgeThreshold;
    QLabel *label_95;
    QSpinBox *spinBox_ORBFirstLevel;
    QLabel *label_127;
    QSpinBox *spinBox_ORBWTA_K;
    QLabel *label_128;
    QSpinBox *spinBox_ORBScoreType;
    QLabel *label_140;
    QSpinBox *spinBox_ORBPatchSize;
    QLabel *label_147;
    QCheckBox *checkBox_ORBGpu;
    QLabel *label_orbGpu;
    QSpacerItem *verticalSpacer_18;
    QWidget *page_11;
    QVBoxLayout *verticalLayout_36;
    QGroupBox *groupBox_detector_freak2;
    QGridLayout *gridLayout_13;
    QCheckBox *checkBox_FREAKOrientationNormalized;
    QLabel *label_6;
    QCheckBox *checkBox_FREAKScaleNormalized;
    QLabel *label_9;
    QDoubleSpinBox *doubleSpinBox_FREAKPatternScale;
    QLabel *label_13;
    QSpinBox *spinBox_FREAKNOctaves;
    QLabel *label_23;
    QSpacerItem *verticalSpacer_13;
    QWidget *page_16;
    QVBoxLayout *verticalLayout_9;
    QGroupBox *groupBox_detector_gftt2;
    QGridLayout *gridLayout_21;
    QDoubleSpinBox *doubleSpinBox_GFTT_qualityLevel;
    QDoubleSpinBox *doubleSpinBox_GFTT_minDistance;
    QCheckBox *checkBox_GFTT_useHarrisDetector;
    QLabel *label_172;
    QDoubleSpinBox *doubleSpinBox_GFTT_k;
    QLabel *label_173;
    QSpinBox *spinBox_GFTT_blockSize;
    QLabel *label_175;
    QLabel *label_176;
    QLabel *label_177;
    QSpacerItem *verticalSpacer_25;
    QWidget *page_25;
    QVBoxLayout *verticalLayout_47;
    QGroupBox *groupBox_detector_brisk2;
    QGridLayout *gridLayout_26;
    QLabel *label_191;
    QDoubleSpinBox *doubleSpinBox_BRISK_patterScale;
    QLabel *label_188;
    QSpinBox *spinBox_BRISK_octaves;
    QLabel *label_189;
    QSpinBox *spinBox_BRISK_thresh;
    QSpacerItem *verticalSpacer_26;
    QWidget *page_71;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *groupBox_detector_kaze2;
    QGridLayout *gridLayout_95;
    QCheckBox *checkBox_kaze_extended;
    QLabel *label_420;
    QLabel *label_416;
    QCheckBox *checkBox_kaze_upright;
    QLabel *label_417;
    QSpinBox *spinBox_kaze_octavelayers;
    QSpinBox *spinBox_kaze_octaves;
    QLabel *label_419;
    QDoubleSpinBox *doubleSpinBox_kaze_threshold;
    QLabel *label_418;
    QLabel *label_421;
    QSpinBox *spinBox_kaze_diffusivity;
    QSpacerItem *verticalSpacer_65;
    QWidget *page_85;
    QVBoxLayout *verticalLayout_146;
    QGroupBox *groupBox_detector_superpoint_torch2;
    QVBoxLayout *verticalLayout_147;
    QLabel *label_575;
    QGridLayout *gridLayout_114;
    QToolButton *toolButton_sptorch_path;
    QLineEdit *lineEdit_sptorch_path;
    QLabel *label_579;
    QDoubleSpinBox *doubleSpinBox_sptorch_threshold;
    QLabel *label_578;
    QCheckBox *checkBox_sptorch_nms;
    QLabel *label_577;
    QCheckBox *checkBox_sptorch_cuda;
    QLabel *label_574;
    QLabel *label_580;
    QSpinBox *spinBox_sptorch_minDistance;
    QSpacerItem *verticalSpacer_84;
    QWidget *page_87;
    QVBoxLayout *verticalLayout_153;
    QGroupBox *groupBox_pydetector2;
    QVBoxLayout *verticalLayout_152;
    QLabel *label_614;
    QGridLayout *gridLayout_118;
    QLabel *label_615;
    QLineEdit *lineEdit_pydetector_path;
    QToolButton *toolButton_pydetector_path;
    QCheckBox *checkBox_pydetector_cuda;
    QLabel *label_618;
    QSpacerItem *verticalSpacer_88;
    QHBoxLayout *horizontalLayout;
    QDialogButtonBox *buttonBox_local;
    QHBoxLayout *horizontalLayout_3;
    QRadioButton *radioButton_basic;
    QRadioButton *radioButton_advanced;
    QDialogButtonBox *buttonBox_global;

    void setupUi(QDialog *preferencesDialog)
    {
        if (preferencesDialog->objectName().isEmpty())
            preferencesDialog->setObjectName(QStringLiteral("preferencesDialog"));
        preferencesDialog->resize(982, 820);
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(preferencesDialog->sizePolicy().hasHeightForWidth());
        preferencesDialog->setSizePolicy(sizePolicy);
        QIcon icon;
        icon.addFile(QStringLiteral(":/images/RTAB-Map.ico"), QSize(), QIcon::Normal, QIcon::Off);
        preferencesDialog->setWindowIcon(icon);
        gridLayout_49 = new QGridLayout(preferencesDialog);
        gridLayout_49->setObjectName(QStringLiteral("gridLayout_49"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(0);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        verticalLayout_18 = new QVBoxLayout();
        verticalLayout_18->setObjectName(QStringLiteral("verticalLayout_18"));
        treeView = new QTreeView(preferencesDialog);
        treeView->setObjectName(QStringLiteral("treeView"));
        treeView->setMinimumSize(QSize(160, 0));
        treeView->setEditTriggers(QAbstractItemView::NoEditTriggers);
        treeView->header()->setVisible(false);

        verticalLayout_18->addWidget(treeView);


        horizontalLayout_2->addLayout(verticalLayout_18);

        verticalLayout_17 = new QVBoxLayout();
        verticalLayout_17->setSpacing(0);
        verticalLayout_17->setObjectName(QStringLiteral("verticalLayout_17"));
        scrollArea = new QScrollArea(preferencesDialog);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, -1011, 686, 3414));
        verticalLayout_16 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_16->setSpacing(0);
        verticalLayout_16->setObjectName(QStringLiteral("verticalLayout_16"));
        verticalLayout_16->setContentsMargins(0, 0, 0, 0);
        verticalLayout_24 = new QVBoxLayout();
        verticalLayout_24->setSpacing(0);
        verticalLayout_24->setObjectName(QStringLiteral("verticalLayout_24"));
        stackedWidget = new QStackedWidget(scrollAreaWidgetContents);
        stackedWidget->setObjectName(QStringLiteral("stackedWidget"));
        stackedWidget->setFrameShadow(QFrame::Raised);
        page_22 = new QWidget();
        page_22->setObjectName(QStringLiteral("page_22"));
        verticalLayout_29 = new QVBoxLayout(page_22);
        verticalLayout_29->setObjectName(QStringLiteral("verticalLayout_29"));
        groupBox_generalSettingsGui0 = new QGroupBox(page_22);
        groupBox_generalSettingsGui0->setObjectName(QStringLiteral("groupBox_generalSettingsGui0"));
        verticalLayout_28 = new QVBoxLayout(groupBox_generalSettingsGui0);
        verticalLayout_28->setObjectName(QStringLiteral("verticalLayout_28"));
        gridLayout_39 = new QGridLayout();
        gridLayout_39->setObjectName(QStringLiteral("gridLayout_39"));
        checkBox_beep = new QCheckBox(groupBox_generalSettingsGui0);
        checkBox_beep->setObjectName(QStringLiteral("checkBox_beep"));

        gridLayout_39->addWidget(checkBox_beep, 2, 0, 1, 1);

        checkBox_notifyWhenNewGlobalPathIsReceived = new QCheckBox(groupBox_generalSettingsGui0);
        checkBox_notifyWhenNewGlobalPathIsReceived->setObjectName(QStringLiteral("checkBox_notifyWhenNewGlobalPathIsReceived"));
        checkBox_notifyWhenNewGlobalPathIsReceived->setChecked(true);

        gridLayout_39->addWidget(checkBox_notifyWhenNewGlobalPathIsReceived, 5, 0, 1, 1);

        label_247 = new QLabel(groupBox_generalSettingsGui0);
        label_247->setObjectName(QStringLiteral("label_247"));
        label_247->setWordWrap(true);
        label_247->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_39->addWidget(label_247, 3, 1, 1, 1);

        label_313 = new QLabel(groupBox_generalSettingsGui0);
        label_313->setObjectName(QStringLiteral("label_313"));
        label_313->setWordWrap(true);
        label_313->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_39->addWidget(label_313, 1, 1, 1, 1);

        label_102 = new QLabel(groupBox_generalSettingsGui0);
        label_102->setObjectName(QStringLiteral("label_102"));
        label_102->setWordWrap(true);
        label_102->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_39->addWidget(label_102, 5, 1, 1, 1);

        general_checkBox_imagesKept = new QCheckBox(groupBox_generalSettingsGui0);
        general_checkBox_imagesKept->setObjectName(QStringLiteral("general_checkBox_imagesKept"));
        general_checkBox_imagesKept->setChecked(true);

        gridLayout_39->addWidget(general_checkBox_imagesKept, 0, 0, 1, 1);

        general_checkBox_cloudsKept = new QCheckBox(groupBox_generalSettingsGui0);
        general_checkBox_cloudsKept->setObjectName(QStringLiteral("general_checkBox_cloudsKept"));
        general_checkBox_cloudsKept->setChecked(true);

        gridLayout_39->addWidget(general_checkBox_cloudsKept, 1, 0, 1, 1);

        checkBox_stamps = new QCheckBox(groupBox_generalSettingsGui0);
        checkBox_stamps->setObjectName(QStringLiteral("checkBox_stamps"));

        gridLayout_39->addWidget(checkBox_stamps, 3, 0, 1, 1);

        label_342 = new QLabel(groupBox_generalSettingsGui0);
        label_342->setObjectName(QStringLiteral("label_342"));
        label_342->setWordWrap(true);
        label_342->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_39->addWidget(label_342, 4, 1, 1, 1);

        label_88 = new QLabel(groupBox_generalSettingsGui0);
        label_88->setObjectName(QStringLiteral("label_88"));
        label_88->setWordWrap(true);
        label_88->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_39->addWidget(label_88, 2, 1, 1, 1);

        label_59 = new QLabel(groupBox_generalSettingsGui0);
        label_59->setObjectName(QStringLiteral("label_59"));
        label_59->setWordWrap(true);
        label_59->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_39->addWidget(label_59, 0, 1, 1, 1);

        checkbox_odomDisabled = new QCheckBox(groupBox_generalSettingsGui0);
        checkbox_odomDisabled->setObjectName(QStringLiteral("checkbox_odomDisabled"));

        gridLayout_39->addWidget(checkbox_odomDisabled, 6, 0, 1, 1);

        label_239 = new QLabel(groupBox_generalSettingsGui0);
        label_239->setObjectName(QStringLiteral("label_239"));
        label_239->setWordWrap(true);
        label_239->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_39->addWidget(label_239, 6, 1, 1, 1);

        checkBox_cacheStatistics = new QCheckBox(groupBox_generalSettingsGui0);
        checkBox_cacheStatistics->setObjectName(QStringLiteral("checkBox_cacheStatistics"));

        gridLayout_39->addWidget(checkBox_cacheStatistics, 4, 0, 1, 1);

        label_347 = new QLabel(groupBox_generalSettingsGui0);
        label_347->setObjectName(QStringLiteral("label_347"));
        label_347->setWordWrap(true);
        label_347->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_39->addWidget(label_347, 7, 1, 1, 1);

        checkbox_groundTruthAlign = new QCheckBox(groupBox_generalSettingsGui0);
        checkbox_groundTruthAlign->setObjectName(QStringLiteral("checkbox_groundTruthAlign"));

        gridLayout_39->addWidget(checkbox_groundTruthAlign, 7, 0, 1, 1);

        gridLayout_39->setColumnStretch(1, 1);

        verticalLayout_28->addLayout(gridLayout_39);

        groupBox_2 = new QGroupBox(groupBox_generalSettingsGui0);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        horizontalLayout_9 = new QHBoxLayout(groupBox_2);
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        pushButton_loadConfig = new QPushButton(groupBox_2);
        pushButton_loadConfig->setObjectName(QStringLiteral("pushButton_loadConfig"));

        horizontalLayout_9->addWidget(pushButton_loadConfig);

        pushButton_saveConfig = new QPushButton(groupBox_2);
        pushButton_saveConfig->setObjectName(QStringLiteral("pushButton_saveConfig"));

        horizontalLayout_9->addWidget(pushButton_saveConfig);

        pushButton_resetConfig = new QPushButton(groupBox_2);
        pushButton_resetConfig->setObjectName(QStringLiteral("pushButton_resetConfig"));

        horizontalLayout_9->addWidget(pushButton_resetConfig);

        horizontalSpacer_4 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_9->addItem(horizontalSpacer_4);


        verticalLayout_28->addWidget(groupBox_2);

        groupBox_5 = new QGroupBox(groupBox_generalSettingsGui0);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));
        gridLayout_31 = new QGridLayout(groupBox_5);
        gridLayout_31->setObjectName(QStringLiteral("gridLayout_31"));
        checkBox_imageRejectedShown = new QCheckBox(groupBox_5);
        checkBox_imageRejectedShown->setObjectName(QStringLiteral("checkBox_imageRejectedShown"));
        checkBox_imageRejectedShown->setChecked(true);

        gridLayout_31->addWidget(checkBox_imageRejectedShown, 1, 0, 1, 1);

        label_105 = new QLabel(groupBox_5);
        label_105->setObjectName(QStringLiteral("label_105"));
        label_105->setWordWrap(true);
        label_105->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_31->addWidget(label_105, 2, 1, 1, 1);

        checkBox_imageHighestHypShown = new QCheckBox(groupBox_5);
        checkBox_imageHighestHypShown->setObjectName(QStringLiteral("checkBox_imageHighestHypShown"));
        checkBox_imageHighestHypShown->setChecked(false);

        gridLayout_31->addWidget(checkBox_imageHighestHypShown, 2, 0, 1, 1);

        label_73 = new QLabel(groupBox_5);
        label_73->setObjectName(QStringLiteral("label_73"));
        label_73->setWordWrap(true);
        label_73->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_31->addWidget(label_73, 1, 1, 1, 1);

        label_70 = new QLabel(groupBox_5);
        label_70->setObjectName(QStringLiteral("label_70"));
        label_70->setWordWrap(false);
        label_70->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_31->addWidget(label_70, 0, 1, 1, 1);

        checkBox_verticalLayoutUsed = new QCheckBox(groupBox_5);
        checkBox_verticalLayoutUsed->setObjectName(QStringLiteral("checkBox_verticalLayoutUsed"));
        checkBox_verticalLayoutUsed->setChecked(true);

        gridLayout_31->addWidget(checkBox_verticalLayoutUsed, 0, 0, 1, 1);

        gridLayout_31->setColumnStretch(1, 1);

        verticalLayout_28->addWidget(groupBox_5);

        groupBox_29 = new QGroupBox(groupBox_generalSettingsGui0);
        groupBox_29->setObjectName(QStringLiteral("groupBox_29"));
        gridLayout_104 = new QGridLayout(groupBox_29);
        gridLayout_104->setObjectName(QStringLiteral("gridLayout_104"));
        label_222 = new QLabel(groupBox_29);
        label_222->setObjectName(QStringLiteral("label_222"));
        label_222->setWordWrap(true);
        label_222->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_104->addWidget(label_222, 0, 1, 1, 1);

        radioButton_wordsGraphView = new QRadioButton(groupBox_29);
        radioButton_wordsGraphView->setObjectName(QStringLiteral("radioButton_wordsGraphView"));
        radioButton_wordsGraphView->setChecked(false);

        gridLayout_104->addWidget(radioButton_wordsGraphView, 1, 0, 1, 1);

        radioButton_posteriorGraphView = new QRadioButton(groupBox_29);
        radioButton_posteriorGraphView->setObjectName(QStringLiteral("radioButton_posteriorGraphView"));
        radioButton_posteriorGraphView->setChecked(true);

        gridLayout_104->addWidget(radioButton_posteriorGraphView, 0, 0, 1, 1);

        label_478 = new QLabel(groupBox_29);
        label_478->setObjectName(QStringLiteral("label_478"));
        label_478->setWordWrap(true);
        label_478->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_104->addWidget(label_478, 1, 1, 1, 1);

        radioButton_nochangeGraphView = new QRadioButton(groupBox_29);
        radioButton_nochangeGraphView->setObjectName(QStringLiteral("radioButton_nochangeGraphView"));

        gridLayout_104->addWidget(radioButton_nochangeGraphView, 3, 0, 1, 1);

        label_479 = new QLabel(groupBox_29);
        label_479->setObjectName(QStringLiteral("label_479"));
        label_479->setWordWrap(true);
        label_479->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_104->addWidget(label_479, 3, 1, 1, 1);

        label_481 = new QLabel(groupBox_29);
        label_481->setObjectName(QStringLiteral("label_481"));
        label_481->setWordWrap(true);
        label_481->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_104->addWidget(label_481, 2, 1, 1, 1);

        radioButton_localizationsGraphView = new QRadioButton(groupBox_29);
        radioButton_localizationsGraphView->setObjectName(QStringLiteral("radioButton_localizationsGraphView"));
        radioButton_localizationsGraphView->setChecked(false);

        gridLayout_104->addWidget(radioButton_localizationsGraphView, 2, 0, 1, 1);

        gridLayout_104->setColumnStretch(1, 1);
        radioButton_posteriorGraphView->raise();
        radioButton_wordsGraphView->raise();
        label_478->raise();
        label_222->raise();
        label_479->raise();
        radioButton_nochangeGraphView->raise();
        label_481->raise();
        radioButton_localizationsGraphView->raise();

        verticalLayout_28->addWidget(groupBox_29);

        groupBox_21 = new QGroupBox(groupBox_generalSettingsGui0);
        groupBox_21->setObjectName(QStringLiteral("groupBox_21"));
        gridLayout_85 = new QGridLayout(groupBox_21);
        gridLayout_85->setObjectName(QStringLiteral("gridLayout_85"));
        label_369 = new QLabel(groupBox_21);
        label_369->setObjectName(QStringLiteral("label_369"));
        label_369->setWordWrap(true);
        label_369->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_85->addWidget(label_369, 0, 1, 1, 1);

        checkBox_odom_onlyInliersShown = new QCheckBox(groupBox_21);
        checkBox_odom_onlyInliersShown->setObjectName(QStringLiteral("checkBox_odom_onlyInliersShown"));
        checkBox_odom_onlyInliersShown->setChecked(false);

        gridLayout_85->addWidget(checkBox_odom_onlyInliersShown, 0, 0, 1, 1);

        label_159 = new QLabel(groupBox_21);
        label_159->setObjectName(QStringLiteral("label_159"));
        label_159->setWordWrap(true);
        label_159->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_85->addWidget(label_159, 1, 1, 1, 1);

        spinBox_odomQualityWarnThr = new QSpinBox(groupBox_21);
        spinBox_odomQualityWarnThr->setObjectName(QStringLiteral("spinBox_odomQualityWarnThr"));
        spinBox_odomQualityWarnThr->setMaximum(9999);
        spinBox_odomQualityWarnThr->setValue(50);

        gridLayout_85->addWidget(spinBox_odomQualityWarnThr, 1, 0, 1, 1);

        gridLayout_85->setColumnStretch(1, 1);

        verticalLayout_28->addWidget(groupBox_21);


        verticalLayout_29->addWidget(groupBox_generalSettingsGui0);

        verticalSpacer_24 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_29->addItem(verticalSpacer_24);

        verticalLayout_29->setStretch(1, 1);
        stackedWidget->addWidget(page_22);
        page_2 = new QWidget();
        page_2->setObjectName(QStringLiteral("page_2"));
        verticalLayout_42 = new QVBoxLayout(page_2);
        verticalLayout_42->setObjectName(QStringLiteral("verticalLayout_42"));
        groupBox_cloudRendering1 = new QGroupBox(page_2);
        groupBox_cloudRendering1->setObjectName(QStringLiteral("groupBox_cloudRendering1"));
        verticalLayout_98 = new QVBoxLayout(groupBox_cloudRendering1);
        verticalLayout_98->setObjectName(QStringLiteral("verticalLayout_98"));
        groupBox_12 = new QGroupBox(groupBox_cloudRendering1);
        groupBox_12->setObjectName(QStringLiteral("groupBox_12"));
        verticalLayout_112 = new QVBoxLayout(groupBox_12);
        verticalLayout_112->setObjectName(QStringLiteral("verticalLayout_112"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        lineEdit_roiRatios = new QLineEdit(groupBox_12);
        lineEdit_roiRatios->setObjectName(QStringLiteral("lineEdit_roiRatios"));

        gridLayout_2->addWidget(lineEdit_roiRatios, 5, 0, 1, 1);

        spinBox_ptsize = new QSpinBox(groupBox_12);
        spinBox_ptsize->setObjectName(QStringLiteral("spinBox_ptsize"));
        spinBox_ptsize->setMinimum(1);
        spinBox_ptsize->setMaximum(64);
        spinBox_ptsize->setValue(2);

        gridLayout_2->addWidget(spinBox_ptsize, 15, 0, 1, 1);

        label_355 = new QLabel(groupBox_12);
        label_355->setObjectName(QStringLiteral("label_355"));
        label_355->setWordWrap(true);
        label_355->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_355, 10, 2, 1, 1);

        label_169 = new QLabel(groupBox_12);
        label_169->setObjectName(QStringLiteral("label_169"));
        label_169->setWordWrap(true);
        label_169->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_169, 8, 2, 1, 1);

        doubleSpinBox_normalRadiusSearch = new QDoubleSpinBox(groupBox_12);
        doubleSpinBox_normalRadiusSearch->setObjectName(QStringLiteral("doubleSpinBox_normalRadiusSearch"));
        doubleSpinBox_normalRadiusSearch->setSingleStep(0.01);

        gridLayout_2->addWidget(doubleSpinBox_normalRadiusSearch, 12, 0, 1, 1);

        label_459 = new QLabel(groupBox_12);
        label_459->setObjectName(QStringLiteral("label_459"));
        label_459->setWordWrap(true);
        label_459->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_459, 13, 2, 1, 1);

        spinBox_colorScheme_odom = new QSpinBox(groupBox_12);
        spinBox_colorScheme_odom->setObjectName(QStringLiteral("spinBox_colorScheme_odom"));

        gridLayout_2->addWidget(spinBox_colorScheme_odom, 13, 1, 1, 1);

        spinBox_colorScheme = new QSpinBox(groupBox_12);
        spinBox_colorScheme->setObjectName(QStringLiteral("spinBox_colorScheme"));

        gridLayout_2->addWidget(spinBox_colorScheme, 13, 0, 1, 1);

        doubleSpinBox_maxDepth_odom = new QDoubleSpinBox(groupBox_12);
        doubleSpinBox_maxDepth_odom->setObjectName(QStringLiteral("doubleSpinBox_maxDepth_odom"));
        doubleSpinBox_maxDepth_odom->setDecimals(1);
        doubleSpinBox_maxDepth_odom->setMaximum(100);
        doubleSpinBox_maxDepth_odom->setSingleStep(0.1);
        doubleSpinBox_maxDepth_odom->setValue(0);

        gridLayout_2->addWidget(doubleSpinBox_maxDepth_odom, 3, 1, 1, 1);

        lineEdit_roiRatios_odom = new QLineEdit(groupBox_12);
        lineEdit_roiRatios_odom->setObjectName(QStringLiteral("lineEdit_roiRatios_odom"));

        gridLayout_2->addWidget(lineEdit_roiRatios_odom, 5, 1, 1, 1);

        label_154 = new QLabel(groupBox_12);
        label_154->setObjectName(QStringLiteral("label_154"));
        label_154->setAlignment(Qt::AlignCenter);
        label_154->setWordWrap(true);

        gridLayout_2->addWidget(label_154, 0, 0, 1, 1);

        label_137 = new QLabel(groupBox_12);
        label_137->setObjectName(QStringLiteral("label_137"));
        label_137->setAlignment(Qt::AlignCenter);
        label_137->setWordWrap(true);

        gridLayout_2->addWidget(label_137, 0, 1, 1, 1);

        checkBox_showClouds = new QCheckBox(groupBox_12);
        checkBox_showClouds->setObjectName(QStringLiteral("checkBox_showClouds"));
        checkBox_showClouds->setChecked(true);

        gridLayout_2->addWidget(checkBox_showClouds, 1, 0, 1, 1);

        checkBox_showOdomClouds = new QCheckBox(groupBox_12);
        checkBox_showOdomClouds->setObjectName(QStringLiteral("checkBox_showOdomClouds"));
        checkBox_showOdomClouds->setChecked(true);

        gridLayout_2->addWidget(checkBox_showOdomClouds, 1, 1, 1, 1);

        label_119 = new QLabel(groupBox_12);
        label_119->setObjectName(QStringLiteral("label_119"));
        label_119->setWordWrap(true);
        label_119->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_119, 1, 2, 1, 1);

        spinBox_decimation_odom = new QSpinBox(groupBox_12);
        spinBox_decimation_odom->setObjectName(QStringLiteral("spinBox_decimation_odom"));
        spinBox_decimation_odom->setMinimum(-32);
        spinBox_decimation_odom->setMaximum(32);
        spinBox_decimation_odom->setValue(1);

        gridLayout_2->addWidget(spinBox_decimation_odom, 2, 1, 1, 1);

        label_108 = new QLabel(groupBox_12);
        label_108->setObjectName(QStringLiteral("label_108"));
        label_108->setWordWrap(true);
        label_108->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_108, 2, 2, 1, 1);

        doubleSpinBox_opacity = new QDoubleSpinBox(groupBox_12);
        doubleSpinBox_opacity->setObjectName(QStringLiteral("doubleSpinBox_opacity"));
        doubleSpinBox_opacity->setDecimals(2);
        doubleSpinBox_opacity->setMaximum(1);
        doubleSpinBox_opacity->setSingleStep(0.1);
        doubleSpinBox_opacity->setValue(1);

        gridLayout_2->addWidget(doubleSpinBox_opacity, 14, 0, 1, 1);

        label_155 = new QLabel(groupBox_12);
        label_155->setObjectName(QStringLiteral("label_155"));
        label_155->setWordWrap(true);
        label_155->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_155, 14, 2, 1, 1);

        label_157 = new QLabel(groupBox_12);
        label_157->setObjectName(QStringLiteral("label_157"));
        label_157->setWordWrap(true);
        label_157->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_157, 15, 2, 1, 1);

        doubleSpinBox_noiseRadius = new QDoubleSpinBox(groupBox_12);
        doubleSpinBox_noiseRadius->setObjectName(QStringLiteral("doubleSpinBox_noiseRadius"));
        doubleSpinBox_noiseRadius->setDecimals(3);
        doubleSpinBox_noiseRadius->setMaximum(1);
        doubleSpinBox_noiseRadius->setSingleStep(0.05);
        doubleSpinBox_noiseRadius->setValue(0);

        gridLayout_2->addWidget(doubleSpinBox_noiseRadius, 7, 0, 1, 1);

        spinBox_decimation = new QSpinBox(groupBox_12);
        spinBox_decimation->setObjectName(QStringLiteral("spinBox_decimation"));
        spinBox_decimation->setMinimum(-32);
        spinBox_decimation->setMaximum(32);
        spinBox_decimation->setValue(1);

        gridLayout_2->addWidget(spinBox_decimation, 2, 0, 1, 1);

        doubleSpinBox_maxDepth = new QDoubleSpinBox(groupBox_12);
        doubleSpinBox_maxDepth->setObjectName(QStringLiteral("doubleSpinBox_maxDepth"));
        doubleSpinBox_maxDepth->setDecimals(1);
        doubleSpinBox_maxDepth->setMaximum(100);
        doubleSpinBox_maxDepth->setSingleStep(0.1);
        doubleSpinBox_maxDepth->setValue(4);

        gridLayout_2->addWidget(doubleSpinBox_maxDepth, 3, 0, 1, 1);

        label_132 = new QLabel(groupBox_12);
        label_132->setObjectName(QStringLiteral("label_132"));
        label_132->setWordWrap(true);
        label_132->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_132, 3, 2, 1, 1);

        doubleSpinBox_minDepth = new QDoubleSpinBox(groupBox_12);
        doubleSpinBox_minDepth->setObjectName(QStringLiteral("doubleSpinBox_minDepth"));
        doubleSpinBox_minDepth->setDecimals(1);
        doubleSpinBox_minDepth->setMaximum(100);
        doubleSpinBox_minDepth->setSingleStep(0.1);
        doubleSpinBox_minDepth->setValue(0);

        gridLayout_2->addWidget(doubleSpinBox_minDepth, 4, 0, 1, 1);

        doubleSpinBox_minDepth_odom = new QDoubleSpinBox(groupBox_12);
        doubleSpinBox_minDepth_odom->setObjectName(QStringLiteral("doubleSpinBox_minDepth_odom"));
        doubleSpinBox_minDepth_odom->setDecimals(1);
        doubleSpinBox_minDepth_odom->setMinimum(0);
        doubleSpinBox_minDepth_odom->setMaximum(100);
        doubleSpinBox_minDepth_odom->setSingleStep(0.1);
        doubleSpinBox_minDepth_odom->setValue(0);

        gridLayout_2->addWidget(doubleSpinBox_minDepth_odom, 4, 1, 1, 1);

        label_150 = new QLabel(groupBox_12);
        label_150->setObjectName(QStringLiteral("label_150"));
        label_150->setWordWrap(true);
        label_150->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_150, 4, 2, 1, 1);

        label_353 = new QLabel(groupBox_12);
        label_353->setObjectName(QStringLiteral("label_353"));
        label_353->setWordWrap(true);
        label_353->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_353, 5, 2, 1, 1);

        doubleSpinBox_floorFilterHeight = new QDoubleSpinBox(groupBox_12);
        doubleSpinBox_floorFilterHeight->setObjectName(QStringLiteral("doubleSpinBox_floorFilterHeight"));
        doubleSpinBox_floorFilterHeight->setDecimals(3);
        doubleSpinBox_floorFilterHeight->setMinimum(-10);
        doubleSpinBox_floorFilterHeight->setMaximum(10);
        doubleSpinBox_floorFilterHeight->setSingleStep(0.1);
        doubleSpinBox_floorFilterHeight->setValue(0);

        gridLayout_2->addWidget(doubleSpinBox_floorFilterHeight, 10, 0, 1, 1);

        doubleSpinBox_voxel = new QDoubleSpinBox(groupBox_12);
        doubleSpinBox_voxel->setObjectName(QStringLiteral("doubleSpinBox_voxel"));
        doubleSpinBox_voxel->setDecimals(3);
        doubleSpinBox_voxel->setMaximum(1);
        doubleSpinBox_voxel->setSingleStep(0.01);
        doubleSpinBox_voxel->setValue(0);

        gridLayout_2->addWidget(doubleSpinBox_voxel, 6, 0, 1, 1);

        spinBox_ptsize_odom = new QSpinBox(groupBox_12);
        spinBox_ptsize_odom->setObjectName(QStringLiteral("spinBox_ptsize_odom"));
        spinBox_ptsize_odom->setMinimum(1);
        spinBox_ptsize_odom->setMaximum(64);
        spinBox_ptsize_odom->setValue(2);

        gridLayout_2->addWidget(spinBox_ptsize_odom, 15, 1, 1, 1);

        label_168 = new QLabel(groupBox_12);
        label_168->setObjectName(QStringLiteral("label_168"));
        label_168->setWordWrap(true);
        label_168->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_168, 7, 2, 1, 1);

        spinBox_noiseMinNeighbors = new QSpinBox(groupBox_12);
        spinBox_noiseMinNeighbors->setObjectName(QStringLiteral("spinBox_noiseMinNeighbors"));
        spinBox_noiseMinNeighbors->setMinimum(1);
        spinBox_noiseMinNeighbors->setMaximum(1000);
        spinBox_noiseMinNeighbors->setValue(5);

        gridLayout_2->addWidget(spinBox_noiseMinNeighbors, 8, 0, 1, 1);

        label_338 = new QLabel(groupBox_12);
        label_338->setObjectName(QStringLiteral("label_338"));
        label_338->setWordWrap(true);
        label_338->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_338, 9, 2, 1, 1);

        label_171 = new QLabel(groupBox_12);
        label_171->setObjectName(QStringLiteral("label_171"));
        label_171->setWordWrap(true);
        label_171->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_171, 6, 2, 1, 1);

        doubleSpinBox_ceilingFilterHeight = new QDoubleSpinBox(groupBox_12);
        doubleSpinBox_ceilingFilterHeight->setObjectName(QStringLiteral("doubleSpinBox_ceilingFilterHeight"));
        doubleSpinBox_ceilingFilterHeight->setDecimals(3);
        doubleSpinBox_ceilingFilterHeight->setMinimum(-10);
        doubleSpinBox_ceilingFilterHeight->setMaximum(10);
        doubleSpinBox_ceilingFilterHeight->setSingleStep(0.1);
        doubleSpinBox_ceilingFilterHeight->setValue(0);

        gridLayout_2->addWidget(doubleSpinBox_ceilingFilterHeight, 9, 0, 1, 1);

        spinBox_normalKSearch = new QSpinBox(groupBox_12);
        spinBox_normalKSearch->setObjectName(QStringLiteral("spinBox_normalKSearch"));
        spinBox_normalKSearch->setMinimum(0);
        spinBox_normalKSearch->setMaximum(1000);
        spinBox_normalKSearch->setValue(10);

        gridLayout_2->addWidget(spinBox_normalKSearch, 11, 0, 1, 1);

        label_210 = new QLabel(groupBox_12);
        label_210->setObjectName(QStringLiteral("label_210"));
        label_210->setWordWrap(true);
        label_210->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_210, 11, 2, 1, 1);

        label_427 = new QLabel(groupBox_12);
        label_427->setObjectName(QStringLiteral("label_427"));
        label_427->setWordWrap(true);
        label_427->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_427, 12, 2, 1, 1);

        doubleSpinBox_opacity_odom = new QDoubleSpinBox(groupBox_12);
        doubleSpinBox_opacity_odom->setObjectName(QStringLiteral("doubleSpinBox_opacity_odom"));
        doubleSpinBox_opacity_odom->setDecimals(2);
        doubleSpinBox_opacity_odom->setMaximum(1);
        doubleSpinBox_opacity_odom->setSingleStep(0.1);
        doubleSpinBox_opacity_odom->setValue(1);

        gridLayout_2->addWidget(doubleSpinBox_opacity_odom, 14, 1, 1, 1);

        gridLayout_2->setColumnStretch(2, 1);

        verticalLayout_112->addLayout(gridLayout_2);

        groupBox_organized = new QGroupBox(groupBox_12);
        groupBox_organized->setObjectName(QStringLiteral("groupBox_organized"));
        groupBox_organized->setCheckable(true);
        verticalLayout_31 = new QVBoxLayout(groupBox_organized);
        verticalLayout_31->setObjectName(QStringLiteral("verticalLayout_31"));
        gridLayout_64 = new QGridLayout();
        gridLayout_64->setObjectName(QStringLiteral("gridLayout_64"));
        label_85 = new QLabel(groupBox_organized);
        label_85->setObjectName(QStringLiteral("label_85"));

        gridLayout_64->addWidget(label_85, 0, 1, 1, 1);

        label_quad = new QLabel(groupBox_organized);
        label_quad->setObjectName(QStringLiteral("label_quad"));

        gridLayout_64->addWidget(label_quad, 1, 1, 1, 1);

        spinBox_mesh_triangleSize = new QSpinBox(groupBox_organized);
        spinBox_mesh_triangleSize->setObjectName(QStringLiteral("spinBox_mesh_triangleSize"));
        spinBox_mesh_triangleSize->setMinimum(2);
        spinBox_mesh_triangleSize->setMaximum(99);

        gridLayout_64->addWidget(spinBox_mesh_triangleSize, 2, 0, 1, 1);

        label_87 = new QLabel(groupBox_organized);
        label_87->setObjectName(QStringLiteral("label_87"));

        gridLayout_64->addWidget(label_87, 2, 1, 1, 1);

        checkBox_mesh_quad = new QCheckBox(groupBox_organized);
        checkBox_mesh_quad->setObjectName(QStringLiteral("checkBox_mesh_quad"));
        checkBox_mesh_quad->setChecked(true);

        gridLayout_64->addWidget(checkBox_mesh_quad, 1, 0, 1, 1);

        doubleSpinBox_mesh_angleTolerance = new QDoubleSpinBox(groupBox_organized);
        doubleSpinBox_mesh_angleTolerance->setObjectName(QStringLiteral("doubleSpinBox_mesh_angleTolerance"));
        doubleSpinBox_mesh_angleTolerance->setDecimals(0);
        doubleSpinBox_mesh_angleTolerance->setMinimum(1);
        doubleSpinBox_mesh_angleTolerance->setMaximum(99);
        doubleSpinBox_mesh_angleTolerance->setValue(15);

        gridLayout_64->addWidget(doubleSpinBox_mesh_angleTolerance, 0, 0, 1, 1);

        label_337 = new QLabel(groupBox_organized);
        label_337->setObjectName(QStringLiteral("label_337"));

        gridLayout_64->addWidget(label_337, 3, 1, 1, 1);

        checkBox_mesh_texture = new QCheckBox(groupBox_organized);
        checkBox_mesh_texture->setObjectName(QStringLiteral("checkBox_mesh_texture"));
        checkBox_mesh_texture->setChecked(true);

        gridLayout_64->addWidget(checkBox_mesh_texture, 3, 0, 1, 1);

        gridLayout_64->setColumnStretch(1, 1);

        verticalLayout_31->addLayout(gridLayout_64);


        verticalLayout_112->addWidget(groupBox_organized);


        verticalLayout_98->addWidget(groupBox_12);

        groupBox_17 = new QGroupBox(groupBox_cloudRendering1);
        groupBox_17->setObjectName(QStringLiteral("groupBox_17"));
        gridLayout_79 = new QGridLayout(groupBox_17);
        gridLayout_79->setObjectName(QStringLiteral("gridLayout_79"));
        doubleSpinBox_opacity_odom_scan = new QDoubleSpinBox(groupBox_17);
        doubleSpinBox_opacity_odom_scan->setObjectName(QStringLiteral("doubleSpinBox_opacity_odom_scan"));
        doubleSpinBox_opacity_odom_scan->setDecimals(2);
        doubleSpinBox_opacity_odom_scan->setMaximum(1);
        doubleSpinBox_opacity_odom_scan->setSingleStep(0.1);
        doubleSpinBox_opacity_odom_scan->setValue(1);

        gridLayout_79->addWidget(doubleSpinBox_opacity_odom_scan, 11, 1, 1, 1);

        label_273 = new QLabel(groupBox_17);
        label_273->setObjectName(QStringLiteral("label_273"));
        label_273->setWordWrap(true);
        label_273->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_273, 2, 2, 1, 1);

        spinBox_ptsize_odom_scan = new QSpinBox(groupBox_17);
        spinBox_ptsize_odom_scan->setObjectName(QStringLiteral("spinBox_ptsize_odom_scan"));
        spinBox_ptsize_odom_scan->setMinimum(1);
        spinBox_ptsize_odom_scan->setMaximum(64);

        gridLayout_79->addWidget(spinBox_ptsize_odom_scan, 12, 1, 1, 1);

        label_110 = new QLabel(groupBox_17);
        label_110->setObjectName(QStringLiteral("label_110"));
        label_110->setWordWrap(true);
        label_110->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_110, 1, 2, 1, 1);

        label_271 = new QLabel(groupBox_17);
        label_271->setObjectName(QStringLiteral("label_271"));
        label_271->setWordWrap(true);
        label_271->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_271, 3, 2, 1, 1);

        label_356 = new QLabel(groupBox_17);
        label_356->setObjectName(QStringLiteral("label_356"));
        label_356->setWordWrap(true);
        label_356->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_356, 0, 1, 1, 1);

        label_359 = new QLabel(groupBox_17);
        label_359->setObjectName(QStringLiteral("label_359"));
        label_359->setWordWrap(true);
        label_359->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_359, 0, 0, 1, 1);

        doubleSpinBox_ceilingFilterHeight_scan = new QDoubleSpinBox(groupBox_17);
        doubleSpinBox_ceilingFilterHeight_scan->setObjectName(QStringLiteral("doubleSpinBox_ceilingFilterHeight_scan"));
        doubleSpinBox_ceilingFilterHeight_scan->setDecimals(3);
        doubleSpinBox_ceilingFilterHeight_scan->setMinimum(-10);
        doubleSpinBox_ceilingFilterHeight_scan->setMaximum(10);
        doubleSpinBox_ceilingFilterHeight_scan->setSingleStep(0.1);
        doubleSpinBox_ceilingFilterHeight_scan->setValue(0);

        gridLayout_79->addWidget(doubleSpinBox_ceilingFilterHeight_scan, 6, 0, 1, 1);

        label_241 = new QLabel(groupBox_17);
        label_241->setObjectName(QStringLiteral("label_241"));
        label_241->setWordWrap(true);
        label_241->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_241, 8, 2, 1, 1);

        label_458 = new QLabel(groupBox_17);
        label_458->setObjectName(QStringLiteral("label_458"));
        label_458->setWordWrap(true);
        label_458->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_458, 10, 2, 1, 1);

        label_460 = new QLabel(groupBox_17);
        label_460->setObjectName(QStringLiteral("label_460"));
        label_460->setWordWrap(true);
        label_460->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_460, 4, 2, 1, 1);

        doubleSpinBox_maxRange = new QDoubleSpinBox(groupBox_17);
        doubleSpinBox_maxRange->setObjectName(QStringLiteral("doubleSpinBox_maxRange"));
        doubleSpinBox_maxRange->setMaximum(999);

        gridLayout_79->addWidget(doubleSpinBox_maxRange, 4, 0, 1, 1);

        label_428 = new QLabel(groupBox_17);
        label_428->setObjectName(QStringLiteral("label_428"));
        label_428->setWordWrap(true);
        label_428->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_428, 9, 2, 1, 1);

        label_158 = new QLabel(groupBox_17);
        label_158->setObjectName(QStringLiteral("label_158"));
        label_158->setWordWrap(true);
        label_158->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_158, 12, 2, 1, 1);

        doubleSpinBox_voxelSizeScan_odom = new QDoubleSpinBox(groupBox_17);
        doubleSpinBox_voxelSizeScan_odom->setObjectName(QStringLiteral("doubleSpinBox_voxelSizeScan_odom"));
        doubleSpinBox_voxelSizeScan_odom->setDecimals(3);
        doubleSpinBox_voxelSizeScan_odom->setMaximum(1);
        doubleSpinBox_voxelSizeScan_odom->setSingleStep(0.01);
        doubleSpinBox_voxelSizeScan_odom->setValue(0);

        gridLayout_79->addWidget(doubleSpinBox_voxelSizeScan_odom, 3, 1, 1, 1);

        checkBox_showScans = new QCheckBox(groupBox_17);
        checkBox_showScans->setObjectName(QStringLiteral("checkBox_showScans"));
        checkBox_showScans->setChecked(true);

        gridLayout_79->addWidget(checkBox_showScans, 1, 0, 1, 1);

        label_156 = new QLabel(groupBox_17);
        label_156->setObjectName(QStringLiteral("label_156"));
        label_156->setWordWrap(true);
        label_156->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_156, 11, 2, 1, 1);

        spinBox_ptsize_scan = new QSpinBox(groupBox_17);
        spinBox_ptsize_scan->setObjectName(QStringLiteral("spinBox_ptsize_scan"));
        spinBox_ptsize_scan->setMinimum(1);
        spinBox_ptsize_scan->setMaximum(64);

        gridLayout_79->addWidget(spinBox_ptsize_scan, 12, 0, 1, 1);

        spinBox_downsamplingScan_odom = new QSpinBox(groupBox_17);
        spinBox_downsamplingScan_odom->setObjectName(QStringLiteral("spinBox_downsamplingScan_odom"));
        spinBox_downsamplingScan_odom->setMinimum(1);
        spinBox_downsamplingScan_odom->setMaximum(9999);

        gridLayout_79->addWidget(spinBox_downsamplingScan_odom, 2, 1, 1, 1);

        checkBox_showOdomScans = new QCheckBox(groupBox_17);
        checkBox_showOdomScans->setObjectName(QStringLiteral("checkBox_showOdomScans"));
        checkBox_showOdomScans->setChecked(true);

        gridLayout_79->addWidget(checkBox_showOdomScans, 1, 1, 1, 1);

        spinBox_downsamplingScan = new QSpinBox(groupBox_17);
        spinBox_downsamplingScan->setObjectName(QStringLiteral("spinBox_downsamplingScan"));
        spinBox_downsamplingScan->setMinimum(1);
        spinBox_downsamplingScan->setMaximum(9999);

        gridLayout_79->addWidget(spinBox_downsamplingScan, 2, 0, 1, 1);

        doubleSpinBox_opacity_scan = new QDoubleSpinBox(groupBox_17);
        doubleSpinBox_opacity_scan->setObjectName(QStringLiteral("doubleSpinBox_opacity_scan"));
        doubleSpinBox_opacity_scan->setDecimals(2);
        doubleSpinBox_opacity_scan->setMaximum(1);
        doubleSpinBox_opacity_scan->setSingleStep(0.1);
        doubleSpinBox_opacity_scan->setValue(1);

        gridLayout_79->addWidget(doubleSpinBox_opacity_scan, 11, 0, 1, 1);

        doubleSpinBox_voxelSizeScan = new QDoubleSpinBox(groupBox_17);
        doubleSpinBox_voxelSizeScan->setObjectName(QStringLiteral("doubleSpinBox_voxelSizeScan"));
        doubleSpinBox_voxelSizeScan->setDecimals(3);
        doubleSpinBox_voxelSizeScan->setMaximum(1);
        doubleSpinBox_voxelSizeScan->setSingleStep(0.01);
        doubleSpinBox_voxelSizeScan->setValue(0);

        gridLayout_79->addWidget(doubleSpinBox_voxelSizeScan, 3, 0, 1, 1);

        label_367 = new QLabel(groupBox_17);
        label_367->setObjectName(QStringLiteral("label_367"));
        label_367->setWordWrap(true);
        label_367->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_367, 6, 2, 1, 1);

        label_368 = new QLabel(groupBox_17);
        label_368->setObjectName(QStringLiteral("label_368"));
        label_368->setWordWrap(true);
        label_368->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_368, 7, 2, 1, 1);

        doubleSpinBox_floorFilterHeight_scan = new QDoubleSpinBox(groupBox_17);
        doubleSpinBox_floorFilterHeight_scan->setObjectName(QStringLiteral("doubleSpinBox_floorFilterHeight_scan"));
        doubleSpinBox_floorFilterHeight_scan->setDecimals(3);
        doubleSpinBox_floorFilterHeight_scan->setMinimum(-10);
        doubleSpinBox_floorFilterHeight_scan->setMaximum(10);
        doubleSpinBox_floorFilterHeight_scan->setSingleStep(0.1);
        doubleSpinBox_floorFilterHeight_scan->setValue(0);

        gridLayout_79->addWidget(doubleSpinBox_floorFilterHeight_scan, 7, 0, 1, 1);

        spinBox_normalKSearch_scan = new QSpinBox(groupBox_17);
        spinBox_normalKSearch_scan->setObjectName(QStringLiteral("spinBox_normalKSearch_scan"));
        spinBox_normalKSearch_scan->setMinimum(0);
        spinBox_normalKSearch_scan->setMaximum(1000);
        spinBox_normalKSearch_scan->setValue(10);

        gridLayout_79->addWidget(spinBox_normalKSearch_scan, 8, 0, 1, 1);

        doubleSpinBox_normalRadiusSearch_scan = new QDoubleSpinBox(groupBox_17);
        doubleSpinBox_normalRadiusSearch_scan->setObjectName(QStringLiteral("doubleSpinBox_normalRadiusSearch_scan"));
        doubleSpinBox_normalRadiusSearch_scan->setSingleStep(0.01);

        gridLayout_79->addWidget(doubleSpinBox_normalRadiusSearch_scan, 9, 0, 1, 1);

        spinBox_colorSchemeScan = new QSpinBox(groupBox_17);
        spinBox_colorSchemeScan->setObjectName(QStringLiteral("spinBox_colorSchemeScan"));

        gridLayout_79->addWidget(spinBox_colorSchemeScan, 10, 0, 1, 1);

        spinBox_colorSchemeScan_odom = new QSpinBox(groupBox_17);
        spinBox_colorSchemeScan_odom->setObjectName(QStringLiteral("spinBox_colorSchemeScan_odom"));

        gridLayout_79->addWidget(spinBox_colorSchemeScan_odom, 10, 1, 1, 1);

        label_461 = new QLabel(groupBox_17);
        label_461->setObjectName(QStringLiteral("label_461"));
        label_461->setWordWrap(true);
        label_461->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_79->addWidget(label_461, 5, 2, 1, 1);

        doubleSpinBox_minRange = new QDoubleSpinBox(groupBox_17);
        doubleSpinBox_minRange->setObjectName(QStringLiteral("doubleSpinBox_minRange"));
        doubleSpinBox_minRange->setMaximum(999);

        gridLayout_79->addWidget(doubleSpinBox_minRange, 5, 0, 1, 1);

        doubleSpinBox_maxRange_odom = new QDoubleSpinBox(groupBox_17);
        doubleSpinBox_maxRange_odom->setObjectName(QStringLiteral("doubleSpinBox_maxRange_odom"));
        doubleSpinBox_maxRange_odom->setMaximum(999);

        gridLayout_79->addWidget(doubleSpinBox_maxRange_odom, 4, 1, 1, 1);

        doubleSpinBox_minRange_odom = new QDoubleSpinBox(groupBox_17);
        doubleSpinBox_minRange_odom->setObjectName(QStringLiteral("doubleSpinBox_minRange_odom"));
        doubleSpinBox_minRange_odom->setMaximum(999);

        gridLayout_79->addWidget(doubleSpinBox_minRange_odom, 5, 1, 1, 1);

        gridLayout_79->setColumnStretch(2, 1);

        verticalLayout_98->addWidget(groupBox_17);

        groupBox_18 = new QGroupBox(groupBox_cloudRendering1);
        groupBox_18->setObjectName(QStringLiteral("groupBox_18"));
        gridLayout_82 = new QGridLayout(groupBox_18);
        gridLayout_82->setObjectName(QStringLiteral("gridLayout_82"));
        label_213 = new QLabel(groupBox_18);
        label_213->setObjectName(QStringLiteral("label_213"));
        label_213->setWordWrap(true);
        label_213->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_82->addWidget(label_213, 3, 2, 1, 1);

        checkBox_showFeatures = new QCheckBox(groupBox_18);
        checkBox_showFeatures->setObjectName(QStringLiteral("checkBox_showFeatures"));
        checkBox_showFeatures->setChecked(true);

        gridLayout_82->addWidget(checkBox_showFeatures, 1, 0, 1, 1);

        checkBox_showGraphs = new QCheckBox(groupBox_18);
        checkBox_showGraphs->setObjectName(QStringLiteral("checkBox_showGraphs"));
        checkBox_showGraphs->setChecked(true);

        gridLayout_82->addWidget(checkBox_showGraphs, 3, 0, 1, 1);

        checkBox_showLabels = new QCheckBox(groupBox_18);
        checkBox_showLabels->setObjectName(QStringLiteral("checkBox_showLabels"));
        checkBox_showLabels->setChecked(true);

        gridLayout_82->addWidget(checkBox_showLabels, 5, 0, 1, 1);

        spinBox_ptsize_odom_features = new QSpinBox(groupBox_18);
        spinBox_ptsize_odom_features->setObjectName(QStringLiteral("spinBox_ptsize_odom_features"));
        spinBox_ptsize_odom_features->setMinimum(1);
        spinBox_ptsize_odom_features->setMaximum(64);

        gridLayout_82->addWidget(spinBox_ptsize_odom_features, 2, 1, 1, 1);

        checkBox_showOdomFeatures = new QCheckBox(groupBox_18);
        checkBox_showOdomFeatures->setObjectName(QStringLiteral("checkBox_showOdomFeatures"));
        checkBox_showOdomFeatures->setChecked(true);

        gridLayout_82->addWidget(checkBox_showOdomFeatures, 1, 1, 1, 1);

        label_123 = new QLabel(groupBox_18);
        label_123->setObjectName(QStringLiteral("label_123"));
        label_123->setWordWrap(true);
        label_123->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_82->addWidget(label_123, 1, 2, 1, 1);

        label_243 = new QLabel(groupBox_18);
        label_243->setObjectName(QStringLiteral("label_243"));
        label_243->setWordWrap(true);
        label_243->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_82->addWidget(label_243, 5, 2, 1, 1);

        label_360 = new QLabel(groupBox_18);
        label_360->setObjectName(QStringLiteral("label_360"));
        label_360->setWordWrap(true);
        label_360->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_82->addWidget(label_360, 0, 0, 1, 1);

        label_361 = new QLabel(groupBox_18);
        label_361->setObjectName(QStringLiteral("label_361"));
        label_361->setWordWrap(true);
        label_361->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_82->addWidget(label_361, 0, 1, 1, 1);

        spinBox_ptsize_features = new QSpinBox(groupBox_18);
        spinBox_ptsize_features->setObjectName(QStringLiteral("spinBox_ptsize_features"));
        spinBox_ptsize_features->setMinimum(1);
        spinBox_ptsize_features->setMaximum(64);

        gridLayout_82->addWidget(spinBox_ptsize_features, 2, 0, 1, 1);

        label_166 = new QLabel(groupBox_18);
        label_166->setObjectName(QStringLiteral("label_166"));
        label_166->setWordWrap(true);
        label_166->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_82->addWidget(label_166, 2, 2, 1, 1);

        label_362 = new QLabel(groupBox_18);
        label_362->setObjectName(QStringLiteral("label_362"));
        label_362->setWordWrap(true);
        label_362->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_82->addWidget(label_362, 4, 2, 1, 1);

        checkBox_showFrustums = new QCheckBox(groupBox_18);
        checkBox_showFrustums->setObjectName(QStringLiteral("checkBox_showFrustums"));
        checkBox_showFrustums->setChecked(true);

        gridLayout_82->addWidget(checkBox_showFrustums, 4, 0, 1, 1);

        checkBox_showOdomFrustums = new QCheckBox(groupBox_18);
        checkBox_showOdomFrustums->setObjectName(QStringLiteral("checkBox_showOdomFrustums"));
        checkBox_showOdomFrustums->setChecked(true);

        gridLayout_82->addWidget(checkBox_showOdomFrustums, 4, 1, 1, 1);

        label_301 = new QLabel(groupBox_18);
        label_301->setObjectName(QStringLiteral("label_301"));
        label_301->setWordWrap(true);
        label_301->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_82->addWidget(label_301, 7, 2, 1, 1);

        checkBox_showLandmarks = new QCheckBox(groupBox_18);
        checkBox_showLandmarks->setObjectName(QStringLiteral("checkBox_showLandmarks"));
        checkBox_showLandmarks->setChecked(true);

        gridLayout_82->addWidget(checkBox_showLandmarks, 7, 0, 1, 1);

        label_528 = new QLabel(groupBox_18);
        label_528->setObjectName(QStringLiteral("label_528"));
        label_528->setWordWrap(true);
        label_528->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_82->addWidget(label_528, 8, 2, 1, 1);

        doubleSpinBox_landmarkSize = new QDoubleSpinBox(groupBox_18);
        doubleSpinBox_landmarkSize->setObjectName(QStringLiteral("doubleSpinBox_landmarkSize"));
        doubleSpinBox_landmarkSize->setMinimum(0);
        doubleSpinBox_landmarkSize->setSingleStep(0.1);
        doubleSpinBox_landmarkSize->setValue(0.1);

        gridLayout_82->addWidget(doubleSpinBox_landmarkSize, 8, 0, 1, 1);

        label_567 = new QLabel(groupBox_18);
        label_567->setObjectName(QStringLiteral("label_567"));
        label_567->setWordWrap(true);
        label_567->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_82->addWidget(label_567, 9, 2, 1, 1);

        label_568 = new QLabel(groupBox_18);
        label_568->setObjectName(QStringLiteral("label_568"));
        label_568->setWordWrap(true);
        label_568->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_82->addWidget(label_568, 11, 2, 1, 1);

        checkBox_showIMUAcc = new QCheckBox(groupBox_18);
        checkBox_showIMUAcc->setObjectName(QStringLiteral("checkBox_showIMUAcc"));
        checkBox_showIMUAcc->setChecked(true);

        gridLayout_82->addWidget(checkBox_showIMUAcc, 11, 1, 1, 1);

        checkBox_showIMUGravity = new QCheckBox(groupBox_18);
        checkBox_showIMUGravity->setObjectName(QStringLiteral("checkBox_showIMUGravity"));
        checkBox_showIMUGravity->setChecked(true);

        gridLayout_82->addWidget(checkBox_showIMUGravity, 9, 0, 1, 1);

        checkBox_showIMUGravity_odom = new QCheckBox(groupBox_18);
        checkBox_showIMUGravity_odom->setObjectName(QStringLiteral("checkBox_showIMUGravity_odom"));
        checkBox_showIMUGravity_odom->setChecked(true);

        gridLayout_82->addWidget(checkBox_showIMUGravity_odom, 9, 1, 1, 1);

        doubleSpinBox_lengthIMUGravity = new QDoubleSpinBox(groupBox_18);
        doubleSpinBox_lengthIMUGravity->setObjectName(QStringLiteral("doubleSpinBox_lengthIMUGravity"));
        doubleSpinBox_lengthIMUGravity->setMinimum(0.01);
        doubleSpinBox_lengthIMUGravity->setSingleStep(1);
        doubleSpinBox_lengthIMUGravity->setValue(1);

        gridLayout_82->addWidget(doubleSpinBox_lengthIMUGravity, 10, 0, 1, 1);

        doubleSpinBox_lengthIMUGravity_odom = new QDoubleSpinBox(groupBox_18);
        doubleSpinBox_lengthIMUGravity_odom->setObjectName(QStringLiteral("doubleSpinBox_lengthIMUGravity_odom"));
        doubleSpinBox_lengthIMUGravity_odom->setMinimum(0.01);
        doubleSpinBox_lengthIMUGravity_odom->setSingleStep(1);
        doubleSpinBox_lengthIMUGravity_odom->setValue(1);

        gridLayout_82->addWidget(doubleSpinBox_lengthIMUGravity_odom, 10, 1, 1, 1);

        label_569 = new QLabel(groupBox_18);
        label_569->setObjectName(QStringLiteral("label_569"));
        label_569->setWordWrap(true);
        label_569->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_82->addWidget(label_569, 10, 2, 1, 1);

        label_572 = new QLabel(groupBox_18);
        label_572->setObjectName(QStringLiteral("label_572"));
        label_572->setWordWrap(true);
        label_572->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_82->addWidget(label_572, 6, 2, 1, 1);

        checkBox_showFrames = new QCheckBox(groupBox_18);
        checkBox_showFrames->setObjectName(QStringLiteral("checkBox_showFrames"));
        checkBox_showFrames->setChecked(true);

        gridLayout_82->addWidget(checkBox_showFrames, 6, 0, 1, 1);


        verticalLayout_98->addWidget(groupBox_18);


        verticalLayout_42->addWidget(groupBox_cloudRendering1);

        verticalSpacer = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_42->addItem(verticalSpacer);

        stackedWidget->addWidget(page_2);
        page_61 = new QWidget();
        page_61->setObjectName(QStringLiteral("page_61"));
        verticalLayout_104 = new QVBoxLayout(page_61);
        verticalLayout_104->setObjectName(QStringLiteral("verticalLayout_104"));
        groupBox_filtering2 = new QGroupBox(page_61);
        groupBox_filtering2->setObjectName(QStringLiteral("groupBox_filtering2"));
        groupBox_filtering2->setCheckable(false);
        groupBox_filtering2->setChecked(false);
        verticalLayout_37 = new QVBoxLayout(groupBox_filtering2);
        verticalLayout_37->setObjectName(QStringLiteral("verticalLayout_37"));
        label_160 = new QLabel(groupBox_filtering2);
        label_160->setObjectName(QStringLiteral("label_160"));
        label_160->setWordWrap(true);
        label_160->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_37->addWidget(label_160);

        gridLayout_65 = new QGridLayout();
        gridLayout_65->setObjectName(QStringLiteral("gridLayout_65"));
        label_71 = new QLabel(groupBox_filtering2);
        label_71->setObjectName(QStringLiteral("label_71"));
        label_71->setWordWrap(true);
        label_71->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_65->addWidget(label_71, 0, 1, 1, 1);

        radioButton_noFiltering = new QRadioButton(groupBox_filtering2);
        radioButton_noFiltering->setObjectName(QStringLiteral("radioButton_noFiltering"));
        radioButton_noFiltering->setChecked(false);

        gridLayout_65->addWidget(radioButton_noFiltering, 0, 0, 1, 1);

        radioButton_subtractFiltering = new QRadioButton(groupBox_filtering2);
        radioButton_subtractFiltering->setObjectName(QStringLiteral("radioButton_subtractFiltering"));

        gridLayout_65->addWidget(radioButton_subtractFiltering, 3, 0, 1, 1);

        gridLayout_17 = new QGridLayout();
        gridLayout_17->setObjectName(QStringLiteral("gridLayout_17"));
        doubleSpinBox_cloudFilterRadius = new QDoubleSpinBox(groupBox_filtering2);
        doubleSpinBox_cloudFilterRadius->setObjectName(QStringLiteral("doubleSpinBox_cloudFilterRadius"));
        doubleSpinBox_cloudFilterRadius->setMinimum(0.01);
        doubleSpinBox_cloudFilterRadius->setValue(0.1);

        gridLayout_17->addWidget(doubleSpinBox_cloudFilterRadius, 0, 0, 1, 1);

        label_37 = new QLabel(groupBox_filtering2);
        label_37->setObjectName(QStringLiteral("label_37"));
        label_37->setWordWrap(true);
        label_37->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_17->addWidget(label_37, 0, 1, 1, 1);

        doubleSpinBox_cloudFilterAngle = new QDoubleSpinBox(groupBox_filtering2);
        doubleSpinBox_cloudFilterAngle->setObjectName(QStringLiteral("doubleSpinBox_cloudFilterAngle"));
        doubleSpinBox_cloudFilterAngle->setDecimals(0);
        doubleSpinBox_cloudFilterAngle->setMaximum(180);
        doubleSpinBox_cloudFilterAngle->setValue(30);

        gridLayout_17->addWidget(doubleSpinBox_cloudFilterAngle, 1, 0, 1, 1);

        label_48 = new QLabel(groupBox_filtering2);
        label_48->setObjectName(QStringLiteral("label_48"));
        label_48->setWordWrap(true);
        label_48->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_17->addWidget(label_48, 1, 1, 1, 1);

        gridLayout_17->setColumnStretch(1, 1);

        gridLayout_65->addLayout(gridLayout_17, 2, 1, 1, 1);

        label_161 = new QLabel(groupBox_filtering2);
        label_161->setObjectName(QStringLiteral("label_161"));
        label_161->setTextFormat(Qt::PlainText);
        label_161->setWordWrap(true);
        label_161->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_65->addWidget(label_161, 3, 1, 1, 1);

        label_33 = new QLabel(groupBox_filtering2);
        label_33->setObjectName(QStringLiteral("label_33"));
        label_33->setWordWrap(true);
        label_33->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_65->addWidget(label_33, 1, 1, 1, 1);

        radioButton_nodeFiltering = new QRadioButton(groupBox_filtering2);
        radioButton_nodeFiltering->setObjectName(QStringLiteral("radioButton_nodeFiltering"));
        radioButton_nodeFiltering->setChecked(false);

        gridLayout_65->addWidget(radioButton_nodeFiltering, 1, 0, 1, 1);

        gridLayout_69 = new QGridLayout();
        gridLayout_69->setObjectName(QStringLiteral("gridLayout_69"));
        doubleSpinBox_subtractFilteringRadius = new QDoubleSpinBox(groupBox_filtering2);
        doubleSpinBox_subtractFilteringRadius->setObjectName(QStringLiteral("doubleSpinBox_subtractFilteringRadius"));
        doubleSpinBox_subtractFilteringRadius->setMinimum(0.01);
        doubleSpinBox_subtractFilteringRadius->setMaximum(1);
        doubleSpinBox_subtractFilteringRadius->setSingleStep(0.01);
        doubleSpinBox_subtractFilteringRadius->setValue(0.02);

        gridLayout_69->addWidget(doubleSpinBox_subtractFilteringRadius, 0, 0, 1, 1);

        label_192 = new QLabel(groupBox_filtering2);
        label_192->setObjectName(QStringLiteral("label_192"));
        label_192->setWordWrap(true);
        label_192->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_69->addWidget(label_192, 0, 1, 1, 1);

        doubleSpinBox_subtractFilteringAngle = new QDoubleSpinBox(groupBox_filtering2);
        doubleSpinBox_subtractFilteringAngle->setObjectName(QStringLiteral("doubleSpinBox_subtractFilteringAngle"));
        doubleSpinBox_subtractFilteringAngle->setDecimals(0);
        doubleSpinBox_subtractFilteringAngle->setMaximum(180);
        doubleSpinBox_subtractFilteringAngle->setValue(30);

        gridLayout_69->addWidget(doubleSpinBox_subtractFilteringAngle, 1, 0, 1, 1);

        label_193 = new QLabel(groupBox_filtering2);
        label_193->setObjectName(QStringLiteral("label_193"));
        label_193->setWordWrap(true);
        label_193->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_69->addWidget(label_193, 1, 1, 1, 1);

        spinBox_subtractFilteringMinPts = new QSpinBox(groupBox_filtering2);
        spinBox_subtractFilteringMinPts->setObjectName(QStringLiteral("spinBox_subtractFilteringMinPts"));
        spinBox_subtractFilteringMinPts->setMinimum(1);

        gridLayout_69->addWidget(spinBox_subtractFilteringMinPts, 2, 0, 1, 1);

        label_174 = new QLabel(groupBox_filtering2);
        label_174->setObjectName(QStringLiteral("label_174"));
        label_174->setWordWrap(true);
        label_174->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_69->addWidget(label_174, 2, 1, 1, 1);

        gridLayout_69->setColumnStretch(1, 1);

        gridLayout_65->addLayout(gridLayout_69, 4, 1, 1, 1);

        gridLayout_65->setColumnStretch(1, 1);

        verticalLayout_37->addLayout(gridLayout_65);


        verticalLayout_104->addWidget(groupBox_filtering2);

        verticalSpacer_55 = new QSpacerItem(20, 1625, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_104->addItem(verticalSpacer_55);

        stackedWidget->addWidget(page_61);
        page_62 = new QWidget();
        page_62->setObjectName(QStringLiteral("page_62"));
        verticalLayout_105 = new QVBoxLayout(page_62);
        verticalLayout_105->setObjectName(QStringLiteral("verticalLayout_105"));
        groupBox_gridMap2 = new QGroupBox(page_62);
        groupBox_gridMap2->setObjectName(QStringLiteral("groupBox_gridMap2"));
        groupBox_gridMap2->setCheckable(false);
        groupBox_gridMap2->setChecked(false);
        verticalLayout_45 = new QVBoxLayout(groupBox_gridMap2);
        verticalLayout_45->setObjectName(QStringLiteral("verticalLayout_45"));
        label_167 = new QLabel(groupBox_gridMap2);
        label_167->setObjectName(QStringLiteral("label_167"));
        label_167->setWordWrap(true);
        label_167->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_45->addWidget(label_167);

        gridLayout_20 = new QGridLayout();
        gridLayout_20->setObjectName(QStringLiteral("gridLayout_20"));
        checkBox_map_shown = new QCheckBox(groupBox_gridMap2);
        checkBox_map_shown->setObjectName(QStringLiteral("checkBox_map_shown"));
        checkBox_map_shown->setChecked(false);

        gridLayout_20->addWidget(checkBox_map_shown, 0, 0, 1, 1);

        label_map_shown = new QLabel(groupBox_gridMap2);
        label_map_shown->setObjectName(QStringLiteral("label_map_shown"));
        label_map_shown->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_20->addWidget(label_map_shown, 0, 1, 1, 1);

        label_170 = new QLabel(groupBox_gridMap2);
        label_170->setObjectName(QStringLiteral("label_170"));
        label_170->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_20->addWidget(label_170, 1, 1, 1, 1);

        doubleSpinBox_map_opacity = new QDoubleSpinBox(groupBox_gridMap2);
        doubleSpinBox_map_opacity->setObjectName(QStringLiteral("doubleSpinBox_map_opacity"));
        doubleSpinBox_map_opacity->setMinimum(0.01);
        doubleSpinBox_map_opacity->setMaximum(1);
        doubleSpinBox_map_opacity->setSingleStep(0.05);
        doubleSpinBox_map_opacity->setValue(0.75);

        gridLayout_20->addWidget(doubleSpinBox_map_opacity, 1, 0, 1, 1);

        gridLayout_20->setColumnStretch(1, 1);

        verticalLayout_45->addLayout(gridLayout_20);

        groupBox_octomap = new QGroupBox(groupBox_gridMap2);
        groupBox_octomap->setObjectName(QStringLiteral("groupBox_octomap"));
        groupBox_octomap->setCheckable(true);
        gridLayout_72 = new QGridLayout(groupBox_octomap);
        gridLayout_72->setObjectName(QStringLiteral("gridLayout_72"));
        checkBox_octomap_show3dMap = new QCheckBox(groupBox_octomap);
        checkBox_octomap_show3dMap->setObjectName(QStringLiteral("checkBox_octomap_show3dMap"));
        checkBox_octomap_show3dMap->setChecked(false);

        gridLayout_72->addWidget(checkBox_octomap_show3dMap, 0, 0, 1, 1);

        label_octomap_treeDepth = new QLabel(groupBox_octomap);
        label_octomap_treeDepth->setObjectName(QStringLiteral("label_octomap_treeDepth"));
        label_octomap_treeDepth->setWordWrap(true);
        label_octomap_treeDepth->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_72->addWidget(label_octomap_treeDepth, 4, 1, 1, 1);

        spinBox_octomap_treeDepth = new QSpinBox(groupBox_octomap);
        spinBox_octomap_treeDepth->setObjectName(QStringLiteral("spinBox_octomap_treeDepth"));
        spinBox_octomap_treeDepth->setMinimum(1);
        spinBox_octomap_treeDepth->setMaximum(16);
        spinBox_octomap_treeDepth->setValue(16);

        gridLayout_72->addWidget(spinBox_octomap_treeDepth, 4, 0, 1, 1);

        spinBox_octomap_pointSize = new QSpinBox(groupBox_octomap);
        spinBox_octomap_pointSize->setObjectName(QStringLiteral("spinBox_octomap_pointSize"));
        spinBox_octomap_pointSize->setMinimum(1);
        spinBox_octomap_pointSize->setMaximum(99);
        spinBox_octomap_pointSize->setValue(5);

        gridLayout_72->addWidget(spinBox_octomap_pointSize, 3, 0, 1, 1);

        label_octomap_treeDepth_5 = new QLabel(groupBox_octomap);
        label_octomap_treeDepth_5->setObjectName(QStringLiteral("label_octomap_treeDepth_5"));
        label_octomap_treeDepth_5->setWordWrap(true);
        label_octomap_treeDepth_5->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_72->addWidget(label_octomap_treeDepth_5, 2, 1, 1, 1);

        checkBox_octomap_2dgrid = new QCheckBox(groupBox_octomap);
        checkBox_octomap_2dgrid->setObjectName(QStringLiteral("checkBox_octomap_2dgrid"));
        checkBox_octomap_2dgrid->setChecked(false);

        gridLayout_72->addWidget(checkBox_octomap_2dgrid, 1, 0, 1, 1);

        label_octomap_treeDepth_3 = new QLabel(groupBox_octomap);
        label_octomap_treeDepth_3->setObjectName(QStringLiteral("label_octomap_treeDepth_3"));
        label_octomap_treeDepth_3->setWordWrap(true);
        label_octomap_treeDepth_3->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_72->addWidget(label_octomap_treeDepth_3, 0, 1, 1, 1);

        label_octomap_treeDepth_6 = new QLabel(groupBox_octomap);
        label_octomap_treeDepth_6->setObjectName(QStringLiteral("label_octomap_treeDepth_6"));
        label_octomap_treeDepth_6->setWordWrap(true);
        label_octomap_treeDepth_6->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_72->addWidget(label_octomap_treeDepth_6, 3, 1, 1, 1);

        label_octomap_treeDepth_8 = new QLabel(groupBox_octomap);
        label_octomap_treeDepth_8->setObjectName(QStringLiteral("label_octomap_treeDepth_8"));
        label_octomap_treeDepth_8->setWordWrap(true);
        label_octomap_treeDepth_8->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_72->addWidget(label_octomap_treeDepth_8, 1, 1, 1, 1);

        comboBox_octomap_renderingType = new QComboBox(groupBox_octomap);
        comboBox_octomap_renderingType->setObjectName(QStringLiteral("comboBox_octomap_renderingType"));
        comboBox_octomap_renderingType->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_72->addWidget(comboBox_octomap_renderingType, 2, 0, 1, 1);

        gridLayout_72->setColumnStretch(1, 1);

        verticalLayout_45->addWidget(groupBox_octomap);


        verticalLayout_105->addWidget(groupBox_gridMap2);

        verticalSpacer_56 = new QSpacerItem(20, 1375, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_105->addItem(verticalSpacer_56);

        stackedWidget->addWidget(page_62);
        page_8 = new QWidget();
        page_8->setObjectName(QStringLiteral("page_8"));
        verticalLayout_43 = new QVBoxLayout(page_8);
        verticalLayout_43->setObjectName(QStringLiteral("verticalLayout_43"));
        groupBox_logging1 = new QGroupBox(page_8);
        groupBox_logging1->setObjectName(QStringLiteral("groupBox_logging1"));
        gridLayout_40 = new QGridLayout(groupBox_logging1);
        gridLayout_40->setObjectName(QStringLiteral("gridLayout_40"));
        label_60 = new QLabel(groupBox_logging1);
        label_60->setObjectName(QStringLiteral("label_60"));
        label_60->setWordWrap(true);
        label_60->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_40->addWidget(label_60, 0, 1, 1, 1);

        comboBox_loggerLevel = new QComboBox(groupBox_logging1);
        comboBox_loggerLevel->setObjectName(QStringLiteral("comboBox_loggerLevel"));
        comboBox_loggerLevel->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_40->addWidget(comboBox_loggerLevel, 0, 0, 1, 1);

        comboBox_loggerEventLevel = new QComboBox(groupBox_logging1);
        comboBox_loggerEventLevel->setObjectName(QStringLiteral("comboBox_loggerEventLevel"));
        comboBox_loggerEventLevel->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_40->addWidget(comboBox_loggerEventLevel, 1, 0, 1, 1);

        label_61 = new QLabel(groupBox_logging1);
        label_61->setObjectName(QStringLiteral("label_61"));
        label_61->setWordWrap(true);
        label_61->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_40->addWidget(label_61, 1, 1, 1, 1);

        comboBox_loggerPauseLevel = new QComboBox(groupBox_logging1);
        comboBox_loggerPauseLevel->setObjectName(QStringLiteral("comboBox_loggerPauseLevel"));
        comboBox_loggerPauseLevel->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_40->addWidget(comboBox_loggerPauseLevel, 2, 0, 1, 1);

        label_62 = new QLabel(groupBox_logging1);
        label_62->setObjectName(QStringLiteral("label_62"));
        label_62->setWordWrap(true);
        label_62->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_40->addWidget(label_62, 2, 1, 1, 1);

        checkBox_logger_printTime = new QCheckBox(groupBox_logging1);
        checkBox_logger_printTime->setObjectName(QStringLiteral("checkBox_logger_printTime"));
        checkBox_logger_printTime->setChecked(true);

        gridLayout_40->addWidget(checkBox_logger_printTime, 3, 0, 1, 1);

        label_67 = new QLabel(groupBox_logging1);
        label_67->setObjectName(QStringLiteral("label_67"));
        label_67->setWordWrap(true);
        label_67->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_40->addWidget(label_67, 3, 1, 1, 1);

        comboBox_loggerType = new QComboBox(groupBox_logging1);
        comboBox_loggerType->setObjectName(QStringLiteral("comboBox_loggerType"));
        comboBox_loggerType->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_40->addWidget(comboBox_loggerType, 5, 0, 1, 1);

        label_68 = new QLabel(groupBox_logging1);
        label_68->setObjectName(QStringLiteral("label_68"));
        label_68->setWordWrap(true);
        label_68->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_40->addWidget(label_68, 5, 1, 1, 1);

        label_118 = new QLabel(groupBox_logging1);
        label_118->setObjectName(QStringLiteral("label_118"));
        label_118->setWordWrap(true);
        label_118->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_40->addWidget(label_118, 4, 1, 1, 1);

        checkBox_logger_printThreadId = new QCheckBox(groupBox_logging1);
        checkBox_logger_printThreadId->setObjectName(QStringLiteral("checkBox_logger_printThreadId"));
        checkBox_logger_printThreadId->setChecked(false);

        gridLayout_40->addWidget(checkBox_logger_printThreadId, 4, 0, 1, 1);

        comboBox_loggerFilter = new QMultiComboBox(groupBox_logging1);
        comboBox_loggerFilter->setObjectName(QStringLiteral("comboBox_loggerFilter"));
        comboBox_loggerFilter->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_40->addWidget(comboBox_loggerFilter, 6, 0, 1, 1);

        label_164 = new QLabel(groupBox_logging1);
        label_164->setObjectName(QStringLiteral("label_164"));
        label_164->setWordWrap(true);
        label_164->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_40->addWidget(label_164, 6, 1, 1, 1);

        gridLayout_40->setColumnStretch(1, 1);

        verticalLayout_43->addWidget(groupBox_logging1);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_43->addItem(verticalSpacer_2);

        stackedWidget->addWidget(page_8);
        page_4 = new QWidget();
        page_4->setObjectName(QStringLiteral("page_4"));
        verticalLayout_27 = new QVBoxLayout(page_4);
        verticalLayout_27->setObjectName(QStringLiteral("verticalLayout_27"));
        groupBox_source0 = new QGroupBox(page_4);
        groupBox_source0->setObjectName(QStringLiteral("groupBox_source0"));
        verticalLayout_7 = new QVBoxLayout(groupBox_source0);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        gridLayout_28 = new QGridLayout();
        gridLayout_28->setObjectName(QStringLiteral("gridLayout_28"));
        label_3 = new QLabel(groupBox_source0);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_28->addWidget(label_3, 1, 1, 1, 1);

        label_42 = new QLabel(groupBox_source0);
        label_42->setObjectName(QStringLiteral("label_42"));
        label_42->setWordWrap(true);
        label_42->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_28->addWidget(label_42, 3, 1, 1, 1);

        label_19 = new QLabel(groupBox_source0);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setWordWrap(true);
        label_19->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_28->addWidget(label_19, 0, 1, 1, 1);

        label_39 = new QLabel(groupBox_source0);
        label_39->setObjectName(QStringLiteral("label_39"));
        label_39->setWordWrap(true);
        label_39->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_28->addWidget(label_39, 2, 1, 1, 1);

        label_244 = new QLabel(groupBox_source0);
        label_244->setObjectName(QStringLiteral("label_244"));
        label_244->setWordWrap(true);
        label_244->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_28->addWidget(label_244, 10, 1, 1, 1);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        toolButton_source_path_calibration = new QToolButton(groupBox_source0);
        toolButton_source_path_calibration->setObjectName(QStringLiteral("toolButton_source_path_calibration"));

        horizontalLayout_5->addWidget(toolButton_source_path_calibration);

        lineEdit_calibrationFile = new QLineEdit(groupBox_source0);
        lineEdit_calibrationFile->setObjectName(QStringLiteral("lineEdit_calibrationFile"));
        lineEdit_calibrationFile->setMinimumSize(QSize(100, 0));

        horizontalLayout_5->addWidget(lineEdit_calibrationFile);

        horizontalLayout_5->setStretch(1, 1);

        gridLayout_28->addLayout(horizontalLayout_5, 7, 0, 1, 1);

        general_doubleSpinBox_imgRate = new QDoubleSpinBox(groupBox_source0);
        general_doubleSpinBox_imgRate->setObjectName(QStringLiteral("general_doubleSpinBox_imgRate"));
        general_doubleSpinBox_imgRate->setDecimals(1);
        general_doubleSpinBox_imgRate->setMaximum(100);
        general_doubleSpinBox_imgRate->setSingleStep(0.1);
        general_doubleSpinBox_imgRate->setValue(0);

        gridLayout_28->addWidget(general_doubleSpinBox_imgRate, 1, 0, 1, 1);

        label_17 = new QLabel(groupBox_source0);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setWordWrap(true);
        label_17->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_28->addWidget(label_17, 4, 1, 1, 1);

        source_mirroring = new QCheckBox(groupBox_source0);
        source_mirroring->setObjectName(QStringLiteral("source_mirroring"));

        gridLayout_28->addWidget(source_mirroring, 4, 0, 1, 1);

        lineEdit_sourceLocalTransform = new QLineEdit(groupBox_source0);
        lineEdit_sourceLocalTransform->setObjectName(QStringLiteral("lineEdit_sourceLocalTransform"));

        gridLayout_28->addWidget(lineEdit_sourceLocalTransform, 3, 0, 1, 1);

        comboBox_sourceType = new QComboBox(groupBox_source0);
        comboBox_sourceType->setObjectName(QStringLiteral("comboBox_sourceType"));
        comboBox_sourceType->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_28->addWidget(comboBox_sourceType, 0, 0, 1, 1);

        pushButton_test_camera = new QPushButton(groupBox_source0);
        pushButton_test_camera->setObjectName(QStringLiteral("pushButton_test_camera"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(pushButton_test_camera->sizePolicy().hasHeightForWidth());
        pushButton_test_camera->setSizePolicy(sizePolicy1);

        gridLayout_28->addWidget(pushButton_test_camera, 11, 0, 1, 1);

        pushButton_calibrate = new QPushButton(groupBox_source0);
        pushButton_calibrate->setObjectName(QStringLiteral("pushButton_calibrate"));
        sizePolicy1.setHeightForWidth(pushButton_calibrate->sizePolicy().hasHeightForWidth());
        pushButton_calibrate->setSizePolicy(sizePolicy1);

        gridLayout_28->addWidget(pushButton_calibrate, 9, 0, 1, 1);

        label_24 = new QLabel(groupBox_source0);
        label_24->setObjectName(QStringLiteral("label_24"));
        label_24->setWordWrap(true);
        label_24->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_28->addWidget(label_24, 9, 1, 1, 1);

        lineEdit_sourceDevice = new QLineEdit(groupBox_source0);
        lineEdit_sourceDevice->setObjectName(QStringLiteral("lineEdit_sourceDevice"));

        gridLayout_28->addWidget(lineEdit_sourceDevice, 2, 0, 1, 1);

        pushButton_calibrate_simple = new QPushButton(groupBox_source0);
        pushButton_calibrate_simple->setObjectName(QStringLiteral("pushButton_calibrate_simple"));
        sizePolicy1.setHeightForWidth(pushButton_calibrate_simple->sizePolicy().hasHeightForWidth());
        pushButton_calibrate_simple->setSizePolicy(sizePolicy1);

        gridLayout_28->addWidget(pushButton_calibrate_simple, 10, 0, 1, 1);

        label_18 = new QLabel(groupBox_source0);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setWordWrap(true);
        label_18->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_28->addWidget(label_18, 7, 1, 1, 1);

        label_36 = new QLabel(groupBox_source0);
        label_36->setObjectName(QStringLiteral("label_36"));
        label_36->setWordWrap(true);
        label_36->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_28->addWidget(label_36, 5, 1, 1, 1);

        spinBox_source_imageDecimation = new QSpinBox(groupBox_source0);
        spinBox_source_imageDecimation->setObjectName(QStringLiteral("spinBox_source_imageDecimation"));
        spinBox_source_imageDecimation->setMinimum(1);

        gridLayout_28->addWidget(spinBox_source_imageDecimation, 5, 0, 1, 1);

        label_229 = new QLabel(groupBox_source0);
        label_229->setObjectName(QStringLiteral("label_229"));
        label_229->setWordWrap(true);
        label_229->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_28->addWidget(label_229, 6, 1, 1, 1);

        checkbox_rgbd_colorOnly = new QCheckBox(groupBox_source0);
        checkbox_rgbd_colorOnly->setObjectName(QStringLiteral("checkbox_rgbd_colorOnly"));
        checkbox_rgbd_colorOnly->setChecked(false);

        gridLayout_28->addWidget(checkbox_rgbd_colorOnly, 6, 0, 1, 1);

        gridLayout_28->setColumnStretch(1, 1);

        verticalLayout_7->addLayout(gridLayout_28);

        stackedWidget_src = new QStackedWidget(groupBox_source0);
        stackedWidget_src->setObjectName(QStringLiteral("stackedWidget_src"));
        page_41 = new QWidget();
        page_41->setObjectName(QStringLiteral("page_41"));
        verticalLayout_64 = new QVBoxLayout(page_41);
        verticalLayout_64->setObjectName(QStringLiteral("verticalLayout_64"));
        groupBox_sourceRGBD = new QGroupBox(page_41);
        groupBox_sourceRGBD->setObjectName(QStringLiteral("groupBox_sourceRGBD"));
        groupBox_sourceRGBD->setCheckable(false);
        groupBox_sourceRGBD->setChecked(false);
        verticalLayout_11 = new QVBoxLayout(groupBox_sourceRGBD);
        verticalLayout_11->setObjectName(QStringLiteral("verticalLayout_11"));
        label_40 = new QLabel(groupBox_sourceRGBD);
        label_40->setObjectName(QStringLiteral("label_40"));
        label_40->setWordWrap(true);
        label_40->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_11->addWidget(label_40);

        gridLayout_55 = new QGridLayout();
        gridLayout_55->setObjectName(QStringLiteral("gridLayout_55"));
        comboBox_cameraRGBD = new QComboBox(groupBox_sourceRGBD);
        comboBox_cameraRGBD->setObjectName(QStringLiteral("comboBox_cameraRGBD"));
        comboBox_cameraRGBD->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_55->addWidget(comboBox_cameraRGBD, 0, 0, 1, 1);

        label_228 = new QLabel(groupBox_sourceRGBD);
        label_228->setObjectName(QStringLiteral("label_228"));
        label_228->setWordWrap(true);
        label_228->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_55->addWidget(label_228, 0, 1, 1, 1);

        gridLayout_55->setColumnStretch(1, 1);

        verticalLayout_11->addLayout(gridLayout_55);

        stackedWidget_rgbd = new QStackedWidget(groupBox_sourceRGBD);
        stackedWidget_rgbd->setObjectName(QStringLiteral("stackedWidget_rgbd"));
        page_32 = new QWidget();
        page_32->setObjectName(QStringLiteral("page_32"));
        verticalLayout_63 = new QVBoxLayout(page_32);
        verticalLayout_63->setObjectName(QStringLiteral("verticalLayout_63"));
        groupBox_openni = new QGroupBox(page_32);
        groupBox_openni->setObjectName(QStringLiteral("groupBox_openni"));
        QSizePolicy sizePolicy2(QSizePolicy::Ignored, QSizePolicy::Ignored);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(groupBox_openni->sizePolicy().hasHeightForWidth());
        groupBox_openni->setSizePolicy(sizePolicy2);
        gridLayout_57 = new QGridLayout(groupBox_openni);
        gridLayout_57->setObjectName(QStringLiteral("gridLayout_57"));
        lineEdit_openniOniPath = new QLineEdit(groupBox_openni);
        lineEdit_openniOniPath->setObjectName(QStringLiteral("lineEdit_openniOniPath"));

        gridLayout_57->addWidget(lineEdit_openniOniPath, 0, 1, 1, 1);

        label_245 = new QLabel(groupBox_openni);
        label_245->setObjectName(QStringLiteral("label_245"));
        label_245->setWordWrap(true);
        label_245->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_57->addWidget(label_245, 0, 2, 1, 1);

        verticalSpacer_34 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_57->addItem(verticalSpacer_34, 1, 0, 1, 1);

        toolButton_openniOniPath = new QToolButton(groupBox_openni);
        toolButton_openniOniPath->setObjectName(QStringLiteral("toolButton_openniOniPath"));

        gridLayout_57->addWidget(toolButton_openniOniPath, 0, 0, 1, 1);

        gridLayout_57->setColumnStretch(2, 1);

        verticalLayout_63->addWidget(groupBox_openni);

        stackedWidget_rgbd->addWidget(page_32);
        page_33 = new QWidget();
        page_33->setObjectName(QStringLiteral("page_33"));
        verticalLayout_80 = new QVBoxLayout(page_33);
        verticalLayout_80->setObjectName(QStringLiteral("verticalLayout_80"));
        verticalSpacer_3 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_80->addItem(verticalSpacer_3);

        stackedWidget_rgbd->addWidget(page_33);
        page_34 = new QWidget();
        page_34->setObjectName(QStringLiteral("page_34"));
        verticalLayout_89 = new QVBoxLayout(page_34);
        verticalLayout_89->setObjectName(QStringLiteral("verticalLayout_89"));
        verticalSpacer_27 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_89->addItem(verticalSpacer_27);

        stackedWidget_rgbd->addWidget(page_34);
        page_35 = new QWidget();
        page_35->setObjectName(QStringLiteral("page_35"));
        verticalLayout_90 = new QVBoxLayout(page_35);
        verticalLayout_90->setObjectName(QStringLiteral("verticalLayout_90"));
        verticalSpacer_28 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_90->addItem(verticalSpacer_28);

        stackedWidget_rgbd->addWidget(page_35);
        page_36 = new QWidget();
        page_36->setObjectName(QStringLiteral("page_36"));
        verticalLayout_51 = new QVBoxLayout(page_36);
        verticalLayout_51->setObjectName(QStringLiteral("verticalLayout_51"));
        groupBox_openni2 = new QGroupBox(page_36);
        groupBox_openni2->setObjectName(QStringLiteral("groupBox_openni2"));
        sizePolicy2.setHeightForWidth(groupBox_openni2->sizePolicy().hasHeightForWidth());
        groupBox_openni2->setSizePolicy(sizePolicy2);
        gridLayout_54 = new QGridLayout(groupBox_openni2);
        gridLayout_54->setObjectName(QStringLiteral("gridLayout_54"));
        label_435 = new QLabel(groupBox_openni2);
        label_435->setObjectName(QStringLiteral("label_435"));
        label_435->setWordWrap(true);

        gridLayout_54->addWidget(label_435, 7, 1, 1, 1);

        openni2_hshift = new QSpinBox(groupBox_openni2);
        openni2_hshift->setObjectName(QStringLiteral("openni2_hshift"));
        openni2_hshift->setMaximum(65535);

        gridLayout_54->addWidget(openni2_hshift, 7, 0, 1, 1);

        openni2_autoWhiteBalance = new QCheckBox(groupBox_openni2);
        openni2_autoWhiteBalance->setObjectName(QStringLiteral("openni2_autoWhiteBalance"));
        openni2_autoWhiteBalance->setChecked(true);

        gridLayout_54->addWidget(openni2_autoWhiteBalance, 1, 0, 1, 1);

        openni2_exposure = new QSpinBox(groupBox_openni2);
        openni2_exposure->setObjectName(QStringLiteral("openni2_exposure"));
        openni2_exposure->setMaximum(65535);

        gridLayout_54->addWidget(openni2_exposure, 3, 0, 1, 1);

        label_264 = new QLabel(groupBox_openni2);
        label_264->setObjectName(QStringLiteral("label_264"));
        label_264->setWordWrap(true);

        gridLayout_54->addWidget(label_264, 5, 1, 1, 1);

        openni2_stampsIdsUsed = new QCheckBox(groupBox_openni2);
        openni2_stampsIdsUsed->setObjectName(QStringLiteral("openni2_stampsIdsUsed"));
        openni2_stampsIdsUsed->setChecked(false);

        gridLayout_54->addWidget(openni2_stampsIdsUsed, 5, 0, 1, 1);

        label_231 = new QLabel(groupBox_openni2);
        label_231->setObjectName(QStringLiteral("label_231"));
        label_231->setWordWrap(true);

        gridLayout_54->addWidget(label_231, 0, 2, 1, 1);

        label_220 = new QLabel(groupBox_openni2);
        label_220->setObjectName(QStringLiteral("label_220"));
        label_220->setWordWrap(true);

        gridLayout_54->addWidget(label_220, 4, 1, 1, 1);

        lineEdit_openni2OniPath = new QLineEdit(groupBox_openni2);
        lineEdit_openni2OniPath->setObjectName(QStringLiteral("lineEdit_openni2OniPath"));

        gridLayout_54->addWidget(lineEdit_openni2OniPath, 0, 1, 1, 1);

        toolButton_openni2OniPath = new QToolButton(groupBox_openni2);
        toolButton_openni2OniPath->setObjectName(QStringLiteral("toolButton_openni2OniPath"));

        gridLayout_54->addWidget(toolButton_openni2OniPath, 0, 0, 1, 1);

        label_217 = new QLabel(groupBox_openni2);
        label_217->setObjectName(QStringLiteral("label_217"));
        label_217->setWordWrap(true);

        gridLayout_54->addWidget(label_217, 1, 1, 1, 1);

        label_223 = new QLabel(groupBox_openni2);
        label_223->setObjectName(QStringLiteral("label_223"));
        label_223->setWordWrap(true);

        gridLayout_54->addWidget(label_223, 6, 1, 1, 1);

        openni2_mirroring = new QCheckBox(groupBox_openni2);
        openni2_mirroring->setObjectName(QStringLiteral("openni2_mirroring"));
        openni2_mirroring->setChecked(false);

        gridLayout_54->addWidget(openni2_mirroring, 6, 0, 1, 1);

        openni2_gain = new QSpinBox(groupBox_openni2);
        openni2_gain->setObjectName(QStringLiteral("openni2_gain"));
        openni2_gain->setMaximum(1000);
        openni2_gain->setValue(100);

        gridLayout_54->addWidget(openni2_gain, 4, 0, 1, 1);

        verticalSpacer_33 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_54->addItem(verticalSpacer_33, 9, 0, 1, 1);

        openni2_autoExposure = new QCheckBox(groupBox_openni2);
        openni2_autoExposure->setObjectName(QStringLiteral("openni2_autoExposure"));
        openni2_autoExposure->setChecked(true);

        gridLayout_54->addWidget(openni2_autoExposure, 2, 0, 1, 1);

        label_218 = new QLabel(groupBox_openni2);
        label_218->setObjectName(QStringLiteral("label_218"));
        label_218->setWordWrap(true);

        gridLayout_54->addWidget(label_218, 2, 1, 1, 1);

        label_219 = new QLabel(groupBox_openni2);
        label_219->setObjectName(QStringLiteral("label_219"));
        label_219->setWordWrap(true);

        gridLayout_54->addWidget(label_219, 3, 1, 1, 1);

        label_436 = new QLabel(groupBox_openni2);
        label_436->setObjectName(QStringLiteral("label_436"));
        label_436->setWordWrap(true);

        gridLayout_54->addWidget(label_436, 8, 1, 1, 1);

        openni2_vshift = new QSpinBox(groupBox_openni2);
        openni2_vshift->setObjectName(QStringLiteral("openni2_vshift"));
        openni2_vshift->setMaximum(65535);

        gridLayout_54->addWidget(openni2_vshift, 8, 0, 1, 1);

        gridLayout_54->setColumnStretch(1, 1);

        verticalLayout_51->addWidget(groupBox_openni2);

        stackedWidget_rgbd->addWidget(page_36);
        page_37 = new QWidget();
        page_37->setObjectName(QStringLiteral("page_37"));
        verticalLayout_62 = new QVBoxLayout(page_37);
        verticalLayout_62->setObjectName(QStringLiteral("verticalLayout_62"));
        groupBox_freenect2 = new QGroupBox(page_37);
        groupBox_freenect2->setObjectName(QStringLiteral("groupBox_freenect2"));
        sizePolicy2.setHeightForWidth(groupBox_freenect2->sizePolicy().hasHeightForWidth());
        groupBox_freenect2->setSizePolicy(sizePolicy2);
        gridLayout_56 = new QGridLayout(groupBox_freenect2);
        gridLayout_56->setObjectName(QStringLiteral("gridLayout_56"));
        label_269 = new QLabel(groupBox_freenect2);
        label_269->setObjectName(QStringLiteral("label_269"));
        label_269->setWordWrap(true);
        label_269->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_56->addWidget(label_269, 4, 1, 1, 1);

        doubleSpinBox_freenect2MaxDepth = new QDoubleSpinBox(groupBox_freenect2);
        doubleSpinBox_freenect2MaxDepth->setObjectName(QStringLiteral("doubleSpinBox_freenect2MaxDepth"));
        doubleSpinBox_freenect2MaxDepth->setMaximum(65);
        doubleSpinBox_freenect2MaxDepth->setValue(12);

        gridLayout_56->addWidget(doubleSpinBox_freenect2MaxDepth, 2, 0, 1, 1);

        comboBox_freenect2Format = new QComboBox(groupBox_freenect2);
        comboBox_freenect2Format->setObjectName(QStringLiteral("comboBox_freenect2Format"));
        comboBox_freenect2Format->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_56->addWidget(comboBox_freenect2Format, 0, 0, 1, 1);

        checkBox_freenect2BilateralFiltering = new QCheckBox(groupBox_freenect2);
        checkBox_freenect2BilateralFiltering->setObjectName(QStringLiteral("checkBox_freenect2BilateralFiltering"));
        checkBox_freenect2BilateralFiltering->setChecked(true);

        gridLayout_56->addWidget(checkBox_freenect2BilateralFiltering, 3, 0, 1, 1);

        label_268 = new QLabel(groupBox_freenect2);
        label_268->setObjectName(QStringLiteral("label_268"));
        label_268->setWordWrap(true);
        label_268->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_56->addWidget(label_268, 3, 1, 1, 1);

        checkBox_freenect2EdgeAwareFiltering = new QCheckBox(groupBox_freenect2);
        checkBox_freenect2EdgeAwareFiltering->setObjectName(QStringLiteral("checkBox_freenect2EdgeAwareFiltering"));
        checkBox_freenect2EdgeAwareFiltering->setChecked(true);

        gridLayout_56->addWidget(checkBox_freenect2EdgeAwareFiltering, 4, 0, 1, 1);

        label_266 = new QLabel(groupBox_freenect2);
        label_266->setObjectName(QStringLiteral("label_266"));
        label_266->setWordWrap(true);
        label_266->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_56->addWidget(label_266, 1, 1, 1, 1);

        label_230 = new QLabel(groupBox_freenect2);
        label_230->setObjectName(QStringLiteral("label_230"));
        label_230->setWordWrap(true);
        label_230->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_56->addWidget(label_230, 0, 1, 1, 1);

        doubleSpinBox_freenect2MinDepth = new QDoubleSpinBox(groupBox_freenect2);
        doubleSpinBox_freenect2MinDepth->setObjectName(QStringLiteral("doubleSpinBox_freenect2MinDepth"));
        doubleSpinBox_freenect2MinDepth->setMaximum(64);
        doubleSpinBox_freenect2MinDepth->setSingleStep(0.1);
        doubleSpinBox_freenect2MinDepth->setValue(0.3);

        gridLayout_56->addWidget(doubleSpinBox_freenect2MinDepth, 1, 0, 1, 1);

        label_270 = new QLabel(groupBox_freenect2);
        label_270->setObjectName(QStringLiteral("label_270"));
        label_270->setWordWrap(true);
        label_270->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_56->addWidget(label_270, 5, 1, 1, 1);

        label_267 = new QLabel(groupBox_freenect2);
        label_267->setObjectName(QStringLiteral("label_267"));
        label_267->setWordWrap(true);
        label_267->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_56->addWidget(label_267, 2, 1, 1, 1);

        checkBox_freenect2NoiseFiltering = new QCheckBox(groupBox_freenect2);
        checkBox_freenect2NoiseFiltering->setObjectName(QStringLiteral("checkBox_freenect2NoiseFiltering"));
        checkBox_freenect2NoiseFiltering->setChecked(true);

        gridLayout_56->addWidget(checkBox_freenect2NoiseFiltering, 5, 0, 1, 1);

        label_434 = new QLabel(groupBox_freenect2);
        label_434->setObjectName(QStringLiteral("label_434"));
        label_434->setWordWrap(true);
        label_434->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_56->addWidget(label_434, 6, 1, 1, 1);

        lineEdit_freenect2Pipeline = new QLineEdit(groupBox_freenect2);
        lineEdit_freenect2Pipeline->setObjectName(QStringLiteral("lineEdit_freenect2Pipeline"));

        gridLayout_56->addWidget(lineEdit_freenect2Pipeline, 6, 0, 1, 1);

        gridLayout_56->setColumnStretch(1, 1);

        verticalLayout_62->addWidget(groupBox_freenect2);

        stackedWidget_rgbd->addWidget(page_37);
        page_63 = new QWidget();
        page_63->setObjectName(QStringLiteral("page_63"));
        verticalLayout_106 = new QVBoxLayout(page_63);
        verticalLayout_106->setObjectName(QStringLiteral("verticalLayout_106"));
        groupBox_realsense = new QGroupBox(page_63);
        groupBox_realsense->setObjectName(QStringLiteral("groupBox_realsense"));
        sizePolicy2.setHeightForWidth(groupBox_realsense->sizePolicy().hasHeightForWidth());
        groupBox_realsense->setSizePolicy(sizePolicy2);
        gridLayout_73 = new QGridLayout(groupBox_realsense);
        gridLayout_73->setObjectName(QStringLiteral("gridLayout_73"));
        comboBox_realsensePresetRGB = new QComboBox(groupBox_realsense);
        comboBox_realsensePresetRGB->setObjectName(QStringLiteral("comboBox_realsensePresetRGB"));
        comboBox_realsensePresetRGB->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_73->addWidget(comboBox_realsensePresetRGB, 0, 0, 1, 1);

        label_318 = new QLabel(groupBox_realsense);
        label_318->setObjectName(QStringLiteral("label_318"));
        label_318->setWordWrap(true);
        label_318->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_73->addWidget(label_318, 1, 1, 1, 1);

        label_317 = new QLabel(groupBox_realsense);
        label_317->setObjectName(QStringLiteral("label_317"));
        label_317->setWordWrap(true);
        label_317->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_73->addWidget(label_317, 0, 1, 1, 1);

        comboBox_realsensePresetDepth = new QComboBox(groupBox_realsense);
        comboBox_realsensePresetDepth->setObjectName(QStringLiteral("comboBox_realsensePresetDepth"));
        comboBox_realsensePresetDepth->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_73->addWidget(comboBox_realsensePresetDepth, 1, 0, 1, 1);

        label_realsenseOdom = new QLabel(groupBox_realsense);
        label_realsenseOdom->setObjectName(QStringLiteral("label_realsenseOdom"));
        label_realsenseOdom->setWordWrap(true);
        label_realsenseOdom->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_73->addWidget(label_realsenseOdom, 2, 1, 1, 1);

        checkbox_realsenseOdom = new QCheckBox(groupBox_realsense);
        checkbox_realsenseOdom->setObjectName(QStringLiteral("checkbox_realsenseOdom"));
        checkbox_realsenseOdom->setChecked(false);

        gridLayout_73->addWidget(checkbox_realsenseOdom, 2, 0, 1, 1);

        checkbox_realsenseDepthScaledToRGBSize = new QCheckBox(groupBox_realsense);
        checkbox_realsenseDepthScaledToRGBSize->setObjectName(QStringLiteral("checkbox_realsenseDepthScaledToRGBSize"));
        checkbox_realsenseDepthScaledToRGBSize->setChecked(false);

        gridLayout_73->addWidget(checkbox_realsenseDepthScaledToRGBSize, 3, 0, 1, 1);

        label_realsenseOdom_2 = new QLabel(groupBox_realsense);
        label_realsenseOdom_2->setObjectName(QStringLiteral("label_realsenseOdom_2"));
        label_realsenseOdom_2->setWordWrap(true);
        label_realsenseOdom_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_73->addWidget(label_realsenseOdom_2, 3, 1, 1, 1);

        comboBox_realsenseRGBSource = new QComboBox(groupBox_realsense);
        comboBox_realsenseRGBSource->setObjectName(QStringLiteral("comboBox_realsenseRGBSource"));
        comboBox_realsenseRGBSource->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_73->addWidget(comboBox_realsenseRGBSource, 4, 0, 1, 1);

        label_realsenseOdom_3 = new QLabel(groupBox_realsense);
        label_realsenseOdom_3->setObjectName(QStringLiteral("label_realsenseOdom_3"));
        label_realsenseOdom_3->setWordWrap(true);
        label_realsenseOdom_3->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_73->addWidget(label_realsenseOdom_3, 4, 1, 1, 1);

        verticalSpacer_32 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_73->addItem(verticalSpacer_32, 5, 0, 1, 1);

        gridLayout_73->setColumnStretch(1, 1);

        verticalLayout_106->addWidget(groupBox_realsense);

        stackedWidget_rgbd->addWidget(page_63);
        page_47 = new QWidget();
        page_47->setObjectName(QStringLiteral("page_47"));
        verticalLayout_79 = new QVBoxLayout(page_47);
        verticalLayout_79->setObjectName(QStringLiteral("verticalLayout_79"));
        groupBox_cameraRGBDImages = new QGroupBox(page_47);
        groupBox_cameraRGBDImages->setObjectName(QStringLiteral("groupBox_cameraRGBDImages"));
        sizePolicy2.setHeightForWidth(groupBox_cameraRGBDImages->sizePolicy().hasHeightForWidth());
        groupBox_cameraRGBDImages->setSizePolicy(sizePolicy2);
        gridLayout_35 = new QGridLayout(groupBox_cameraRGBDImages);
        gridLayout_35->setObjectName(QStringLiteral("gridLayout_35"));
        label_252 = new QLabel(groupBox_cameraRGBDImages);
        label_252->setObjectName(QStringLiteral("label_252"));
        label_252->setWordWrap(true);
        label_252->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_35->addWidget(label_252, 0, 2, 1, 1);

        toolButton_cameraRGBDImages_path_rgb = new QToolButton(groupBox_cameraRGBDImages);
        toolButton_cameraRGBDImages_path_rgb->setObjectName(QStringLiteral("toolButton_cameraRGBDImages_path_rgb"));

        gridLayout_35->addWidget(toolButton_cameraRGBDImages_path_rgb, 0, 0, 1, 1);

        label_257 = new QLabel(groupBox_cameraRGBDImages);
        label_257->setObjectName(QStringLiteral("label_257"));
        label_257->setWordWrap(true);
        label_257->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_35->addWidget(label_257, 2, 2, 1, 1);

        toolButton_cameraRGBDImages_path_depth = new QToolButton(groupBox_cameraRGBDImages);
        toolButton_cameraRGBDImages_path_depth->setObjectName(QStringLiteral("toolButton_cameraRGBDImages_path_depth"));

        gridLayout_35->addWidget(toolButton_cameraRGBDImages_path_depth, 1, 0, 1, 1);

        lineEdit_cameraRGBDImages_path_rgb = new QLineEdit(groupBox_cameraRGBDImages);
        lineEdit_cameraRGBDImages_path_rgb->setObjectName(QStringLiteral("lineEdit_cameraRGBDImages_path_rgb"));

        gridLayout_35->addWidget(lineEdit_cameraRGBDImages_path_rgb, 0, 1, 1, 1);

        lineEdit_cameraRGBDImages_path_depth = new QLineEdit(groupBox_cameraRGBDImages);
        lineEdit_cameraRGBDImages_path_depth->setObjectName(QStringLiteral("lineEdit_cameraRGBDImages_path_depth"));

        gridLayout_35->addWidget(lineEdit_cameraRGBDImages_path_depth, 1, 1, 1, 1);

        label_254 = new QLabel(groupBox_cameraRGBDImages);
        label_254->setObjectName(QStringLiteral("label_254"));
        label_254->setWordWrap(true);
        label_254->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_35->addWidget(label_254, 1, 2, 1, 1);

        doubleSpinBox_cameraRGBDImages_scale = new QDoubleSpinBox(groupBox_cameraRGBDImages);
        doubleSpinBox_cameraRGBDImages_scale->setObjectName(QStringLiteral("doubleSpinBox_cameraRGBDImages_scale"));
        doubleSpinBox_cameraRGBDImages_scale->setDecimals(3);
        doubleSpinBox_cameraRGBDImages_scale->setMinimum(1);
        doubleSpinBox_cameraRGBDImages_scale->setMaximum(999999);

        gridLayout_35->addWidget(doubleSpinBox_cameraRGBDImages_scale, 2, 1, 1, 1);

        verticalSpacer_40 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_35->addItem(verticalSpacer_40, 5, 1, 1, 1);

        label_441 = new QLabel(groupBox_cameraRGBDImages);
        label_441->setObjectName(QStringLiteral("label_441"));
        label_441->setWordWrap(true);
        label_441->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_35->addWidget(label_441, 3, 2, 1, 1);

        spinBox_cameraRGBDImages_startIndex = new QSpinBox(groupBox_cameraRGBDImages);
        spinBox_cameraRGBDImages_startIndex->setObjectName(QStringLiteral("spinBox_cameraRGBDImages_startIndex"));
        spinBox_cameraRGBDImages_startIndex->setMaximum(9999999);

        gridLayout_35->addWidget(spinBox_cameraRGBDImages_startIndex, 3, 1, 1, 1);

        label_540 = new QLabel(groupBox_cameraRGBDImages);
        label_540->setObjectName(QStringLiteral("label_540"));
        label_540->setWordWrap(true);
        label_540->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_35->addWidget(label_540, 4, 2, 1, 1);

        spinBox_cameraRGBDImages_maxFrames = new QSpinBox(groupBox_cameraRGBDImages);
        spinBox_cameraRGBDImages_maxFrames->setObjectName(QStringLiteral("spinBox_cameraRGBDImages_maxFrames"));
        spinBox_cameraRGBDImages_maxFrames->setMaximum(9999999);

        gridLayout_35->addWidget(spinBox_cameraRGBDImages_maxFrames, 4, 1, 1, 1);

        gridLayout_35->setColumnStretch(2, 1);

        verticalLayout_79->addWidget(groupBox_cameraRGBDImages);

        stackedWidget_rgbd->addWidget(page_47);
        page_72 = new QWidget();
        page_72->setObjectName(QStringLiteral("page_72"));
        verticalLayout_127 = new QVBoxLayout(page_72);
        verticalLayout_127->setObjectName(QStringLiteral("verticalLayout_127"));
        groupBox_k4w2 = new QGroupBox(page_72);
        groupBox_k4w2->setObjectName(QStringLiteral("groupBox_k4w2"));
        sizePolicy2.setHeightForWidth(groupBox_k4w2->sizePolicy().hasHeightForWidth());
        groupBox_k4w2->setSizePolicy(sizePolicy2);
        gridLayout_96 = new QGridLayout(groupBox_k4w2);
        gridLayout_96->setObjectName(QStringLiteral("gridLayout_96"));
        comboBox_k4w2Format = new QComboBox(groupBox_k4w2);
        comboBox_k4w2Format->setObjectName(QStringLiteral("comboBox_k4w2Format"));
        comboBox_k4w2Format->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_96->addWidget(comboBox_k4w2Format, 0, 0, 1, 1);

        label_440 = new QLabel(groupBox_k4w2);
        label_440->setObjectName(QStringLiteral("label_440"));
        label_440->setWordWrap(true);
        label_440->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_96->addWidget(label_440, 0, 1, 1, 1);

        verticalSpacer_68 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_96->addItem(verticalSpacer_68, 1, 0, 1, 1);

        gridLayout_96->setColumnStretch(1, 1);

        verticalLayout_127->addWidget(groupBox_k4w2);

        verticalSpacer_67 = new QSpacerItem(20, 283, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_127->addItem(verticalSpacer_67);

        verticalLayout_127->setStretch(0, 1);
        stackedWidget_rgbd->addWidget(page_72);
        page_55 = new QWidget();
        page_55->setObjectName(QStringLiteral("page_55"));
        verticalLayout_103 = new QVBoxLayout(page_55);
        verticalLayout_103->setObjectName(QStringLiteral("verticalLayout_103"));
        groupBox_realsense2 = new QGroupBox(page_55);
        groupBox_realsense2->setObjectName(QStringLiteral("groupBox_realsense2"));
        sizePolicy2.setHeightForWidth(groupBox_realsense2->sizePolicy().hasHeightForWidth());
        groupBox_realsense2->setSizePolicy(sizePolicy2);
        gridLayout_100 = new QGridLayout(groupBox_realsense2);
        gridLayout_100->setObjectName(QStringLiteral("gridLayout_100"));
        spinBox_rs2_width_depth = new QSpinBox(groupBox_realsense2);
        spinBox_rs2_width_depth->setObjectName(QStringLiteral("spinBox_rs2_width_depth"));
        spinBox_rs2_width_depth->setMaximum(9999);

        gridLayout_100->addWidget(spinBox_rs2_width_depth, 6, 1, 1, 1);

        spinBox_rs2_height_depth = new QSpinBox(groupBox_realsense2);
        spinBox_rs2_height_depth->setObjectName(QStringLiteral("spinBox_rs2_height_depth"));
        spinBox_rs2_height_depth->setMaximum(9999);

        gridLayout_100->addWidget(spinBox_rs2_height_depth, 7, 1, 1, 1);

        lineEdit_rs2_jsonFile = new QLineEdit(groupBox_realsense2);
        lineEdit_rs2_jsonFile->setObjectName(QStringLiteral("lineEdit_rs2_jsonFile"));

        gridLayout_100->addWidget(lineEdit_rs2_jsonFile, 12, 1, 1, 1);

        label_565 = new QLabel(groupBox_realsense2);
        label_565->setObjectName(QStringLiteral("label_565"));
        label_565->setWordWrap(true);
        label_565->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_100->addWidget(label_565, 10, 2, 1, 1);

        label_549 = new QLabel(groupBox_realsense2);
        label_549->setObjectName(QStringLiteral("label_549"));
        label_549->setWordWrap(true);
        label_549->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_100->addWidget(label_549, 3, 2, 1, 1);

        spinBox_rs2_height = new QSpinBox(groupBox_realsense2);
        spinBox_rs2_height->setObjectName(QStringLiteral("spinBox_rs2_height"));
        spinBox_rs2_height->setMaximum(9999);

        gridLayout_100->addWidget(spinBox_rs2_height, 4, 1, 1, 1);

        spinBox_rs2_rate = new QSpinBox(groupBox_realsense2);
        spinBox_rs2_rate->setObjectName(QStringLiteral("spinBox_rs2_rate"));
        spinBox_rs2_rate->setMaximum(999);

        gridLayout_100->addWidget(spinBox_rs2_rate, 5, 1, 1, 1);

        checkbox_rs2_emitter = new QCheckBox(groupBox_realsense2);
        checkbox_rs2_emitter->setObjectName(QStringLiteral("checkbox_rs2_emitter"));
        checkbox_rs2_emitter->setChecked(false);

        gridLayout_100->addWidget(checkbox_rs2_emitter, 0, 1, 1, 1);

        label_468 = new QLabel(groupBox_realsense2);
        label_468->setObjectName(QStringLiteral("label_468"));
        label_468->setWordWrap(true);
        label_468->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_100->addWidget(label_468, 0, 2, 1, 1);

        checkbox_rs2_irMode = new QCheckBox(groupBox_realsense2);
        checkbox_rs2_irMode->setObjectName(QStringLiteral("checkbox_rs2_irMode"));
        checkbox_rs2_irMode->setChecked(false);

        gridLayout_100->addWidget(checkbox_rs2_irMode, 1, 1, 1, 1);

        label_471 = new QLabel(groupBox_realsense2);
        label_471->setObjectName(QStringLiteral("label_471"));
        label_471->setWordWrap(true);
        label_471->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_100->addWidget(label_471, 1, 2, 1, 1);

        spinBox_rs2_width = new QSpinBox(groupBox_realsense2);
        spinBox_rs2_width->setObjectName(QStringLiteral("spinBox_rs2_width"));
        spinBox_rs2_width->setMaximum(9999);

        gridLayout_100->addWidget(spinBox_rs2_width, 3, 1, 1, 1);

        label_550 = new QLabel(groupBox_realsense2);
        label_550->setObjectName(QStringLiteral("label_550"));
        label_550->setWordWrap(true);
        label_550->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_100->addWidget(label_550, 4, 2, 1, 1);

        verticalSpacer_71 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_100->addItem(verticalSpacer_71, 13, 1, 1, 1);

        label_551 = new QLabel(groupBox_realsense2);
        label_551->setObjectName(QStringLiteral("label_551"));
        label_551->setWordWrap(true);
        label_551->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_100->addWidget(label_551, 5, 2, 1, 1);

        label_562 = new QLabel(groupBox_realsense2);
        label_562->setObjectName(QStringLiteral("label_562"));
        label_562->setWordWrap(true);
        label_562->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_100->addWidget(label_562, 2, 2, 1, 1);

        checkbox_rs2_irDepth = new QCheckBox(groupBox_realsense2);
        checkbox_rs2_irDepth->setObjectName(QStringLiteral("checkbox_rs2_irDepth"));
        checkbox_rs2_irDepth->setChecked(false);

        gridLayout_100->addWidget(checkbox_rs2_irDepth, 2, 1, 1, 1);

        checkbox_rs2_dualMode = new QCheckBox(groupBox_realsense2);
        checkbox_rs2_dualMode->setObjectName(QStringLiteral("checkbox_rs2_dualMode"));
        checkbox_rs2_dualMode->setChecked(false);

        gridLayout_100->addWidget(checkbox_rs2_dualMode, 10, 1, 1, 1);

        lineEdit_rs2_dualModeExtrinsics = new QLineEdit(groupBox_realsense2);
        lineEdit_rs2_dualModeExtrinsics->setObjectName(QStringLiteral("lineEdit_rs2_dualModeExtrinsics"));

        gridLayout_100->addWidget(lineEdit_rs2_dualModeExtrinsics, 11, 1, 1, 1);

        label_571 = new QLabel(groupBox_realsense2);
        label_571->setObjectName(QStringLiteral("label_571"));
        label_571->setWordWrap(true);
        label_571->setOpenExternalLinks(true);
        label_571->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_100->addWidget(label_571, 12, 2, 1, 1);

        toolButton_rs2_jsonFile = new QToolButton(groupBox_realsense2);
        toolButton_rs2_jsonFile->setObjectName(QStringLiteral("toolButton_rs2_jsonFile"));

        gridLayout_100->addWidget(toolButton_rs2_jsonFile, 12, 0, 1, 1);

        label_606 = new QLabel(groupBox_realsense2);
        label_606->setObjectName(QStringLiteral("label_606"));
        label_606->setWordWrap(true);
        label_606->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_100->addWidget(label_606, 9, 2, 1, 1);

        checkbox_rs2_globalTimeStync = new QCheckBox(groupBox_realsense2);
        checkbox_rs2_globalTimeStync->setObjectName(QStringLiteral("checkbox_rs2_globalTimeStync"));
        checkbox_rs2_globalTimeStync->setChecked(false);

        gridLayout_100->addWidget(checkbox_rs2_globalTimeStync, 9, 1, 1, 1);

        label_611 = new QLabel(groupBox_realsense2);
        label_611->setObjectName(QStringLiteral("label_611"));
        label_611->setWordWrap(true);
        label_611->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_100->addWidget(label_611, 6, 2, 1, 1);

        spinBox_rs2_rate_depth = new QSpinBox(groupBox_realsense2);
        spinBox_rs2_rate_depth->setObjectName(QStringLiteral("spinBox_rs2_rate_depth"));
        spinBox_rs2_rate_depth->setMaximum(999);

        gridLayout_100->addWidget(spinBox_rs2_rate_depth, 8, 1, 1, 1);

        label_612 = new QLabel(groupBox_realsense2);
        label_612->setObjectName(QStringLiteral("label_612"));
        label_612->setWordWrap(true);
        label_612->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_100->addWidget(label_612, 7, 2, 1, 1);

        label_613 = new QLabel(groupBox_realsense2);
        label_613->setObjectName(QStringLiteral("label_613"));
        label_613->setWordWrap(true);
        label_613->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_100->addWidget(label_613, 8, 2, 1, 1);

        label_566 = new QLabel(groupBox_realsense2);
        label_566->setObjectName(QStringLiteral("label_566"));
        label_566->setWordWrap(true);
        label_566->setOpenExternalLinks(true);
        label_566->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_100->addWidget(label_566, 11, 2, 1, 1);

        gridLayout_100->setColumnStretch(2, 1);

        verticalLayout_103->addWidget(groupBox_realsense2);

        stackedWidget_rgbd->addWidget(page_55);
        page_84 = new QWidget();
        page_84->setObjectName(QStringLiteral("page_84"));
        verticalLayout_145 = new QVBoxLayout(page_84);
        verticalLayout_145->setObjectName(QStringLiteral("verticalLayout_145"));
        groupBox_k4a = new QGroupBox(page_84);
        groupBox_k4a->setObjectName(QStringLiteral("groupBox_k4a"));
        sizePolicy2.setHeightForWidth(groupBox_k4a->sizePolicy().hasHeightForWidth());
        groupBox_k4a->setSizePolicy(sizePolicy2);
        gridLayout_113 = new QGridLayout(groupBox_k4a);
        gridLayout_113->setObjectName(QStringLiteral("gridLayout_113"));
        checkbox_k4a_irDepth = new QCheckBox(groupBox_k4a);
        checkbox_k4a_irDepth->setObjectName(QStringLiteral("checkbox_k4a_irDepth"));
        checkbox_k4a_irDepth->setChecked(false);

        gridLayout_113->addWidget(checkbox_k4a_irDepth, 3, 1, 1, 1);

        source_checkBox_useMKVStamps = new QCheckBox(groupBox_k4a);
        source_checkBox_useMKVStamps->setObjectName(QStringLiteral("source_checkBox_useMKVStamps"));

        gridLayout_113->addWidget(source_checkBox_useMKVStamps, 6, 1, 1, 1);

        label_604 = new QLabel(groupBox_k4a);
        label_604->setObjectName(QStringLiteral("label_604"));
        label_604->setWordWrap(true);
        label_604->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_113->addWidget(label_604, 6, 2, 1, 1);

        label_601 = new QLabel(groupBox_k4a);
        label_601->setObjectName(QStringLiteral("label_601"));
        label_601->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_113->addWidget(label_601, 4, 2, 1, 1);

        toolButton_k4a_mkv = new QToolButton(groupBox_k4a);
        toolButton_k4a_mkv->setObjectName(QStringLiteral("toolButton_k4a_mkv"));

        gridLayout_113->addWidget(toolButton_k4a_mkv, 5, 0, 1, 1);

        lineEdit_k4a_mkv = new QLineEdit(groupBox_k4a);
        lineEdit_k4a_mkv->setObjectName(QStringLiteral("lineEdit_k4a_mkv"));

        gridLayout_113->addWidget(lineEdit_k4a_mkv, 5, 1, 1, 1);

        comboBox_k4a_framerate = new QComboBox(groupBox_k4a);
        comboBox_k4a_framerate->setObjectName(QStringLiteral("comboBox_k4a_framerate"));
        QSizePolicy sizePolicy3(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(comboBox_k4a_framerate->sizePolicy().hasHeightForWidth());
        comboBox_k4a_framerate->setSizePolicy(sizePolicy3);

        gridLayout_113->addWidget(comboBox_k4a_framerate, 4, 1, 1, 1);

        comboBox_k4a_rgb_resolution = new QComboBox(groupBox_k4a);
        comboBox_k4a_rgb_resolution->setObjectName(QStringLiteral("comboBox_k4a_rgb_resolution"));

        gridLayout_113->addWidget(comboBox_k4a_rgb_resolution, 0, 1, 1, 1);

        label_603 = new QLabel(groupBox_k4a);
        label_603->setObjectName(QStringLiteral("label_603"));
        label_603->setWordWrap(true);
        label_603->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_113->addWidget(label_603, 5, 2, 1, 1);

        label_558 = new QLabel(groupBox_k4a);
        label_558->setObjectName(QStringLiteral("label_558"));
        label_558->setWordWrap(true);
        label_558->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_113->addWidget(label_558, 3, 2, 1, 1);

        comboBox_k4a_depth_resolution = new QComboBox(groupBox_k4a);
        comboBox_k4a_depth_resolution->setObjectName(QStringLiteral("comboBox_k4a_depth_resolution"));

        gridLayout_113->addWidget(comboBox_k4a_depth_resolution, 1, 1, 1, 1);

        label_602 = new QLabel(groupBox_k4a);
        label_602->setObjectName(QStringLiteral("label_602"));
        label_602->setWordWrap(true);
        label_602->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_113->addWidget(label_602, 1, 2, 1, 1);

        label_600 = new QLabel(groupBox_k4a);
        label_600->setObjectName(QStringLiteral("label_600"));
        label_600->setWordWrap(true);
        label_600->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_113->addWidget(label_600, 0, 2, 1, 1);

        verticalSpacer_42 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_113->addItem(verticalSpacer_42, 7, 1, 1, 1);

        gridLayout_113->setColumnStretch(2, 1);

        verticalLayout_145->addWidget(groupBox_k4a);

        stackedWidget_rgbd->addWidget(page_84);

        verticalLayout_11->addWidget(stackedWidget_rgbd);

        verticalSpacer_83 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_11->addItem(verticalSpacer_83);

        verticalLayout_11->setStretch(3, 1);

        verticalLayout_64->addWidget(groupBox_sourceRGBD);

        stackedWidget_src->addWidget(page_41);
        page_44 = new QWidget();
        page_44->setObjectName(QStringLiteral("page_44"));
        verticalLayout_71 = new QVBoxLayout(page_44);
        verticalLayout_71->setObjectName(QStringLiteral("verticalLayout_71"));
        groupBox_sourceStereo = new QGroupBox(page_44);
        groupBox_sourceStereo->setObjectName(QStringLiteral("groupBox_sourceStereo"));
        groupBox_sourceStereo->setCheckable(false);
        groupBox_sourceStereo->setChecked(false);
        verticalLayout_67 = new QVBoxLayout(groupBox_sourceStereo);
        verticalLayout_67->setObjectName(QStringLiteral("verticalLayout_67"));
        label_46 = new QLabel(groupBox_sourceStereo);
        label_46->setObjectName(QStringLiteral("label_46"));
        label_46->setWordWrap(true);
        label_46->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_67->addWidget(label_46);

        gridLayout_58 = new QGridLayout();
        gridLayout_58->setObjectName(QStringLiteral("gridLayout_58"));
        comboBox_cameraStereo = new QComboBox(groupBox_sourceStereo);
        comboBox_cameraStereo->setObjectName(QStringLiteral("comboBox_cameraStereo"));
        comboBox_cameraStereo->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_58->addWidget(comboBox_cameraStereo, 0, 1, 1, 1);

        label_240 = new QLabel(groupBox_sourceStereo);
        label_240->setObjectName(QStringLiteral("label_240"));
        label_240->setWordWrap(true);
        label_240->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_58->addWidget(label_240, 0, 2, 1, 1);

        checkbox_stereo_depthGenerated = new QCheckBox(groupBox_sourceStereo);
        checkbox_stereo_depthGenerated->setObjectName(QStringLiteral("checkbox_stereo_depthGenerated"));

        gridLayout_58->addWidget(checkbox_stereo_depthGenerated, 1, 1, 1, 1);

        label_stereo_depthGenerated = new QLabel(groupBox_sourceStereo);
        label_stereo_depthGenerated->setObjectName(QStringLiteral("label_stereo_depthGenerated"));
        label_stereo_depthGenerated->setWordWrap(true);
        label_stereo_depthGenerated->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_58->addWidget(label_stereo_depthGenerated, 1, 2, 1, 1);

        label_stereo_rectify = new QLabel(groupBox_sourceStereo);
        label_stereo_rectify->setObjectName(QStringLiteral("label_stereo_rectify"));
        label_stereo_rectify->setWordWrap(true);
        label_stereo_rectify->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_58->addWidget(label_stereo_rectify, 2, 2, 1, 1);

        checkBox_stereo_rectify = new QCheckBox(groupBox_sourceStereo);
        checkBox_stereo_rectify->setObjectName(QStringLiteral("checkBox_stereo_rectify"));

        gridLayout_58->addWidget(checkBox_stereo_rectify, 2, 1, 1, 1);

        label_stereo_rectify_2 = new QLabel(groupBox_sourceStereo);
        label_stereo_rectify_2->setObjectName(QStringLiteral("label_stereo_rectify_2"));
        label_stereo_rectify_2->setWordWrap(true);
        label_stereo_rectify_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_58->addWidget(label_stereo_rectify_2, 3, 2, 1, 1);

        checkBox_stereo_exposureCompensation = new QCheckBox(groupBox_sourceStereo);
        checkBox_stereo_exposureCompensation->setObjectName(QStringLiteral("checkBox_stereo_exposureCompensation"));

        gridLayout_58->addWidget(checkBox_stereo_exposureCompensation, 3, 1, 1, 1);

        gridLayout_58->setColumnStretch(2, 1);

        verticalLayout_67->addLayout(gridLayout_58);

        stackedWidget_stereo = new QStackedWidget(groupBox_sourceStereo);
        stackedWidget_stereo->setObjectName(QStringLiteral("stackedWidget_stereo"));
        page_49 = new QWidget();
        page_49->setObjectName(QStringLiteral("page_49"));
        verticalLayout_91 = new QVBoxLayout(page_49);
        verticalLayout_91->setObjectName(QStringLiteral("verticalLayout_91"));
        verticalSpacer_30 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_91->addItem(verticalSpacer_30);

        stackedWidget_stereo->addWidget(page_49);
        page_48 = new QWidget();
        page_48->setObjectName(QStringLiteral("page_48"));
        verticalLayout_92 = new QVBoxLayout(page_48);
        verticalLayout_92->setObjectName(QStringLiteral("verticalLayout_92"));
        verticalSpacer_47 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_92->addItem(verticalSpacer_47);

        stackedWidget_stereo->addWidget(page_48);
        page_53 = new QWidget();
        page_53->setObjectName(QStringLiteral("page_53"));
        verticalLayout_70 = new QVBoxLayout(page_53);
        verticalLayout_70->setObjectName(QStringLiteral("verticalLayout_70"));
        groupBox_cameraStereoImages = new QGroupBox(page_53);
        groupBox_cameraStereoImages->setObjectName(QStringLiteral("groupBox_cameraStereoImages"));
        sizePolicy2.setHeightForWidth(groupBox_cameraStereoImages->sizePolicy().hasHeightForWidth());
        groupBox_cameraStereoImages->setSizePolicy(sizePolicy2);
        gridLayout_61 = new QGridLayout(groupBox_cameraStereoImages);
        gridLayout_61->setObjectName(QStringLiteral("gridLayout_61"));
        toolButton_cameraStereoImages_path_left = new QToolButton(groupBox_cameraStereoImages);
        toolButton_cameraStereoImages_path_left->setObjectName(QStringLiteral("toolButton_cameraStereoImages_path_left"));

        gridLayout_61->addWidget(toolButton_cameraStereoImages_path_left, 0, 0, 1, 1);

        verticalSpacer_38 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_61->addItem(verticalSpacer_38, 4, 0, 1, 1);

        label_249 = new QLabel(groupBox_cameraStereoImages);
        label_249->setObjectName(QStringLiteral("label_249"));
        label_249->setWordWrap(true);
        label_249->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_61->addWidget(label_249, 0, 2, 1, 1);

        lineEdit_cameraStereoImages_path_left = new QLineEdit(groupBox_cameraStereoImages);
        lineEdit_cameraStereoImages_path_left->setObjectName(QStringLiteral("lineEdit_cameraStereoImages_path_left"));

        gridLayout_61->addWidget(lineEdit_cameraStereoImages_path_left, 0, 1, 1, 1);

        label_253 = new QLabel(groupBox_cameraStereoImages);
        label_253->setObjectName(QStringLiteral("label_253"));
        label_253->setWordWrap(true);
        label_253->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_61->addWidget(label_253, 1, 2, 1, 1);

        lineEdit_cameraStereoImages_path_right = new QLineEdit(groupBox_cameraStereoImages);
        lineEdit_cameraStereoImages_path_right->setObjectName(QStringLiteral("lineEdit_cameraStereoImages_path_right"));

        gridLayout_61->addWidget(lineEdit_cameraStereoImages_path_right, 1, 1, 1, 1);

        toolButton_cameraStereoImages_path_right = new QToolButton(groupBox_cameraStereoImages);
        toolButton_cameraStereoImages_path_right->setObjectName(QStringLiteral("toolButton_cameraStereoImages_path_right"));

        gridLayout_61->addWidget(toolButton_cameraStereoImages_path_right, 1, 0, 1, 1);

        label_442 = new QLabel(groupBox_cameraStereoImages);
        label_442->setObjectName(QStringLiteral("label_442"));
        label_442->setWordWrap(true);
        label_442->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_61->addWidget(label_442, 2, 2, 1, 1);

        spinBox_cameraStereoImages_startIndex = new QSpinBox(groupBox_cameraStereoImages);
        spinBox_cameraStereoImages_startIndex->setObjectName(QStringLiteral("spinBox_cameraStereoImages_startIndex"));
        spinBox_cameraStereoImages_startIndex->setMaximum(9999999);

        gridLayout_61->addWidget(spinBox_cameraStereoImages_startIndex, 2, 1, 1, 1);

        label_530 = new QLabel(groupBox_cameraStereoImages);
        label_530->setObjectName(QStringLiteral("label_530"));
        label_530->setWordWrap(true);
        label_530->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_61->addWidget(label_530, 3, 2, 1, 1);

        spinBox_cameraStereoImages_maxFrames = new QSpinBox(groupBox_cameraStereoImages);
        spinBox_cameraStereoImages_maxFrames->setObjectName(QStringLiteral("spinBox_cameraStereoImages_maxFrames"));
        spinBox_cameraStereoImages_maxFrames->setMaximum(9999999);

        gridLayout_61->addWidget(spinBox_cameraStereoImages_maxFrames, 3, 1, 1, 1);

        gridLayout_61->setColumnStretch(2, 1);

        verticalLayout_70->addWidget(groupBox_cameraStereoImages);

        stackedWidget_stereo->addWidget(page_53);
        page_50 = new QWidget();
        page_50->setObjectName(QStringLiteral("page_50"));
        verticalLayout_78 = new QVBoxLayout(page_50);
        verticalLayout_78->setObjectName(QStringLiteral("verticalLayout_78"));
        groupBox_cameraStereoVideo = new QGroupBox(page_50);
        groupBox_cameraStereoVideo->setObjectName(QStringLiteral("groupBox_cameraStereoVideo"));
        sizePolicy2.setHeightForWidth(groupBox_cameraStereoVideo->sizePolicy().hasHeightForWidth());
        groupBox_cameraStereoVideo->setSizePolicy(sizePolicy2);
        gridLayout_66 = new QGridLayout(groupBox_cameraStereoVideo);
        gridLayout_66->setObjectName(QStringLiteral("gridLayout_66"));
        toolButton_cameraStereoVideo_path = new QToolButton(groupBox_cameraStereoVideo);
        toolButton_cameraStereoVideo_path->setObjectName(QStringLiteral("toolButton_cameraStereoVideo_path"));

        gridLayout_66->addWidget(toolButton_cameraStereoVideo_path, 0, 0, 1, 1);

        lineEdit_cameraStereoVideo_path = new QLineEdit(groupBox_cameraStereoVideo);
        lineEdit_cameraStereoVideo_path->setObjectName(QStringLiteral("lineEdit_cameraStereoVideo_path"));

        gridLayout_66->addWidget(lineEdit_cameraStereoVideo_path, 0, 1, 1, 1);

        verticalSpacer_39 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_66->addItem(verticalSpacer_39, 2, 0, 1, 1);

        label_339 = new QLabel(groupBox_cameraStereoVideo);
        label_339->setObjectName(QStringLiteral("label_339"));
        label_339->setWordWrap(true);
        label_339->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_66->addWidget(label_339, 0, 2, 1, 1);

        label_340 = new QLabel(groupBox_cameraStereoVideo);
        label_340->setObjectName(QStringLiteral("label_340"));
        label_340->setWordWrap(true);
        label_340->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_66->addWidget(label_340, 1, 2, 1, 1);

        lineEdit_cameraStereoVideo_path_2 = new QLineEdit(groupBox_cameraStereoVideo);
        lineEdit_cameraStereoVideo_path_2->setObjectName(QStringLiteral("lineEdit_cameraStereoVideo_path_2"));

        gridLayout_66->addWidget(lineEdit_cameraStereoVideo_path_2, 1, 1, 1, 1);

        toolButton_cameraStereoVideo_path_2 = new QToolButton(groupBox_cameraStereoVideo);
        toolButton_cameraStereoVideo_path_2->setObjectName(QStringLiteral("toolButton_cameraStereoVideo_path_2"));

        gridLayout_66->addWidget(toolButton_cameraStereoVideo_path_2, 1, 0, 1, 1);

        gridLayout_66->setColumnStretch(2, 1);

        verticalLayout_78->addWidget(groupBox_cameraStereoVideo);

        stackedWidget_stereo->addWidget(page_50);
        page_59 = new QWidget();
        page_59->setObjectName(QStringLiteral("page_59"));
        verticalLayout_101 = new QVBoxLayout(page_59);
        verticalLayout_101->setObjectName(QStringLiteral("verticalLayout_101"));
        groupBox_cameraStereoZed = new QGroupBox(page_59);
        groupBox_cameraStereoZed->setObjectName(QStringLiteral("groupBox_cameraStereoZed"));
        sizePolicy2.setHeightForWidth(groupBox_cameraStereoZed->sizePolicy().hasHeightForWidth());
        groupBox_cameraStereoZed->setSizePolicy(sizePolicy2);
        gridLayout_37 = new QGridLayout(groupBox_cameraStereoZed);
        gridLayout_37->setObjectName(QStringLiteral("gridLayout_37"));
        verticalSpacer_54 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_37->addItem(verticalSpacer_54, 8, 0, 1, 1);

        label_305 = new QLabel(groupBox_cameraStereoZed);
        label_305->setObjectName(QStringLiteral("label_305"));
        label_305->setWordWrap(true);
        label_305->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_37->addWidget(label_305, 7, 1, 1, 1);

        comboBox_stereoZed_resolution = new QComboBox(groupBox_cameraStereoZed);
        comboBox_stereoZed_resolution->setObjectName(QStringLiteral("comboBox_stereoZed_resolution"));

        gridLayout_37->addWidget(comboBox_stereoZed_resolution, 0, 0, 1, 1);

        label_291 = new QLabel(groupBox_cameraStereoZed);
        label_291->setObjectName(QStringLiteral("label_291"));
        label_291->setWordWrap(true);
        label_291->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_37->addWidget(label_291, 3, 1, 1, 1);

        comboBox_stereoZed_sensingMode = new QComboBox(groupBox_cameraStereoZed);
        comboBox_stereoZed_sensingMode->setObjectName(QStringLiteral("comboBox_stereoZed_sensingMode"));

        gridLayout_37->addWidget(comboBox_stereoZed_sensingMode, 3, 0, 1, 1);

        comboBox_stereoZed_quality = new QComboBox(groupBox_cameraStereoZed);
        comboBox_stereoZed_quality->setObjectName(QStringLiteral("comboBox_stereoZed_quality"));

        gridLayout_37->addWidget(comboBox_stereoZed_quality, 1, 0, 1, 1);

        label_306 = new QLabel(groupBox_cameraStereoZed);
        label_306->setObjectName(QStringLiteral("label_306"));
        label_306->setWordWrap(true);
        label_306->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_37->addWidget(label_306, 4, 1, 1, 1);

        spinBox_stereoZed_confidenceThr = new QSpinBox(groupBox_cameraStereoZed);
        spinBox_stereoZed_confidenceThr->setObjectName(QStringLiteral("spinBox_stereoZed_confidenceThr"));
        spinBox_stereoZed_confidenceThr->setMaximum(100);

        gridLayout_37->addWidget(spinBox_stereoZed_confidenceThr, 4, 0, 1, 1);

        label_274 = new QLabel(groupBox_cameraStereoZed);
        label_274->setObjectName(QStringLiteral("label_274"));
        label_274->setWordWrap(true);
        label_274->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_37->addWidget(label_274, 0, 1, 1, 1);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        toolButton_zedSvoPath = new QToolButton(groupBox_cameraStereoZed);
        toolButton_zedSvoPath->setObjectName(QStringLiteral("toolButton_zedSvoPath"));

        horizontalLayout_8->addWidget(toolButton_zedSvoPath);

        lineEdit_zedSvoPath = new QLineEdit(groupBox_cameraStereoZed);
        lineEdit_zedSvoPath->setObjectName(QStringLiteral("lineEdit_zedSvoPath"));

        horizontalLayout_8->addWidget(lineEdit_zedSvoPath);


        gridLayout_37->addLayout(horizontalLayout_8, 7, 0, 1, 1);

        label_290 = new QLabel(groupBox_cameraStereoZed);
        label_290->setObjectName(QStringLiteral("label_290"));
        label_290->setWordWrap(true);
        label_290->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_37->addWidget(label_290, 1, 1, 1, 1);

        checkbox_stereoZed_odom = new QCheckBox(groupBox_cameraStereoZed);
        checkbox_stereoZed_odom->setObjectName(QStringLiteral("checkbox_stereoZed_odom"));

        gridLayout_37->addWidget(checkbox_stereoZed_odom, 6, 0, 1, 1);

        label_316 = new QLabel(groupBox_cameraStereoZed);
        label_316->setObjectName(QStringLiteral("label_316"));
        label_316->setWordWrap(true);
        label_316->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_37->addWidget(label_316, 6, 1, 1, 1);

        label_343 = new QLabel(groupBox_cameraStereoZed);
        label_343->setObjectName(QStringLiteral("label_343"));
        label_343->setWordWrap(true);
        label_343->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_37->addWidget(label_343, 2, 1, 1, 1);

        checkbox_stereoZed_selfCalibration = new QCheckBox(groupBox_cameraStereoZed);
        checkbox_stereoZed_selfCalibration->setObjectName(QStringLiteral("checkbox_stereoZed_selfCalibration"));

        gridLayout_37->addWidget(checkbox_stereoZed_selfCalibration, 2, 0, 1, 1);

        label_570 = new QLabel(groupBox_cameraStereoZed);
        label_570->setObjectName(QStringLiteral("label_570"));
        label_570->setWordWrap(true);
        label_570->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_37->addWidget(label_570, 5, 1, 1, 1);

        spinBox_stereoZed_texturenessConfidenceThr = new QSpinBox(groupBox_cameraStereoZed);
        spinBox_stereoZed_texturenessConfidenceThr->setObjectName(QStringLiteral("spinBox_stereoZed_texturenessConfidenceThr"));
        spinBox_stereoZed_texturenessConfidenceThr->setMaximum(100);

        gridLayout_37->addWidget(spinBox_stereoZed_texturenessConfidenceThr, 5, 0, 1, 1);

        gridLayout_37->setColumnStretch(1, 1);

        verticalLayout_101->addWidget(groupBox_cameraStereoZed);

        stackedWidget_stereo->addWidget(page_59);
        page_60 = new QWidget();
        page_60->setObjectName(QStringLiteral("page_60"));
        verticalLayout_86 = new QVBoxLayout(page_60);
        verticalLayout_86->setObjectName(QStringLiteral("verticalLayout_86"));
        groupBox_cameraStereoUsb = new QGroupBox(page_60);
        groupBox_cameraStereoUsb->setObjectName(QStringLiteral("groupBox_cameraStereoUsb"));
        sizePolicy2.setHeightForWidth(groupBox_cameraStereoUsb->sizePolicy().hasHeightForWidth());
        groupBox_cameraStereoUsb->setSizePolicy(sizePolicy2);
        gridLayout_99 = new QGridLayout(groupBox_cameraStereoUsb);
        gridLayout_99->setObjectName(QStringLiteral("gridLayout_99"));
        label_470 = new QLabel(groupBox_cameraStereoUsb);
        label_470->setObjectName(QStringLiteral("label_470"));
        label_470->setWordWrap(true);
        label_470->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_99->addWidget(label_470, 0, 1, 1, 1);

        spinBox_stereousbcam_streamHeight = new QSpinBox(groupBox_cameraStereoUsb);
        spinBox_stereousbcam_streamHeight->setObjectName(QStringLiteral("spinBox_stereousbcam_streamHeight"));
        spinBox_stereousbcam_streamHeight->setMinimum(0);
        spinBox_stereousbcam_streamHeight->setMaximum(10000);

        gridLayout_99->addWidget(spinBox_stereousbcam_streamHeight, 3, 0, 1, 1);

        label_620 = new QLabel(groupBox_cameraStereoUsb);
        label_620->setObjectName(QStringLiteral("label_620"));
        label_620->setWordWrap(true);
        label_620->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_99->addWidget(label_620, 1, 1, 1, 1);

        label_619 = new QLabel(groupBox_cameraStereoUsb);
        label_619->setObjectName(QStringLiteral("label_619"));
        label_619->setWordWrap(true);
        label_619->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_99->addWidget(label_619, 3, 1, 1, 1);

        spinBox_stereo_right_device = new QSpinBox(groupBox_cameraStereoUsb);
        spinBox_stereo_right_device->setObjectName(QStringLiteral("spinBox_stereo_right_device"));
        spinBox_stereo_right_device->setMinimum(-1);

        gridLayout_99->addWidget(spinBox_stereo_right_device, 0, 0, 1, 1);

        spinBox_stereousbcam_streamWidth = new QSpinBox(groupBox_cameraStereoUsb);
        spinBox_stereousbcam_streamWidth->setObjectName(QStringLiteral("spinBox_stereousbcam_streamWidth"));
        spinBox_stereousbcam_streamWidth->setMinimum(0);
        spinBox_stereousbcam_streamWidth->setMaximum(10000);

        gridLayout_99->addWidget(spinBox_stereousbcam_streamWidth, 1, 0, 1, 1);

        verticalSpacer_51 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_99->addItem(verticalSpacer_51, 4, 0, 1, 1);

        gridLayout_99->setColumnStretch(1, 1);

        verticalLayout_86->addWidget(groupBox_cameraStereoUsb);

        stackedWidget_stereo->addWidget(page_60);
        page_77 = new QWidget();
        page_77->setObjectName(QStringLiteral("page_77"));
        verticalLayout_135 = new QVBoxLayout(page_77);
        verticalLayout_135->setObjectName(QStringLiteral("verticalLayout_135"));
        stackedWidget_stereo->addWidget(page_77);
        page_80 = new QWidget();
        page_80->setObjectName(QStringLiteral("page_80"));
        verticalLayout_140 = new QVBoxLayout(page_80);
        verticalLayout_140->setObjectName(QStringLiteral("verticalLayout_140"));
        groupBox_cameraStereoRealSense2 = new QGroupBox(page_80);
        groupBox_cameraStereoRealSense2->setObjectName(QStringLiteral("groupBox_cameraStereoRealSense2"));
        sizePolicy2.setHeightForWidth(groupBox_cameraStereoRealSense2->sizePolicy().hasHeightForWidth());
        groupBox_cameraStereoRealSense2->setSizePolicy(sizePolicy2);
        gridLayout_108 = new QGridLayout(groupBox_cameraStereoRealSense2);
        gridLayout_108->setObjectName(QStringLiteral("gridLayout_108"));
        checkbox_stereoRealSense2_odom = new QCheckBox(groupBox_cameraStereoRealSense2);
        checkbox_stereoRealSense2_odom->setObjectName(QStringLiteral("checkbox_stereoRealSense2_odom"));

        gridLayout_108->addWidget(checkbox_stereoRealSense2_odom, 0, 0, 1, 1);

        label_546 = new QLabel(groupBox_cameraStereoRealSense2);
        label_546->setObjectName(QStringLiteral("label_546"));
        label_546->setWordWrap(true);
        label_546->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_108->addWidget(label_546, 0, 1, 1, 1);

        gridLayout_108->setColumnStretch(1, 1);

        verticalLayout_140->addWidget(groupBox_cameraStereoRealSense2);

        verticalSpacer_78 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_140->addItem(verticalSpacer_78);

        stackedWidget_stereo->addWidget(page_80);
        page_86 = new QWidget();
        page_86->setObjectName(QStringLiteral("page_86"));
        verticalLayout_148 = new QVBoxLayout(page_86);
        verticalLayout_148->setObjectName(QStringLiteral("verticalLayout_148"));
        groupBox_cameraMyntEye = new QGroupBox(page_86);
        groupBox_cameraMyntEye->setObjectName(QStringLiteral("groupBox_cameraMyntEye"));
        sizePolicy2.setHeightForWidth(groupBox_cameraMyntEye->sizePolicy().hasHeightForWidth());
        groupBox_cameraMyntEye->setSizePolicy(sizePolicy2);
        gridLayout_115 = new QGridLayout(groupBox_cameraMyntEye);
        gridLayout_115->setObjectName(QStringLiteral("gridLayout_115"));
        checkbox_stereoMyntEye_depth = new QCheckBox(groupBox_cameraMyntEye);
        checkbox_stereoMyntEye_depth->setObjectName(QStringLiteral("checkbox_stereoMyntEye_depth"));

        gridLayout_115->addWidget(checkbox_stereoMyntEye_depth, 1, 0, 1, 1);

        label_576 = new QLabel(groupBox_cameraMyntEye);
        label_576->setObjectName(QStringLiteral("label_576"));
        label_576->setWordWrap(true);
        label_576->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_115->addWidget(label_576, 1, 1, 1, 1);

        label_581 = new QLabel(groupBox_cameraMyntEye);
        label_581->setObjectName(QStringLiteral("label_581"));
        label_581->setWordWrap(true);
        label_581->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_115->addWidget(label_581, 0, 1, 1, 1);

        checkbox_stereoMyntEye_rectify = new QCheckBox(groupBox_cameraMyntEye);
        checkbox_stereoMyntEye_rectify->setObjectName(QStringLiteral("checkbox_stereoMyntEye_rectify"));

        gridLayout_115->addWidget(checkbox_stereoMyntEye_rectify, 0, 0, 1, 1);

        spinBox_stereoMyntEye_brightness = new QSpinBox(groupBox_cameraMyntEye);
        spinBox_stereoMyntEye_brightness->setObjectName(QStringLiteral("spinBox_stereoMyntEye_brightness"));
        spinBox_stereoMyntEye_brightness->setMaximum(240);
        spinBox_stereoMyntEye_brightness->setValue(120);

        gridLayout_115->addWidget(spinBox_stereoMyntEye_brightness, 4, 0, 1, 1);

        spinBox_stereoMyntEye_gain = new QSpinBox(groupBox_cameraMyntEye);
        spinBox_stereoMyntEye_gain->setObjectName(QStringLiteral("spinBox_stereoMyntEye_gain"));
        spinBox_stereoMyntEye_gain->setMaximum(48);
        spinBox_stereoMyntEye_gain->setValue(24);

        gridLayout_115->addWidget(spinBox_stereoMyntEye_gain, 3, 0, 1, 1);

        checkbox_stereoMyntEye_autoExposure = new QCheckBox(groupBox_cameraMyntEye);
        checkbox_stereoMyntEye_autoExposure->setObjectName(QStringLiteral("checkbox_stereoMyntEye_autoExposure"));
        checkbox_stereoMyntEye_autoExposure->setChecked(true);

        gridLayout_115->addWidget(checkbox_stereoMyntEye_autoExposure, 2, 0, 1, 1);

        label_595 = new QLabel(groupBox_cameraMyntEye);
        label_595->setObjectName(QStringLiteral("label_595"));
        label_595->setWordWrap(true);
        label_595->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_115->addWidget(label_595, 3, 1, 1, 1);

        label_596 = new QLabel(groupBox_cameraMyntEye);
        label_596->setObjectName(QStringLiteral("label_596"));
        label_596->setWordWrap(true);
        label_596->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_115->addWidget(label_596, 4, 1, 1, 1);

        label_594 = new QLabel(groupBox_cameraMyntEye);
        label_594->setObjectName(QStringLiteral("label_594"));
        label_594->setWordWrap(true);
        label_594->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_115->addWidget(label_594, 2, 1, 1, 1);

        label_597 = new QLabel(groupBox_cameraMyntEye);
        label_597->setObjectName(QStringLiteral("label_597"));
        label_597->setWordWrap(true);
        label_597->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_115->addWidget(label_597, 5, 1, 1, 1);

        spinBox_stereoMyntEye_contrast = new QSpinBox(groupBox_cameraMyntEye);
        spinBox_stereoMyntEye_contrast->setObjectName(QStringLiteral("spinBox_stereoMyntEye_contrast"));
        spinBox_stereoMyntEye_contrast->setMaximum(254);
        spinBox_stereoMyntEye_contrast->setValue(116);

        gridLayout_115->addWidget(spinBox_stereoMyntEye_contrast, 5, 0, 1, 1);

        label_599 = new QLabel(groupBox_cameraMyntEye);
        label_599->setObjectName(QStringLiteral("label_599"));
        label_599->setWordWrap(true);
        label_599->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_115->addWidget(label_599, 6, 1, 1, 1);

        spinBox_stereoMyntEye_irControl = new QSpinBox(groupBox_cameraMyntEye);
        spinBox_stereoMyntEye_irControl->setObjectName(QStringLiteral("spinBox_stereoMyntEye_irControl"));
        spinBox_stereoMyntEye_irControl->setMaximum(160);
        spinBox_stereoMyntEye_irControl->setValue(116);

        gridLayout_115->addWidget(spinBox_stereoMyntEye_irControl, 6, 0, 1, 1);

        gridLayout_115->setColumnStretch(1, 1);

        verticalLayout_148->addWidget(groupBox_cameraMyntEye);

        verticalSpacer_85 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_148->addItem(verticalSpacer_85);

        stackedWidget_stereo->addWidget(page_86);
        page_88 = new QWidget();
        page_88->setObjectName(QStringLiteral("page_88"));
        verticalLayout_154 = new QVBoxLayout(page_88);
        verticalLayout_154->setObjectName(QStringLiteral("verticalLayout_154"));
        groupBox_cameraStereoZedOC = new QGroupBox(page_88);
        groupBox_cameraStereoZedOC->setObjectName(QStringLiteral("groupBox_cameraStereoZedOC"));
        sizePolicy2.setHeightForWidth(groupBox_cameraStereoZedOC->sizePolicy().hasHeightForWidth());
        groupBox_cameraStereoZedOC->setSizePolicy(sizePolicy2);
        gridLayout_119 = new QGridLayout(groupBox_cameraStereoZedOC);
        gridLayout_119->setObjectName(QStringLiteral("gridLayout_119"));
        comboBox_stereoZedOC_resolution = new QComboBox(groupBox_cameraStereoZedOC);
        comboBox_stereoZedOC_resolution->setObjectName(QStringLiteral("comboBox_stereoZedOC_resolution"));

        gridLayout_119->addWidget(comboBox_stereoZedOC_resolution, 0, 0, 1, 1);

        label_624 = new QLabel(groupBox_cameraStereoZedOC);
        label_624->setObjectName(QStringLiteral("label_624"));
        label_624->setWordWrap(true);
        label_624->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_119->addWidget(label_624, 0, 1, 1, 1);

        gridLayout_119->setColumnStretch(1, 1);

        verticalLayout_154->addWidget(groupBox_cameraStereoZedOC);

        verticalSpacer_70 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_154->addItem(verticalSpacer_70);

        stackedWidget_stereo->addWidget(page_88);
        page_89 = new QWidget();
        page_89->setObjectName(QStringLiteral("page_89"));
        verticalLayout_155 = new QVBoxLayout(page_89);
        verticalLayout_155->setObjectName(QStringLiteral("verticalLayout_155"));
        groupBox_cameraDepthAI = new QGroupBox(page_89);
        groupBox_cameraDepthAI->setObjectName(QStringLiteral("groupBox_cameraDepthAI"));
        sizePolicy2.setHeightForWidth(groupBox_cameraDepthAI->sizePolicy().hasHeightForWidth());
        groupBox_cameraDepthAI->setSizePolicy(sizePolicy2);
        gridLayout_120 = new QGridLayout(groupBox_cameraDepthAI);
        gridLayout_120->setObjectName(QStringLiteral("gridLayout_120"));
        label_626 = new QLabel(groupBox_cameraDepthAI);
        label_626->setObjectName(QStringLiteral("label_626"));
        label_626->setWordWrap(true);
        label_626->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_120->addWidget(label_626, 0, 1, 1, 1);

        label_625 = new QLabel(groupBox_cameraDepthAI);
        label_625->setObjectName(QStringLiteral("label_625"));
        label_625->setWordWrap(true);
        label_625->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_120->addWidget(label_625, 1, 1, 1, 1);

        checkBox_depthai_depth = new QCheckBox(groupBox_cameraDepthAI);
        checkBox_depthai_depth->setObjectName(QStringLiteral("checkBox_depthai_depth"));

        gridLayout_120->addWidget(checkBox_depthai_depth, 1, 0, 1, 1);

        comboBox_depthai_resolution = new QComboBox(groupBox_cameraDepthAI);
        comboBox_depthai_resolution->setObjectName(QStringLiteral("comboBox_depthai_resolution"));
        comboBox_depthai_resolution->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_120->addWidget(comboBox_depthai_resolution, 0, 0, 1, 1);

        label_627 = new QLabel(groupBox_cameraDepthAI);
        label_627->setObjectName(QStringLiteral("label_627"));
        label_627->setWordWrap(true);
        label_627->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_120->addWidget(label_627, 2, 1, 1, 1);

        spinBox_depthai_confidence = new QSpinBox(groupBox_cameraDepthAI);
        spinBox_depthai_confidence->setObjectName(QStringLiteral("spinBox_depthai_confidence"));
        spinBox_depthai_confidence->setMinimum(0);
        spinBox_depthai_confidence->setMaximum(255);
        spinBox_depthai_confidence->setValue(200);

        gridLayout_120->addWidget(spinBox_depthai_confidence, 2, 0, 1, 1);

        gridLayout_120->setColumnStretch(1, 1);

        verticalLayout_155->addWidget(groupBox_cameraDepthAI);

        verticalSpacer_72 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_155->addItem(verticalSpacer_72);

        stackedWidget_stereo->addWidget(page_89);

        verticalLayout_67->addWidget(stackedWidget_stereo);


        verticalLayout_71->addWidget(groupBox_sourceStereo);

        stackedWidget_src->addWidget(page_44);
        page_42 = new QWidget();
        page_42->setObjectName(QStringLiteral("page_42"));
        verticalLayout_65 = new QVBoxLayout(page_42);
        verticalLayout_65->setObjectName(QStringLiteral("verticalLayout_65"));
        groupBox_sourceRGB = new QGroupBox(page_42);
        groupBox_sourceRGB->setObjectName(QStringLiteral("groupBox_sourceRGB"));
        groupBox_sourceRGB->setCheckable(false);
        groupBox_sourceRGB->setChecked(false);
        verticalLayout_5 = new QVBoxLayout(groupBox_sourceRGB);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        gridLayout_5 = new QGridLayout();
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        label_20 = new QLabel(groupBox_sourceRGB);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setWordWrap(true);
        label_20->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_5->addWidget(label_20, 0, 1, 1, 1);

        source_comboBox_image_type = new QComboBox(groupBox_sourceRGB);
        source_comboBox_image_type->setObjectName(QStringLiteral("source_comboBox_image_type"));

        gridLayout_5->addWidget(source_comboBox_image_type, 0, 0, 1, 1);

        label_242 = new QLabel(groupBox_sourceRGB);
        label_242->setObjectName(QStringLiteral("label_242"));
        label_242->setWordWrap(true);
        label_242->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_5->addWidget(label_242, 1, 1, 1, 1);

        checkBox_rgb_rectify = new QCheckBox(groupBox_sourceRGB);
        checkBox_rgb_rectify->setObjectName(QStringLiteral("checkBox_rgb_rectify"));

        gridLayout_5->addWidget(checkBox_rgb_rectify, 1, 0, 1, 1);

        gridLayout_5->setColumnStretch(1, 1);

        verticalLayout_5->addLayout(gridLayout_5);

        stackedWidget_image = new QStackedWidget(groupBox_sourceRGB);
        stackedWidget_image->setObjectName(QStringLiteral("stackedWidget_image"));
        page_7 = new QWidget();
        page_7->setObjectName(QStringLiteral("page_7"));
        verticalLayout_30 = new QVBoxLayout(page_7);
        verticalLayout_30->setObjectName(QStringLiteral("verticalLayout_30"));
        source_groupBox_usbcam = new QGroupBox(page_7);
        source_groupBox_usbcam->setObjectName(QStringLiteral("source_groupBox_usbcam"));
        gridLayout_112 = new QGridLayout(source_groupBox_usbcam);
        gridLayout_112->setObjectName(QStringLiteral("gridLayout_112"));
        spinBox_usbcam_streamHeight = new QSpinBox(source_groupBox_usbcam);
        spinBox_usbcam_streamHeight->setObjectName(QStringLiteral("spinBox_usbcam_streamHeight"));
        spinBox_usbcam_streamHeight->setMinimum(0);
        spinBox_usbcam_streamHeight->setMaximum(10000);

        gridLayout_112->addWidget(spinBox_usbcam_streamHeight, 1, 0, 1, 1);

        label_553 = new QLabel(source_groupBox_usbcam);
        label_553->setObjectName(QStringLiteral("label_553"));
        label_553->setWordWrap(true);
        label_553->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_112->addWidget(label_553, 1, 1, 1, 1);

        label_552 = new QLabel(source_groupBox_usbcam);
        label_552->setObjectName(QStringLiteral("label_552"));
        label_552->setWordWrap(true);
        label_552->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_112->addWidget(label_552, 0, 1, 1, 1);

        spinBox_usbcam_streamWidth = new QSpinBox(source_groupBox_usbcam);
        spinBox_usbcam_streamWidth->setObjectName(QStringLiteral("spinBox_usbcam_streamWidth"));
        spinBox_usbcam_streamWidth->setMinimum(0);
        spinBox_usbcam_streamWidth->setMaximum(10000);

        gridLayout_112->addWidget(spinBox_usbcam_streamWidth, 0, 0, 1, 1);

        gridLayout_112->setColumnStretch(1, 1);

        verticalLayout_30->addWidget(source_groupBox_usbcam);

        verticalSpacer_8 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_30->addItem(verticalSpacer_8);

        stackedWidget_image->addWidget(page_7);
        page_5 = new QWidget();
        page_5->setObjectName(QStringLiteral("page_5"));
        verticalLayout_6 = new QVBoxLayout(page_5);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        source_groupBox_images = new QGroupBox(page_5);
        source_groupBox_images->setObjectName(QStringLiteral("source_groupBox_images"));
        gridLayout_7 = new QGridLayout(source_groupBox_images);
        gridLayout_7->setObjectName(QStringLiteral("gridLayout_7"));
        source_images_lineEdit_path = new QLineEdit(source_groupBox_images);
        source_images_lineEdit_path->setObjectName(QStringLiteral("source_images_lineEdit_path"));
        source_images_lineEdit_path->setReadOnly(false);

        gridLayout_7->addWidget(source_images_lineEdit_path, 0, 1, 1, 1);

        label_21 = new QLabel(source_groupBox_images);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setWordWrap(true);
        label_21->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_7->addWidget(label_21, 1, 1, 1, 1);

        source_images_spinBox_startPos = new QSpinBox(source_groupBox_images);
        source_images_spinBox_startPos->setObjectName(QStringLiteral("source_images_spinBox_startPos"));
        source_images_spinBox_startPos->setMinimum(0);
        source_images_spinBox_startPos->setMaximum(999999999);

        gridLayout_7->addWidget(source_images_spinBox_startPos, 1, 0, 1, 1);

        source_images_toolButton_selectSource = new QToolButton(source_groupBox_images);
        source_images_toolButton_selectSource->setObjectName(QStringLiteral("source_images_toolButton_selectSource"));

        gridLayout_7->addWidget(source_images_toolButton_selectSource, 0, 0, 1, 1);

        label_34 = new QLabel(source_groupBox_images);
        label_34->setObjectName(QStringLiteral("label_34"));
        label_34->setWordWrap(true);
        label_34->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_7->addWidget(label_34, 2, 1, 1, 1);

        source_images_spinBox_maxFrames = new QSpinBox(source_groupBox_images);
        source_images_spinBox_maxFrames->setObjectName(QStringLiteral("source_images_spinBox_maxFrames"));
        source_images_spinBox_maxFrames->setMinimum(0);
        source_images_spinBox_maxFrames->setMaximum(999999999);

        gridLayout_7->addWidget(source_images_spinBox_maxFrames, 2, 0, 1, 1);


        verticalLayout_6->addWidget(source_groupBox_images);

        verticalSpacer_9 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_6->addItem(verticalSpacer_9);

        stackedWidget_image->addWidget(page_5);
        page_6 = new QWidget();
        page_6->setObjectName(QStringLiteral("page_6"));
        verticalLayout_8 = new QVBoxLayout(page_6);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        source_groupBox_video = new QGroupBox(page_6);
        source_groupBox_video->setObjectName(QStringLiteral("source_groupBox_video"));
        gridLayout_8 = new QGridLayout(source_groupBox_video);
        gridLayout_8->setObjectName(QStringLiteral("gridLayout_8"));
        source_video_lineEdit_path = new QLineEdit(source_groupBox_video);
        source_video_lineEdit_path->setObjectName(QStringLiteral("source_video_lineEdit_path"));
        source_video_lineEdit_path->setReadOnly(false);

        gridLayout_8->addWidget(source_video_lineEdit_path, 0, 1, 1, 1);

        source_video_toolButton_selectSource = new QToolButton(source_groupBox_video);
        source_video_toolButton_selectSource->setObjectName(QStringLiteral("source_video_toolButton_selectSource"));

        gridLayout_8->addWidget(source_video_toolButton_selectSource, 0, 0, 1, 1);


        verticalLayout_8->addWidget(source_groupBox_video);

        verticalSpacer_10 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_8->addItem(verticalSpacer_10);

        stackedWidget_image->addWidget(page_6);

        verticalLayout_5->addWidget(stackedWidget_image);


        verticalLayout_65->addWidget(groupBox_sourceRGB);

        stackedWidget_src->addWidget(page_42);
        page_43 = new QWidget();
        page_43->setObjectName(QStringLiteral("page_43"));
        verticalLayout_66 = new QVBoxLayout(page_43);
        verticalLayout_66->setObjectName(QStringLiteral("verticalLayout_66"));
        groupBox_sourceDatabase = new QGroupBox(page_43);
        groupBox_sourceDatabase->setObjectName(QStringLiteral("groupBox_sourceDatabase"));
        groupBox_sourceDatabase->setCheckable(false);
        groupBox_sourceDatabase->setChecked(false);
        gridLayout_9 = new QGridLayout(groupBox_sourceDatabase);
        gridLayout_9->setObjectName(QStringLiteral("gridLayout_9"));
        label_263 = new QLabel(groupBox_sourceDatabase);
        label_263->setObjectName(QStringLiteral("label_263"));
        label_263->setWordWrap(true);
        label_263->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_9->addWidget(label_263, 3, 1, 1, 1);

        source_checkBox_ignoreGoals = new QCheckBox(groupBox_sourceDatabase);
        source_checkBox_ignoreGoals->setObjectName(QStringLiteral("source_checkBox_ignoreGoals"));

        gridLayout_9->addWidget(source_checkBox_ignoreGoals, 3, 0, 1, 1);

        source_checkBox_ignoreOdometry = new QCheckBox(groupBox_sourceDatabase);
        source_checkBox_ignoreOdometry->setObjectName(QStringLiteral("source_checkBox_ignoreOdometry"));

        gridLayout_9->addWidget(source_checkBox_ignoreOdometry, 1, 0, 1, 1);

        toolButton_dbViewer = new QToolButton(groupBox_sourceDatabase);
        toolButton_dbViewer->setObjectName(QStringLiteral("toolButton_dbViewer"));
        QIcon icon1;
        icon1.addFile(QStringLiteral(":/images/mag_glass.png"), QSize(), QIcon::Normal, QIcon::Off);
        toolButton_dbViewer->setIcon(icon1);

        gridLayout_9->addWidget(toolButton_dbViewer, 0, 2, 1, 1);

        source_database_toolButton_selectSource = new QToolButton(groupBox_sourceDatabase);
        source_database_toolButton_selectSource->setObjectName(QStringLiteral("source_database_toolButton_selectSource"));

        gridLayout_9->addWidget(source_database_toolButton_selectSource, 0, 0, 1, 1);

        source_database_lineEdit_path = new QLineEdit(groupBox_sourceDatabase);
        source_database_lineEdit_path->setObjectName(QStringLiteral("source_database_lineEdit_path"));

        gridLayout_9->addWidget(source_database_lineEdit_path, 0, 1, 1, 1);

        verticalSpacer_35 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_9->addItem(verticalSpacer_35, 9, 1, 1, 1);

        label_58 = new QLabel(groupBox_sourceDatabase);
        label_58->setObjectName(QStringLiteral("label_58"));
        label_58->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_9->addWidget(label_58, 5, 1, 1, 1);

        source_spinBox_databaseStartId = new QSpinBox(groupBox_sourceDatabase);
        source_spinBox_databaseStartId->setObjectName(QStringLiteral("source_spinBox_databaseStartId"));
        source_spinBox_databaseStartId->setMinimum(0);
        source_spinBox_databaseStartId->setMaximum(99999);

        gridLayout_9->addWidget(source_spinBox_databaseStartId, 5, 0, 1, 1);

        label_72 = new QLabel(groupBox_sourceDatabase);
        label_72->setObjectName(QStringLiteral("label_72"));
        label_72->setWordWrap(true);
        label_72->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_9->addWidget(label_72, 1, 1, 1, 1);

        label_80 = new QLabel(groupBox_sourceDatabase);
        label_80->setObjectName(QStringLiteral("label_80"));
        label_80->setWordWrap(true);
        label_80->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_9->addWidget(label_80, 4, 1, 1, 1);

        source_checkBox_ignoreGoalDelay = new QCheckBox(groupBox_sourceDatabase);
        source_checkBox_ignoreGoalDelay->setObjectName(QStringLiteral("source_checkBox_ignoreGoalDelay"));

        gridLayout_9->addWidget(source_checkBox_ignoreGoalDelay, 4, 0, 1, 1);

        source_spinBox_database_cameraIndex = new QSpinBox(groupBox_sourceDatabase);
        source_spinBox_database_cameraIndex->setObjectName(QStringLiteral("source_spinBox_database_cameraIndex"));
        source_spinBox_database_cameraIndex->setMinimum(-1);
        source_spinBox_database_cameraIndex->setMaximum(9999);

        gridLayout_9->addWidget(source_spinBox_database_cameraIndex, 7, 0, 1, 1);

        label_90 = new QLabel(groupBox_sourceDatabase);
        label_90->setObjectName(QStringLiteral("label_90"));
        label_90->setWordWrap(true);
        label_90->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_9->addWidget(label_90, 2, 1, 1, 1);

        source_checkBox_useDbStamps = new QCheckBox(groupBox_sourceDatabase);
        source_checkBox_useDbStamps->setObjectName(QStringLiteral("source_checkBox_useDbStamps"));

        gridLayout_9->addWidget(source_checkBox_useDbStamps, 2, 0, 1, 1);

        label_315 = new QLabel(groupBox_sourceDatabase);
        label_315->setObjectName(QStringLiteral("label_315"));
        label_315->setWordWrap(true);
        label_315->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_9->addWidget(label_315, 7, 1, 1, 1);

        label_529 = new QLabel(groupBox_sourceDatabase);
        label_529->setObjectName(QStringLiteral("label_529"));
        label_529->setWordWrap(true);
        label_529->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_9->addWidget(label_529, 6, 1, 1, 1);

        source_spinBox_databaseStopId = new QSpinBox(groupBox_sourceDatabase);
        source_spinBox_databaseStopId->setObjectName(QStringLiteral("source_spinBox_databaseStopId"));
        source_spinBox_databaseStopId->setMinimum(0);
        source_spinBox_databaseStopId->setMaximum(99999);

        gridLayout_9->addWidget(source_spinBox_databaseStopId, 6, 0, 1, 1);

        label_557 = new QLabel(groupBox_sourceDatabase);
        label_557->setObjectName(QStringLiteral("label_557"));
        label_557->setWordWrap(true);
        label_557->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_9->addWidget(label_557, 8, 1, 1, 1);

        source_checkBox_stereoToDepthDB = new QCheckBox(groupBox_sourceDatabase);
        source_checkBox_stereoToDepthDB->setObjectName(QStringLiteral("source_checkBox_stereoToDepthDB"));

        gridLayout_9->addWidget(source_checkBox_stereoToDepthDB, 8, 0, 1, 1);


        verticalLayout_66->addWidget(groupBox_sourceDatabase);

        stackedWidget_src->addWidget(page_43);

        verticalLayout_7->addWidget(stackedWidget_src);

        groupBox_sourceImages_optional = new QGroupBox(groupBox_source0);
        groupBox_sourceImages_optional->setObjectName(QStringLiteral("groupBox_sourceImages_optional"));
        verticalLayout_93 = new QVBoxLayout(groupBox_sourceImages_optional);
        verticalLayout_93->setObjectName(QStringLiteral("verticalLayout_93"));
        verticalLayout_93->setContentsMargins(0, 0, 0, 0);
        gridLayout_68 = new QGridLayout();
        gridLayout_68->setObjectName(QStringLiteral("gridLayout_68"));
        label_288 = new QLabel(groupBox_sourceImages_optional);
        label_288->setObjectName(QStringLiteral("label_288"));
        label_288->setWordWrap(true);
        label_288->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_288, 8, 2, 1, 1);

        label_289 = new QLabel(groupBox_sourceImages_optional);
        label_289->setObjectName(QStringLiteral("label_289"));
        label_289->setWordWrap(true);
        label_289->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_289, 9, 2, 1, 1);

        label_265 = new QLabel(groupBox_sourceImages_optional);
        label_265->setObjectName(QStringLiteral("label_265"));
        label_265->setWordWrap(true);
        label_265->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_265, 0, 2, 1, 1);

        toolButton_cameraImages_gt = new QToolButton(groupBox_sourceImages_optional);
        toolButton_cameraImages_gt->setObjectName(QStringLiteral("toolButton_cameraImages_gt"));

        gridLayout_68->addWidget(toolButton_cameraImages_gt, 8, 0, 1, 1);

        lineEdit_cameraImages_odom = new QLineEdit(groupBox_sourceImages_optional);
        lineEdit_cameraImages_odom->setObjectName(QStringLiteral("lineEdit_cameraImages_odom"));

        gridLayout_68->addWidget(lineEdit_cameraImages_odom, 6, 1, 1, 1);

        label_293 = new QLabel(groupBox_sourceImages_optional);
        label_293->setObjectName(QStringLiteral("label_293"));
        label_293->setWordWrap(true);
        label_293->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_293, 11, 2, 1, 1);

        label_348 = new QLabel(groupBox_sourceImages_optional);
        label_348->setObjectName(QStringLiteral("label_348"));
        label_348->setWordWrap(true);
        label_348->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_348, 6, 2, 1, 1);

        label_255 = new QLabel(groupBox_sourceImages_optional);
        label_255->setObjectName(QStringLiteral("label_255"));
        label_255->setWordWrap(true);
        label_255->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_255, 2, 2, 1, 1);

        toolButton_cameraImages_timestamps = new QToolButton(groupBox_sourceImages_optional);
        toolButton_cameraImages_timestamps->setObjectName(QStringLiteral("toolButton_cameraImages_timestamps"));

        gridLayout_68->addWidget(toolButton_cameraImages_timestamps, 4, 0, 1, 1);

        lineEdit_cameraImages_timestamps = new QLineEdit(groupBox_sourceImages_optional);
        lineEdit_cameraImages_timestamps->setObjectName(QStringLiteral("lineEdit_cameraImages_timestamps"));

        gridLayout_68->addWidget(lineEdit_cameraImages_timestamps, 4, 1, 1, 1);

        lineEdit_cameraImages_gt = new QLineEdit(groupBox_sourceImages_optional);
        lineEdit_cameraImages_gt->setObjectName(QStringLiteral("lineEdit_cameraImages_gt"));

        gridLayout_68->addWidget(lineEdit_cameraImages_gt, 8, 1, 1, 1);

        checkBox_cameraImages_syncTimeStamps = new QCheckBox(groupBox_sourceImages_optional);
        checkBox_cameraImages_syncTimeStamps->setObjectName(QStringLiteral("checkBox_cameraImages_syncTimeStamps"));

        gridLayout_68->addWidget(checkBox_cameraImages_syncTimeStamps, 3, 1, 1, 1);

        checkBox_cameraImages_timestamps = new QCheckBox(groupBox_sourceImages_optional);
        checkBox_cameraImages_timestamps->setObjectName(QStringLiteral("checkBox_cameraImages_timestamps"));

        gridLayout_68->addWidget(checkBox_cameraImages_timestamps, 2, 1, 1, 1);

        label_292 = new QLabel(groupBox_sourceImages_optional);
        label_292->setObjectName(QStringLiteral("label_292"));
        label_292->setWordWrap(true);
        label_292->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_292, 13, 2, 1, 1);

        label_256 = new QLabel(groupBox_sourceImages_optional);
        label_256->setObjectName(QStringLiteral("label_256"));
        label_256->setWordWrap(true);
        label_256->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_256, 3, 2, 1, 1);

        lineEdit_cameraImages_path_scans = new QLineEdit(groupBox_sourceImages_optional);
        lineEdit_cameraImages_path_scans->setObjectName(QStringLiteral("lineEdit_cameraImages_path_scans"));

        gridLayout_68->addWidget(lineEdit_cameraImages_path_scans, 11, 1, 1, 1);

        label_251 = new QLabel(groupBox_sourceImages_optional);
        label_251->setObjectName(QStringLiteral("label_251"));
        label_251->setWordWrap(true);
        label_251->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_251, 4, 2, 1, 1);

        spinBox_cameraImages_max_scan_pts = new QSpinBox(groupBox_sourceImages_optional);
        spinBox_cameraImages_max_scan_pts->setObjectName(QStringLiteral("spinBox_cameraImages_max_scan_pts"));
        spinBox_cameraImages_max_scan_pts->setMaximum(99999999);

        gridLayout_68->addWidget(spinBox_cameraImages_max_scan_pts, 13, 1, 1, 1);

        comboBox_cameraImages_gtFormat = new QComboBox(groupBox_sourceImages_optional);
        comboBox_cameraImages_gtFormat->setObjectName(QStringLiteral("comboBox_cameraImages_gtFormat"));
        comboBox_cameraImages_gtFormat->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_68->addWidget(comboBox_cameraImages_gtFormat, 9, 1, 1, 1);

        toolButton_cameraImages_path_scans = new QToolButton(groupBox_sourceImages_optional);
        toolButton_cameraImages_path_scans->setObjectName(QStringLiteral("toolButton_cameraImages_path_scans"));

        gridLayout_68->addWidget(toolButton_cameraImages_path_scans, 11, 0, 1, 1);

        lineEdit_cameraImages_laser_transform = new QLineEdit(groupBox_sourceImages_optional);
        lineEdit_cameraImages_laser_transform->setObjectName(QStringLiteral("lineEdit_cameraImages_laser_transform"));

        gridLayout_68->addWidget(lineEdit_cameraImages_laser_transform, 12, 1, 1, 1);

        label_294 = new QLabel(groupBox_sourceImages_optional);
        label_294->setObjectName(QStringLiteral("label_294"));
        label_294->setWordWrap(true);
        label_294->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_294, 12, 2, 1, 1);

        comboBox_cameraImages_bayerMode = new QComboBox(groupBox_sourceImages_optional);
        comboBox_cameraImages_bayerMode->setObjectName(QStringLiteral("comboBox_cameraImages_bayerMode"));
        comboBox_cameraImages_bayerMode->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_68->addWidget(comboBox_cameraImages_bayerMode, 0, 1, 1, 1);

        label_349 = new QLabel(groupBox_sourceImages_optional);
        label_349->setObjectName(QStringLiteral("label_349"));
        label_349->setWordWrap(true);
        label_349->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_349, 7, 2, 1, 1);

        comboBox_cameraImages_odomFormat = new QComboBox(groupBox_sourceImages_optional);
        comboBox_cameraImages_odomFormat->setObjectName(QStringLiteral("comboBox_cameraImages_odomFormat"));
        comboBox_cameraImages_odomFormat->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_68->addWidget(comboBox_cameraImages_odomFormat, 7, 1, 1, 1);

        toolButton_cameraImages_odom = new QToolButton(groupBox_sourceImages_optional);
        toolButton_cameraImages_odom->setObjectName(QStringLiteral("toolButton_cameraImages_odom"));

        gridLayout_68->addWidget(toolButton_cameraImages_odom, 6, 0, 1, 1);

        label_443 = new QLabel(groupBox_sourceImages_optional);
        label_443->setObjectName(QStringLiteral("label_443"));
        label_443->setWordWrap(true);
        label_443->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_443, 10, 2, 1, 1);

        doubleSpinBox_maxPoseTimeDiff = new QDoubleSpinBox(groupBox_sourceImages_optional);
        doubleSpinBox_maxPoseTimeDiff->setObjectName(QStringLiteral("doubleSpinBox_maxPoseTimeDiff"));
        doubleSpinBox_maxPoseTimeDiff->setDecimals(4);
        doubleSpinBox_maxPoseTimeDiff->setMaximum(9.99);
        doubleSpinBox_maxPoseTimeDiff->setSingleStep(0.01);
        doubleSpinBox_maxPoseTimeDiff->setValue(0.02);

        gridLayout_68->addWidget(doubleSpinBox_maxPoseTimeDiff, 10, 1, 1, 1);

        label_464 = new QLabel(groupBox_sourceImages_optional);
        label_464->setObjectName(QStringLiteral("label_464"));
        label_464->setWordWrap(true);
        label_464->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_464, 15, 2, 1, 1);

        label_463 = new QLabel(groupBox_sourceImages_optional);
        label_463->setObjectName(QStringLiteral("label_463"));
        label_463->setWordWrap(true);
        label_463->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_463, 14, 2, 1, 1);

        toolButton_cameraImages_path_imu = new QToolButton(groupBox_sourceImages_optional);
        toolButton_cameraImages_path_imu->setObjectName(QStringLiteral("toolButton_cameraImages_path_imu"));

        gridLayout_68->addWidget(toolButton_cameraImages_path_imu, 14, 0, 1, 1);

        lineEdit_cameraImages_path_imu = new QLineEdit(groupBox_sourceImages_optional);
        lineEdit_cameraImages_path_imu->setObjectName(QStringLiteral("lineEdit_cameraImages_path_imu"));

        gridLayout_68->addWidget(lineEdit_cameraImages_path_imu, 14, 1, 1, 1);

        lineEdit_cameraImages_imu_transform = new QLineEdit(groupBox_sourceImages_optional);
        lineEdit_cameraImages_imu_transform->setObjectName(QStringLiteral("lineEdit_cameraImages_imu_transform"));

        gridLayout_68->addWidget(lineEdit_cameraImages_imu_transform, 15, 1, 1, 1);

        label_465 = new QLabel(groupBox_sourceImages_optional);
        label_465->setObjectName(QStringLiteral("label_465"));
        label_465->setWordWrap(true);
        label_465->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_465, 16, 2, 1, 1);

        spinBox_cameraImages_max_imu_rate = new QSpinBox(groupBox_sourceImages_optional);
        spinBox_cameraImages_max_imu_rate->setObjectName(QStringLiteral("spinBox_cameraImages_max_imu_rate"));
        spinBox_cameraImages_max_imu_rate->setMaximum(99999999);

        gridLayout_68->addWidget(spinBox_cameraImages_max_imu_rate, 16, 1, 1, 1);

        label_605 = new QLabel(groupBox_sourceImages_optional);
        label_605->setObjectName(QStringLiteral("label_605"));
        label_605->setWordWrap(true);
        label_605->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_68->addWidget(label_605, 1, 2, 1, 1);

        checkBox_cameraImages_configForEachFrame = new QCheckBox(groupBox_sourceImages_optional);
        checkBox_cameraImages_configForEachFrame->setObjectName(QStringLiteral("checkBox_cameraImages_configForEachFrame"));

        gridLayout_68->addWidget(checkBox_cameraImages_configForEachFrame, 1, 1, 1, 1);

        gridLayout_68->setColumnStretch(2, 1);

        verticalLayout_93->addLayout(gridLayout_68);

        groupBox_depthFromScan = new QGroupBox(groupBox_sourceImages_optional);
        groupBox_depthFromScan->setObjectName(QStringLiteral("groupBox_depthFromScan"));
        groupBox_depthFromScan->setCheckable(true);
        groupBox_depthFromScan->setChecked(false);
        verticalLayout_100 = new QVBoxLayout(groupBox_depthFromScan);
        verticalLayout_100->setObjectName(QStringLiteral("verticalLayout_100"));
        groupBox_depthFromScan_fillHoles = new QGroupBox(groupBox_depthFromScan);
        groupBox_depthFromScan_fillHoles->setObjectName(QStringLiteral("groupBox_depthFromScan_fillHoles"));
        groupBox_depthFromScan_fillHoles->setCheckable(true);
        verticalLayout_99 = new QVBoxLayout(groupBox_depthFromScan_fillHoles);
        verticalLayout_99->setObjectName(QStringLiteral("verticalLayout_99"));
        radioButton_depthFromScan_vertical = new QRadioButton(groupBox_depthFromScan_fillHoles);
        radioButton_depthFromScan_vertical->setObjectName(QStringLiteral("radioButton_depthFromScan_vertical"));
        radioButton_depthFromScan_vertical->setChecked(true);

        verticalLayout_99->addWidget(radioButton_depthFromScan_vertical);

        radioButton_depthFromScan_horizontal = new QRadioButton(groupBox_depthFromScan_fillHoles);
        radioButton_depthFromScan_horizontal->setObjectName(QStringLiteral("radioButton_depthFromScan_horizontal"));

        verticalLayout_99->addWidget(radioButton_depthFromScan_horizontal);


        verticalLayout_100->addWidget(groupBox_depthFromScan_fillHoles);

        checkBox_depthFromScan_fillBorders = new QCheckBox(groupBox_depthFromScan);
        checkBox_depthFromScan_fillBorders->setObjectName(QStringLiteral("checkBox_depthFromScan_fillBorders"));

        verticalLayout_100->addWidget(checkBox_depthFromScan_fillBorders);


        verticalLayout_93->addWidget(groupBox_depthFromScan);


        verticalLayout_7->addWidget(groupBox_sourceImages_optional);

        groupBox_depthImageFiltering = new QGroupBox(groupBox_source0);
        groupBox_depthImageFiltering->setObjectName(QStringLiteral("groupBox_depthImageFiltering"));
        verticalLayout_94 = new QVBoxLayout(groupBox_depthImageFiltering);
        verticalLayout_94->setObjectName(QStringLiteral("verticalLayout_94"));
        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setObjectName(QStringLiteral("horizontalLayout_10"));
        toolButton_source_distortionModel = new QToolButton(groupBox_depthImageFiltering);
        toolButton_source_distortionModel->setObjectName(QStringLiteral("toolButton_source_distortionModel"));

        horizontalLayout_10->addWidget(toolButton_source_distortionModel);

        lineEdit_source_distortionModel = new QLineEdit(groupBox_depthImageFiltering);
        lineEdit_source_distortionModel->setObjectName(QStringLiteral("lineEdit_source_distortionModel"));
        lineEdit_source_distortionModel->setMinimumSize(QSize(100, 0));

        horizontalLayout_10->addWidget(lineEdit_source_distortionModel);

        toolButton_distortionModel = new QToolButton(groupBox_depthImageFiltering);
        toolButton_distortionModel->setObjectName(QStringLiteral("toolButton_distortionModel"));
        toolButton_distortionModel->setIcon(icon1);

        horizontalLayout_10->addWidget(toolButton_distortionModel);

        label_341 = new QLabel(groupBox_depthImageFiltering);
        label_341->setObjectName(QStringLiteral("label_341"));
        label_341->setWordWrap(true);

        horizontalLayout_10->addWidget(label_341);

        horizontalLayout_10->setStretch(3, 1);

        verticalLayout_94->addLayout(horizontalLayout_10);

        groupBox_bilateral = new QGroupBox(groupBox_depthImageFiltering);
        groupBox_bilateral->setObjectName(QStringLiteral("groupBox_bilateral"));
        groupBox_bilateral->setCheckable(true);
        gridLayout_78 = new QGridLayout(groupBox_bilateral);
        gridLayout_78->setObjectName(QStringLiteral("gridLayout_78"));
        doubleSpinBox_bilateral_sigmaR = new QDoubleSpinBox(groupBox_bilateral);
        doubleSpinBox_bilateral_sigmaR->setObjectName(QStringLiteral("doubleSpinBox_bilateral_sigmaR"));
        doubleSpinBox_bilateral_sigmaR->setDecimals(4);
        doubleSpinBox_bilateral_sigmaR->setMinimum(0.0001);
        doubleSpinBox_bilateral_sigmaR->setMaximum(1);
        doubleSpinBox_bilateral_sigmaR->setSingleStep(0.001);
        doubleSpinBox_bilateral_sigmaR->setValue(0.005);

        gridLayout_78->addWidget(doubleSpinBox_bilateral_sigmaR, 2, 0, 1, 1);

        label_344 = new QLabel(groupBox_bilateral);
        label_344->setObjectName(QStringLiteral("label_344"));
        label_344->setWordWrap(true);
        label_344->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_78->addWidget(label_344, 2, 1, 1, 1);

        doubleSpinBox_bilateral_sigmaS = new QDoubleSpinBox(groupBox_bilateral);
        doubleSpinBox_bilateral_sigmaS->setObjectName(QStringLiteral("doubleSpinBox_bilateral_sigmaS"));
        doubleSpinBox_bilateral_sigmaS->setDecimals(1);
        doubleSpinBox_bilateral_sigmaS->setMinimum(0.1);
        doubleSpinBox_bilateral_sigmaS->setMaximum(100);
        doubleSpinBox_bilateral_sigmaS->setSingleStep(1);
        doubleSpinBox_bilateral_sigmaS->setValue(5);

        gridLayout_78->addWidget(doubleSpinBox_bilateral_sigmaS, 1, 0, 1, 1);

        label_345 = new QLabel(groupBox_bilateral);
        label_345->setObjectName(QStringLiteral("label_345"));
        label_345->setWordWrap(true);
        label_345->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_78->addWidget(label_345, 1, 1, 1, 1);

        gridLayout_78->setColumnStretch(1, 1);

        verticalLayout_94->addWidget(groupBox_bilateral);


        verticalLayout_7->addWidget(groupBox_depthImageFiltering);

        groupBox_imuFiltering = new QGroupBox(groupBox_source0);
        groupBox_imuFiltering->setObjectName(QStringLiteral("groupBox_imuFiltering"));
        verticalLayout_141 = new QVBoxLayout(groupBox_imuFiltering);
        verticalLayout_141->setObjectName(QStringLiteral("verticalLayout_141"));
        gridLayout_109 = new QGridLayout();
        gridLayout_109->setObjectName(QStringLiteral("gridLayout_109"));
        comboBox_imuFilter_strategy = new QComboBox(groupBox_imuFiltering);
        comboBox_imuFilter_strategy->setObjectName(QStringLiteral("comboBox_imuFilter_strategy"));

        gridLayout_109->addWidget(comboBox_imuFilter_strategy, 0, 0, 1, 1);

        label_541 = new QLabel(groupBox_imuFiltering);
        label_541->setObjectName(QStringLiteral("label_541"));
        label_541->setWordWrap(true);
        label_541->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_109->addWidget(label_541, 0, 1, 1, 1);

        checkbox_publishInterIMU = new QCheckBox(groupBox_imuFiltering);
        checkbox_publishInterIMU->setObjectName(QStringLiteral("checkbox_publishInterIMU"));
        checkbox_publishInterIMU->setChecked(false);

        gridLayout_109->addWidget(checkbox_publishInterIMU, 2, 0, 1, 1);

        label_563 = new QLabel(groupBox_imuFiltering);
        label_563->setObjectName(QStringLiteral("label_563"));
        label_563->setWordWrap(true);
        label_563->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_109->addWidget(label_563, 2, 1, 1, 1);

        label_621 = new QLabel(groupBox_imuFiltering);
        label_621->setObjectName(QStringLiteral("label_621"));
        label_621->setWordWrap(true);
        label_621->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_109->addWidget(label_621, 1, 1, 1, 1);

        checkBox_imuFilter_baseFrameConversion = new QCheckBox(groupBox_imuFiltering);
        checkBox_imuFilter_baseFrameConversion->setObjectName(QStringLiteral("checkBox_imuFilter_baseFrameConversion"));
        checkBox_imuFilter_baseFrameConversion->setChecked(false);

        gridLayout_109->addWidget(checkBox_imuFilter_baseFrameConversion, 1, 0, 1, 1);

        gridLayout_109->setColumnStretch(1, 1);

        verticalLayout_141->addLayout(gridLayout_109);

        stackedWidget_imuFilter = new QStackedWidget(groupBox_imuFiltering);
        stackedWidget_imuFilter->setObjectName(QStringLiteral("stackedWidget_imuFilter"));
        page_83 = new QWidget();
        page_83->setObjectName(QStringLiteral("page_83"));
        verticalLayout_144 = new QVBoxLayout(page_83);
        verticalLayout_144->setObjectName(QStringLiteral("verticalLayout_144"));
        verticalSpacer_80 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_144->addItem(verticalSpacer_80);

        stackedWidget_imuFilter->addWidget(page_83);
        page_81 = new QWidget();
        page_81->setObjectName(QStringLiteral("page_81"));
        verticalLayout_142 = new QVBoxLayout(page_81);
        verticalLayout_142->setObjectName(QStringLiteral("verticalLayout_142"));
        groupBox_madgwickfilter = new QGroupBox(page_81);
        groupBox_madgwickfilter->setObjectName(QStringLiteral("groupBox_madgwickfilter"));
        gridLayout_110 = new QGridLayout(groupBox_madgwickfilter);
        gridLayout_110->setObjectName(QStringLiteral("gridLayout_110"));
        doubleSpinBox_imuFilterMadgwickGain = new QDoubleSpinBox(groupBox_madgwickfilter);
        doubleSpinBox_imuFilterMadgwickGain->setObjectName(QStringLiteral("doubleSpinBox_imuFilterMadgwickGain"));
        doubleSpinBox_imuFilterMadgwickGain->setDecimals(4);
        doubleSpinBox_imuFilterMadgwickGain->setMinimum(0);
        doubleSpinBox_imuFilterMadgwickGain->setMaximum(1);
        doubleSpinBox_imuFilterMadgwickGain->setSingleStep(0.01);
        doubleSpinBox_imuFilterMadgwickGain->setValue(0.01);

        gridLayout_110->addWidget(doubleSpinBox_imuFilterMadgwickGain, 0, 0, 1, 1);

        label_542 = new QLabel(groupBox_madgwickfilter);
        label_542->setObjectName(QStringLiteral("label_542"));
        label_542->setWordWrap(true);
        label_542->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_110->addWidget(label_542, 0, 1, 1, 1);

        doubleSpinBox_imuFilterMadgwickZeta = new QDoubleSpinBox(groupBox_madgwickfilter);
        doubleSpinBox_imuFilterMadgwickZeta->setObjectName(QStringLiteral("doubleSpinBox_imuFilterMadgwickZeta"));
        doubleSpinBox_imuFilterMadgwickZeta->setDecimals(4);
        doubleSpinBox_imuFilterMadgwickZeta->setMinimum(0);
        doubleSpinBox_imuFilterMadgwickZeta->setMaximum(1);
        doubleSpinBox_imuFilterMadgwickZeta->setSingleStep(0.01);
        doubleSpinBox_imuFilterMadgwickZeta->setValue(0.01);

        gridLayout_110->addWidget(doubleSpinBox_imuFilterMadgwickZeta, 1, 0, 1, 1);

        label_543 = new QLabel(groupBox_madgwickfilter);
        label_543->setObjectName(QStringLiteral("label_543"));
        label_543->setWordWrap(true);
        label_543->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_110->addWidget(label_543, 1, 1, 1, 1);

        gridLayout_110->setColumnStretch(1, 1);

        verticalLayout_142->addWidget(groupBox_madgwickfilter);

        verticalSpacer_79 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_142->addItem(verticalSpacer_79);

        stackedWidget_imuFilter->addWidget(page_81);
        page_82 = new QWidget();
        page_82->setObjectName(QStringLiteral("page_82"));
        verticalLayout_143 = new QVBoxLayout(page_82);
        verticalLayout_143->setObjectName(QStringLiteral("verticalLayout_143"));
        groupBox_complementaryfilter = new QGroupBox(page_82);
        groupBox_complementaryfilter->setObjectName(QStringLiteral("groupBox_complementaryfilter"));
        gridLayout_111 = new QGridLayout(groupBox_complementaryfilter);
        gridLayout_111->setObjectName(QStringLiteral("gridLayout_111"));
        label_547 = new QLabel(groupBox_complementaryfilter);
        label_547->setObjectName(QStringLiteral("label_547"));
        label_547->setWordWrap(true);
        label_547->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_111->addWidget(label_547, 2, 1, 1, 1);

        label_544 = new QLabel(groupBox_complementaryfilter);
        label_544->setObjectName(QStringLiteral("label_544"));
        label_544->setWordWrap(true);
        label_544->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_111->addWidget(label_544, 0, 1, 1, 1);

        checkBox_imuFilterComplementaryDoBiasEstimation = new QCheckBox(groupBox_complementaryfilter);
        checkBox_imuFilterComplementaryDoBiasEstimation->setObjectName(QStringLiteral("checkBox_imuFilterComplementaryDoBiasEstimation"));

        gridLayout_111->addWidget(checkBox_imuFilterComplementaryDoBiasEstimation, 2, 0, 1, 1);

        doubleSpinBox_imuFilterComplementaryGainAcc = new QDoubleSpinBox(groupBox_complementaryfilter);
        doubleSpinBox_imuFilterComplementaryGainAcc->setObjectName(QStringLiteral("doubleSpinBox_imuFilterComplementaryGainAcc"));
        doubleSpinBox_imuFilterComplementaryGainAcc->setDecimals(4);
        doubleSpinBox_imuFilterComplementaryGainAcc->setMinimum(0);
        doubleSpinBox_imuFilterComplementaryGainAcc->setMaximum(1);
        doubleSpinBox_imuFilterComplementaryGainAcc->setSingleStep(0.01);
        doubleSpinBox_imuFilterComplementaryGainAcc->setValue(0.01);

        gridLayout_111->addWidget(doubleSpinBox_imuFilterComplementaryGainAcc, 0, 0, 1, 1);

        label_545 = new QLabel(groupBox_complementaryfilter);
        label_545->setObjectName(QStringLiteral("label_545"));
        label_545->setWordWrap(true);
        label_545->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_111->addWidget(label_545, 1, 1, 1, 1);

        doubleSpinBox_imuFilterComplementaryBiasAlpha = new QDoubleSpinBox(groupBox_complementaryfilter);
        doubleSpinBox_imuFilterComplementaryBiasAlpha->setObjectName(QStringLiteral("doubleSpinBox_imuFilterComplementaryBiasAlpha"));
        doubleSpinBox_imuFilterComplementaryBiasAlpha->setDecimals(4);
        doubleSpinBox_imuFilterComplementaryBiasAlpha->setMinimum(0);
        doubleSpinBox_imuFilterComplementaryBiasAlpha->setMaximum(1);
        doubleSpinBox_imuFilterComplementaryBiasAlpha->setSingleStep(0.01);
        doubleSpinBox_imuFilterComplementaryBiasAlpha->setValue(0.01);

        gridLayout_111->addWidget(doubleSpinBox_imuFilterComplementaryBiasAlpha, 1, 0, 1, 1);

        checkBox_imuFilterComplementaryDoAdaptiveGain = new QCheckBox(groupBox_complementaryfilter);
        checkBox_imuFilterComplementaryDoAdaptiveGain->setObjectName(QStringLiteral("checkBox_imuFilterComplementaryDoAdaptiveGain"));

        gridLayout_111->addWidget(checkBox_imuFilterComplementaryDoAdaptiveGain, 3, 0, 1, 1);

        label_548 = new QLabel(groupBox_complementaryfilter);
        label_548->setObjectName(QStringLiteral("label_548"));
        label_548->setWordWrap(true);
        label_548->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_111->addWidget(label_548, 3, 1, 1, 1);

        gridLayout_111->setColumnStretch(1, 1);

        verticalLayout_143->addWidget(groupBox_complementaryfilter);

        verticalSpacer_81 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_143->addItem(verticalSpacer_81);

        stackedWidget_imuFilter->addWidget(page_82);

        verticalLayout_141->addWidget(stackedWidget_imuFilter);


        verticalLayout_7->addWidget(groupBox_imuFiltering);

        groupBox_scan = new QGroupBox(groupBox_source0);
        groupBox_scan->setObjectName(QStringLiteral("groupBox_scan"));
        groupBox_scan->setCheckable(false);
        groupBox_scan->setChecked(false);
        verticalLayout_13 = new QVBoxLayout(groupBox_scan);
        verticalLayout_13->setObjectName(QStringLiteral("verticalLayout_13"));
        label_16 = new QLabel(groupBox_scan);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setWordWrap(true);

        verticalLayout_13->addWidget(label_16);

        gridLayout_62 = new QGridLayout();
        gridLayout_62->setObjectName(QStringLiteral("gridLayout_62"));
        doubleSpinBox_source_scanRangeMin = new QDoubleSpinBox(groupBox_scan);
        doubleSpinBox_source_scanRangeMin->setObjectName(QStringLiteral("doubleSpinBox_source_scanRangeMin"));
        doubleSpinBox_source_scanRangeMin->setMaximum(9999);

        gridLayout_62->addWidget(doubleSpinBox_source_scanRangeMin, 2, 0, 1, 1);

        spinBox_source_scanNormalsK = new QSpinBox(groupBox_scan);
        spinBox_source_scanNormalsK->setObjectName(QStringLiteral("spinBox_source_scanNormalsK"));
        spinBox_source_scanNormalsK->setMinimum(0);
        spinBox_source_scanNormalsK->setMaximum(99999999);
        spinBox_source_scanNormalsK->setValue(0);

        gridLayout_62->addWidget(spinBox_source_scanNormalsK, 5, 0, 1, 1);

        label_300 = new QLabel(groupBox_scan);
        label_300->setObjectName(QStringLiteral("label_300"));
        label_300->setWordWrap(true);
        label_300->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_62->addWidget(label_300, 1, 1, 1, 1);

        doubleSpinBox_source_scanVoxelSize = new QDoubleSpinBox(groupBox_scan);
        doubleSpinBox_source_scanVoxelSize->setObjectName(QStringLiteral("doubleSpinBox_source_scanVoxelSize"));
        doubleSpinBox_source_scanVoxelSize->setDecimals(3);
        doubleSpinBox_source_scanVoxelSize->setSingleStep(0.01);

        gridLayout_62->addWidget(doubleSpinBox_source_scanVoxelSize, 4, 0, 1, 1);

        spinBox_source_scanDownsampleStep = new QSpinBox(groupBox_scan);
        spinBox_source_scanDownsampleStep->setObjectName(QStringLiteral("spinBox_source_scanDownsampleStep"));
        spinBox_source_scanDownsampleStep->setMinimum(1);
        spinBox_source_scanDownsampleStep->setMaximum(99999999);

        gridLayout_62->addWidget(spinBox_source_scanDownsampleStep, 1, 0, 1, 1);

        label_302 = new QLabel(groupBox_scan);
        label_302->setObjectName(QStringLiteral("label_302"));
        label_302->setWordWrap(true);
        label_302->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_62->addWidget(label_302, 4, 1, 1, 1);

        label_304 = new QLabel(groupBox_scan);
        label_304->setObjectName(QStringLiteral("label_304"));
        label_304->setWordWrap(true);
        label_304->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_62->addWidget(label_304, 5, 1, 1, 1);

        label_425 = new QLabel(groupBox_scan);
        label_425->setObjectName(QStringLiteral("label_425"));
        label_425->setWordWrap(true);
        label_425->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_62->addWidget(label_425, 6, 1, 1, 1);

        doubleSpinBox_source_scanNormalsRadius = new QDoubleSpinBox(groupBox_scan);
        doubleSpinBox_source_scanNormalsRadius->setObjectName(QStringLiteral("doubleSpinBox_source_scanNormalsRadius"));
        doubleSpinBox_source_scanNormalsRadius->setSingleStep(0.01);

        gridLayout_62->addWidget(doubleSpinBox_source_scanNormalsRadius, 6, 0, 1, 1);

        label_453 = new QLabel(groupBox_scan);
        label_453->setObjectName(QStringLiteral("label_453"));
        label_453->setWordWrap(true);
        label_453->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_62->addWidget(label_453, 7, 1, 1, 1);

        doubleSpinBox_source_scanRangeMax = new QDoubleSpinBox(groupBox_scan);
        doubleSpinBox_source_scanRangeMax->setObjectName(QStringLiteral("doubleSpinBox_source_scanRangeMax"));
        doubleSpinBox_source_scanRangeMax->setMaximum(9999);

        gridLayout_62->addWidget(doubleSpinBox_source_scanRangeMax, 3, 0, 1, 1);

        label_522 = new QLabel(groupBox_scan);
        label_522->setObjectName(QStringLiteral("label_522"));
        label_522->setWordWrap(true);
        label_522->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_62->addWidget(label_522, 2, 1, 1, 1);

        label_523 = new QLabel(groupBox_scan);
        label_523->setObjectName(QStringLiteral("label_523"));
        label_523->setWordWrap(true);
        label_523->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_62->addWidget(label_523, 3, 1, 1, 1);

        checkBox_source_scanFromDepth = new QCheckBox(groupBox_scan);
        checkBox_source_scanFromDepth->setObjectName(QStringLiteral("checkBox_source_scanFromDepth"));

        gridLayout_62->addWidget(checkBox_source_scanFromDepth, 0, 0, 1, 1);

        label_source_scanFromDepth = new QLabel(groupBox_scan);
        label_source_scanFromDepth->setObjectName(QStringLiteral("label_source_scanFromDepth"));
        label_source_scanFromDepth->setWordWrap(true);
        label_source_scanFromDepth->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_62->addWidget(label_source_scanFromDepth, 0, 1, 1, 1);

        doubleSpinBox_source_scanNormalsForceGroundUp = new QDoubleSpinBox(groupBox_scan);
        doubleSpinBox_source_scanNormalsForceGroundUp->setObjectName(QStringLiteral("doubleSpinBox_source_scanNormalsForceGroundUp"));
        doubleSpinBox_source_scanNormalsForceGroundUp->setMaximum(1);
        doubleSpinBox_source_scanNormalsForceGroundUp->setSingleStep(0.01);

        gridLayout_62->addWidget(doubleSpinBox_source_scanNormalsForceGroundUp, 7, 0, 1, 1);

        gridLayout_62->setColumnStretch(1, 1);

        verticalLayout_13->addLayout(gridLayout_62);


        verticalLayout_7->addWidget(groupBox_scan);

        verticalLayout_7->setStretch(5, 1);

        verticalLayout_27->addWidget(groupBox_source0);

        verticalSpacer_7 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_27->addItem(verticalSpacer_7);

        verticalLayout_27->setStretch(1, 1);
        stackedWidget->addWidget(page_4);
        page_20 = new QWidget();
        page_20->setObjectName(QStringLiteral("page_20"));
        verticalLayout_23 = new QVBoxLayout(page_20);
        verticalLayout_23->setObjectName(QStringLiteral("verticalLayout_23"));
        groupBox_rtabmap_basic0 = new QGroupBox(page_20);
        groupBox_rtabmap_basic0->setObjectName(QStringLiteral("groupBox_rtabmap_basic0"));
        gridLayout_3 = new QGridLayout(groupBox_rtabmap_basic0);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        general_doubleSpinBox_detectionRate_2 = new QDoubleSpinBox(groupBox_rtabmap_basic0);
        general_doubleSpinBox_detectionRate_2->setObjectName(QStringLiteral("general_doubleSpinBox_detectionRate_2"));
        general_doubleSpinBox_detectionRate_2->setDecimals(3);
        general_doubleSpinBox_detectionRate_2->setValue(1);

        gridLayout_3->addWidget(general_doubleSpinBox_detectionRate_2, 3, 0, 1, 1);

        general_checkBox_SLAM_mode_2 = new QCheckBox(groupBox_rtabmap_basic0);
        general_checkBox_SLAM_mode_2->setObjectName(QStringLiteral("general_checkBox_SLAM_mode_2"));
        general_checkBox_SLAM_mode_2->setChecked(true);

        gridLayout_3->addWidget(general_checkBox_SLAM_mode_2, 0, 0, 1, 1);

        label_79 = new QLabel(groupBox_rtabmap_basic0);
        label_79->setObjectName(QStringLiteral("label_79"));
        label_79->setWordWrap(true);
        label_79->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_79, 0, 1, 1, 1);

        general_checkBox_activateRGBD_2 = new QCheckBox(groupBox_rtabmap_basic0);
        general_checkBox_activateRGBD_2->setObjectName(QStringLiteral("general_checkBox_activateRGBD_2"));
        general_checkBox_activateRGBD_2->setChecked(true);

        gridLayout_3->addWidget(general_checkBox_activateRGBD_2, 1, 0, 1, 1);

        label_activateRGBD_2 = new QLabel(groupBox_rtabmap_basic0);
        label_activateRGBD_2->setObjectName(QStringLiteral("label_activateRGBD_2"));
        label_activateRGBD_2->setWordWrap(true);
        label_activateRGBD_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_activateRGBD_2, 1, 1, 1, 1);

        general_checkBox_publishStats_2 = new QCheckBox(groupBox_rtabmap_basic0);
        general_checkBox_publishStats_2->setObjectName(QStringLiteral("general_checkBox_publishStats_2"));
        general_checkBox_publishStats_2->setChecked(true);

        gridLayout_3->addWidget(general_checkBox_publishStats_2, 2, 0, 1, 1);

        label_publishStat = new QLabel(groupBox_rtabmap_basic0);
        label_publishStat->setObjectName(QStringLiteral("label_publishStat"));
        label_publishStat->setWordWrap(true);
        label_publishStat->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_publishStat, 2, 1, 1, 1);

        label_134 = new QLabel(groupBox_rtabmap_basic0);
        label_134->setObjectName(QStringLiteral("label_134"));
        label_134->setWordWrap(true);
        label_134->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_134, 3, 1, 1, 1);

        general_spinBox_imagesBufferSize_2 = new QSpinBox(groupBox_rtabmap_basic0);
        general_spinBox_imagesBufferSize_2->setObjectName(QStringLiteral("general_spinBox_imagesBufferSize_2"));
        general_spinBox_imagesBufferSize_2->setMaximum(999);
        general_spinBox_imagesBufferSize_2->setValue(1);

        gridLayout_3->addWidget(general_spinBox_imagesBufferSize_2, 4, 0, 1, 1);

        label_112 = new QLabel(groupBox_rtabmap_basic0);
        label_112->setObjectName(QStringLiteral("label_112"));
        label_112->setWordWrap(true);
        label_112->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_112, 4, 1, 1, 1);

        general_doubleSpinBox_timeThr_2 = new QDoubleSpinBox(groupBox_rtabmap_basic0);
        general_doubleSpinBox_timeThr_2->setObjectName(QStringLiteral("general_doubleSpinBox_timeThr_2"));
        general_doubleSpinBox_timeThr_2->setDecimals(0);
        general_doubleSpinBox_timeThr_2->setMaximum(10000);
        general_doubleSpinBox_timeThr_2->setSingleStep(50);
        general_doubleSpinBox_timeThr_2->setValue(700);

        gridLayout_3->addWidget(general_doubleSpinBox_timeThr_2, 5, 0, 1, 1);

        label_timeLimit_2 = new QLabel(groupBox_rtabmap_basic0);
        label_timeLimit_2->setObjectName(QStringLiteral("label_timeLimit_2"));
        label_timeLimit_2->setWordWrap(true);
        label_timeLimit_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_timeLimit_2, 5, 1, 1, 1);

        general_doubleSpinBox_hardThr_2 = new QDoubleSpinBox(groupBox_rtabmap_basic0);
        general_doubleSpinBox_hardThr_2->setObjectName(QStringLiteral("general_doubleSpinBox_hardThr_2"));
        general_doubleSpinBox_hardThr_2->setMaximum(1);
        general_doubleSpinBox_hardThr_2->setSingleStep(0.01);
        general_doubleSpinBox_hardThr_2->setValue(0.95);

        gridLayout_3->addWidget(general_doubleSpinBox_hardThr_2, 6, 0, 1, 1);

        label_121 = new QLabel(groupBox_rtabmap_basic0);
        label_121->setObjectName(QStringLiteral("label_121"));
        label_121->setWordWrap(true);
        label_121->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_121, 6, 1, 1, 1);

        doubleSpinBox_similarityThreshold_2 = new QDoubleSpinBox(groupBox_rtabmap_basic0);
        doubleSpinBox_similarityThreshold_2->setObjectName(QStringLiteral("doubleSpinBox_similarityThreshold_2"));
        doubleSpinBox_similarityThreshold_2->setMaximum(1);
        doubleSpinBox_similarityThreshold_2->setValue(0.2);

        gridLayout_3->addWidget(doubleSpinBox_similarityThreshold_2, 7, 0, 1, 1);

        label_similarity_2 = new QLabel(groupBox_rtabmap_basic0);
        label_similarity_2->setObjectName(QStringLiteral("label_similarity_2"));
        label_similarity_2->setWordWrap(true);
        label_similarity_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_similarity_2, 7, 1, 1, 1);

        general_spinBox_maxStMemSize_2 = new QSpinBox(groupBox_rtabmap_basic0);
        general_spinBox_maxStMemSize_2->setObjectName(QStringLiteral("general_spinBox_maxStMemSize_2"));
        general_spinBox_maxStMemSize_2->setMinimum(1);
        general_spinBox_maxStMemSize_2->setMaximum(9999);
        general_spinBox_maxStMemSize_2->setValue(15);

        gridLayout_3->addWidget(general_spinBox_maxStMemSize_2, 8, 0, 1, 1);

        label_126 = new QLabel(groupBox_rtabmap_basic0);
        label_126->setObjectName(QStringLiteral("label_126"));
        label_126->setWordWrap(true);
        label_126->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_126, 8, 1, 1, 1);

        gridLayout_3->setColumnStretch(1, 1);

        verticalLayout_23->addWidget(groupBox_rtabmap_basic0);

        verticalSpacer_5 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_23->addItem(verticalSpacer_5);

        stackedWidget->addWidget(page_20);
        page = new QWidget();
        page->setObjectName(QStringLiteral("page"));
        verticalLayout_44 = new QVBoxLayout(page);
        verticalLayout_44->setObjectName(QStringLiteral("verticalLayout_44"));
        groupBox_rtabmap_advanced0 = new QGroupBox(page);
        groupBox_rtabmap_advanced0->setObjectName(QStringLiteral("groupBox_rtabmap_advanced0"));
        verticalLayout_15 = new QVBoxLayout(groupBox_rtabmap_advanced0);
        verticalLayout_15->setObjectName(QStringLiteral("verticalLayout_15"));
        gridLayout_43 = new QGridLayout();
        gridLayout_43->setObjectName(QStringLiteral("gridLayout_43"));
        general_doubleSpinBox_detectionRate = new QDoubleSpinBox(groupBox_rtabmap_advanced0);
        general_doubleSpinBox_detectionRate->setObjectName(QStringLiteral("general_doubleSpinBox_detectionRate"));
        general_doubleSpinBox_detectionRate->setDecimals(3);
        general_doubleSpinBox_detectionRate->setValue(1);

        gridLayout_43->addWidget(general_doubleSpinBox_detectionRate, 1, 0, 1, 1);

        general_checkBox_SLAM_mode = new QCheckBox(groupBox_rtabmap_advanced0);
        general_checkBox_SLAM_mode->setObjectName(QStringLiteral("general_checkBox_SLAM_mode"));
        general_checkBox_SLAM_mode->setChecked(true);

        gridLayout_43->addWidget(general_checkBox_SLAM_mode, 0, 0, 1, 1);

        label_92 = new QLabel(groupBox_rtabmap_advanced0);
        label_92->setObjectName(QStringLiteral("label_92"));
        label_92->setWordWrap(true);
        label_92->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_43->addWidget(label_92, 0, 1, 1, 1);

        general_checkBox_startNewMapOnLoopClosure = new QCheckBox(groupBox_rtabmap_advanced0);
        general_checkBox_startNewMapOnLoopClosure->setObjectName(QStringLiteral("general_checkBox_startNewMapOnLoopClosure"));
        general_checkBox_startNewMapOnLoopClosure->setChecked(true);

        gridLayout_43->addWidget(general_checkBox_startNewMapOnLoopClosure, 4, 0, 1, 1);

        label_135 = new QLabel(groupBox_rtabmap_advanced0);
        label_135->setObjectName(QStringLiteral("label_135"));
        label_135->setWordWrap(true);
        label_135->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_43->addWidget(label_135, 1, 1, 1, 1);

        general_spinBox_imagesBufferSize = new QSpinBox(groupBox_rtabmap_advanced0);
        general_spinBox_imagesBufferSize->setObjectName(QStringLiteral("general_spinBox_imagesBufferSize"));
        general_spinBox_imagesBufferSize->setMaximum(999);
        general_spinBox_imagesBufferSize->setValue(1);

        gridLayout_43->addWidget(general_spinBox_imagesBufferSize, 2, 0, 1, 1);

        general_checkBox_createIntermediateNodes = new QCheckBox(groupBox_rtabmap_advanced0);
        general_checkBox_createIntermediateNodes->setObjectName(QStringLiteral("general_checkBox_createIntermediateNodes"));
        general_checkBox_createIntermediateNodes->setChecked(false);

        gridLayout_43->addWidget(general_checkBox_createIntermediateNodes, 3, 0, 1, 1);

        label_185 = new QLabel(groupBox_rtabmap_advanced0);
        label_185->setObjectName(QStringLiteral("label_185"));
        label_185->setWordWrap(true);
        label_185->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_43->addWidget(label_185, 4, 1, 1, 1);

        label_84 = new QLabel(groupBox_rtabmap_advanced0);
        label_84->setObjectName(QStringLiteral("label_84"));
        label_84->setWordWrap(true);
        label_84->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_43->addWidget(label_84, 2, 1, 1, 1);

        label_165 = new QLabel(groupBox_rtabmap_advanced0);
        label_165->setObjectName(QStringLiteral("label_165"));
        label_165->setWordWrap(true);
        label_165->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_43->addWidget(label_165, 3, 1, 1, 1);

        label_467 = new QLabel(groupBox_rtabmap_advanced0);
        label_467->setObjectName(QStringLiteral("label_467"));
        label_467->setWordWrap(true);
        label_467->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_43->addWidget(label_467, 6, 1, 1, 1);

        general_checkBox_imagesAlreadyRectified = new QCheckBox(groupBox_rtabmap_advanced0);
        general_checkBox_imagesAlreadyRectified->setObjectName(QStringLiteral("general_checkBox_imagesAlreadyRectified"));
        general_checkBox_imagesAlreadyRectified->setChecked(true);

        gridLayout_43->addWidget(general_checkBox_imagesAlreadyRectified, 6, 0, 1, 1);

        label_329 = new QLabel(groupBox_rtabmap_advanced0);
        label_329->setObjectName(QStringLiteral("label_329"));
        label_329->setWordWrap(true);
        label_329->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_43->addWidget(label_329, 5, 1, 1, 1);

        general_checkBox_startNewMapOnGoodSignature = new QCheckBox(groupBox_rtabmap_advanced0);
        general_checkBox_startNewMapOnGoodSignature->setObjectName(QStringLiteral("general_checkBox_startNewMapOnGoodSignature"));
        general_checkBox_startNewMapOnGoodSignature->setChecked(true);

        gridLayout_43->addWidget(general_checkBox_startNewMapOnGoodSignature, 5, 0, 1, 1);

        label_498 = new QLabel(groupBox_rtabmap_advanced0);
        label_498->setObjectName(QStringLiteral("label_498"));
        label_498->setWordWrap(true);
        label_498->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_43->addWidget(label_498, 7, 1, 1, 1);

        general_checkBox_rectifyOnlyFeatures = new QCheckBox(groupBox_rtabmap_advanced0);
        general_checkBox_rectifyOnlyFeatures->setObjectName(QStringLiteral("general_checkBox_rectifyOnlyFeatures"));
        general_checkBox_rectifyOnlyFeatures->setChecked(true);

        gridLayout_43->addWidget(general_checkBox_rectifyOnlyFeatures, 7, 0, 1, 1);

        label_573 = new QLabel(groupBox_rtabmap_advanced0);
        label_573->setObjectName(QStringLiteral("label_573"));
        label_573->setWordWrap(true);
        label_573->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_43->addWidget(label_573, 8, 1, 1, 1);

        general_checkBox_saveLocalizationData = new QCheckBox(groupBox_rtabmap_advanced0);
        general_checkBox_saveLocalizationData->setObjectName(QStringLiteral("general_checkBox_saveLocalizationData"));
        general_checkBox_saveLocalizationData->setChecked(true);

        gridLayout_43->addWidget(general_checkBox_saveLocalizationData, 8, 0, 1, 1);

        gridLayout_43->setColumnStretch(1, 1);

        verticalLayout_15->addLayout(gridLayout_43);

        groupBox_6 = new QGroupBox(groupBox_rtabmap_advanced0);
        groupBox_6->setObjectName(QStringLiteral("groupBox_6"));
        gridLayout_44 = new QGridLayout(groupBox_6);
        gridLayout_44->setObjectName(QStringLiteral("gridLayout_44"));
        general_doubleSpinBox_timeThr = new QDoubleSpinBox(groupBox_6);
        general_doubleSpinBox_timeThr->setObjectName(QStringLiteral("general_doubleSpinBox_timeThr"));
        general_doubleSpinBox_timeThr->setDecimals(0);
        general_doubleSpinBox_timeThr->setMaximum(10000);
        general_doubleSpinBox_timeThr->setSingleStep(50);
        general_doubleSpinBox_timeThr->setValue(700);

        gridLayout_44->addWidget(general_doubleSpinBox_timeThr, 0, 0, 1, 1);

        label_timeLimit = new QLabel(groupBox_6);
        label_timeLimit->setObjectName(QStringLiteral("label_timeLimit"));
        label_timeLimit->setWordWrap(true);
        label_timeLimit->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_44->addWidget(label_timeLimit, 0, 1, 1, 1);

        general_spinBox_memoryThr = new QSpinBox(groupBox_6);
        general_spinBox_memoryThr->setObjectName(QStringLiteral("general_spinBox_memoryThr"));
        general_spinBox_memoryThr->setMaximum(99999);
        general_spinBox_memoryThr->setValue(0);

        gridLayout_44->addWidget(general_spinBox_memoryThr, 1, 0, 1, 1);

        label_maxWmSize = new QLabel(groupBox_6);
        label_maxWmSize->setObjectName(QStringLiteral("label_maxWmSize"));
        label_maxWmSize->setWordWrap(true);
        label_maxWmSize->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_44->addWidget(label_maxWmSize, 1, 1, 1, 1);

        general_doubleSpinBox_hardThr = new QDoubleSpinBox(groupBox_6);
        general_doubleSpinBox_hardThr->setObjectName(QStringLiteral("general_doubleSpinBox_hardThr"));
        general_doubleSpinBox_hardThr->setMaximum(1);
        general_doubleSpinBox_hardThr->setSingleStep(0.01);
        general_doubleSpinBox_hardThr->setValue(0.95);

        gridLayout_44->addWidget(general_doubleSpinBox_hardThr, 2, 0, 1, 1);

        label_93 = new QLabel(groupBox_6);
        label_93->setObjectName(QStringLiteral("label_93"));
        label_93->setWordWrap(true);
        label_93->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_44->addWidget(label_93, 2, 1, 1, 1);

        general_doubleSpinBox_loopRatio = new QDoubleSpinBox(groupBox_6);
        general_doubleSpinBox_loopRatio->setObjectName(QStringLiteral("general_doubleSpinBox_loopRatio"));
        general_doubleSpinBox_loopRatio->setMaximum(1);
        general_doubleSpinBox_loopRatio->setSingleStep(0.01);
        general_doubleSpinBox_loopRatio->setValue(0.7);

        gridLayout_44->addWidget(general_doubleSpinBox_loopRatio, 3, 0, 1, 1);

        label_96 = new QLabel(groupBox_6);
        label_96->setObjectName(QStringLiteral("label_96"));
        label_96->setWordWrap(true);
        label_96->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_44->addWidget(label_96, 3, 1, 1, 1);

        gridLayout_44->setColumnStretch(1, 1);

        verticalLayout_15->addWidget(groupBox_6);

        groupBox_publishing = new QGroupBox(groupBox_rtabmap_advanced0);
        groupBox_publishing->setObjectName(QStringLiteral("groupBox_publishing"));
        groupBox_publishing->setCheckable(true);
        gridLayout_45 = new QGridLayout(groupBox_publishing);
        gridLayout_45->setObjectName(QStringLiteral("gridLayout_45"));
        general_checkBox_publishPdf = new QCheckBox(groupBox_publishing);
        general_checkBox_publishPdf->setObjectName(QStringLiteral("general_checkBox_publishPdf"));
        general_checkBox_publishPdf->setChecked(true);

        gridLayout_45->addWidget(general_checkBox_publishPdf, 1, 0, 1, 1);

        label_106 = new QLabel(groupBox_publishing);
        label_106->setObjectName(QStringLiteral("label_106"));
        label_106->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_45->addWidget(label_106, 1, 1, 1, 1);

        general_checkBox_publishLikelihood = new QCheckBox(groupBox_publishing);
        general_checkBox_publishLikelihood->setObjectName(QStringLiteral("general_checkBox_publishLikelihood"));
        general_checkBox_publishLikelihood->setChecked(true);

        gridLayout_45->addWidget(general_checkBox_publishLikelihood, 2, 0, 1, 1);

        label_462 = new QLabel(groupBox_publishing);
        label_462->setObjectName(QStringLiteral("label_462"));
        label_462->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_45->addWidget(label_462, 5, 1, 1, 1);

        general_checkBox_publishRawData = new QCheckBox(groupBox_publishing);
        general_checkBox_publishRawData->setObjectName(QStringLiteral("general_checkBox_publishRawData"));
        general_checkBox_publishRawData->setChecked(true);

        gridLayout_45->addWidget(general_checkBox_publishRawData, 0, 0, 1, 1);

        label_91 = new QLabel(groupBox_publishing);
        label_91->setObjectName(QStringLiteral("label_91"));
        label_91->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_45->addWidget(label_91, 0, 1, 1, 1);

        label_116 = new QLabel(groupBox_publishing);
        label_116->setObjectName(QStringLiteral("label_116"));
        label_116->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_45->addWidget(label_116, 2, 1, 1, 1);

        label_452 = new QLabel(groupBox_publishing);
        label_452->setObjectName(QStringLiteral("label_452"));
        label_452->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_45->addWidget(label_452, 4, 1, 1, 1);

        general_checkBox_publishRAM = new QCheckBox(groupBox_publishing);
        general_checkBox_publishRAM->setObjectName(QStringLiteral("general_checkBox_publishRAM"));
        general_checkBox_publishRAM->setChecked(true);

        gridLayout_45->addWidget(general_checkBox_publishRAM, 4, 0, 1, 1);

        label_437 = new QLabel(groupBox_publishing);
        label_437->setObjectName(QStringLiteral("label_437"));
        label_437->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_45->addWidget(label_437, 3, 1, 1, 1);

        general_checkBox_publishRMSE = new QCheckBox(groupBox_publishing);
        general_checkBox_publishRMSE->setObjectName(QStringLiteral("general_checkBox_publishRMSE"));
        general_checkBox_publishRMSE->setChecked(true);

        gridLayout_45->addWidget(general_checkBox_publishRMSE, 3, 0, 1, 1);

        general_checkBox_saveWMState = new QCheckBox(groupBox_publishing);
        general_checkBox_saveWMState->setObjectName(QStringLiteral("general_checkBox_saveWMState"));
        general_checkBox_saveWMState->setChecked(true);

        gridLayout_45->addWidget(general_checkBox_saveWMState, 5, 0, 1, 1);

        gridLayout_45->setColumnStretch(1, 1);

        verticalLayout_15->addWidget(groupBox_publishing);

        groupBox_statistics = new QGroupBox(groupBox_rtabmap_advanced0);
        groupBox_statistics->setObjectName(QStringLiteral("groupBox_statistics"));
        groupBox_statistics->setCheckable(true);
        gridLayout_46 = new QGridLayout(groupBox_statistics);
        gridLayout_46->setObjectName(QStringLiteral("gridLayout_46"));
        general_checkBox_statisticLogsBufferedInRAM = new QCheckBox(groupBox_statistics);
        general_checkBox_statisticLogsBufferedInRAM->setObjectName(QStringLiteral("general_checkBox_statisticLogsBufferedInRAM"));
        general_checkBox_statisticLogsBufferedInRAM->setChecked(true);

        gridLayout_46->addWidget(general_checkBox_statisticLogsBufferedInRAM, 0, 0, 1, 1);

        label_143 = new QLabel(groupBox_statistics);
        label_143->setObjectName(QStringLiteral("label_143"));
        label_143->setWordWrap(true);
        label_143->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_46->addWidget(label_143, 0, 1, 1, 1);

        general_checkBox_statisticLoggedHeaders = new QCheckBox(groupBox_statistics);
        general_checkBox_statisticLoggedHeaders->setObjectName(QStringLiteral("general_checkBox_statisticLoggedHeaders"));
        general_checkBox_statisticLoggedHeaders->setChecked(true);

        gridLayout_46->addWidget(general_checkBox_statisticLoggedHeaders, 1, 0, 1, 1);

        label_209 = new QLabel(groupBox_statistics);
        label_209->setObjectName(QStringLiteral("label_209"));
        label_209->setWordWrap(true);
        label_209->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_46->addWidget(label_209, 1, 1, 1, 1);

        gridLayout_46->setColumnStretch(1, 1);

        verticalLayout_15->addWidget(groupBox_statistics);

        gridLayout_4 = new QGridLayout();
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        label_workingDirectory = new QLabel(groupBox_rtabmap_advanced0);
        label_workingDirectory->setObjectName(QStringLiteral("label_workingDirectory"));
        label_workingDirectory->setWordWrap(true);
        label_workingDirectory->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_4->addWidget(label_workingDirectory, 0, 0, 1, 1);

        toolButton_workingDirectory = new QToolButton(groupBox_rtabmap_advanced0);
        toolButton_workingDirectory->setObjectName(QStringLiteral("toolButton_workingDirectory"));

        gridLayout_4->addWidget(toolButton_workingDirectory, 0, 1, 1, 1);

        lineEdit_workingDirectory = new QLineEdit(groupBox_rtabmap_advanced0);
        lineEdit_workingDirectory->setObjectName(QStringLiteral("lineEdit_workingDirectory"));
        lineEdit_workingDirectory->setReadOnly(true);

        gridLayout_4->addWidget(lineEdit_workingDirectory, 0, 2, 1, 1);


        verticalLayout_15->addLayout(gridLayout_4);


        verticalLayout_44->addWidget(groupBox_rtabmap_advanced0);

        verticalSpacer_4 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_44->addItem(verticalSpacer_4);

        stackedWidget->addWidget(page);
        page_3 = new QWidget();
        page_3->setObjectName(QStringLiteral("page_3"));
        verticalLayout_4 = new QVBoxLayout(page_3);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        groupBox_memory1 = new QGroupBox(page_3);
        groupBox_memory1->setObjectName(QStringLiteral("groupBox_memory1"));
        verticalLayout_10 = new QVBoxLayout(groupBox_memory1);
        verticalLayout_10->setObjectName(QStringLiteral("verticalLayout_10"));
        gridLayout_42 = new QGridLayout();
        gridLayout_42->setObjectName(QStringLiteral("gridLayout_42"));
        spinBox_imagePreDecimation = new QSpinBox(groupBox_memory1);
        spinBox_imagePreDecimation->setObjectName(QStringLiteral("spinBox_imagePreDecimation"));
        spinBox_imagePreDecimation->setMinimum(1);
        spinBox_imagePreDecimation->setMaximum(16);

        gridLayout_42->addWidget(spinBox_imagePreDecimation, 14, 0, 1, 1);

        spinBox_imagePostDecimation = new QSpinBox(groupBox_memory1);
        spinBox_imagePostDecimation->setObjectName(QStringLiteral("spinBox_imagePostDecimation"));
        spinBox_imagePostDecimation->setMinimum(1);
        spinBox_imagePostDecimation->setMaximum(16);

        gridLayout_42->addWidget(spinBox_imagePostDecimation, 15, 0, 1, 1);

        general_spinBox_maxStMemSize = new QSpinBox(groupBox_memory1);
        general_spinBox_maxStMemSize->setObjectName(QStringLiteral("general_spinBox_maxStMemSize"));
        general_spinBox_maxStMemSize->setMinimum(1);
        general_spinBox_maxStMemSize->setMaximum(9999);
        general_spinBox_maxStMemSize->setValue(15);

        gridLayout_42->addWidget(general_spinBox_maxStMemSize, 0, 0, 1, 1);

        label_retrieved_10 = new QLabel(groupBox_memory1);
        label_retrieved_10->setObjectName(QStringLiteral("label_retrieved_10"));
        label_retrieved_10->setWordWrap(true);
        label_retrieved_10->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved_10, 7, 1, 1, 1);

        general_doubleSpinBox_recentWmRatio = new QDoubleSpinBox(groupBox_memory1);
        general_doubleSpinBox_recentWmRatio->setObjectName(QStringLiteral("general_doubleSpinBox_recentWmRatio"));
        general_doubleSpinBox_recentWmRatio->setMinimumSize(QSize(50, 0));
        general_doubleSpinBox_recentWmRatio->setMaximum(1);
        general_doubleSpinBox_recentWmRatio->setSingleStep(0.01);
        general_doubleSpinBox_recentWmRatio->setValue(0.2);

        gridLayout_42->addWidget(general_doubleSpinBox_recentWmRatio, 1, 0, 1, 1);

        label_retrieved_4 = new QLabel(groupBox_memory1);
        label_retrieved_4->setObjectName(QStringLiteral("label_retrieved_4"));
        label_retrieved_4->setWordWrap(true);
        label_retrieved_4->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved_4, 8, 1, 1, 1);

        general_checkBox_initWMWithAllNodes = new QCheckBox(groupBox_memory1);
        general_checkBox_initWMWithAllNodes->setObjectName(QStringLiteral("general_checkBox_initWMWithAllNodes"));
        general_checkBox_initWMWithAllNodes->setChecked(false);

        gridLayout_42->addWidget(general_checkBox_initWMWithAllNodes, 8, 0, 1, 1);

        general_spinBox_maxRetrieved = new QSpinBox(groupBox_memory1);
        general_spinBox_maxRetrieved->setObjectName(QStringLiteral("general_spinBox_maxRetrieved"));
        general_spinBox_maxRetrieved->setMaximum(999);
        general_spinBox_maxRetrieved->setSingleStep(1);
        general_spinBox_maxRetrieved->setValue(2);

        gridLayout_42->addWidget(general_spinBox_maxRetrieved, 2, 0, 1, 1);

        general_checkBox_keepRawData = new QCheckBox(groupBox_memory1);
        general_checkBox_keepRawData->setObjectName(QStringLiteral("general_checkBox_keepRawData"));
        general_checkBox_keepRawData->setChecked(false);

        gridLayout_42->addWidget(general_checkBox_keepRawData, 9, 0, 1, 1);

        general_checkBox_reduceGraph = new QCheckBox(groupBox_memory1);
        general_checkBox_reduceGraph->setObjectName(QStringLiteral("general_checkBox_reduceGraph"));
        general_checkBox_reduceGraph->setChecked(false);

        gridLayout_42->addWidget(general_checkBox_reduceGraph, 7, 0, 1, 1);

        label_retrieved_3 = new QLabel(groupBox_memory1);
        label_retrieved_3->setObjectName(QStringLiteral("label_retrieved_3"));
        label_retrieved_3->setWordWrap(true);
        label_retrieved_3->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved_3, 6, 1, 1, 1);

        general_checkBox_createMapLabels = new QCheckBox(groupBox_memory1);
        general_checkBox_createMapLabels->setObjectName(QStringLiteral("general_checkBox_createMapLabels"));
        general_checkBox_createMapLabels->setChecked(false);

        gridLayout_42->addWidget(general_checkBox_createMapLabels, 5, 0, 1, 1);

        label_retrieved_5 = new QLabel(groupBox_memory1);
        label_retrieved_5->setObjectName(QStringLiteral("label_retrieved_5"));
        label_retrieved_5->setWordWrap(true);
        label_retrieved_5->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved_5, 9, 1, 1, 1);

        label_retrieved_11 = new QLabel(groupBox_memory1);
        label_retrieved_11->setObjectName(QStringLiteral("label_retrieved_11"));
        label_retrieved_11->setWordWrap(true);
        label_retrieved_11->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved_11, 5, 1, 1, 1);

        general_checkBox_keepDescriptors = new QCheckBox(groupBox_memory1);
        general_checkBox_keepDescriptors->setObjectName(QStringLiteral("general_checkBox_keepDescriptors"));
        general_checkBox_keepDescriptors->setChecked(false);

        gridLayout_42->addWidget(general_checkBox_keepDescriptors, 11, 0, 1, 1);

        label_retrieved_12 = new QLabel(groupBox_memory1);
        label_retrieved_12->setObjectName(QStringLiteral("label_retrieved_12"));
        label_retrieved_12->setWordWrap(true);
        label_retrieved_12->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved_12, 11, 1, 1, 1);

        label_retrieved_13 = new QLabel(groupBox_memory1);
        label_retrieved_13->setObjectName(QStringLiteral("label_retrieved_13"));
        label_retrieved_13->setWordWrap(true);
        label_retrieved_13->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved_13, 14, 1, 1, 1);

        label_retrieved_2 = new QLabel(groupBox_memory1);
        label_retrieved_2->setObjectName(QStringLiteral("label_retrieved_2"));
        label_retrieved_2->setWordWrap(true);
        label_retrieved_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved_2, 4, 1, 1, 1);

        label_retrieved = new QLabel(groupBox_memory1);
        label_retrieved->setObjectName(QStringLiteral("label_retrieved"));
        label_retrieved->setWordWrap(true);
        label_retrieved->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved, 2, 1, 1, 1);

        general_checkBox_generateIds = new QCheckBox(groupBox_memory1);
        general_checkBox_generateIds->setObjectName(QStringLiteral("general_checkBox_generateIds"));
        general_checkBox_generateIds->setChecked(true);

        gridLayout_42->addWidget(general_checkBox_generateIds, 4, 0, 1, 1);

        general_checkBox_badSignaturesIgnored = new QCheckBox(groupBox_memory1);
        general_checkBox_badSignaturesIgnored->setObjectName(QStringLiteral("general_checkBox_badSignaturesIgnored"));
        general_checkBox_badSignaturesIgnored->setChecked(false);

        gridLayout_42->addWidget(general_checkBox_badSignaturesIgnored, 6, 0, 1, 1);

        general_checkBox_transferSortingByWeightId = new QCheckBox(groupBox_memory1);
        general_checkBox_transferSortingByWeightId->setObjectName(QStringLiteral("general_checkBox_transferSortingByWeightId"));
        general_checkBox_transferSortingByWeightId->setChecked(true);

        gridLayout_42->addWidget(general_checkBox_transferSortingByWeightId, 3, 0, 1, 1);

        label_retrieved_9 = new QLabel(groupBox_memory1);
        label_retrieved_9->setObjectName(QStringLiteral("label_retrieved_9"));
        label_retrieved_9->setWordWrap(true);
        label_retrieved_9->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved_9, 12, 1, 1, 1);

        label_29 = new QLabel(groupBox_memory1);
        label_29->setObjectName(QStringLiteral("label_29"));
        label_29->setWordWrap(false);
        label_29->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_29, 0, 1, 1, 1);

        general_checkBox_saveDepth16bits = new QCheckBox(groupBox_memory1);
        general_checkBox_saveDepth16bits->setObjectName(QStringLiteral("general_checkBox_saveDepth16bits"));
        general_checkBox_saveDepth16bits->setChecked(false);

        gridLayout_42->addWidget(general_checkBox_saveDepth16bits, 12, 0, 1, 1);

        label_ratioRecent = new QLabel(groupBox_memory1);
        label_ratioRecent->setObjectName(QStringLiteral("label_ratioRecent"));
        label_ratioRecent->setWordWrap(true);
        label_ratioRecent->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_ratioRecent, 1, 1, 1, 1);

        label_retrieved_6 = new QLabel(groupBox_memory1);
        label_retrieved_6->setObjectName(QStringLiteral("label_retrieved_6"));
        label_retrieved_6->setWordWrap(true);
        label_retrieved_6->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved_6, 15, 1, 1, 1);

        label_retrieved_7 = new QLabel(groupBox_memory1);
        label_retrieved_7->setObjectName(QStringLiteral("label_retrieved_7"));
        label_retrieved_7->setWordWrap(true);
        label_retrieved_7->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved_7, 3, 1, 1, 1);

        general_checkBox_compressionParallelized = new QCheckBox(groupBox_memory1);
        general_checkBox_compressionParallelized->setObjectName(QStringLiteral("general_checkBox_compressionParallelized"));
        general_checkBox_compressionParallelized->setChecked(false);

        gridLayout_42->addWidget(general_checkBox_compressionParallelized, 13, 0, 1, 1);

        general_checkBox_saveIntermediateNodeData = new QCheckBox(groupBox_memory1);
        general_checkBox_saveIntermediateNodeData->setObjectName(QStringLiteral("general_checkBox_saveIntermediateNodeData"));
        general_checkBox_saveIntermediateNodeData->setChecked(false);

        gridLayout_42->addWidget(general_checkBox_saveIntermediateNodeData, 10, 0, 1, 1);

        label_retrieved_15 = new QLabel(groupBox_memory1);
        label_retrieved_15->setObjectName(QStringLiteral("label_retrieved_15"));
        label_retrieved_15->setWordWrap(true);
        label_retrieved_15->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved_15, 13, 1, 1, 1);

        label_retrieved_16 = new QLabel(groupBox_memory1);
        label_retrieved_16->setObjectName(QStringLiteral("label_retrieved_16"));
        label_retrieved_16->setWordWrap(true);
        label_retrieved_16->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_42->addWidget(label_retrieved_16, 10, 1, 1, 1);

        gridLayout_42->setColumnStretch(1, 1);

        verticalLayout_10->addLayout(gridLayout_42);

        groupBox_27 = new QGroupBox(groupBox_memory1);
        groupBox_27->setObjectName(QStringLiteral("groupBox_27"));
        gridLayout_10 = new QGridLayout(groupBox_27);
        gridLayout_10->setObjectName(QStringLiteral("gridLayout_10"));
        general_spinBox_laserScanDownsample = new QSpinBox(groupBox_27);
        general_spinBox_laserScanDownsample->setObjectName(QStringLiteral("general_spinBox_laserScanDownsample"));
        general_spinBox_laserScanDownsample->setMinimumSize(QSize(50, 0));
        general_spinBox_laserScanDownsample->setMinimum(1);
        general_spinBox_laserScanDownsample->setMaximum(9999);

        gridLayout_10->addWidget(general_spinBox_laserScanDownsample, 0, 0, 1, 1);

        label_retrieved_8 = new QLabel(groupBox_27);
        label_retrieved_8->setObjectName(QStringLiteral("label_retrieved_8"));
        label_retrieved_8->setWordWrap(true);
        label_retrieved_8->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_10->addWidget(label_retrieved_8, 0, 1, 1, 1);

        general_doubleSpinBox_laserScanNormalRadius = new QDoubleSpinBox(groupBox_27);
        general_doubleSpinBox_laserScanNormalRadius->setObjectName(QStringLiteral("general_doubleSpinBox_laserScanNormalRadius"));
        general_doubleSpinBox_laserScanNormalRadius->setMinimumSize(QSize(50, 0));
        general_doubleSpinBox_laserScanNormalRadius->setSingleStep(0.01);

        gridLayout_10->addWidget(general_doubleSpinBox_laserScanNormalRadius, 3, 0, 1, 1);

        general_spinBox_laserScanNormalK = new QSpinBox(groupBox_27);
        general_spinBox_laserScanNormalK->setObjectName(QStringLiteral("general_spinBox_laserScanNormalK"));
        general_spinBox_laserScanNormalK->setMinimumSize(QSize(50, 0));
        general_spinBox_laserScanNormalK->setMinimum(0);
        general_spinBox_laserScanNormalK->setMaximum(99);
        general_spinBox_laserScanNormalK->setValue(0);

        gridLayout_10->addWidget(general_spinBox_laserScanNormalK, 2, 0, 1, 1);

        label_retrieved_14 = new QLabel(groupBox_27);
        label_retrieved_14->setObjectName(QStringLiteral("label_retrieved_14"));
        label_retrieved_14->setWordWrap(true);
        label_retrieved_14->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_10->addWidget(label_retrieved_14, 2, 1, 1, 1);

        label_retrieved_17 = new QLabel(groupBox_27);
        label_retrieved_17->setObjectName(QStringLiteral("label_retrieved_17"));
        label_retrieved_17->setWordWrap(true);
        label_retrieved_17->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_10->addWidget(label_retrieved_17, 3, 1, 1, 1);

        label_retrieved_18 = new QLabel(groupBox_27);
        label_retrieved_18->setObjectName(QStringLiteral("label_retrieved_18"));
        label_retrieved_18->setWordWrap(true);
        label_retrieved_18->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_10->addWidget(label_retrieved_18, 1, 1, 1, 1);

        general_doubleSpinBox_laserScanVoxelSize = new QDoubleSpinBox(groupBox_27);
        general_doubleSpinBox_laserScanVoxelSize->setObjectName(QStringLiteral("general_doubleSpinBox_laserScanVoxelSize"));
        general_doubleSpinBox_laserScanVoxelSize->setMinimumSize(QSize(50, 0));
        general_doubleSpinBox_laserScanVoxelSize->setDecimals(3);
        general_doubleSpinBox_laserScanVoxelSize->setSingleStep(0.01);

        gridLayout_10->addWidget(general_doubleSpinBox_laserScanVoxelSize, 1, 0, 1, 1);

        gridLayout_10->setColumnStretch(1, 1);

        verticalLayout_10->addWidget(groupBox_27);

        groupBox_rehearsal2 = new QGroupBox(groupBox_memory1);
        groupBox_rehearsal2->setObjectName(QStringLiteral("groupBox_rehearsal2"));
        gridLayout_12 = new QGridLayout(groupBox_rehearsal2);
        gridLayout_12->setObjectName(QStringLiteral("gridLayout_12"));
        general_checkBox_RehearsalIdUpdatedToNewOne = new QCheckBox(groupBox_rehearsal2);
        general_checkBox_RehearsalIdUpdatedToNewOne->setObjectName(QStringLiteral("general_checkBox_RehearsalIdUpdatedToNewOne"));

        gridLayout_12->addWidget(general_checkBox_RehearsalIdUpdatedToNewOne, 2, 0, 1, 1);

        label_similarity = new QLabel(groupBox_rehearsal2);
        label_similarity->setObjectName(QStringLiteral("label_similarity"));
        label_similarity->setWordWrap(true);
        label_similarity->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_12->addWidget(label_similarity, 0, 1, 1, 1);

        doubleSpinBox_similarityThreshold = new QDoubleSpinBox(groupBox_rehearsal2);
        doubleSpinBox_similarityThreshold->setObjectName(QStringLiteral("doubleSpinBox_similarityThreshold"));
        doubleSpinBox_similarityThreshold->setMaximum(1);
        doubleSpinBox_similarityThreshold->setValue(0.2);

        gridLayout_12->addWidget(doubleSpinBox_similarityThreshold, 0, 0, 1, 1);

        label_rehearsalIdUpdate = new QLabel(groupBox_rehearsal2);
        label_rehearsalIdUpdate->setObjectName(QStringLiteral("label_rehearsalIdUpdate"));
        label_rehearsalIdUpdate->setWordWrap(true);
        label_rehearsalIdUpdate->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_12->addWidget(label_rehearsalIdUpdate, 2, 1, 1, 1);

        label_rehearsalIdUpdate_2 = new QLabel(groupBox_rehearsal2);
        label_rehearsalIdUpdate_2->setObjectName(QStringLiteral("label_rehearsalIdUpdate_2"));
        label_rehearsalIdUpdate_2->setWordWrap(true);
        label_rehearsalIdUpdate_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_12->addWidget(label_rehearsalIdUpdate_2, 3, 1, 1, 1);

        rgdb_rehearsalWeightIgnoredWhileMoving = new QCheckBox(groupBox_rehearsal2);
        rgdb_rehearsalWeightIgnoredWhileMoving->setObjectName(QStringLiteral("rgdb_rehearsalWeightIgnoredWhileMoving"));

        gridLayout_12->addWidget(rgdb_rehearsalWeightIgnoredWhileMoving, 3, 0, 1, 1);

        gridLayout_12->setColumnStretch(1, 1);

        verticalLayout_10->addWidget(groupBox_rehearsal2);


        verticalLayout_4->addWidget(groupBox_memory1);

        verticalSpacer_6 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer_6);

        stackedWidget->addWidget(page_3);
        page_9 = new QWidget();
        page_9->setObjectName(QStringLiteral("page_9"));
        verticalLayout_58 = new QVBoxLayout(page_9);
        verticalLayout_58->setObjectName(QStringLiteral("verticalLayout_58"));
        groupBox_signature_keypoint2 = new QGroupBox(page_9);
        groupBox_signature_keypoint2->setObjectName(QStringLiteral("groupBox_signature_keypoint2"));
        verticalLayout_12 = new QVBoxLayout(groupBox_signature_keypoint2);
        verticalLayout_12->setObjectName(QStringLiteral("verticalLayout_12"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        checkBox_useOdomFeatures = new QCheckBox(groupBox_signature_keypoint2);
        checkBox_useOdomFeatures->setObjectName(QStringLiteral("checkBox_useOdomFeatures"));
        checkBox_useOdomFeatures->setChecked(false);

        horizontalLayout_7->addWidget(checkBox_useOdomFeatures);

        label_145 = new QLabel(groupBox_signature_keypoint2);
        label_145->setObjectName(QStringLiteral("label_145"));
        label_145->setWordWrap(true);
        label_145->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        horizontalLayout_7->addWidget(label_145);

        horizontalLayout_7->setStretch(1, 1);

        verticalLayout_12->addLayout(horizontalLayout_7);

        groupBox_8 = new QGroupBox(groupBox_signature_keypoint2);
        groupBox_8->setObjectName(QStringLiteral("groupBox_8"));
        verticalLayout_60 = new QVBoxLayout(groupBox_8);
        verticalLayout_60->setObjectName(QStringLiteral("verticalLayout_60"));
        gridLayout_32 = new QGridLayout();
        gridLayout_32->setObjectName(QStringLiteral("gridLayout_32"));
        label_22 = new QLabel(groupBox_8);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setWordWrap(true);
        label_22->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_22, 5, 1, 1, 1);

        label_81 = new QLabel(groupBox_8);
        label_81->setObjectName(QStringLiteral("label_81"));
        label_81->setWordWrap(true);
        label_81->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_81, 6, 1, 1, 1);

        label_99 = new QLabel(groupBox_8);
        label_99->setObjectName(QStringLiteral("label_99"));
        label_99->setWordWrap(true);
        label_99->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_99, 10, 1, 1, 1);

        surf_doubleSpinBox_ratioBadSign = new QDoubleSpinBox(groupBox_8);
        surf_doubleSpinBox_ratioBadSign->setObjectName(QStringLiteral("surf_doubleSpinBox_ratioBadSign"));
        surf_doubleSpinBox_ratioBadSign->setDecimals(2);
        surf_doubleSpinBox_ratioBadSign->setMinimum(0);
        surf_doubleSpinBox_ratioBadSign->setMaximum(1);
        surf_doubleSpinBox_ratioBadSign->setSingleStep(0.05);
        surf_doubleSpinBox_ratioBadSign->setValue(0.25);

        gridLayout_32->addWidget(surf_doubleSpinBox_ratioBadSign, 6, 0, 1, 1);

        label_57 = new QLabel(groupBox_8);
        label_57->setObjectName(QStringLiteral("label_57"));
        label_57->setWordWrap(true);
        label_57->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_57, 2, 1, 1, 1);

        comboBox_detector_strategy = new QComboBox(groupBox_8);
        comboBox_detector_strategy->setObjectName(QStringLiteral("comboBox_detector_strategy"));

        gridLayout_32->addWidget(comboBox_detector_strategy, 0, 0, 1, 1);

        lineEdit_kp_roi = new QLineEdit(groupBox_8);
        lineEdit_kp_roi->setObjectName(QStringLiteral("lineEdit_kp_roi"));
        lineEdit_kp_roi->setReadOnly(true);

        gridLayout_32->addWidget(lineEdit_kp_roi, 7, 0, 1, 1);

        label_101 = new QLabel(groupBox_8);
        label_101->setObjectName(QStringLiteral("label_101"));
        label_101->setWordWrap(true);
        label_101->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_101, 7, 1, 1, 1);

        label_53 = new QLabel(groupBox_8);
        label_53->setObjectName(QStringLiteral("label_53"));
        label_53->setWordWrap(true);
        label_53->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_53, 1, 1, 1, 1);

        doubleSpinBox_kp_roi1 = new QDoubleSpinBox(groupBox_8);
        doubleSpinBox_kp_roi1->setObjectName(QStringLiteral("doubleSpinBox_kp_roi1"));
        doubleSpinBox_kp_roi1->setDecimals(0);

        gridLayout_32->addWidget(doubleSpinBox_kp_roi1, 9, 0, 1, 1);

        label_117 = new QLabel(groupBox_8);
        label_117->setObjectName(QStringLiteral("label_117"));
        label_117->setWordWrap(true);
        label_117->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_117, 0, 1, 1, 1);

        surf_doubleSpinBox_maxDepth = new QDoubleSpinBox(groupBox_8);
        surf_doubleSpinBox_maxDepth->setObjectName(QStringLiteral("surf_doubleSpinBox_maxDepth"));
        surf_doubleSpinBox_maxDepth->setDecimals(2);
        surf_doubleSpinBox_maxDepth->setMinimum(0);
        surf_doubleSpinBox_maxDepth->setMaximum(99);
        surf_doubleSpinBox_maxDepth->setSingleStep(1);
        surf_doubleSpinBox_maxDepth->setValue(0);

        gridLayout_32->addWidget(surf_doubleSpinBox_maxDepth, 1, 0, 1, 1);

        surf_spinBox_wordsPerImageTarget = new QSpinBox(groupBox_8);
        surf_spinBox_wordsPerImageTarget->setObjectName(QStringLiteral("surf_spinBox_wordsPerImageTarget"));
        surf_spinBox_wordsPerImageTarget->setMinimum(-1);
        surf_spinBox_wordsPerImageTarget->setMaximum(2000);
        surf_spinBox_wordsPerImageTarget->setValue(150);

        gridLayout_32->addWidget(surf_spinBox_wordsPerImageTarget, 5, 0, 1, 1);

        doubleSpinBox_kp_roi0 = new QDoubleSpinBox(groupBox_8);
        doubleSpinBox_kp_roi0->setObjectName(QStringLiteral("doubleSpinBox_kp_roi0"));
        doubleSpinBox_kp_roi0->setDecimals(0);

        gridLayout_32->addWidget(doubleSpinBox_kp_roi0, 8, 0, 1, 1);

        label_97 = new QLabel(groupBox_8);
        label_97->setObjectName(QStringLiteral("label_97"));
        label_97->setWordWrap(true);
        label_97->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_97, 8, 1, 1, 1);

        label_98 = new QLabel(groupBox_8);
        label_98->setObjectName(QStringLiteral("label_98"));
        label_98->setWordWrap(true);
        label_98->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_98, 9, 1, 1, 1);

        doubleSpinBox_kp_roi2 = new QDoubleSpinBox(groupBox_8);
        doubleSpinBox_kp_roi2->setObjectName(QStringLiteral("doubleSpinBox_kp_roi2"));
        doubleSpinBox_kp_roi2->setDecimals(0);

        gridLayout_32->addWidget(doubleSpinBox_kp_roi2, 10, 0, 1, 1);

        doubleSpinBox_kp_roi3 = new QDoubleSpinBox(groupBox_8);
        doubleSpinBox_kp_roi3->setObjectName(QStringLiteral("doubleSpinBox_kp_roi3"));
        doubleSpinBox_kp_roi3->setDecimals(0);

        gridLayout_32->addWidget(doubleSpinBox_kp_roi3, 11, 0, 1, 1);

        label_100 = new QLabel(groupBox_8);
        label_100->setObjectName(QStringLiteral("label_100"));
        label_100->setWordWrap(true);
        label_100->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_100, 11, 1, 1, 1);

        surf_doubleSpinBox_minDepth = new QDoubleSpinBox(groupBox_8);
        surf_doubleSpinBox_minDepth->setObjectName(QStringLiteral("surf_doubleSpinBox_minDepth"));
        surf_doubleSpinBox_minDepth->setDecimals(2);
        surf_doubleSpinBox_minDepth->setMinimum(0);
        surf_doubleSpinBox_minDepth->setMaximum(99);
        surf_doubleSpinBox_minDepth->setSingleStep(1);
        surf_doubleSpinBox_minDepth->setValue(0);

        gridLayout_32->addWidget(surf_doubleSpinBox_minDepth, 2, 0, 1, 1);

        label_50 = new QLabel(groupBox_8);
        label_50->setObjectName(QStringLiteral("label_50"));
        label_50->setWordWrap(true);
        label_50->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_50, 12, 1, 1, 1);

        spinBox_KPGridRows = new QSpinBox(groupBox_8);
        spinBox_KPGridRows->setObjectName(QStringLiteral("spinBox_KPGridRows"));
        spinBox_KPGridRows->setMinimum(1);
        spinBox_KPGridRows->setMaximum(99);
        spinBox_KPGridRows->setValue(1);

        gridLayout_32->addWidget(spinBox_KPGridRows, 12, 0, 1, 1);

        spinBox_KPGridCols = new QSpinBox(groupBox_8);
        spinBox_KPGridCols->setObjectName(QStringLiteral("spinBox_KPGridCols"));
        spinBox_KPGridCols->setMinimum(1);
        spinBox_KPGridCols->setMaximum(99);
        spinBox_KPGridCols->setValue(1);

        gridLayout_32->addWidget(spinBox_KPGridCols, 13, 0, 1, 1);

        label_86 = new QLabel(groupBox_8);
        label_86->setObjectName(QStringLiteral("label_86"));
        label_86->setWordWrap(true);
        label_86->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_86, 13, 1, 1, 1);

        label_262 = new QLabel(groupBox_8);
        label_262->setObjectName(QStringLiteral("label_262"));
        label_262->setWordWrap(true);
        label_262->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_262, 3, 1, 1, 1);

        checkBox_memDepthAsMask = new QCheckBox(groupBox_8);
        checkBox_memDepthAsMask->setObjectName(QStringLiteral("checkBox_memDepthAsMask"));

        gridLayout_32->addWidget(checkBox_memDepthAsMask, 3, 0, 1, 1);

        label_582 = new QLabel(groupBox_8);
        label_582->setObjectName(QStringLiteral("label_582"));
        label_582->setWordWrap(true);
        label_582->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_32->addWidget(label_582, 4, 1, 1, 1);

        checkBox_memStereoFromMotion = new QCheckBox(groupBox_8);
        checkBox_memStereoFromMotion->setObjectName(QStringLiteral("checkBox_memStereoFromMotion"));

        gridLayout_32->addWidget(checkBox_memStereoFromMotion, 4, 0, 1, 1);

        gridLayout_32->setColumnStretch(1, 1);

        verticalLayout_60->addLayout(gridLayout_32);

        groupBox_15 = new QGroupBox(groupBox_8);
        groupBox_15->setObjectName(QStringLiteral("groupBox_15"));
        verticalLayout_76 = new QVBoxLayout(groupBox_15);
        verticalLayout_76->setObjectName(QStringLiteral("verticalLayout_76"));
        label_44 = new QLabel(groupBox_15);
        label_44->setObjectName(QStringLiteral("label_44"));
        label_44->setWordWrap(true);
        label_44->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_76->addWidget(label_44);

        gridLayout_41 = new QGridLayout();
        gridLayout_41->setObjectName(QStringLiteral("gridLayout_41"));
        subpix_winSize_kp = new QSpinBox(groupBox_15);
        subpix_winSize_kp->setObjectName(QStringLiteral("subpix_winSize_kp"));
        subpix_winSize_kp->setMinimum(0);
        subpix_winSize_kp->setMaximum(999999);
        subpix_winSize_kp->setSingleStep(1);
        subpix_winSize_kp->setValue(5);

        gridLayout_41->addWidget(subpix_winSize_kp, 0, 0, 1, 1);

        label_202 = new QLabel(groupBox_15);
        label_202->setObjectName(QStringLiteral("label_202"));
        label_202->setWordWrap(true);
        label_202->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_41->addWidget(label_202, 0, 1, 1, 1);

        subpix_iterations_kp = new QSpinBox(groupBox_15);
        subpix_iterations_kp->setObjectName(QStringLiteral("subpix_iterations_kp"));
        subpix_iterations_kp->setMinimum(0);
        subpix_iterations_kp->setMaximum(999999);
        subpix_iterations_kp->setSingleStep(1);
        subpix_iterations_kp->setValue(20);

        gridLayout_41->addWidget(subpix_iterations_kp, 1, 0, 1, 1);

        label_225 = new QLabel(groupBox_15);
        label_225->setObjectName(QStringLiteral("label_225"));
        label_225->setWordWrap(true);
        label_225->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_41->addWidget(label_225, 1, 1, 1, 1);

        subpix_eps_kp = new QDoubleSpinBox(groupBox_15);
        subpix_eps_kp->setObjectName(QStringLiteral("subpix_eps_kp"));
        subpix_eps_kp->setDecimals(3);
        subpix_eps_kp->setMinimum(0.001);
        subpix_eps_kp->setMaximum(0.1);
        subpix_eps_kp->setSingleStep(0.01);
        subpix_eps_kp->setValue(0.03);

        gridLayout_41->addWidget(subpix_eps_kp, 2, 0, 1, 1);

        label_227 = new QLabel(groupBox_15);
        label_227->setObjectName(QStringLiteral("label_227"));
        label_227->setWordWrap(true);
        label_227->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_41->addWidget(label_227, 2, 1, 1, 1);

        gridLayout_41->setColumnStretch(1, 1);

        verticalLayout_76->addLayout(gridLayout_41);


        verticalLayout_60->addWidget(groupBox_15);


        verticalLayout_12->addWidget(groupBox_8);

        groupBox_dictionary_strategy2 = new QGroupBox(groupBox_signature_keypoint2);
        groupBox_dictionary_strategy2->setObjectName(QStringLiteral("groupBox_dictionary_strategy2"));
        gridLayout_18 = new QGridLayout(groupBox_dictionary_strategy2);
        gridLayout_18->setObjectName(QStringLiteral("gridLayout_18"));
        toolButton_dictionaryPath = new QToolButton(groupBox_dictionary_strategy2);
        toolButton_dictionaryPath->setObjectName(QStringLiteral("toolButton_dictionaryPath"));

        gridLayout_18->addWidget(toolButton_dictionaryPath, 3, 1, 1, 1);

        label_78 = new QLabel(groupBox_dictionary_strategy2);
        label_78->setObjectName(QStringLiteral("label_78"));
        label_78->setWordWrap(true);
        label_78->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_18->addWidget(label_78, 4, 2, 1, 1);

        surf_doubleSpinBox_nndrRatio = new QDoubleSpinBox(groupBox_dictionary_strategy2);
        surf_doubleSpinBox_nndrRatio->setObjectName(QStringLiteral("surf_doubleSpinBox_nndrRatio"));
        surf_doubleSpinBox_nndrRatio->setDecimals(2);
        surf_doubleSpinBox_nndrRatio->setMinimum(0.01);
        surf_doubleSpinBox_nndrRatio->setMaximum(1);
        surf_doubleSpinBox_nndrRatio->setSingleStep(0.1);
        surf_doubleSpinBox_nndrRatio->setValue(0.7);

        gridLayout_18->addWidget(surf_doubleSpinBox_nndrRatio, 4, 0, 1, 1);

        lineEdit_dictionaryPath = new QLineEdit(groupBox_dictionary_strategy2);
        lineEdit_dictionaryPath->setObjectName(QStringLiteral("lineEdit_dictionaryPath"));

        gridLayout_18->addWidget(lineEdit_dictionaryPath, 3, 0, 1, 1);

        label_dictionaryPath = new QLabel(groupBox_dictionary_strategy2);
        label_dictionaryPath->setObjectName(QStringLiteral("label_dictionaryPath"));
        label_dictionaryPath->setWordWrap(true);
        label_dictionaryPath->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_18->addWidget(label_dictionaryPath, 3, 2, 1, 1);

        checkBox_dictionary_incremental = new QCheckBox(groupBox_dictionary_strategy2);
        checkBox_dictionary_incremental->setObjectName(QStringLiteral("checkBox_dictionary_incremental"));
        checkBox_dictionary_incremental->setChecked(true);

        gridLayout_18->addWidget(checkBox_dictionary_incremental, 1, 0, 1, 1);

        label_12 = new QLabel(groupBox_dictionary_strategy2);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_18->addWidget(label_12, 5, 2, 1, 1);

        comboBox_dictionary_strategy = new QComboBox(groupBox_dictionary_strategy2);
        comboBox_dictionary_strategy->setObjectName(QStringLiteral("comboBox_dictionary_strategy"));
        comboBox_dictionary_strategy->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_18->addWidget(comboBox_dictionary_strategy, 5, 0, 1, 1);

        label_incrementalDict = new QLabel(groupBox_dictionary_strategy2);
        label_incrementalDict->setObjectName(QStringLiteral("label_incrementalDict"));
        label_incrementalDict->setWordWrap(true);
        label_incrementalDict->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_18->addWidget(label_incrementalDict, 1, 2, 1, 1);

        label_94 = new QLabel(groupBox_dictionary_strategy2);
        label_94->setObjectName(QStringLiteral("label_94"));
        label_94->setWordWrap(true);
        label_94->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_18->addWidget(label_94, 2, 2, 1, 1);

        checkBox_kp_parallelized = new QCheckBox(groupBox_dictionary_strategy2);
        checkBox_kp_parallelized->setObjectName(QStringLiteral("checkBox_kp_parallelized"));
        checkBox_kp_parallelized->setChecked(true);

        gridLayout_18->addWidget(checkBox_kp_parallelized, 2, 0, 1, 1);

        checkBox_kp_newWordsComparedTogether = new QCheckBox(groupBox_dictionary_strategy2);
        checkBox_kp_newWordsComparedTogether->setObjectName(QStringLiteral("checkBox_kp_newWordsComparedTogether"));
        checkBox_kp_newWordsComparedTogether->setChecked(true);

        gridLayout_18->addWidget(checkBox_kp_newWordsComparedTogether, 6, 0, 1, 1);

        label_186 = new QLabel(groupBox_dictionary_strategy2);
        label_186->setObjectName(QStringLiteral("label_186"));
        label_186->setWordWrap(true);
        label_186->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_18->addWidget(label_186, 6, 2, 1, 1);

        label_260 = new QLabel(groupBox_dictionary_strategy2);
        label_260->setObjectName(QStringLiteral("label_260"));
        label_260->setWordWrap(true);
        label_260->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_18->addWidget(label_260, 8, 2, 1, 1);

        checkBox_kp_incrementalFlann = new QCheckBox(groupBox_dictionary_strategy2);
        checkBox_kp_incrementalFlann->setObjectName(QStringLiteral("checkBox_kp_incrementalFlann"));
        checkBox_kp_incrementalFlann->setChecked(true);

        gridLayout_18->addWidget(checkBox_kp_incrementalFlann, 8, 0, 1, 1);

        label_451 = new QLabel(groupBox_dictionary_strategy2);
        label_451->setObjectName(QStringLiteral("label_451"));
        label_451->setWordWrap(true);
        label_451->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_18->addWidget(label_451, 9, 2, 1, 1);

        surf_doubleSpinBox_rebalancingFactor = new QDoubleSpinBox(groupBox_dictionary_strategy2);
        surf_doubleSpinBox_rebalancingFactor->setObjectName(QStringLiteral("surf_doubleSpinBox_rebalancingFactor"));
        surf_doubleSpinBox_rebalancingFactor->setDecimals(1);
        surf_doubleSpinBox_rebalancingFactor->setMinimum(1);
        surf_doubleSpinBox_rebalancingFactor->setMaximum(99);
        surf_doubleSpinBox_rebalancingFactor->setSingleStep(0.1);
        surf_doubleSpinBox_rebalancingFactor->setValue(2);

        gridLayout_18->addWidget(surf_doubleSpinBox_rebalancingFactor, 9, 0, 1, 1);

        label_556 = new QLabel(groupBox_dictionary_strategy2);
        label_556->setObjectName(QStringLiteral("label_556"));
        label_556->setWordWrap(true);
        label_556->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_18->addWidget(label_556, 7, 2, 1, 1);

        checkBox_kp_byteToFloat = new QCheckBox(groupBox_dictionary_strategy2);
        checkBox_kp_byteToFloat->setObjectName(QStringLiteral("checkBox_kp_byteToFloat"));
        checkBox_kp_byteToFloat->setChecked(true);

        gridLayout_18->addWidget(checkBox_kp_byteToFloat, 7, 0, 1, 1);

        gridLayout_18->setColumnStretch(2, 1);

        verticalLayout_12->addWidget(groupBox_dictionary_strategy2);


        verticalLayout_58->addWidget(groupBox_signature_keypoint2);

        verticalSpacer_15 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_58->addItem(verticalSpacer_15);

        stackedWidget->addWidget(page_9);
        page_24 = new QWidget();
        page_24->setObjectName(QStringLiteral("page_24"));
        verticalLayout_33 = new QVBoxLayout(page_24);
        verticalLayout_33->setObjectName(QStringLiteral("verticalLayout_33"));
        groupBox_database2 = new QGroupBox(page_24);
        groupBox_database2->setObjectName(QStringLiteral("groupBox_database2"));
        gridLayout_11 = new QGridLayout(groupBox_database2);
        gridLayout_11->setObjectName(QStringLiteral("gridLayout_11"));
        general_checkBox_keepBinaryData = new QCheckBox(groupBox_database2);
        general_checkBox_keepBinaryData->setObjectName(QStringLiteral("general_checkBox_keepBinaryData"));
        general_checkBox_keepBinaryData->setChecked(true);

        gridLayout_11->addWidget(general_checkBox_keepBinaryData, 0, 0, 1, 1);

        label_keepRawData = new QLabel(groupBox_database2);
        label_keepRawData->setObjectName(QStringLiteral("label_keepRawData"));
        label_keepRawData->setWordWrap(true);
        label_keepRawData->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_11->addWidget(label_keepRawData, 0, 1, 1, 1);

        checkBox_dbInMemory = new QCheckBox(groupBox_database2);
        checkBox_dbInMemory->setObjectName(QStringLiteral("checkBox_dbInMemory"));

        gridLayout_11->addWidget(checkBox_dbInMemory, 2, 0, 1, 1);

        comboBox_dbJournalMode = new QComboBox(groupBox_database2);
        comboBox_dbJournalMode->setObjectName(QStringLiteral("comboBox_dbJournalMode"));

        gridLayout_11->addWidget(comboBox_dbJournalMode, 6, 0, 1, 1);

        label_74 = new QLabel(groupBox_database2);
        label_74->setObjectName(QStringLiteral("label_74"));
        label_74->setWordWrap(true);
        label_74->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_11->addWidget(label_74, 2, 1, 1, 1);

        label_120 = new QLabel(groupBox_database2);
        label_120->setObjectName(QStringLiteral("label_120"));
        label_120->setWordWrap(true);
        label_120->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_11->addWidget(label_120, 8, 1, 1, 1);

        comboBox_dbTempStore = new QComboBox(groupBox_database2);
        comboBox_dbTempStore->setObjectName(QStringLiteral("comboBox_dbTempStore"));

        gridLayout_11->addWidget(comboBox_dbTempStore, 8, 0, 1, 1);

        general_checkBox_keepNotLinkedNodes = new QCheckBox(groupBox_database2);
        general_checkBox_keepNotLinkedNodes->setObjectName(QStringLiteral("general_checkBox_keepNotLinkedNodes"));
        general_checkBox_keepNotLinkedNodes->setChecked(true);

        gridLayout_11->addWidget(general_checkBox_keepNotLinkedNodes, 1, 0, 1, 1);

        spinBox_dbCacheSize = new QSpinBox(groupBox_database2);
        spinBox_dbCacheSize->setObjectName(QStringLiteral("spinBox_dbCacheSize"));
        spinBox_dbCacheSize->setMinimum(10);
        spinBox_dbCacheSize->setMaximum(999999);
        spinBox_dbCacheSize->setSingleStep(100);
        spinBox_dbCacheSize->setValue(2000);

        gridLayout_11->addWidget(spinBox_dbCacheSize, 5, 0, 1, 1);

        label_75 = new QLabel(groupBox_database2);
        label_75->setObjectName(QStringLiteral("label_75"));
        label_75->setWordWrap(true);
        label_75->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_11->addWidget(label_75, 5, 1, 1, 1);

        label_77 = new QLabel(groupBox_database2);
        label_77->setObjectName(QStringLiteral("label_77"));
        label_77->setWordWrap(true);
        label_77->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_11->addWidget(label_77, 6, 1, 1, 1);

        label_89 = new QLabel(groupBox_database2);
        label_89->setObjectName(QStringLiteral("label_89"));
        label_89->setWordWrap(true);
        label_89->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_11->addWidget(label_89, 7, 1, 1, 1);

        comboBox_dbSynchronous = new QComboBox(groupBox_database2);
        comboBox_dbSynchronous->setObjectName(QStringLiteral("comboBox_dbSynchronous"));

        gridLayout_11->addWidget(comboBox_dbSynchronous, 7, 0, 1, 1);

        label_keepRehearsed = new QLabel(groupBox_database2);
        label_keepRehearsed->setObjectName(QStringLiteral("label_keepRehearsed"));
        label_keepRehearsed->setWordWrap(true);
        label_keepRehearsed->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_11->addWidget(label_keepRehearsed, 1, 1, 1, 1);

        label_299 = new QLabel(groupBox_database2);
        label_299->setObjectName(QStringLiteral("label_299"));
        label_299->setWordWrap(true);
        label_299->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_11->addWidget(label_299, 3, 1, 1, 1);

        lineEdit_rgbCompressionFormat = new QLineEdit(groupBox_database2);
        lineEdit_rgbCompressionFormat->setObjectName(QStringLiteral("lineEdit_rgbCompressionFormat"));

        gridLayout_11->addWidget(lineEdit_rgbCompressionFormat, 3, 0, 1, 1);

        label_610 = new QLabel(groupBox_database2);
        label_610->setObjectName(QStringLiteral("label_610"));
        label_610->setWordWrap(true);
        label_610->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_11->addWidget(label_610, 4, 1, 1, 1);

        lineEdit_targetDatabaseVersion = new QLineEdit(groupBox_database2);
        lineEdit_targetDatabaseVersion->setObjectName(QStringLiteral("lineEdit_targetDatabaseVersion"));

        gridLayout_11->addWidget(lineEdit_targetDatabaseVersion, 4, 0, 1, 1);

        gridLayout_11->setColumnStretch(1, 1);

        verticalLayout_33->addWidget(groupBox_database2);

        verticalSpacer_12 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_33->addItem(verticalSpacer_12);

        stackedWidget->addWidget(page_24);
        page_23 = new QWidget();
        page_23->setObjectName(QStringLiteral("page_23"));
        verticalLayout_2 = new QVBoxLayout(page_23);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        groupBox_bayes1 = new QGroupBox(page_23);
        groupBox_bayes1->setObjectName(QStringLiteral("groupBox_bayes1"));
        verticalLayout_22 = new QVBoxLayout(groupBox_bayes1);
        verticalLayout_22->setObjectName(QStringLiteral("verticalLayout_22"));
        label = new QLabel(groupBox_bayes1);
        label->setObjectName(QStringLiteral("label"));
        label->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_22->addWidget(label);

        label_76 = new QLabel(groupBox_bayes1);
        label_76->setObjectName(QStringLiteral("label_76"));
        label_76->setWordWrap(true);
        label_76->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_22->addWidget(label_76);

        gridLayout_24 = new QGridLayout();
        gridLayout_24->setObjectName(QStringLiteral("gridLayout_24"));
        label_65 = new QLabel(groupBox_bayes1);
        label_65->setObjectName(QStringLiteral("label_65"));

        gridLayout_24->addWidget(label_65, 0, 0, 1, 1);

        label_66 = new QLabel(groupBox_bayes1);
        label_66->setObjectName(QStringLiteral("label_66"));

        gridLayout_24->addWidget(label_66, 1, 0, 1, 1);

        label_prediction_sum = new QLabel(groupBox_bayes1);
        label_prediction_sum->setObjectName(QStringLiteral("label_prediction_sum"));

        gridLayout_24->addWidget(label_prediction_sum, 1, 1, 1, 1);

        lineEdit_bayes_predictionLC = new QLineEdit(groupBox_bayes1);
        lineEdit_bayes_predictionLC->setObjectName(QStringLiteral("lineEdit_bayes_predictionLC"));
        lineEdit_bayes_predictionLC->setReadOnly(false);

        gridLayout_24->addWidget(lineEdit_bayes_predictionLC, 0, 1, 1, 1);


        verticalLayout_22->addLayout(gridLayout_24);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        label_63 = new QLabel(groupBox_bayes1);
        label_63->setObjectName(QStringLiteral("label_63"));

        horizontalLayout_6->addWidget(label_63);

        predictionPlot = new UPlot(groupBox_bayes1);
        predictionPlot->setObjectName(QStringLiteral("predictionPlot"));

        horizontalLayout_6->addWidget(predictionPlot);

        horizontalLayout_6->setStretch(1, 1);

        verticalLayout_22->addLayout(horizontalLayout_6);

        gridLayout_22 = new QGridLayout();
        gridLayout_22->setObjectName(QStringLiteral("gridLayout_22"));
        checkBox_bayes_fullPredictionUpdate = new QCheckBox(groupBox_bayes1);
        checkBox_bayes_fullPredictionUpdate->setObjectName(QStringLiteral("checkBox_bayes_fullPredictionUpdate"));

        gridLayout_22->addWidget(checkBox_bayes_fullPredictionUpdate, 0, 0, 1, 1);

        general_doubleSpinBox_vp = new QDoubleSpinBox(groupBox_bayes1);
        general_doubleSpinBox_vp->setObjectName(QStringLiteral("general_doubleSpinBox_vp"));
        general_doubleSpinBox_vp->setMaximum(1);
        general_doubleSpinBox_vp->setSingleStep(0.01);
        general_doubleSpinBox_vp->setValue(0.9);

        gridLayout_22->addWidget(general_doubleSpinBox_vp, 1, 0, 1, 1);

        label_113 = new QLabel(groupBox_bayes1);
        label_113->setObjectName(QStringLiteral("label_113"));
        label_113->setWordWrap(true);
        label_113->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_22->addWidget(label_113, 1, 1, 1, 1);

        label_142 = new QLabel(groupBox_bayes1);
        label_142->setObjectName(QStringLiteral("label_142"));
        label_142->setWordWrap(true);
        label_142->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_22->addWidget(label_142, 0, 1, 1, 1);

        label_82 = new QLabel(groupBox_bayes1);
        label_82->setObjectName(QStringLiteral("label_82"));
        label_82->setWordWrap(true);
        label_82->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_22->addWidget(label_82, 2, 1, 1, 1);

        checkBox_kp_tfIdfLikelihoodUsed = new QCheckBox(groupBox_bayes1);
        checkBox_kp_tfIdfLikelihoodUsed->setObjectName(QStringLiteral("checkBox_kp_tfIdfLikelihoodUsed"));
        checkBox_kp_tfIdfLikelihoodUsed->setChecked(true);

        gridLayout_22->addWidget(checkBox_kp_tfIdfLikelihoodUsed, 2, 0, 1, 1);

        label_502 = new QLabel(groupBox_bayes1);
        label_502->setObjectName(QStringLiteral("label_502"));
        label_502->setWordWrap(true);
        label_502->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_22->addWidget(label_502, 3, 1, 1, 1);

        checkBox_rtabmap_loopGPS = new QCheckBox(groupBox_bayes1);
        checkBox_rtabmap_loopGPS->setObjectName(QStringLiteral("checkBox_rtabmap_loopGPS"));

        gridLayout_22->addWidget(checkBox_rtabmap_loopGPS, 3, 0, 1, 1);

        gridLayout_22->setColumnStretch(1, 1);

        verticalLayout_22->addLayout(gridLayout_22);

        groupBox_vh_epipolar2 = new QGroupBox(groupBox_bayes1);
        groupBox_vh_epipolar2->setObjectName(QStringLiteral("groupBox_vh_epipolar2"));
        groupBox_vh_epipolar2->setCheckable(true);
        gridLayout = new QGridLayout(groupBox_vh_epipolar2);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        surf_spinBox_matchCountMinAccepted = new QSpinBox(groupBox_vh_epipolar2);
        surf_spinBox_matchCountMinAccepted->setObjectName(QStringLiteral("surf_spinBox_matchCountMinAccepted"));
        surf_spinBox_matchCountMinAccepted->setMinimum(8);
        surf_spinBox_matchCountMinAccepted->setMaximum(100000);
        surf_spinBox_matchCountMinAccepted->setValue(11);

        gridLayout->addWidget(surf_spinBox_matchCountMinAccepted, 0, 0, 1, 1);

        label_25 = new QLabel(groupBox_vh_epipolar2);
        label_25->setObjectName(QStringLiteral("label_25"));
        label_25->setWordWrap(true);
        label_25->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout->addWidget(label_25, 0, 1, 1, 1);

        surf_doubleSpinBox_ransacParam1 = new QDoubleSpinBox(groupBox_vh_epipolar2);
        surf_doubleSpinBox_ransacParam1->setObjectName(QStringLiteral("surf_doubleSpinBox_ransacParam1"));
        surf_doubleSpinBox_ransacParam1->setDecimals(1);
        surf_doubleSpinBox_ransacParam1->setMaximum(10);
        surf_doubleSpinBox_ransacParam1->setSingleStep(0.1);
        surf_doubleSpinBox_ransacParam1->setValue(3);

        gridLayout->addWidget(surf_doubleSpinBox_ransacParam1, 1, 0, 1, 1);

        label_27 = new QLabel(groupBox_vh_epipolar2);
        label_27->setObjectName(QStringLiteral("label_27"));
        label_27->setWordWrap(true);
        label_27->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout->addWidget(label_27, 1, 1, 1, 1);

        surf_doubleSpinBox_ransacParam2 = new QDoubleSpinBox(groupBox_vh_epipolar2);
        surf_doubleSpinBox_ransacParam2->setObjectName(QStringLiteral("surf_doubleSpinBox_ransacParam2"));
        surf_doubleSpinBox_ransacParam2->setDecimals(2);
        surf_doubleSpinBox_ransacParam2->setMaximum(0.99);
        surf_doubleSpinBox_ransacParam2->setSingleStep(0.01);
        surf_doubleSpinBox_ransacParam2->setValue(0.99);

        gridLayout->addWidget(surf_doubleSpinBox_ransacParam2, 2, 0, 1, 1);

        label_28 = new QLabel(groupBox_vh_epipolar2);
        label_28->setObjectName(QStringLiteral("label_28"));
        label_28->setWordWrap(true);
        label_28->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout->addWidget(label_28, 2, 1, 1, 1);

        gridLayout->setColumnStretch(1, 10);

        verticalLayout_22->addWidget(groupBox_vh_epipolar2);

        verticalLayout_22->setStretch(3, 1);

        verticalLayout_2->addWidget(groupBox_bayes1);

        verticalSpacer_23 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_23);

        verticalLayout_2->setStretch(1, 1);
        stackedWidget->addWidget(page_23);
        page_10 = new QWidget();
        page_10->setObjectName(QStringLiteral("page_10"));
        verticalLayout_19 = new QVBoxLayout(page_10);
        verticalLayout_19->setObjectName(QStringLiteral("verticalLayout_19"));
        groupBox_slam1 = new QGroupBox(page_10);
        groupBox_slam1->setObjectName(QStringLiteral("groupBox_slam1"));
        groupBox_slam1->setCheckable(false);
        verticalLayout_25 = new QVBoxLayout(groupBox_slam1);
        verticalLayout_25->setObjectName(QStringLiteral("verticalLayout_25"));
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        general_checkBox_activateRGBD = new QCheckBox(groupBox_slam1);
        general_checkBox_activateRGBD->setObjectName(QStringLiteral("general_checkBox_activateRGBD"));

        horizontalLayout_4->addWidget(general_checkBox_activateRGBD);

        label_activateRGBD = new QLabel(groupBox_slam1);
        label_activateRGBD->setObjectName(QStringLiteral("label_activateRGBD"));
        label_activateRGBD->setWordWrap(true);
        label_activateRGBD->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        horizontalLayout_4->addWidget(label_activateRGBD);

        horizontalLayout_4->setStretch(1, 1);

        verticalLayout_25->addLayout(horizontalLayout_4);

        label_83 = new QLabel(groupBox_slam1);
        label_83->setObjectName(QStringLiteral("label_83"));
        label_83->setWordWrap(true);
        label_83->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_25->addWidget(label_83);

        groupBox_slam_update = new QGroupBox(groupBox_slam1);
        groupBox_slam_update->setObjectName(QStringLiteral("groupBox_slam_update"));
        gridLayout_47 = new QGridLayout(groupBox_slam_update);
        gridLayout_47->setObjectName(QStringLiteral("gridLayout_47"));
        label_432 = new QLabel(groupBox_slam_update);
        label_432->setObjectName(QStringLiteral("label_432"));
        label_432->setWordWrap(true);
        label_432->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_432, 2, 1, 1, 1);

        label_scanMatching = new QLabel(groupBox_slam_update);
        label_scanMatching->setObjectName(QStringLiteral("label_scanMatching"));
        label_scanMatching->setWordWrap(true);
        label_scanMatching->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_scanMatching, 6, 1, 1, 1);

        label_152 = new QLabel(groupBox_slam_update);
        label_152->setObjectName(QStringLiteral("label_152"));
        label_152->setWordWrap(true);
        label_152->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_152, 1, 1, 1, 1);

        spinBox_maxLocalLocationsRetrieved = new QSpinBox(groupBox_slam_update);
        spinBox_maxLocalLocationsRetrieved->setObjectName(QStringLiteral("spinBox_maxLocalLocationsRetrieved"));

        gridLayout_47->addWidget(spinBox_maxLocalLocationsRetrieved, 13, 0, 1, 1);

        loopClosure_reextract = new QCheckBox(groupBox_slam_update);
        loopClosure_reextract->setObjectName(QStringLiteral("loopClosure_reextract"));

        gridLayout_47->addWidget(loopClosure_reextract, 9, 0, 1, 1);

        rgdb_newMapOdomChange = new QDoubleSpinBox(groupBox_slam_update);
        rgdb_newMapOdomChange->setObjectName(QStringLiteral("rgdb_newMapOdomChange"));
        rgdb_newMapOdomChange->setDecimals(1);
        rgdb_newMapOdomChange->setMaximum(99);
        rgdb_newMapOdomChange->setSingleStep(0.1);
        rgdb_newMapOdomChange->setValue(1);

        gridLayout_47->addWidget(rgdb_newMapOdomChange, 5, 0, 1, 1);

        label_scanMatching_5 = new QLabel(groupBox_slam_update);
        label_scanMatching_5->setObjectName(QStringLiteral("label_scanMatching_5"));
        label_scanMatching_5->setWordWrap(true);
        label_scanMatching_5->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_scanMatching_5, 14, 1, 1, 1);

        rgbd_savedLocalizationIgnored = new QCheckBox(groupBox_slam_update);
        rgbd_savedLocalizationIgnored->setObjectName(QStringLiteral("rgbd_savedLocalizationIgnored"));

        gridLayout_47->addWidget(rgbd_savedLocalizationIgnored, 4, 0, 1, 1);

        label_scanMatching_9 = new QLabel(groupBox_slam_update);
        label_scanMatching_9->setObjectName(QStringLiteral("label_scanMatching_9"));
        label_scanMatching_9->setWordWrap(true);
        label_scanMatching_9->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_scanMatching_9, 9, 1, 1, 1);

        rgdb_linearSpeedUpdate = new QDoubleSpinBox(groupBox_slam_update);
        rgdb_linearSpeedUpdate->setObjectName(QStringLiteral("rgdb_linearSpeedUpdate"));
        rgdb_linearSpeedUpdate->setDecimals(3);
        rgdb_linearSpeedUpdate->setSingleStep(0.1);

        gridLayout_47->addWidget(rgdb_linearSpeedUpdate, 2, 0, 1, 1);

        odomScanHistory = new QCheckBox(groupBox_slam_update);
        odomScanHistory->setObjectName(QStringLiteral("odomScanHistory"));

        gridLayout_47->addWidget(odomScanHistory, 6, 0, 1, 1);

        localDetection_radius = new QDoubleSpinBox(groupBox_slam_update);
        localDetection_radius->setObjectName(QStringLiteral("localDetection_radius"));
        localDetection_radius->setValue(1);

        gridLayout_47->addWidget(localDetection_radius, 15, 0, 1, 1);

        rgdb_localImmunizationRatio = new QDoubleSpinBox(groupBox_slam_update);
        rgdb_localImmunizationRatio->setObjectName(QStringLiteral("rgdb_localImmunizationRatio"));
        rgdb_localImmunizationRatio->setDecimals(2);
        rgdb_localImmunizationRatio->setMaximum(1);
        rgdb_localImmunizationRatio->setSingleStep(0.1);
        rgdb_localImmunizationRatio->setValue(0.1);

        gridLayout_47->addWidget(rgdb_localImmunizationRatio, 14, 0, 1, 1);

        memCovOffDiagIgnored = new QCheckBox(groupBox_slam_update);
        memCovOffDiagIgnored->setObjectName(QStringLiteral("memCovOffDiagIgnored"));

        gridLayout_47->addWidget(memCovOffDiagIgnored, 11, 0, 1, 1);

        label_163 = new QLabel(groupBox_slam_update);
        label_163->setObjectName(QStringLiteral("label_163"));
        label_163->setWordWrap(true);
        label_163->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_163, 5, 1, 1, 1);

        label_433 = new QLabel(groupBox_slam_update);
        label_433->setObjectName(QStringLiteral("label_433"));
        label_433->setWordWrap(true);
        label_433->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_433, 3, 1, 1, 1);

        loopClosure_bunlde = new QCheckBox(groupBox_slam_update);
        loopClosure_bunlde->setObjectName(QStringLiteral("loopClosure_bunlde"));

        gridLayout_47->addWidget(loopClosure_bunlde, 10, 0, 1, 1);

        label_scanMatching_10 = new QLabel(groupBox_slam_update);
        label_scanMatching_10->setObjectName(QStringLiteral("label_scanMatching_10"));
        label_scanMatching_10->setWordWrap(true);
        label_scanMatching_10->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_scanMatching_10, 10, 1, 1, 1);

        label_scanMatching_11 = new QLabel(groupBox_slam_update);
        label_scanMatching_11->setObjectName(QStringLiteral("label_scanMatching_11"));
        label_scanMatching_11->setWordWrap(true);
        label_scanMatching_11->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_scanMatching_11, 11, 1, 1, 1);

        rgdb_linearUpdate = new QDoubleSpinBox(groupBox_slam_update);
        rgdb_linearUpdate->setObjectName(QStringLiteral("rgdb_linearUpdate"));
        rgdb_linearUpdate->setDecimals(3);
        rgdb_linearUpdate->setSingleStep(0.1);

        gridLayout_47->addWidget(rgdb_linearUpdate, 0, 0, 1, 1);

        label_153 = new QLabel(groupBox_slam_update);
        label_153->setObjectName(QStringLiteral("label_153"));
        label_153->setWordWrap(true);
        label_153->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_153, 0, 1, 1, 1);

        rgbd_loopCovLimited = new QCheckBox(groupBox_slam_update);
        rgbd_loopCovLimited->setObjectName(QStringLiteral("rgbd_loopCovLimited"));

        gridLayout_47->addWidget(rgbd_loopCovLimited, 12, 0, 1, 1);

        rgdb_angularUpdate = new QDoubleSpinBox(groupBox_slam_update);
        rgdb_angularUpdate->setObjectName(QStringLiteral("rgdb_angularUpdate"));
        rgdb_angularUpdate->setDecimals(2);
        rgdb_angularUpdate->setMaximum(3.14);
        rgdb_angularUpdate->setSingleStep(0.1);

        gridLayout_47->addWidget(rgdb_angularUpdate, 1, 0, 1, 1);

        label_scanMatching_2 = new QLabel(groupBox_slam_update);
        label_scanMatching_2->setObjectName(QStringLiteral("label_scanMatching_2"));
        label_scanMatching_2->setWordWrap(true);
        label_scanMatching_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_scanMatching_2, 4, 1, 1, 1);

        label_space2 = new QLabel(groupBox_slam_update);
        label_space2->setObjectName(QStringLiteral("label_space2"));
        label_space2->setWordWrap(true);
        label_space2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_space2, 15, 1, 1, 1);

        label_scanMatching_3 = new QLabel(groupBox_slam_update);
        label_scanMatching_3->setObjectName(QStringLiteral("label_scanMatching_3"));
        label_scanMatching_3->setWordWrap(true);
        label_scanMatching_3->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_scanMatching_3, 13, 1, 1, 1);

        spinBox_maxOdomCacheSize = new QSpinBox(groupBox_slam_update);
        spinBox_maxOdomCacheSize->setObjectName(QStringLiteral("spinBox_maxOdomCacheSize"));
        spinBox_maxOdomCacheSize->setMaximum(9999);

        gridLayout_47->addWidget(spinBox_maxOdomCacheSize, 17, 0, 1, 1);

        rgdb_angularSpeedUpdate = new QDoubleSpinBox(groupBox_slam_update);
        rgdb_angularSpeedUpdate->setObjectName(QStringLiteral("rgdb_angularSpeedUpdate"));
        rgdb_angularSpeedUpdate->setDecimals(2);
        rgdb_angularSpeedUpdate->setMaximum(3.14);
        rgdb_angularSpeedUpdate->setSingleStep(0.1);

        gridLayout_47->addWidget(rgdb_angularSpeedUpdate, 3, 0, 1, 1);

        label_scanMatching_12 = new QLabel(groupBox_slam_update);
        label_scanMatching_12->setObjectName(QStringLiteral("label_scanMatching_12"));
        label_scanMatching_12->setWordWrap(true);
        label_scanMatching_12->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_scanMatching_12, 12, 1, 1, 1);

        odomGravity = new QCheckBox(groupBox_slam_update);
        odomGravity->setObjectName(QStringLiteral("odomGravity"));

        gridLayout_47->addWidget(odomGravity, 7, 0, 1, 1);

        label_space2_5 = new QLabel(groupBox_slam_update);
        label_space2_5->setObjectName(QStringLiteral("label_space2_5"));
        label_space2_5->setWordWrap(true);
        label_space2_5->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_space2_5, 17, 1, 1, 1);

        label_scanMatching_7 = new QLabel(groupBox_slam_update);
        label_scanMatching_7->setObjectName(QStringLiteral("label_scanMatching_7"));
        label_scanMatching_7->setWordWrap(true);
        label_scanMatching_7->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_scanMatching_7, 7, 1, 1, 1);

        label_space2_12 = new QLabel(groupBox_slam_update);
        label_space2_12->setObjectName(QStringLiteral("label_space2_12"));
        label_space2_12->setWordWrap(true);
        label_space2_12->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_space2_12, 16, 1, 1, 1);

        maxLocalizationDistance = new QDoubleSpinBox(groupBox_slam_update);
        maxLocalizationDistance->setObjectName(QStringLiteral("maxLocalizationDistance"));
        maxLocalizationDistance->setValue(1);

        gridLayout_47->addWidget(maxLocalizationDistance, 16, 0, 1, 1);

        label_scanMatching_14 = new QLabel(groupBox_slam_update);
        label_scanMatching_14->setObjectName(QStringLiteral("label_scanMatching_14"));
        label_scanMatching_14->setWordWrap(true);
        label_scanMatching_14->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_47->addWidget(label_scanMatching_14, 8, 1, 1, 1);

        loopClosure_identityGuess = new QCheckBox(groupBox_slam_update);
        loopClosure_identityGuess->setObjectName(QStringLiteral("loopClosure_identityGuess"));

        gridLayout_47->addWidget(loopClosure_identityGuess, 8, 0, 1, 1);

        gridLayout_47->setColumnStretch(1, 1);

        verticalLayout_25->addWidget(groupBox_slam_update);


        verticalLayout_19->addWidget(groupBox_slam1);

        verticalSpacer_11 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_19->addItem(verticalSpacer_11);

        stackedWidget->addWidget(page_10);
        page_28 = new QWidget();
        page_28->setObjectName(QStringLiteral("page_28"));
        verticalLayout_95 = new QVBoxLayout(page_28);
        verticalLayout_95->setObjectName(QStringLiteral("verticalLayout_95"));
        groupBox_proximityDetection2 = new QGroupBox(page_28);
        groupBox_proximityDetection2->setObjectName(QStringLiteral("groupBox_proximityDetection2"));
        verticalLayout_96 = new QVBoxLayout(groupBox_proximityDetection2);
        verticalLayout_96->setObjectName(QStringLiteral("verticalLayout_96"));
        groupBox_localDetection_time = new QGroupBox(groupBox_proximityDetection2);
        groupBox_localDetection_time->setObjectName(QStringLiteral("groupBox_localDetection_time"));
        groupBox_localDetection_time->setCheckable(true);
        verticalLayout_21 = new QVBoxLayout(groupBox_localDetection_time);
        verticalLayout_21->setObjectName(QStringLiteral("verticalLayout_21"));
        label_54 = new QLabel(groupBox_localDetection_time);
        label_54->setObjectName(QStringLiteral("label_54"));
        label_54->setWordWrap(true);
        label_54->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_21->addWidget(label_54);


        verticalLayout_96->addWidget(groupBox_localDetection_time);

        groupBox_localDetection_space = new QGroupBox(groupBox_proximityDetection2);
        groupBox_localDetection_space->setObjectName(QStringLiteral("groupBox_localDetection_space"));
        groupBox_localDetection_space->setCheckable(true);
        verticalLayout_48 = new QVBoxLayout(groupBox_localDetection_space);
        verticalLayout_48->setObjectName(QStringLiteral("verticalLayout_48"));
        label_space1_2 = new QLabel(groupBox_localDetection_space);
        label_space1_2->setObjectName(QStringLiteral("label_space1_2"));
        label_space1_2->setWordWrap(true);
        label_space1_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_48->addWidget(label_space1_2);

        gridLayout_50 = new QGridLayout();
        gridLayout_50->setObjectName(QStringLiteral("gridLayout_50"));
        localDetection_pathFilteringRadius = new QDoubleSpinBox(groupBox_localDetection_space);
        localDetection_pathFilteringRadius->setObjectName(QStringLiteral("localDetection_pathFilteringRadius"));
        localDetection_pathFilteringRadius->setDecimals(2);
        localDetection_pathFilteringRadius->setSingleStep(0.1);
        localDetection_pathFilteringRadius->setValue(1);

        gridLayout_50->addWidget(localDetection_pathFilteringRadius, 5, 0, 1, 1);

        label_scanMatching_4 = new QLabel(groupBox_localDetection_space);
        label_scanMatching_4->setObjectName(QStringLiteral("label_scanMatching_4"));
        label_scanMatching_4->setWordWrap(true);
        label_scanMatching_4->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_50->addWidget(label_scanMatching_4, 6, 1, 1, 1);

        label_space3_3 = new QLabel(groupBox_localDetection_space);
        label_space3_3->setObjectName(QStringLiteral("label_space3_3"));
        label_space3_3->setWordWrap(true);
        label_space3_3->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_50->addWidget(label_space3_3, 5, 1, 1, 1);

        checkBox_localSpacePathOdomPosesUsed = new QCheckBox(groupBox_localDetection_space);
        checkBox_localSpacePathOdomPosesUsed->setObjectName(QStringLiteral("checkBox_localSpacePathOdomPosesUsed"));

        gridLayout_50->addWidget(checkBox_localSpacePathOdomPosesUsed, 6, 0, 1, 1);

        localDetection_maxDiffID = new QSpinBox(groupBox_localDetection_space);
        localDetection_maxDiffID->setObjectName(QStringLiteral("localDetection_maxDiffID"));
        localDetection_maxDiffID->setMaximum(999);
        localDetection_maxDiffID->setValue(0);

        gridLayout_50->addWidget(localDetection_maxDiffID, 0, 0, 1, 1);

        label_space3_2 = new QLabel(groupBox_localDetection_space);
        label_space3_2->setObjectName(QStringLiteral("label_space3_2"));
        label_space3_2->setWordWrap(true);
        label_space3_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_50->addWidget(label_space3_2, 0, 1, 1, 1);

        label_scanMatching_6 = new QLabel(groupBox_localDetection_space);
        label_scanMatching_6->setObjectName(QStringLiteral("label_scanMatching_6"));
        label_scanMatching_6->setWordWrap(true);
        label_scanMatching_6->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_50->addWidget(label_scanMatching_6, 8, 1, 1, 1);

        checkBox_localSpaceScanMatchingIDsSaved = new QCheckBox(groupBox_localDetection_space);
        checkBox_localSpaceScanMatchingIDsSaved->setObjectName(QStringLiteral("checkBox_localSpaceScanMatchingIDsSaved"));

        gridLayout_50->addWidget(checkBox_localSpaceScanMatchingIDsSaved, 8, 0, 1, 1);

        label_scanMatching_8 = new QLabel(groupBox_localDetection_space);
        label_scanMatching_8->setObjectName(QStringLiteral("label_scanMatching_8"));
        label_scanMatching_8->setWordWrap(true);
        label_scanMatching_8->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_50->addWidget(label_scanMatching_8, 3, 1, 1, 1);

        localDetection_angle = new QDoubleSpinBox(groupBox_localDetection_space);
        localDetection_angle->setObjectName(QStringLiteral("localDetection_angle"));
        localDetection_angle->setDecimals(0);
        localDetection_angle->setMaximum(360);
        localDetection_angle->setSingleStep(10);

        gridLayout_50->addWidget(localDetection_angle, 3, 0, 1, 1);

        label_space3_4 = new QLabel(groupBox_localDetection_space);
        label_space3_4->setObjectName(QStringLiteral("label_space3_4"));
        label_space3_4->setWordWrap(true);
        label_space3_4->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_50->addWidget(label_space3_4, 1, 1, 1, 1);

        localDetection_maxPaths = new QSpinBox(groupBox_localDetection_space);
        localDetection_maxPaths->setObjectName(QStringLiteral("localDetection_maxPaths"));
        localDetection_maxPaths->setMaximum(999);
        localDetection_maxPaths->setValue(0);

        gridLayout_50->addWidget(localDetection_maxPaths, 1, 0, 1, 1);

        label_space3_8 = new QLabel(groupBox_localDetection_space);
        label_space3_8->setObjectName(QStringLiteral("label_space3_8"));
        label_space3_8->setWordWrap(true);
        label_space3_8->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_50->addWidget(label_space3_8, 2, 1, 1, 1);

        localDetection_maxNeighbors = new QSpinBox(groupBox_localDetection_space);
        localDetection_maxNeighbors->setObjectName(QStringLiteral("localDetection_maxNeighbors"));
        localDetection_maxNeighbors->setMaximum(999);
        localDetection_maxNeighbors->setValue(0);

        gridLayout_50->addWidget(localDetection_maxNeighbors, 2, 0, 1, 1);

        label_scanMatching_13 = new QLabel(groupBox_localDetection_space);
        label_scanMatching_13->setObjectName(QStringLiteral("label_scanMatching_13"));
        label_scanMatching_13->setWordWrap(true);
        label_scanMatching_13->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_50->addWidget(label_scanMatching_13, 4, 1, 1, 1);

        checkBox_localSpaceOdomGuess = new QCheckBox(groupBox_localDetection_space);
        checkBox_localSpaceOdomGuess->setObjectName(QStringLiteral("checkBox_localSpaceOdomGuess"));

        gridLayout_50->addWidget(checkBox_localSpaceOdomGuess, 4, 0, 1, 1);

        gridLayout_50->setColumnStretch(1, 1);

        verticalLayout_48->addLayout(gridLayout_50);


        verticalLayout_96->addWidget(groupBox_localDetection_space);


        verticalLayout_95->addWidget(groupBox_proximityDetection2);

        verticalSpacer_49 = new QSpacerItem(20, 1396, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_95->addItem(verticalSpacer_49);

        stackedWidget->addWidget(page_28);
        page_19 = new QWidget();
        page_19->setObjectName(QStringLiteral("page_19"));
        verticalLayout_26 = new QVBoxLayout(page_19);
        verticalLayout_26->setObjectName(QStringLiteral("verticalLayout_26"));
        groupBox_graphOptimization2 = new QGroupBox(page_19);
        groupBox_graphOptimization2->setObjectName(QStringLiteral("groupBox_graphOptimization2"));
        gridLayout_81 = new QGridLayout(groupBox_graphOptimization2);
        gridLayout_81->setObjectName(QStringLiteral("gridLayout_81"));
        graphOptimization_type = new QComboBox(groupBox_graphOptimization2);
        graphOptimization_type->setObjectName(QStringLiteral("graphOptimization_type"));

        gridLayout_81->addWidget(graphOptimization_type, 0, 0, 1, 1);

        label_45 = new QLabel(groupBox_graphOptimization2);
        label_45->setObjectName(QStringLiteral("label_45"));
        label_45->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_81->addWidget(label_45, 0, 1, 1, 1);

        graphOptimization_robust = new QCheckBox(groupBox_graphOptimization2);
        graphOptimization_robust->setObjectName(QStringLiteral("graphOptimization_robust"));

        gridLayout_81->addWidget(graphOptimization_robust, 1, 0, 1, 1);

        label_258 = new QLabel(groupBox_graphOptimization2);
        label_258->setObjectName(QStringLiteral("label_258"));
        label_258->setWordWrap(true);
        label_258->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_81->addWidget(label_258, 1, 1, 1, 1);

        graphOptimization_maxError = new QDoubleSpinBox(groupBox_graphOptimization2);
        graphOptimization_maxError->setObjectName(QStringLiteral("graphOptimization_maxError"));
        graphOptimization_maxError->setDecimals(2);
        graphOptimization_maxError->setMinimum(0);
        graphOptimization_maxError->setMaximum(100);
        graphOptimization_maxError->setSingleStep(1);
        graphOptimization_maxError->setValue(1);

        gridLayout_81->addWidget(graphOptimization_maxError, 2, 0, 1, 1);

        label_259 = new QLabel(groupBox_graphOptimization2);
        label_259->setObjectName(QStringLiteral("label_259"));
        label_259->setWordWrap(true);
        label_259->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_81->addWidget(label_259, 2, 1, 1, 1);

        graphOptimization_fromGraphEnd = new QCheckBox(groupBox_graphOptimization2);
        graphOptimization_fromGraphEnd->setObjectName(QStringLiteral("graphOptimization_fromGraphEnd"));

        gridLayout_81->addWidget(graphOptimization_fromGraphEnd, 6, 0, 2, 1);

        line = new QFrame(groupBox_graphOptimization2);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        gridLayout_81->addWidget(line, 6, 1, 1, 1);

        label_151 = new QLabel(groupBox_graphOptimization2);
        label_151->setObjectName(QStringLiteral("label_151"));
        label_151->setWordWrap(true);
        label_151->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_81->addWidget(label_151, 7, 1, 1, 1);

        label_211 = new QLabel(groupBox_graphOptimization2);
        label_211->setObjectName(QStringLiteral("label_211"));
        label_211->setWordWrap(true);
        label_211->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_81->addWidget(label_211, 8, 1, 1, 1);

        label_183 = new QLabel(groupBox_graphOptimization2);
        label_183->setObjectName(QStringLiteral("label_183"));
        label_183->setWordWrap(true);
        label_183->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_81->addWidget(label_183, 9, 1, 1, 1);

        label_431 = new QLabel(groupBox_graphOptimization2);
        label_431->setObjectName(QStringLiteral("label_431"));
        label_431->setWordWrap(true);
        label_431->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_81->addWidget(label_431, 4, 1, 1, 1);

        graphOptimization_priorsIgnored = new QCheckBox(groupBox_graphOptimization2);
        graphOptimization_priorsIgnored->setObjectName(QStringLiteral("graphOptimization_priorsIgnored"));

        gridLayout_81->addWidget(graphOptimization_priorsIgnored, 4, 0, 1, 1);

        graphOptimization_landmarksIgnored = new QCheckBox(groupBox_graphOptimization2);
        graphOptimization_landmarksIgnored->setObjectName(QStringLiteral("graphOptimization_landmarksIgnored"));

        gridLayout_81->addWidget(graphOptimization_landmarksIgnored, 5, 0, 1, 1);

        label_521 = new QLabel(groupBox_graphOptimization2);
        label_521->setObjectName(QStringLiteral("label_521"));
        label_521->setWordWrap(true);
        label_521->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_81->addWidget(label_521, 5, 1, 1, 1);

        label_527 = new QLabel(groupBox_graphOptimization2);
        label_527->setObjectName(QStringLiteral("label_527"));
        label_527->setWordWrap(true);
        label_527->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_81->addWidget(label_527, 3, 1, 1, 1);

        graphOptimization_gravitySigma = new QDoubleSpinBox(groupBox_graphOptimization2);
        graphOptimization_gravitySigma->setObjectName(QStringLiteral("graphOptimization_gravitySigma"));
        graphOptimization_gravitySigma->setDecimals(4);
        graphOptimization_gravitySigma->setMinimum(0);
        graphOptimization_gravitySigma->setMaximum(10);
        graphOptimization_gravitySigma->setSingleStep(0.1);
        graphOptimization_gravitySigma->setValue(0);

        gridLayout_81->addWidget(graphOptimization_gravitySigma, 3, 0, 1, 1);

        gridLayout_81->setColumnStretch(1, 1);

        verticalLayout_26->addWidget(groupBox_graphOptimization2);

        verticalSpacer_48 = new QSpacerItem(17, 2098, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_26->addItem(verticalSpacer_48);

        stackedWidget->addWidget(page_19);
        page_64 = new QWidget();
        page_64->setObjectName(QStringLiteral("page_64"));
        verticalLayout_109 = new QVBoxLayout(page_64);
        verticalLayout_109->setObjectName(QStringLiteral("verticalLayout_109"));
        groupBox_occupancy2 = new QGroupBox(page_64);
        groupBox_occupancy2->setObjectName(QStringLiteral("groupBox_occupancy2"));
        groupBox_occupancy2->setCheckable(false);
        verticalLayout_110 = new QVBoxLayout(groupBox_occupancy2);
        verticalLayout_110->setObjectName(QStringLiteral("verticalLayout_110"));
        gridLayout_75 = new QGridLayout();
        gridLayout_75->setObjectName(QStringLiteral("gridLayout_75"));
        doubleSpinBox_grid_noiseRadius = new QDoubleSpinBox(groupBox_occupancy2);
        doubleSpinBox_grid_noiseRadius->setObjectName(QStringLiteral("doubleSpinBox_grid_noiseRadius"));
        doubleSpinBox_grid_noiseRadius->setDecimals(3);
        doubleSpinBox_grid_noiseRadius->setMaximum(1);
        doubleSpinBox_grid_noiseRadius->setSingleStep(0.05);
        doubleSpinBox_grid_noiseRadius->setValue(0);

        gridLayout_75->addWidget(doubleSpinBox_grid_noiseRadius, 8, 0, 1, 1);

        doubleSpinBox_grid_minGroundHeight = new QDoubleSpinBox(groupBox_occupancy2);
        doubleSpinBox_grid_minGroundHeight->setObjectName(QStringLiteral("doubleSpinBox_grid_minGroundHeight"));
        doubleSpinBox_grid_minGroundHeight->setDecimals(2);
        doubleSpinBox_grid_minGroundHeight->setMinimum(-99999);
        doubleSpinBox_grid_minGroundHeight->setMaximum(0);
        doubleSpinBox_grid_minGroundHeight->setSingleStep(0.01);
        doubleSpinBox_grid_minGroundHeight->setValue(-0.05);

        gridLayout_75->addWidget(doubleSpinBox_grid_minGroundHeight, 5, 0, 1, 1);

        label_335 = new QLabel(groupBox_occupancy2);
        label_335->setObjectName(QStringLiteral("label_335"));
        label_335->setWordWrap(true);
        label_335->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_335, 12, 1, 1, 1);

        doubleSpinBox_grid_maxObstacleHeight = new QDoubleSpinBox(groupBox_occupancy2);
        doubleSpinBox_grid_maxObstacleHeight->setObjectName(QStringLiteral("doubleSpinBox_grid_maxObstacleHeight"));
        doubleSpinBox_grid_maxObstacleHeight->setDecimals(2);
        doubleSpinBox_grid_maxObstacleHeight->setMaximum(9999);

        gridLayout_75->addWidget(doubleSpinBox_grid_maxObstacleHeight, 7, 0, 1, 1);

        doubleSpinBox_grid_maxGroundHeight = new QDoubleSpinBox(groupBox_occupancy2);
        doubleSpinBox_grid_maxGroundHeight->setObjectName(QStringLiteral("doubleSpinBox_grid_maxGroundHeight"));
        doubleSpinBox_grid_maxGroundHeight->setDecimals(2);
        doubleSpinBox_grid_maxGroundHeight->setMaximum(99999);
        doubleSpinBox_grid_maxGroundHeight->setSingleStep(0.01);

        gridLayout_75->addWidget(doubleSpinBox_grid_maxGroundHeight, 6, 0, 1, 1);

        label_321 = new QLabel(groupBox_occupancy2);
        label_321->setObjectName(QStringLiteral("label_321"));
        label_321->setWordWrap(true);
        label_321->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_321, 9, 1, 1, 1);

        label_310 = new QLabel(groupBox_occupancy2);
        label_310->setObjectName(QStringLiteral("label_310"));
        label_310->setWordWrap(true);
        label_310->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_310, 6, 1, 1, 1);

        label_309 = new QLabel(groupBox_occupancy2);
        label_309->setObjectName(QStringLiteral("label_309"));
        label_309->setWordWrap(true);
        label_309->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_309, 7, 1, 1, 1);

        spinBox_grid_noiseMinNeighbors = new QSpinBox(groupBox_occupancy2);
        spinBox_grid_noiseMinNeighbors->setObjectName(QStringLiteral("spinBox_grid_noiseMinNeighbors"));
        spinBox_grid_noiseMinNeighbors->setMinimum(1);
        spinBox_grid_noiseMinNeighbors->setMaximum(1000);
        spinBox_grid_noiseMinNeighbors->setValue(5);

        gridLayout_75->addWidget(spinBox_grid_noiseMinNeighbors, 9, 0, 1, 1);

        doubleSpinBox_grid_resolution = new QDoubleSpinBox(groupBox_occupancy2);
        doubleSpinBox_grid_resolution->setObjectName(QStringLiteral("doubleSpinBox_grid_resolution"));
        doubleSpinBox_grid_resolution->setMinimum(0.01);
        doubleSpinBox_grid_resolution->setMaximum(1);
        doubleSpinBox_grid_resolution->setValue(0.05);

        gridLayout_75->addWidget(doubleSpinBox_grid_resolution, 2, 0, 1, 1);

        label_331 = new QLabel(groupBox_occupancy2);
        label_331->setObjectName(QStringLiteral("label_331"));
        label_331->setWordWrap(true);
        label_331->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_331, 13, 1, 1, 1);

        spinBox_grid_scanDecimation = new QSpinBox(groupBox_occupancy2);
        spinBox_grid_scanDecimation->setObjectName(QStringLiteral("spinBox_grid_scanDecimation"));
        spinBox_grid_scanDecimation->setMinimum(1);
        spinBox_grid_scanDecimation->setMaximum(100);
        spinBox_grid_scanDecimation->setValue(1);

        gridLayout_75->addWidget(spinBox_grid_scanDecimation, 13, 0, 1, 1);

        label_333 = new QLabel(groupBox_occupancy2);
        label_333->setObjectName(QStringLiteral("label_333"));
        label_333->setWordWrap(true);
        label_333->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_333, 10, 1, 1, 1);

        label_334 = new QLabel(groupBox_occupancy2);
        label_334->setObjectName(QStringLiteral("label_334"));
        label_334->setWordWrap(true);
        label_334->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_334, 11, 1, 1, 1);

        doubleSpinBox_grid_footprintWidth = new QDoubleSpinBox(groupBox_occupancy2);
        doubleSpinBox_grid_footprintWidth->setObjectName(QStringLiteral("doubleSpinBox_grid_footprintWidth"));
        doubleSpinBox_grid_footprintWidth->setDecimals(2);
        doubleSpinBox_grid_footprintWidth->setMaximum(10);
        doubleSpinBox_grid_footprintWidth->setSingleStep(0.1);
        doubleSpinBox_grid_footprintWidth->setValue(0);

        gridLayout_75->addWidget(doubleSpinBox_grid_footprintWidth, 11, 0, 1, 1);

        doubleSpinBox_grid_footprintHeight = new QDoubleSpinBox(groupBox_occupancy2);
        doubleSpinBox_grid_footprintHeight->setObjectName(QStringLiteral("doubleSpinBox_grid_footprintHeight"));
        doubleSpinBox_grid_footprintHeight->setDecimals(2);
        doubleSpinBox_grid_footprintHeight->setMaximum(10);
        doubleSpinBox_grid_footprintHeight->setSingleStep(0.1);
        doubleSpinBox_grid_footprintHeight->setValue(0);

        gridLayout_75->addWidget(doubleSpinBox_grid_footprintHeight, 12, 0, 1, 1);

        label_327 = new QLabel(groupBox_occupancy2);
        label_327->setObjectName(QStringLiteral("label_327"));
        label_327->setWordWrap(true);
        label_327->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_327, 5, 1, 1, 1);

        label_326 = new QLabel(groupBox_occupancy2);
        label_326->setObjectName(QStringLiteral("label_326"));
        label_326->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_326, 2, 1, 1, 1);

        checkbox_rgbd_createOccupancyGrid = new QCheckBox(groupBox_occupancy2);
        checkbox_rgbd_createOccupancyGrid->setObjectName(QStringLiteral("checkbox_rgbd_createOccupancyGrid"));
        checkbox_rgbd_createOccupancyGrid->setChecked(false);

        gridLayout_75->addWidget(checkbox_rgbd_createOccupancyGrid, 0, 0, 1, 1);

        label_325 = new QLabel(groupBox_occupancy2);
        label_325->setObjectName(QStringLiteral("label_325"));
        label_325->setWordWrap(true);
        label_325->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_325, 8, 1, 1, 1);

        doubleSpinBox_grid_footprintLength = new QDoubleSpinBox(groupBox_occupancy2);
        doubleSpinBox_grid_footprintLength->setObjectName(QStringLiteral("doubleSpinBox_grid_footprintLength"));
        doubleSpinBox_grid_footprintLength->setDecimals(2);
        doubleSpinBox_grid_footprintLength->setMaximum(10);
        doubleSpinBox_grid_footprintLength->setSingleStep(0.1);
        doubleSpinBox_grid_footprintLength->setValue(0);

        gridLayout_75->addWidget(doubleSpinBox_grid_footprintLength, 10, 0, 1, 1);

        checkBox_grid_projMapFrame = new QCheckBox(groupBox_occupancy2);
        checkBox_grid_projMapFrame->setObjectName(QStringLiteral("checkBox_grid_projMapFrame"));
        checkBox_grid_projMapFrame->setChecked(false);

        gridLayout_75->addWidget(checkBox_grid_projMapFrame, 1, 0, 1, 1);

        label_365 = new QLabel(groupBox_occupancy2);
        label_365->setObjectName(QStringLiteral("label_365"));
        label_365->setWordWrap(true);
        label_365->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_365, 0, 1, 1, 1);

        label_311 = new QLabel(groupBox_occupancy2);
        label_311->setObjectName(QStringLiteral("label_311"));
        label_311->setWordWrap(true);
        label_311->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_311, 1, 1, 1, 1);

        label_323 = new QLabel(groupBox_occupancy2);
        label_323->setObjectName(QStringLiteral("label_323"));
        label_323->setWordWrap(true);
        label_323->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_323, 3, 1, 1, 1);

        doubleSpinBox_grid_minDepth = new QDoubleSpinBox(groupBox_occupancy2);
        doubleSpinBox_grid_minDepth->setObjectName(QStringLiteral("doubleSpinBox_grid_minDepth"));
        doubleSpinBox_grid_minDepth->setDecimals(1);
        doubleSpinBox_grid_minDepth->setMaximum(100);
        doubleSpinBox_grid_minDepth->setSingleStep(0.1);
        doubleSpinBox_grid_minDepth->setValue(0);

        gridLayout_75->addWidget(doubleSpinBox_grid_minDepth, 3, 0, 1, 1);

        label_320 = new QLabel(groupBox_occupancy2);
        label_320->setObjectName(QStringLiteral("label_320"));
        label_320->setWordWrap(true);
        label_320->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_320, 4, 1, 1, 1);

        doubleSpinBox_grid_maxDepth = new QDoubleSpinBox(groupBox_occupancy2);
        doubleSpinBox_grid_maxDepth->setObjectName(QStringLiteral("doubleSpinBox_grid_maxDepth"));
        doubleSpinBox_grid_maxDepth->setDecimals(1);
        doubleSpinBox_grid_maxDepth->setMaximum(100);
        doubleSpinBox_grid_maxDepth->setSingleStep(1);
        doubleSpinBox_grid_maxDepth->setValue(4);

        gridLayout_75->addWidget(doubleSpinBox_grid_maxDepth, 4, 0, 1, 1);

        label_456 = new QLabel(groupBox_occupancy2);
        label_456->setObjectName(QStringLiteral("label_456"));
        label_456->setWordWrap(true);
        label_456->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_75->addWidget(label_456, 14, 1, 1, 1);

        checkBox_grid_preVoxelFiltering = new QCheckBox(groupBox_occupancy2);
        checkBox_grid_preVoxelFiltering->setObjectName(QStringLiteral("checkBox_grid_preVoxelFiltering"));
        checkBox_grid_preVoxelFiltering->setChecked(false);

        gridLayout_75->addWidget(checkBox_grid_preVoxelFiltering, 14, 0, 1, 1);

        gridLayout_75->setColumnStretch(1, 1);

        verticalLayout_110->addLayout(gridLayout_75);

        groupBox_grid_fromDepthImage = new QGroupBox(groupBox_occupancy2);
        groupBox_grid_fromDepthImage->setObjectName(QStringLiteral("groupBox_grid_fromDepthImage"));
        groupBox_grid_fromDepthImage->setCheckable(true);
        verticalLayout_108 = new QVBoxLayout(groupBox_grid_fromDepthImage);
        verticalLayout_108->setObjectName(QStringLiteral("verticalLayout_108"));
        label_octomap_groundObstacle_2 = new QLabel(groupBox_grid_fromDepthImage);
        label_octomap_groundObstacle_2->setObjectName(QStringLiteral("label_octomap_groundObstacle_2"));
        label_octomap_groundObstacle_2->setWordWrap(true);
        label_octomap_groundObstacle_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_108->addWidget(label_octomap_groundObstacle_2);

        gridLayout_76 = new QGridLayout();
        gridLayout_76->setObjectName(QStringLiteral("gridLayout_76"));
        spinBox_grid_decimation = new QSpinBox(groupBox_grid_fromDepthImage);
        spinBox_grid_decimation->setObjectName(QStringLiteral("spinBox_grid_decimation"));
        spinBox_grid_decimation->setMinimum(-32);
        spinBox_grid_decimation->setMaximum(32);
        spinBox_grid_decimation->setValue(4);

        gridLayout_76->addWidget(spinBox_grid_decimation, 0, 0, 1, 1);

        label_322 = new QLabel(groupBox_grid_fromDepthImage);
        label_322->setObjectName(QStringLiteral("label_322"));
        label_322->setWordWrap(true);
        label_322->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_76->addWidget(label_322, 0, 1, 1, 1);

        lineEdit_grid_roi = new QLineEdit(groupBox_grid_fromDepthImage);
        lineEdit_grid_roi->setObjectName(QStringLiteral("lineEdit_grid_roi"));
        lineEdit_grid_roi->setReadOnly(true);

        gridLayout_76->addWidget(lineEdit_grid_roi, 3, 0, 1, 1);

        label_324 = new QLabel(groupBox_grid_fromDepthImage);
        label_324->setObjectName(QStringLiteral("label_324"));
        label_324->setWordWrap(true);
        label_324->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_76->addWidget(label_324, 3, 1, 1, 1);

        label_363 = new QLabel(groupBox_grid_fromDepthImage);
        label_363->setObjectName(QStringLiteral("label_363"));
        label_363->setWordWrap(true);
        label_363->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_76->addWidget(label_363, 4, 1, 1, 1);

        checkBox_grid_projRayTracing = new QCheckBox(groupBox_grid_fromDepthImage);
        checkBox_grid_projRayTracing->setObjectName(QStringLiteral("checkBox_grid_projRayTracing"));
        checkBox_grid_projRayTracing->setChecked(false);

        gridLayout_76->addWidget(checkBox_grid_projRayTracing, 4, 0, 1, 1);

        gridLayout_76->setColumnStretch(1, 1);

        verticalLayout_108->addLayout(gridLayout_76);


        verticalLayout_110->addWidget(groupBox_grid_fromDepthImage);

        groupBox_grid_normalsSegmentation = new QGroupBox(groupBox_occupancy2);
        groupBox_grid_normalsSegmentation->setObjectName(QStringLiteral("groupBox_grid_normalsSegmentation"));
        groupBox_grid_normalsSegmentation->setCheckable(true);
        verticalLayout_107 = new QVBoxLayout(groupBox_grid_normalsSegmentation);
        verticalLayout_107->setObjectName(QStringLiteral("verticalLayout_107"));
        label_312 = new QLabel(groupBox_grid_normalsSegmentation);
        label_312->setObjectName(QStringLiteral("label_312"));
        label_312->setWordWrap(true);
        label_312->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_107->addWidget(label_312);

        gridLayout_74 = new QGridLayout();
        gridLayout_74->setObjectName(QStringLiteral("gridLayout_74"));
        doubleSpinBox_grid_maxGroundAngle = new QDoubleSpinBox(groupBox_grid_normalsSegmentation);
        doubleSpinBox_grid_maxGroundAngle->setObjectName(QStringLiteral("doubleSpinBox_grid_maxGroundAngle"));
        doubleSpinBox_grid_maxGroundAngle->setDecimals(0);
        doubleSpinBox_grid_maxGroundAngle->setMinimum(0);
        doubleSpinBox_grid_maxGroundAngle->setMaximum(180);
        doubleSpinBox_grid_maxGroundAngle->setValue(30);

        gridLayout_74->addWidget(doubleSpinBox_grid_maxGroundAngle, 0, 0, 1, 1);

        spinBox_grid_minClusterSize = new QSpinBox(groupBox_grid_normalsSegmentation);
        spinBox_grid_minClusterSize->setObjectName(QStringLiteral("spinBox_grid_minClusterSize"));
        spinBox_grid_minClusterSize->setMinimum(1);
        spinBox_grid_minClusterSize->setMaximum(10000);
        spinBox_grid_minClusterSize->setValue(20);

        gridLayout_74->addWidget(spinBox_grid_minClusterSize, 3, 0, 1, 1);

        label_307 = new QLabel(groupBox_grid_normalsSegmentation);
        label_307->setObjectName(QStringLiteral("label_307"));
        label_307->setWordWrap(true);
        label_307->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_74->addWidget(label_307, 0, 1, 1, 1);

        label_308 = new QLabel(groupBox_grid_normalsSegmentation);
        label_308->setObjectName(QStringLiteral("label_308"));
        label_308->setWordWrap(true);
        label_308->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_74->addWidget(label_308, 3, 1, 1, 1);

        label_272 = new QLabel(groupBox_grid_normalsSegmentation);
        label_272->setObjectName(QStringLiteral("label_272"));
        label_272->setWordWrap(true);
        label_272->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_74->addWidget(label_272, 4, 1, 1, 1);

        checkBox_grid_flatObstaclesDetected = new QCheckBox(groupBox_grid_normalsSegmentation);
        checkBox_grid_flatObstaclesDetected->setObjectName(QStringLiteral("checkBox_grid_flatObstaclesDetected"));
        checkBox_grid_flatObstaclesDetected->setChecked(false);

        gridLayout_74->addWidget(checkBox_grid_flatObstaclesDetected, 4, 0, 1, 1);

        spinBox_grid_normalK = new QSpinBox(groupBox_grid_normalsSegmentation);
        spinBox_grid_normalK->setObjectName(QStringLiteral("spinBox_grid_normalK"));
        spinBox_grid_normalK->setMinimum(3);
        spinBox_grid_normalK->setMaximum(1000);
        spinBox_grid_normalK->setValue(20);

        gridLayout_74->addWidget(spinBox_grid_normalK, 1, 0, 1, 1);

        label_330 = new QLabel(groupBox_grid_normalsSegmentation);
        label_330->setObjectName(QStringLiteral("label_330"));
        label_330->setWordWrap(true);
        label_330->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_74->addWidget(label_330, 1, 1, 1, 1);

        label_336 = new QLabel(groupBox_grid_normalsSegmentation);
        label_336->setObjectName(QStringLiteral("label_336"));
        label_336->setWordWrap(true);
        label_336->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_74->addWidget(label_336, 2, 1, 1, 1);

        doubleSpinBox_grid_clusterRadius = new QDoubleSpinBox(groupBox_grid_normalsSegmentation);
        doubleSpinBox_grid_clusterRadius->setObjectName(QStringLiteral("doubleSpinBox_grid_clusterRadius"));
        doubleSpinBox_grid_clusterRadius->setDecimals(3);
        doubleSpinBox_grid_clusterRadius->setMinimum(0.001);
        doubleSpinBox_grid_clusterRadius->setMaximum(1);
        doubleSpinBox_grid_clusterRadius->setSingleStep(0.01);
        doubleSpinBox_grid_clusterRadius->setValue(0.1);

        gridLayout_74->addWidget(doubleSpinBox_grid_clusterRadius, 2, 0, 1, 1);

        gridLayout_74->setColumnStretch(1, 1);

        verticalLayout_107->addLayout(gridLayout_74);


        verticalLayout_110->addWidget(groupBox_grid_normalsSegmentation);

        groupBox_grid_3d = new QGroupBox(groupBox_occupancy2);
        groupBox_grid_3d->setObjectName(QStringLiteral("groupBox_grid_3d"));
        groupBox_grid_3d->setCheckable(true);
        verticalLayout_102 = new QVBoxLayout(groupBox_grid_3d);
        verticalLayout_102->setObjectName(QStringLiteral("verticalLayout_102"));
        label_octomap_groundObstacle_3 = new QLabel(groupBox_grid_3d);
        label_octomap_groundObstacle_3->setObjectName(QStringLiteral("label_octomap_groundObstacle_3"));
        label_octomap_groundObstacle_3->setWordWrap(true);
        label_octomap_groundObstacle_3->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_102->addWidget(label_octomap_groundObstacle_3);

        gridLayout_67 = new QGridLayout();
        gridLayout_67->setObjectName(QStringLiteral("gridLayout_67"));
        label_octomap_groundObstacle = new QLabel(groupBox_grid_3d);
        label_octomap_groundObstacle->setObjectName(QStringLiteral("label_octomap_groundObstacle"));
        label_octomap_groundObstacle->setWordWrap(true);
        label_octomap_groundObstacle->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_67->addWidget(label_octomap_groundObstacle, 0, 1, 1, 1);

        checkBox_grid_groundObstacle = new QCheckBox(groupBox_grid_3d);
        checkBox_grid_groundObstacle->setObjectName(QStringLiteral("checkBox_grid_groundObstacle"));
        checkBox_grid_groundObstacle->setChecked(true);

        gridLayout_67->addWidget(checkBox_grid_groundObstacle, 0, 0, 1, 1);

        gridLayout_67->setColumnStretch(1, 1);

        verticalLayout_102->addLayout(gridLayout_67);


        verticalLayout_110->addWidget(groupBox_grid_3d);

        groupBox_9 = new QGroupBox(groupBox_occupancy2);
        groupBox_9->setObjectName(QStringLiteral("groupBox_9"));
        verticalLayout_111 = new QVBoxLayout(groupBox_9);
        verticalLayout_111->setObjectName(QStringLiteral("verticalLayout_111"));
        label_octomap_groundObstacle_5 = new QLabel(groupBox_9);
        label_octomap_groundObstacle_5->setObjectName(QStringLiteral("label_octomap_groundObstacle_5"));
        label_octomap_groundObstacle_5->setWordWrap(true);
        label_octomap_groundObstacle_5->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_111->addWidget(label_octomap_groundObstacle_5);

        gridLayout_77 = new QGridLayout();
        gridLayout_77->setObjectName(QStringLiteral("gridLayout_77"));
        checkBox_grid_unknownSpaceFilled = new QCheckBox(groupBox_9);
        checkBox_grid_unknownSpaceFilled->setObjectName(QStringLiteral("checkBox_grid_unknownSpaceFilled"));

        gridLayout_77->addWidget(checkBox_grid_unknownSpaceFilled, 0, 0, 1, 1);

        label_328 = new QLabel(groupBox_9);
        label_328->setObjectName(QStringLiteral("label_328"));
        label_328->setWordWrap(true);
        label_328->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_77->addWidget(label_328, 0, 1, 1, 1);

        gridLayout_77->setColumnStretch(1, 1);

        verticalLayout_111->addLayout(gridLayout_77);


        verticalLayout_110->addWidget(groupBox_9);


        verticalLayout_109->addWidget(groupBox_occupancy2);

        verticalSpacer_57 = new QSpacerItem(20, 1177, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_109->addItem(verticalSpacer_57);

        stackedWidget->addWidget(page_64);
        page_66 = new QWidget();
        page_66->setObjectName(QStringLiteral("page_66"));
        verticalLayout_116 = new QVBoxLayout(page_66);
        verticalLayout_116->setObjectName(QStringLiteral("verticalLayout_116"));
        groupBox_occupancyGlobal2 = new QGroupBox(page_66);
        groupBox_occupancyGlobal2->setObjectName(QStringLiteral("groupBox_occupancyGlobal2"));
        groupBox_occupancyGlobal2->setCheckable(false);
        verticalLayout_115 = new QVBoxLayout(groupBox_occupancyGlobal2);
        verticalLayout_115->setObjectName(QStringLiteral("verticalLayout_115"));
        gridLayout_88 = new QGridLayout();
        gridLayout_88->setObjectName(QStringLiteral("gridLayout_88"));
        label_484 = new QLabel(groupBox_occupancyGlobal2);
        label_484->setObjectName(QStringLiteral("label_484"));
        label_484->setWordWrap(true);
        label_484->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_88->addWidget(label_484, 9, 1, 1, 1);

        checkBox_grid_fullUpdate = new QCheckBox(groupBox_occupancyGlobal2);
        checkBox_grid_fullUpdate->setObjectName(QStringLiteral("checkBox_grid_fullUpdate"));
        checkBox_grid_fullUpdate->setChecked(false);

        gridLayout_88->addWidget(checkBox_grid_fullUpdate, 0, 0, 1, 1);

        label_332 = new QLabel(groupBox_occupancyGlobal2);
        label_332->setObjectName(QStringLiteral("label_332"));
        label_332->setWordWrap(true);
        label_332->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_88->addWidget(label_332, 0, 1, 1, 1);

        label_224 = new QLabel(groupBox_occupancyGlobal2);
        label_224->setObjectName(QStringLiteral("label_224"));
        label_224->setWordWrap(true);
        label_224->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_88->addWidget(label_224, 11, 1, 1, 1);

        label_482 = new QLabel(groupBox_occupancyGlobal2);
        label_482->setObjectName(QStringLiteral("label_482"));
        label_482->setWordWrap(true);
        label_482->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_88->addWidget(label_482, 7, 1, 1, 1);

        label_319 = new QLabel(groupBox_occupancyGlobal2);
        label_319->setObjectName(QStringLiteral("label_319"));
        label_319->setWordWrap(true);
        label_319->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_88->addWidget(label_319, 5, 1, 1, 1);

        checkBox_grid_erode = new QCheckBox(groupBox_occupancyGlobal2);
        checkBox_grid_erode->setObjectName(QStringLiteral("checkBox_grid_erode"));
        checkBox_grid_erode->setChecked(false);

        gridLayout_88->addWidget(checkBox_grid_erode, 11, 0, 1, 1);

        spinBox_grid_maxNodes = new QSpinBox(groupBox_occupancyGlobal2);
        spinBox_grid_maxNodes->setObjectName(QStringLiteral("spinBox_grid_maxNodes"));
        spinBox_grid_maxNodes->setMaximum(9999);

        gridLayout_88->addWidget(spinBox_grid_maxNodes, 3, 0, 1, 1);

        doubleSpinBox_grid_footprintRadius = new QDoubleSpinBox(groupBox_occupancyGlobal2);
        doubleSpinBox_grid_footprintRadius->setObjectName(QStringLiteral("doubleSpinBox_grid_footprintRadius"));
        doubleSpinBox_grid_footprintRadius->setMinimum(0);
        doubleSpinBox_grid_footprintRadius->setMaximum(10);
        doubleSpinBox_grid_footprintRadius->setSingleStep(0.1);
        doubleSpinBox_grid_footprintRadius->setValue(0);

        gridLayout_88->addWidget(doubleSpinBox_grid_footprintRadius, 5, 0, 1, 1);

        label_366 = new QLabel(groupBox_occupancyGlobal2);
        label_366->setObjectName(QStringLiteral("label_366"));
        label_366->setWordWrap(true);
        label_366->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_88->addWidget(label_366, 2, 1, 1, 1);

        doubleSpinBox_grid_minMapSize = new QDoubleSpinBox(groupBox_occupancyGlobal2);
        doubleSpinBox_grid_minMapSize->setObjectName(QStringLiteral("doubleSpinBox_grid_minMapSize"));
        doubleSpinBox_grid_minMapSize->setDecimals(0);
        doubleSpinBox_grid_minMapSize->setMinimum(0);
        doubleSpinBox_grid_minMapSize->setMaximum(9999);
        doubleSpinBox_grid_minMapSize->setSingleStep(10);
        doubleSpinBox_grid_minMapSize->setValue(0);

        gridLayout_88->addWidget(doubleSpinBox_grid_minMapSize, 2, 0, 1, 1);

        label_455 = new QLabel(groupBox_occupancyGlobal2);
        label_455->setObjectName(QStringLiteral("label_455"));
        label_455->setWordWrap(true);
        label_455->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_88->addWidget(label_455, 3, 1, 1, 1);

        label_454 = new QLabel(groupBox_occupancyGlobal2);
        label_454->setObjectName(QStringLiteral("label_454"));
        label_454->setWordWrap(true);
        label_454->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_88->addWidget(label_454, 1, 1, 1, 1);

        doubleSpinBox_grid_updateError = new QDoubleSpinBox(groupBox_occupancyGlobal2);
        doubleSpinBox_grid_updateError->setObjectName(QStringLiteral("doubleSpinBox_grid_updateError"));
        doubleSpinBox_grid_updateError->setDecimals(3);
        doubleSpinBox_grid_updateError->setMinimum(0.001);
        doubleSpinBox_grid_updateError->setMaximum(99);
        doubleSpinBox_grid_updateError->setSingleStep(0.1);
        doubleSpinBox_grid_updateError->setValue(0.01);

        gridLayout_88->addWidget(doubleSpinBox_grid_updateError, 1, 0, 1, 1);

        label_457 = new QLabel(groupBox_occupancyGlobal2);
        label_457->setObjectName(QStringLiteral("label_457"));
        label_457->setWordWrap(true);
        label_457->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_88->addWidget(label_457, 6, 1, 1, 1);

        doubleSpinBox_grid_occThr = new QDoubleSpinBox(groupBox_occupancyGlobal2);
        doubleSpinBox_grid_occThr->setObjectName(QStringLiteral("doubleSpinBox_grid_occThr"));
        doubleSpinBox_grid_occThr->setMinimum(0);
        doubleSpinBox_grid_occThr->setMaximum(1);
        doubleSpinBox_grid_occThr->setSingleStep(0.1);
        doubleSpinBox_grid_occThr->setValue(0);

        gridLayout_88->addWidget(doubleSpinBox_grid_occThr, 6, 0, 1, 1);

        label_483 = new QLabel(groupBox_occupancyGlobal2);
        label_483->setObjectName(QStringLiteral("label_483"));
        label_483->setWordWrap(true);
        label_483->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_88->addWidget(label_483, 8, 1, 1, 1);

        doubleSpinBox_grid_probHit = new QDoubleSpinBox(groupBox_occupancyGlobal2);
        doubleSpinBox_grid_probHit->setObjectName(QStringLiteral("doubleSpinBox_grid_probHit"));
        doubleSpinBox_grid_probHit->setMinimum(0.5);
        doubleSpinBox_grid_probHit->setMaximum(1);
        doubleSpinBox_grid_probHit->setSingleStep(0.1);
        doubleSpinBox_grid_probHit->setValue(0.5);

        gridLayout_88->addWidget(doubleSpinBox_grid_probHit, 7, 0, 1, 1);

        doubleSpinBox_grid_probMiss = new QDoubleSpinBox(groupBox_occupancyGlobal2);
        doubleSpinBox_grid_probMiss->setObjectName(QStringLiteral("doubleSpinBox_grid_probMiss"));
        doubleSpinBox_grid_probMiss->setMinimum(0);
        doubleSpinBox_grid_probMiss->setMaximum(0.5);
        doubleSpinBox_grid_probMiss->setSingleStep(0.1);
        doubleSpinBox_grid_probMiss->setValue(0.5);

        gridLayout_88->addWidget(doubleSpinBox_grid_probMiss, 8, 0, 1, 1);

        label_485 = new QLabel(groupBox_occupancyGlobal2);
        label_485->setObjectName(QStringLiteral("label_485"));
        label_485->setWordWrap(true);
        label_485->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_88->addWidget(label_485, 10, 1, 1, 1);

        doubleSpinBox_grid_clampingMin = new QDoubleSpinBox(groupBox_occupancyGlobal2);
        doubleSpinBox_grid_clampingMin->setObjectName(QStringLiteral("doubleSpinBox_grid_clampingMin"));
        doubleSpinBox_grid_clampingMin->setDecimals(4);
        doubleSpinBox_grid_clampingMin->setMinimum(0);
        doubleSpinBox_grid_clampingMin->setMaximum(1);
        doubleSpinBox_grid_clampingMin->setSingleStep(0.1);
        doubleSpinBox_grid_clampingMin->setValue(0.5);

        gridLayout_88->addWidget(doubleSpinBox_grid_clampingMin, 9, 0, 1, 1);

        doubleSpinBox_grid_clampingMax = new QDoubleSpinBox(groupBox_occupancyGlobal2);
        doubleSpinBox_grid_clampingMax->setObjectName(QStringLiteral("doubleSpinBox_grid_clampingMax"));
        doubleSpinBox_grid_clampingMax->setDecimals(4);
        doubleSpinBox_grid_clampingMax->setMinimum(0);
        doubleSpinBox_grid_clampingMax->setMaximum(1);
        doubleSpinBox_grid_clampingMax->setSingleStep(0.1);
        doubleSpinBox_grid_clampingMax->setValue(0.5);

        gridLayout_88->addWidget(doubleSpinBox_grid_clampingMax, 10, 0, 1, 1);

        label_616 = new QLabel(groupBox_occupancyGlobal2);
        label_616->setObjectName(QStringLiteral("label_616"));
        label_616->setWordWrap(true);
        label_616->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_88->addWidget(label_616, 4, 1, 1, 1);

        doubleSpinBox_grid_altitudeDelta = new QDoubleSpinBox(groupBox_occupancyGlobal2);
        doubleSpinBox_grid_altitudeDelta->setObjectName(QStringLiteral("doubleSpinBox_grid_altitudeDelta"));
        doubleSpinBox_grid_altitudeDelta->setDecimals(2);
        doubleSpinBox_grid_altitudeDelta->setMinimum(0);
        doubleSpinBox_grid_altitudeDelta->setMaximum(99);
        doubleSpinBox_grid_altitudeDelta->setSingleStep(0.1);
        doubleSpinBox_grid_altitudeDelta->setValue(0);

        gridLayout_88->addWidget(doubleSpinBox_grid_altitudeDelta, 4, 0, 1, 1);

        gridLayout_88->setColumnStretch(1, 1);

        verticalLayout_115->addLayout(gridLayout_88);


        verticalLayout_116->addWidget(groupBox_occupancyGlobal2);

        verticalSpacer_59 = new QSpacerItem(20, 2473, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_116->addItem(verticalSpacer_59);

        stackedWidget->addWidget(page_66);
        page_30 = new QWidget();
        page_30->setObjectName(QStringLiteral("page_30"));
        verticalLayout_97 = new QVBoxLayout(page_30);
        verticalLayout_97->setObjectName(QStringLiteral("verticalLayout_97"));
        groupBox_pathPlanning2 = new QGroupBox(page_30);
        groupBox_pathPlanning2->setObjectName(QStringLiteral("groupBox_pathPlanning2"));
        verticalLayout_61 = new QVBoxLayout(groupBox_pathPlanning2);
        verticalLayout_61->setObjectName(QStringLiteral("verticalLayout_61"));
        label_space2_3 = new QLabel(groupBox_pathPlanning2);
        label_space2_3->setObjectName(QStringLiteral("label_space2_3"));
        label_space2_3->setWordWrap(true);
        label_space2_3->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_61->addWidget(label_space2_3);

        gridLayout_51 = new QGridLayout();
        gridLayout_51->setObjectName(QStringLiteral("gridLayout_51"));
        graphPlan_goalReachedRadius = new QDoubleSpinBox(groupBox_pathPlanning2);
        graphPlan_goalReachedRadius->setObjectName(QStringLiteral("graphPlan_goalReachedRadius"));
        graphPlan_goalReachedRadius->setValue(1);

        gridLayout_51->addWidget(graphPlan_goalReachedRadius, 0, 0, 1, 1);

        label_space2_2 = new QLabel(groupBox_pathPlanning2);
        label_space2_2->setObjectName(QStringLiteral("label_space2_2"));
        label_space2_2->setWordWrap(true);
        label_space2_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_51->addWidget(label_space2_2, 0, 1, 1, 1);

        label_space3_5 = new QLabel(groupBox_pathPlanning2);
        label_space3_5->setObjectName(QStringLiteral("label_space3_5"));
        label_space3_5->setWordWrap(true);
        label_space3_5->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_51->addWidget(label_space3_5, 2, 1, 1, 1);

        graphPlan_goalsSavedInUserData = new QCheckBox(groupBox_pathPlanning2);
        graphPlan_goalsSavedInUserData->setObjectName(QStringLiteral("graphPlan_goalsSavedInUserData"));

        gridLayout_51->addWidget(graphPlan_goalsSavedInUserData, 2, 0, 1, 1);

        label_space2_4 = new QLabel(groupBox_pathPlanning2);
        label_space2_4->setObjectName(QStringLiteral("label_space2_4"));
        label_space2_4->setWordWrap(true);
        label_space2_4->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_51->addWidget(label_space2_4, 1, 1, 1, 1);

        graphPlan_stuckIterations = new QSpinBox(groupBox_pathPlanning2);
        graphPlan_stuckIterations->setObjectName(QStringLiteral("graphPlan_stuckIterations"));
        graphPlan_stuckIterations->setMaximum(9999);

        gridLayout_51->addWidget(graphPlan_stuckIterations, 1, 0, 1, 1);

        graphPlan_linearVelocity = new QDoubleSpinBox(groupBox_pathPlanning2);
        graphPlan_linearVelocity->setObjectName(QStringLiteral("graphPlan_linearVelocity"));
        graphPlan_linearVelocity->setSingleStep(0.1);
        graphPlan_linearVelocity->setValue(0);

        gridLayout_51->addWidget(graphPlan_linearVelocity, 3, 0, 1, 1);

        graphPlan_angularVelocity = new QDoubleSpinBox(groupBox_pathPlanning2);
        graphPlan_angularVelocity->setObjectName(QStringLiteral("graphPlan_angularVelocity"));
        graphPlan_angularVelocity->setMaximum(10);
        graphPlan_angularVelocity->setValue(0);

        gridLayout_51->addWidget(graphPlan_angularVelocity, 4, 0, 1, 1);

        label_space3_6 = new QLabel(groupBox_pathPlanning2);
        label_space3_6->setObjectName(QStringLiteral("label_space3_6"));
        label_space3_6->setWordWrap(true);
        label_space3_6->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_51->addWidget(label_space3_6, 3, 1, 1, 1);

        label_space3_7 = new QLabel(groupBox_pathPlanning2);
        label_space3_7->setObjectName(QStringLiteral("label_space3_7"));
        label_space3_7->setWordWrap(true);
        label_space3_7->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_51->addWidget(label_space3_7, 4, 1, 1, 1);

        gridLayout_51->setColumnStretch(1, 1);

        verticalLayout_61->addLayout(gridLayout_51);


        verticalLayout_97->addWidget(groupBox_pathPlanning2);

        verticalSpacer_50 = new QSpacerItem(20, 1529, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_97->addItem(verticalSpacer_50);

        stackedWidget->addWidget(page_30);
        page_78 = new QWidget();
        page_78->setObjectName(QStringLiteral("page_78"));
        verticalLayout_137 = new QVBoxLayout(page_78);
        verticalLayout_137->setObjectName(QStringLiteral("verticalLayout_137"));
        groupBox_markerDetection2 = new QGroupBox(page_78);
        groupBox_markerDetection2->setObjectName(QStringLiteral("groupBox_markerDetection2"));
        verticalLayout_136 = new QVBoxLayout(groupBox_markerDetection2);
        verticalLayout_136->setObjectName(QStringLiteral("verticalLayout_136"));
        gridLayout_63 = new QGridLayout();
        gridLayout_63->setObjectName(QStringLiteral("gridLayout_63"));
        label_space2_9 = new QLabel(groupBox_markerDetection2);
        label_space2_9->setObjectName(QStringLiteral("label_space2_9"));
        label_space2_9->setWordWrap(true);
        label_space2_9->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_63->addWidget(label_space2_9, 4, 1, 1, 1);

        ArucoVarianceLinear = new QDoubleSpinBox(groupBox_markerDetection2);
        ArucoVarianceLinear->setObjectName(QStringLiteral("ArucoVarianceLinear"));
        ArucoVarianceLinear->setDecimals(6);
        ArucoVarianceLinear->setMinimum(1e-6);
        ArucoVarianceLinear->setSingleStep(0.001);
        ArucoVarianceLinear->setValue(0.001);

        gridLayout_63->addWidget(ArucoVarianceLinear, 3, 0, 1, 1);

        label_markerDetection = new QLabel(groupBox_markerDetection2);
        label_markerDetection->setObjectName(QStringLiteral("label_markerDetection"));
        label_markerDetection->setWordWrap(true);
        label_markerDetection->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_63->addWidget(label_markerDetection, 0, 1, 1, 1);

        label_space2_6 = new QLabel(groupBox_markerDetection2);
        label_space2_6->setObjectName(QStringLiteral("label_space2_6"));
        label_space2_6->setWordWrap(true);
        label_space2_6->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_63->addWidget(label_space2_6, 3, 1, 1, 1);

        label_space2_7 = new QLabel(groupBox_markerDetection2);
        label_space2_7->setObjectName(QStringLiteral("label_space2_7"));
        label_space2_7->setWordWrap(true);
        label_space2_7->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_63->addWidget(label_space2_7, 1, 1, 1, 1);

        RGBDMarkerDetection = new QCheckBox(groupBox_markerDetection2);
        RGBDMarkerDetection->setObjectName(QStringLiteral("RGBDMarkerDetection"));

        gridLayout_63->addWidget(RGBDMarkerDetection, 0, 0, 1, 1);

        ArucoVarianceAngular = new QDoubleSpinBox(groupBox_markerDetection2);
        ArucoVarianceAngular->setObjectName(QStringLiteral("ArucoVarianceAngular"));
        ArucoVarianceAngular->setDecimals(6);
        ArucoVarianceAngular->setMinimum(1e-6);
        ArucoVarianceAngular->setMaximum(9999);
        ArucoVarianceAngular->setSingleStep(0.001);
        ArucoVarianceAngular->setValue(0.001);

        gridLayout_63->addWidget(ArucoVarianceAngular, 4, 0, 1, 1);

        ArucoMarkerLength = new QDoubleSpinBox(groupBox_markerDetection2);
        ArucoMarkerLength->setObjectName(QStringLiteral("ArucoMarkerLength"));
        ArucoMarkerLength->setDecimals(4);
        ArucoMarkerLength->setMinimum(0);
        ArucoMarkerLength->setSingleStep(0.01);
        ArucoMarkerLength->setValue(0.1);

        gridLayout_63->addWidget(ArucoMarkerLength, 1, 0, 1, 1);

        label_space2_8 = new QLabel(groupBox_markerDetection2);
        label_space2_8->setObjectName(QStringLiteral("label_space2_8"));
        label_space2_8->setWordWrap(true);
        label_space2_8->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_63->addWidget(label_space2_8, 2, 1, 1, 1);

        ArucoMaxDepthError = new QDoubleSpinBox(groupBox_markerDetection2);
        ArucoMaxDepthError->setObjectName(QStringLiteral("ArucoMaxDepthError"));
        ArucoMaxDepthError->setDecimals(4);
        ArucoMaxDepthError->setMinimum(0.0001);
        ArucoMaxDepthError->setSingleStep(0.01);
        ArucoMaxDepthError->setValue(0.1);

        gridLayout_63->addWidget(ArucoMaxDepthError, 2, 0, 1, 1);

        label_space2_13 = new QLabel(groupBox_markerDetection2);
        label_space2_13->setObjectName(QStringLiteral("label_space2_13"));
        label_space2_13->setWordWrap(true);
        label_space2_13->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_63->addWidget(label_space2_13, 5, 1, 1, 1);

        label_space2_14 = new QLabel(groupBox_markerDetection2);
        label_space2_14->setObjectName(QStringLiteral("label_space2_14"));
        label_space2_14->setWordWrap(true);
        label_space2_14->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_63->addWidget(label_space2_14, 6, 1, 1, 1);

        ArucoMarkerRangeMin = new QDoubleSpinBox(groupBox_markerDetection2);
        ArucoMarkerRangeMin->setObjectName(QStringLiteral("ArucoMarkerRangeMin"));
        ArucoMarkerRangeMin->setDecimals(2);
        ArucoMarkerRangeMin->setMinimum(0);
        ArucoMarkerRangeMin->setMaximum(999);
        ArucoMarkerRangeMin->setSingleStep(1);
        ArucoMarkerRangeMin->setValue(0);

        gridLayout_63->addWidget(ArucoMarkerRangeMin, 5, 0, 1, 1);

        ArucoMarkerRangeMax = new QDoubleSpinBox(groupBox_markerDetection2);
        ArucoMarkerRangeMax->setObjectName(QStringLiteral("ArucoMarkerRangeMax"));
        ArucoMarkerRangeMax->setDecimals(2);
        ArucoMarkerRangeMax->setMinimum(0);
        ArucoMarkerRangeMax->setMaximum(999);
        ArucoMarkerRangeMax->setSingleStep(1);
        ArucoMarkerRangeMax->setValue(0);

        gridLayout_63->addWidget(ArucoMarkerRangeMax, 6, 0, 1, 1);

        gridLayout_63->setColumnStretch(1, 1);

        verticalLayout_136->addLayout(gridLayout_63);

        groupBox_31 = new QGroupBox(groupBox_markerDetection2);
        groupBox_31->setObjectName(QStringLiteral("groupBox_31"));
        gridLayout_106 = new QGridLayout(groupBox_31);
        gridLayout_106->setObjectName(QStringLiteral("gridLayout_106"));
        ArucoDictionary = new QComboBox(groupBox_31);
        ArucoDictionary->setObjectName(QStringLiteral("ArucoDictionary"));

        gridLayout_106->addWidget(ArucoDictionary, 0, 0, 1, 1);

        label_space2_10 = new QLabel(groupBox_31);
        label_space2_10->setObjectName(QStringLiteral("label_space2_10"));
        label_space2_10->setWordWrap(true);
        label_space2_10->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_106->addWidget(label_space2_10, 0, 1, 1, 1);

        ArucoCornerRefinementMethod = new QComboBox(groupBox_31);
        ArucoCornerRefinementMethod->setObjectName(QStringLiteral("ArucoCornerRefinementMethod"));

        gridLayout_106->addWidget(ArucoCornerRefinementMethod, 1, 0, 1, 1);

        label_space2_11 = new QLabel(groupBox_31);
        label_space2_11->setObjectName(QStringLiteral("label_space2_11"));
        label_space2_11->setWordWrap(true);
        label_space2_11->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_106->addWidget(label_space2_11, 1, 1, 1, 1);

        gridLayout_106->setColumnStretch(1, 1);

        verticalLayout_136->addWidget(groupBox_31);


        verticalLayout_137->addWidget(groupBox_markerDetection2);

        verticalSpacer_76 = new QSpacerItem(20, 2627, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_137->addItem(verticalSpacer_76);

        stackedWidget->addWidget(page_78);
        page_12 = new QWidget();
        page_12->setObjectName(QStringLiteral("page_12"));
        verticalLayout_32 = new QVBoxLayout(page_12);
        verticalLayout_32->setObjectName(QStringLiteral("verticalLayout_32"));
        groupBox_odometry1 = new QGroupBox(page_12);
        groupBox_odometry1->setObjectName(QStringLiteral("groupBox_odometry1"));
        verticalLayout_34 = new QVBoxLayout(groupBox_odometry1);
        verticalLayout_34->setObjectName(QStringLiteral("verticalLayout_34"));
        label_114 = new QLabel(groupBox_odometry1);
        label_114->setObjectName(QStringLiteral("label_114"));
        label_114->setWordWrap(true);
        label_114->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_34->addWidget(label_114);

        gridLayout_27 = new QGridLayout();
        gridLayout_27->setObjectName(QStringLiteral("gridLayout_27"));
        label_198 = new QLabel(groupBox_odometry1);
        label_198->setObjectName(QStringLiteral("label_198"));
        label_198->setWordWrap(true);
        label_198->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_198, 7, 1, 1, 1);

        label_196 = new QLabel(groupBox_odometry1);
        label_196->setObjectName(QStringLiteral("label_196"));
        label_196->setWordWrap(true);
        label_196->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_196, 11, 1, 1, 1);

        label_246 = new QLabel(groupBox_odometry1);
        label_246->setObjectName(QStringLiteral("label_246"));
        label_246->setWordWrap(true);
        label_246->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_246, 12, 1, 1, 1);

        label_103 = new QLabel(groupBox_odometry1);
        label_103->setObjectName(QStringLiteral("label_103"));
        label_103->setWordWrap(true);
        label_103->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_103, 0, 1, 1, 1);

        odom_strategy = new QComboBox(groupBox_odometry1);
        odom_strategy->setObjectName(QStringLiteral("odom_strategy"));
        odom_strategy->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_27->addWidget(odom_strategy, 0, 0, 1, 1);

        label_314 = new QLabel(groupBox_odometry1);
        label_314->setObjectName(QStringLiteral("label_314"));
        label_314->setWordWrap(true);
        label_314->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_314, 2, 1, 1, 1);

        odom_alignWithGround = new QCheckBox(groupBox_odometry1);
        odom_alignWithGround->setObjectName(QStringLiteral("odom_alignWithGround"));

        gridLayout_27->addWidget(odom_alignWithGround, 2, 0, 1, 1);

        label_238 = new QLabel(groupBox_odometry1);
        label_238->setObjectName(QStringLiteral("label_238"));
        label_238->setWordWrap(true);
        label_238->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_238, 4, 1, 1, 1);

        odom_flow_guessMotion = new QCheckBox(groupBox_odometry1);
        odom_flow_guessMotion->setObjectName(QStringLiteral("odom_flow_guessMotion"));

        gridLayout_27->addWidget(odom_flow_guessMotion, 7, 0, 1, 1);

        odom_dataBufferSize = new QSpinBox(groupBox_odometry1);
        odom_dataBufferSize->setObjectName(QStringLiteral("odom_dataBufferSize"));
        odom_dataBufferSize->setMaximum(999999);

        gridLayout_27->addWidget(odom_dataBufferSize, 13, 0, 1, 1);

        pushButton_testOdometry = new QPushButton(groupBox_odometry1);
        pushButton_testOdometry->setObjectName(QStringLiteral("pushButton_testOdometry"));

        gridLayout_27->addWidget(pushButton_testOdometry, 14, 0, 1, 1);

        label_221 = new QLabel(groupBox_odometry1);
        label_221->setObjectName(QStringLiteral("label_221"));
        label_221->setWordWrap(true);
        label_221->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_221, 6, 1, 1, 1);

        label_232 = new QLabel(groupBox_odometry1);
        label_232->setObjectName(QStringLiteral("label_232"));
        label_232->setWordWrap(true);
        label_232->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_232, 13, 1, 1, 1);

        odom_imageDecimation = new QSpinBox(groupBox_odometry1);
        odom_imageDecimation->setObjectName(QStringLiteral("odom_imageDecimation"));
        odom_imageDecimation->setMinimum(-32);
        odom_imageDecimation->setMaximum(32);

        gridLayout_27->addWidget(odom_imageDecimation, 9, 0, 1, 1);

        label_47 = new QLabel(groupBox_odometry1);
        label_47->setObjectName(QStringLiteral("label_47"));
        label_47->setWordWrap(true);
        label_47->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_47, 3, 1, 1, 1);

        label_248 = new QLabel(groupBox_odometry1);
        label_248->setObjectName(QStringLiteral("label_248"));
        label_248->setWordWrap(true);
        label_248->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_248, 9, 1, 1, 1);

        odom_filteringStrategy = new QComboBox(groupBox_odometry1);
        odom_filteringStrategy->setObjectName(QStringLiteral("odom_filteringStrategy"));
        odom_filteringStrategy->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_27->addWidget(odom_filteringStrategy, 5, 0, 1, 1);

        odom_fillInfoData = new QCheckBox(groupBox_odometry1);
        odom_fillInfoData->setObjectName(QStringLiteral("odom_fillInfoData"));

        gridLayout_27->addWidget(odom_fillInfoData, 6, 0, 1, 1);

        odom_flow_keyframeThr = new QDoubleSpinBox(groupBox_odometry1);
        odom_flow_keyframeThr->setObjectName(QStringLiteral("odom_flow_keyframeThr"));
        odom_flow_keyframeThr->setMaximum(1);
        odom_flow_keyframeThr->setSingleStep(0.1);
        odom_flow_keyframeThr->setValue(0);

        gridLayout_27->addWidget(odom_flow_keyframeThr, 11, 0, 1, 1);

        odom_holonomic = new QCheckBox(groupBox_odometry1);
        odom_holonomic->setObjectName(QStringLiteral("odom_holonomic"));

        gridLayout_27->addWidget(odom_holonomic, 4, 0, 1, 1);

        label_233 = new QLabel(groupBox_odometry1);
        label_233->setObjectName(QStringLiteral("label_233"));
        label_233->setWordWrap(true);
        label_233->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_233, 5, 1, 1, 1);

        odom_countdown = new QSpinBox(groupBox_odometry1);
        odom_countdown->setObjectName(QStringLiteral("odom_countdown"));
        odom_countdown->setMaximum(999999);

        gridLayout_27->addWidget(odom_countdown, 3, 0, 1, 1);

        odom_flow_scanKeyframeThr = new QDoubleSpinBox(groupBox_odometry1);
        odom_flow_scanKeyframeThr->setObjectName(QStringLiteral("odom_flow_scanKeyframeThr"));
        odom_flow_scanKeyframeThr->setMaximum(1);
        odom_flow_scanKeyframeThr->setSingleStep(0.1);
        odom_flow_scanKeyframeThr->setValue(0.5);

        gridLayout_27->addWidget(odom_flow_scanKeyframeThr, 12, 0, 1, 1);

        label_351 = new QLabel(groupBox_odometry1);
        label_351->setObjectName(QStringLiteral("label_351"));
        label_351->setWordWrap(true);
        label_351->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_351, 1, 1, 1, 1);

        odom_registration = new QComboBox(groupBox_odometry1);
        odom_registration->setObjectName(QStringLiteral("odom_registration"));
        odom_registration->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_27->addWidget(odom_registration, 1, 0, 1, 1);

        label_354 = new QLabel(groupBox_odometry1);
        label_354->setObjectName(QStringLiteral("label_354"));
        label_354->setWordWrap(true);
        label_354->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_354, 10, 1, 1, 1);

        odom_VisKeyFrameThr = new QSpinBox(groupBox_odometry1);
        odom_VisKeyFrameThr->setObjectName(QStringLiteral("odom_VisKeyFrameThr"));
        odom_VisKeyFrameThr->setMaximum(9999);
        odom_VisKeyFrameThr->setValue(100);

        gridLayout_27->addWidget(odom_VisKeyFrameThr, 10, 0, 1, 1);

        label_520 = new QLabel(groupBox_odometry1);
        label_520->setObjectName(QStringLiteral("label_520"));
        label_520->setWordWrap(true);
        label_520->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_27->addWidget(label_520, 8, 1, 1, 1);

        odom_guess_smoothing_delay = new QDoubleSpinBox(groupBox_odometry1);
        odom_guess_smoothing_delay->setObjectName(QStringLiteral("odom_guess_smoothing_delay"));
        odom_guess_smoothing_delay->setDecimals(3);
        odom_guess_smoothing_delay->setMaximum(10);
        odom_guess_smoothing_delay->setSingleStep(0.1);
        odom_guess_smoothing_delay->setValue(1);

        gridLayout_27->addWidget(odom_guess_smoothing_delay, 8, 0, 1, 1);

        gridLayout_27->setColumnStretch(1, 1);

        verticalLayout_34->addLayout(gridLayout_27);

        stackedWidget_odometryType = new QStackedWidget(groupBox_odometry1);
        stackedWidget_odometryType->setObjectName(QStringLiteral("stackedWidget_odometryType"));
        page_52 = new QWidget();
        page_52->setObjectName(QStringLiteral("page_52"));
        verticalLayout_77 = new QVBoxLayout(page_52);
        verticalLayout_77->setObjectName(QStringLiteral("verticalLayout_77"));
        groupBox_odomF2M = new QGroupBox(page_52);
        groupBox_odomF2M->setObjectName(QStringLiteral("groupBox_odomF2M"));
        verticalLayout_54 = new QVBoxLayout(groupBox_odomF2M);
        verticalLayout_54->setObjectName(QStringLiteral("verticalLayout_54"));
        label_226 = new QLabel(groupBox_odomF2M);
        label_226->setObjectName(QStringLiteral("label_226"));
        label_226->setWordWrap(true);
        label_226->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_54->addWidget(label_226);

        gridLayout_29 = new QGridLayout();
        gridLayout_29->setObjectName(QStringLiteral("gridLayout_29"));
        spinBox_odom_f2m_maxNewFeatures = new QSpinBox(groupBox_odomF2M);
        spinBox_odom_f2m_maxNewFeatures->setObjectName(QStringLiteral("spinBox_odom_f2m_maxNewFeatures"));
        spinBox_odom_f2m_maxNewFeatures->setMinimum(0);
        spinBox_odom_f2m_maxNewFeatures->setMaximum(999999);
        spinBox_odom_f2m_maxNewFeatures->setSingleStep(1);
        spinBox_odom_f2m_maxNewFeatures->setValue(0);

        gridLayout_29->addWidget(spinBox_odom_f2m_maxNewFeatures, 1, 0, 1, 1);

        doubleSpinBox_odom_f2m_scanRadius = new QDoubleSpinBox(groupBox_odomF2M);
        doubleSpinBox_odom_f2m_scanRadius->setObjectName(QStringLiteral("doubleSpinBox_odom_f2m_scanRadius"));
        doubleSpinBox_odom_f2m_scanRadius->setDecimals(3);
        doubleSpinBox_odom_f2m_scanRadius->setValue(0.025);

        gridLayout_29->addWidget(doubleSpinBox_odom_f2m_scanRadius, 3, 0, 1, 1);

        label_215 = new QLabel(groupBox_odomF2M);
        label_215->setObjectName(QStringLiteral("label_215"));
        label_215->setWordWrap(true);
        label_215->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_29->addWidget(label_215, 3, 1, 1, 1);

        label_195 = new QLabel(groupBox_odomF2M);
        label_195->setObjectName(QStringLiteral("label_195"));
        label_195->setWordWrap(true);
        label_195->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_29->addWidget(label_195, 2, 1, 1, 1);

        label_190 = new QLabel(groupBox_odomF2M);
        label_190->setObjectName(QStringLiteral("label_190"));
        label_190->setWordWrap(true);
        label_190->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_29->addWidget(label_190, 0, 1, 1, 1);

        odom_localHistory = new QSpinBox(groupBox_odomF2M);
        odom_localHistory->setObjectName(QStringLiteral("odom_localHistory"));
        odom_localHistory->setMinimum(0);
        odom_localHistory->setMaximum(999999999);
        odom_localHistory->setSingleStep(1);
        odom_localHistory->setValue(0);

        gridLayout_29->addWidget(odom_localHistory, 0, 0, 1, 1);

        spinBox_odom_f2m_scanMaxSize = new QSpinBox(groupBox_odomF2M);
        spinBox_odom_f2m_scanMaxSize->setObjectName(QStringLiteral("spinBox_odom_f2m_scanMaxSize"));
        spinBox_odom_f2m_scanMaxSize->setMaximum(999999999);

        gridLayout_29->addWidget(spinBox_odom_f2m_scanMaxSize, 2, 0, 1, 1);

        label_194 = new QLabel(groupBox_odomF2M);
        label_194->setObjectName(QStringLiteral("label_194"));
        label_194->setWordWrap(true);
        label_194->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_29->addWidget(label_194, 1, 1, 1, 1);

        label_357 = new QLabel(groupBox_odomF2M);
        label_357->setObjectName(QStringLiteral("label_357"));
        label_357->setWordWrap(true);
        label_357->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_29->addWidget(label_357, 7, 1, 1, 1);

        odom_f2m_bundleMaxFrames = new QSpinBox(groupBox_odomF2M);
        odom_f2m_bundleMaxFrames->setObjectName(QStringLiteral("odom_f2m_bundleMaxFrames"));
        odom_f2m_bundleMaxFrames->setMaximum(999999);

        gridLayout_29->addWidget(odom_f2m_bundleMaxFrames, 8, 0, 1, 1);

        label_358 = new QLabel(groupBox_odomF2M);
        label_358->setObjectName(QStringLiteral("label_358"));
        label_358->setWordWrap(true);
        label_358->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_29->addWidget(label_358, 8, 1, 1, 1);

        odom_f2m_bundleStrategy = new QComboBox(groupBox_odomF2M);
        odom_f2m_bundleStrategy->setObjectName(QStringLiteral("odom_f2m_bundleStrategy"));
        odom_f2m_bundleStrategy->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_29->addWidget(odom_f2m_bundleStrategy, 7, 0, 1, 1);

        label_430 = new QLabel(groupBox_odomF2M);
        label_430->setObjectName(QStringLiteral("label_430"));
        label_430->setWordWrap(true);
        label_430->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_29->addWidget(label_430, 4, 1, 1, 1);

        doubleSpinBox_odom_f2m_scanAngle = new QDoubleSpinBox(groupBox_odomF2M);
        doubleSpinBox_odom_f2m_scanAngle->setObjectName(QStringLiteral("doubleSpinBox_odom_f2m_scanAngle"));
        doubleSpinBox_odom_f2m_scanAngle->setDecimals(0);
        doubleSpinBox_odom_f2m_scanAngle->setMaximum(180);
        doubleSpinBox_odom_f2m_scanAngle->setValue(0);

        gridLayout_29->addWidget(doubleSpinBox_odom_f2m_scanAngle, 4, 0, 1, 1);

        label_524 = new QLabel(groupBox_odomF2M);
        label_524->setObjectName(QStringLiteral("label_524"));
        label_524->setWordWrap(true);
        label_524->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_29->addWidget(label_524, 6, 1, 1, 1);

        odom_f2m_validDepthRatio = new QDoubleSpinBox(groupBox_odomF2M);
        odom_f2m_validDepthRatio->setObjectName(QStringLiteral("odom_f2m_validDepthRatio"));
        odom_f2m_validDepthRatio->setDecimals(2);
        odom_f2m_validDepthRatio->setMaximum(1);
        odom_f2m_validDepthRatio->setSingleStep(0.05);
        odom_f2m_validDepthRatio->setValue(0.75);

        gridLayout_29->addWidget(odom_f2m_validDepthRatio, 6, 0, 1, 1);

        label_564 = new QLabel(groupBox_odomF2M);
        label_564->setObjectName(QStringLiteral("label_564"));
        label_564->setWordWrap(true);
        label_564->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_29->addWidget(label_564, 5, 1, 1, 1);

        doubleSpinBox_odom_f2m_scanRange = new QDoubleSpinBox(groupBox_odomF2M);
        doubleSpinBox_odom_f2m_scanRange->setObjectName(QStringLiteral("doubleSpinBox_odom_f2m_scanRange"));
        doubleSpinBox_odom_f2m_scanRange->setDecimals(0);
        doubleSpinBox_odom_f2m_scanRange->setMaximum(9999);
        doubleSpinBox_odom_f2m_scanRange->setValue(0);

        gridLayout_29->addWidget(doubleSpinBox_odom_f2m_scanRange, 5, 0, 1, 1);

        label_598 = new QLabel(groupBox_odomF2M);
        label_598->setObjectName(QStringLiteral("label_598"));
        label_598->setWordWrap(true);
        label_598->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_29->addWidget(label_598, 9, 1, 1, 1);

        odom_f2m_gravitySigma = new QDoubleSpinBox(groupBox_odomF2M);
        odom_f2m_gravitySigma->setObjectName(QStringLiteral("odom_f2m_gravitySigma"));
        odom_f2m_gravitySigma->setDecimals(4);
        odom_f2m_gravitySigma->setMinimum(-1);
        odom_f2m_gravitySigma->setMaximum(10);
        odom_f2m_gravitySigma->setSingleStep(0.1);
        odom_f2m_gravitySigma->setValue(-1);

        gridLayout_29->addWidget(odom_f2m_gravitySigma, 9, 0, 1, 1);

        gridLayout_29->setColumnStretch(1, 1);

        verticalLayout_54->addLayout(gridLayout_29);

        verticalSpacer_46 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_54->addItem(verticalSpacer_46);


        verticalLayout_77->addWidget(groupBox_odomF2M);

        stackedWidget_odometryType->addWidget(page_52);
        page_58 = new QWidget();
        page_58->setObjectName(QStringLiteral("page_58"));
        verticalLayout_55 = new QVBoxLayout(page_58);
        verticalLayout_55->setObjectName(QStringLiteral("verticalLayout_55"));
        groupBox_odomF2F = new QGroupBox(page_58);
        groupBox_odomF2F->setObjectName(QStringLiteral("groupBox_odomF2F"));
        verticalLayout_53 = new QVBoxLayout(groupBox_odomF2F);
        verticalLayout_53->setObjectName(QStringLiteral("verticalLayout_53"));
        label_199 = new QLabel(groupBox_odomF2F);
        label_199->setObjectName(QStringLiteral("label_199"));
        label_199->setWordWrap(true);
        label_199->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_53->addWidget(label_199);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setObjectName(QStringLiteral("horizontalLayout_12"));
        comboBox_odomf2f_corType = new QComboBox(groupBox_odomF2F);
        comboBox_odomf2f_corType->setObjectName(QStringLiteral("comboBox_odomf2f_corType"));
        QSizePolicy sizePolicy4(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(comboBox_odomf2f_corType->sizePolicy().hasHeightForWidth());
        comboBox_odomf2f_corType->setSizePolicy(sizePolicy4);
        comboBox_odomf2f_corType->setSizeAdjustPolicy(QComboBox::AdjustToContentsOnFirstShow);

        horizontalLayout_12->addWidget(comboBox_odomf2f_corType);

        label_149 = new QLabel(groupBox_odomF2F);
        label_149->setObjectName(QStringLiteral("label_149"));
        label_149->setWordWrap(true);
        label_149->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        horizontalLayout_12->addWidget(label_149);

        horizontalLayout_12->setStretch(1, 1);

        verticalLayout_53->addLayout(horizontalLayout_12);

        groupBox_odomf2f_corType = new QGroupBox(groupBox_odomF2F);
        groupBox_odomf2f_corType->setObjectName(QStringLiteral("groupBox_odomf2f_corType"));
        gridLayout_52 = new QGridLayout(groupBox_odomf2f_corType);
        gridLayout_52->setObjectName(QStringLiteral("gridLayout_52"));
        odom_flow_winSize_2 = new QSpinBox(groupBox_odomf2f_corType);
        odom_flow_winSize_2->setObjectName(QStringLiteral("odom_flow_winSize_2"));
        odom_flow_winSize_2->setMinimum(1);
        odom_flow_winSize_2->setMaximum(999999);
        odom_flow_winSize_2->setSingleStep(1);
        odom_flow_winSize_2->setValue(21);

        gridLayout_52->addWidget(odom_flow_winSize_2, 0, 0, 1, 1);

        label_201 = new QLabel(groupBox_odomf2f_corType);
        label_201->setObjectName(QStringLiteral("label_201"));
        label_201->setWordWrap(true);
        label_201->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_52->addWidget(label_201, 0, 1, 1, 1);

        odom_flow_maxLevel_2 = new QSpinBox(groupBox_odomf2f_corType);
        odom_flow_maxLevel_2->setObjectName(QStringLiteral("odom_flow_maxLevel_2"));
        odom_flow_maxLevel_2->setMinimum(0);
        odom_flow_maxLevel_2->setMaximum(999999);
        odom_flow_maxLevel_2->setSingleStep(1);
        odom_flow_maxLevel_2->setValue(3);

        gridLayout_52->addWidget(odom_flow_maxLevel_2, 3, 0, 1, 1);

        label_295 = new QLabel(groupBox_odomf2f_corType);
        label_295->setObjectName(QStringLiteral("label_295"));
        label_295->setWordWrap(true);
        label_295->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_52->addWidget(label_295, 1, 1, 1, 1);

        label_296 = new QLabel(groupBox_odomf2f_corType);
        label_296->setObjectName(QStringLiteral("label_296"));
        label_296->setWordWrap(true);
        label_296->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_52->addWidget(label_296, 2, 1, 1, 1);

        odom_flow_iterations_2 = new QSpinBox(groupBox_odomf2f_corType);
        odom_flow_iterations_2->setObjectName(QStringLiteral("odom_flow_iterations_2"));
        odom_flow_iterations_2->setMinimum(1);
        odom_flow_iterations_2->setMaximum(999999);
        odom_flow_iterations_2->setSingleStep(1);
        odom_flow_iterations_2->setValue(30);

        gridLayout_52->addWidget(odom_flow_iterations_2, 1, 0, 1, 1);

        label_297 = new QLabel(groupBox_odomf2f_corType);
        label_297->setObjectName(QStringLiteral("label_297"));
        label_297->setWordWrap(true);
        label_297->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_52->addWidget(label_297, 3, 1, 1, 1);

        odom_flow_eps_2 = new QDoubleSpinBox(groupBox_odomf2f_corType);
        odom_flow_eps_2->setObjectName(QStringLiteral("odom_flow_eps_2"));
        odom_flow_eps_2->setDecimals(3);
        odom_flow_eps_2->setMinimum(0.001);
        odom_flow_eps_2->setMaximum(1);
        odom_flow_eps_2->setSingleStep(0.01);
        odom_flow_eps_2->setValue(0.01);

        gridLayout_52->addWidget(odom_flow_eps_2, 2, 0, 1, 1);

        verticalSpacer_45 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout_52->addItem(verticalSpacer_45, 4, 0, 1, 1);

        gridLayout_52->setColumnStretch(1, 1);

        verticalLayout_53->addWidget(groupBox_odomf2f_corType);

        verticalSpacer_43 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_53->addItem(verticalSpacer_43);


        verticalLayout_55->addWidget(groupBox_odomF2F);

        stackedWidget_odometryType->addWidget(page_58);
        page_67 = new QWidget();
        page_67->setObjectName(QStringLiteral("page_67"));
        verticalLayout_119 = new QVBoxLayout(page_67);
        verticalLayout_119->setObjectName(QStringLiteral("verticalLayout_119"));
        groupBox_odomFovis = new QGroupBox(page_67);
        groupBox_odomFovis->setObjectName(QStringLiteral("groupBox_odomFovis"));
        verticalLayout_118 = new QVBoxLayout(groupBox_odomFovis);
        verticalLayout_118->setObjectName(QStringLiteral("verticalLayout_118"));
        label_234 = new QLabel(groupBox_odomFovis);
        label_234->setObjectName(QStringLiteral("label_234"));
        label_234->setWordWrap(true);
        label_234->setOpenExternalLinks(true);
        label_234->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_118->addWidget(label_234);

        groupBox_23 = new QGroupBox(groupBox_odomFovis);
        groupBox_23->setObjectName(QStringLiteral("groupBox_23"));
        gridLayout_90 = new QGridLayout(groupBox_23);
        gridLayout_90->setObjectName(QStringLiteral("gridLayout_90"));
        spinBox_OdomFovisFeatureSearchWindow = new QSpinBox(groupBox_23);
        spinBox_OdomFovisFeatureSearchWindow->setObjectName(QStringLiteral("spinBox_OdomFovisFeatureSearchWindow"));
        spinBox_OdomFovisFeatureSearchWindow->setMinimum(1);
        spinBox_OdomFovisFeatureSearchWindow->setMaximum(999);
        spinBox_OdomFovisFeatureSearchWindow->setValue(25);

        gridLayout_90->addWidget(spinBox_OdomFovisFeatureSearchWindow, 5, 0, 1, 1);

        spinBox_OdomFovisMinFeaturesForEstimate = new QSpinBox(groupBox_23);
        spinBox_OdomFovisMinFeaturesForEstimate->setObjectName(QStringLiteral("spinBox_OdomFovisMinFeaturesForEstimate"));
        spinBox_OdomFovisMinFeaturesForEstimate->setMinimum(3);
        spinBox_OdomFovisMinFeaturesForEstimate->setMaximum(999);
        spinBox_OdomFovisMinFeaturesForEstimate->setValue(10);

        gridLayout_90->addWidget(spinBox_OdomFovisMinFeaturesForEstimate, 2, 0, 1, 1);

        label_371 = new QLabel(groupBox_23);
        label_371->setObjectName(QStringLiteral("label_371"));
        label_371->setWordWrap(true);
        label_371->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_90->addWidget(label_371, 0, 1, 1, 1);

        label_377 = new QLabel(groupBox_23);
        label_377->setObjectName(QStringLiteral("label_377"));
        label_377->setWordWrap(true);
        label_377->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_90->addWidget(label_377, 5, 1, 1, 1);

        checkBox_OdomFovisUpdateTargetFeaturesWithRefined = new QCheckBox(groupBox_23);
        checkBox_OdomFovisUpdateTargetFeaturesWithRefined->setObjectName(QStringLiteral("checkBox_OdomFovisUpdateTargetFeaturesWithRefined"));

        gridLayout_90->addWidget(checkBox_OdomFovisUpdateTargetFeaturesWithRefined, 6, 0, 1, 1);

        label_374 = new QLabel(groupBox_23);
        label_374->setObjectName(QStringLiteral("label_374"));
        label_374->setWordWrap(true);
        label_374->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_90->addWidget(label_374, 2, 1, 1, 1);

        doubleSpinBox_OdomFovisInlierMaxReprojectionError = new QDoubleSpinBox(groupBox_23);
        doubleSpinBox_OdomFovisInlierMaxReprojectionError->setObjectName(QStringLiteral("doubleSpinBox_OdomFovisInlierMaxReprojectionError"));
        doubleSpinBox_OdomFovisInlierMaxReprojectionError->setDecimals(1);
        doubleSpinBox_OdomFovisInlierMaxReprojectionError->setMinimum(0.1);
        doubleSpinBox_OdomFovisInlierMaxReprojectionError->setMaximum(9.9);
        doubleSpinBox_OdomFovisInlierMaxReprojectionError->setSingleStep(0.1);
        doubleSpinBox_OdomFovisInlierMaxReprojectionError->setValue(1.5);

        gridLayout_90->addWidget(doubleSpinBox_OdomFovisInlierMaxReprojectionError, 0, 0, 1, 1);

        checkBox_OdomFovisUseSubpixelRefinement = new QCheckBox(groupBox_23);
        checkBox_OdomFovisUseSubpixelRefinement->setObjectName(QStringLiteral("checkBox_OdomFovisUseSubpixelRefinement"));
        checkBox_OdomFovisUseSubpixelRefinement->setChecked(true);

        gridLayout_90->addWidget(checkBox_OdomFovisUseSubpixelRefinement, 4, 0, 1, 1);

        label_375 = new QLabel(groupBox_23);
        label_375->setObjectName(QStringLiteral("label_375"));
        label_375->setWordWrap(true);
        label_375->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_90->addWidget(label_375, 3, 1, 1, 1);

        doubleSpinBox_OdomFovisCliqueInlierThreshold = new QDoubleSpinBox(groupBox_23);
        doubleSpinBox_OdomFovisCliqueInlierThreshold->setObjectName(QStringLiteral("doubleSpinBox_OdomFovisCliqueInlierThreshold"));
        doubleSpinBox_OdomFovisCliqueInlierThreshold->setMinimum(0.01);
        doubleSpinBox_OdomFovisCliqueInlierThreshold->setMaximum(9.99);
        doubleSpinBox_OdomFovisCliqueInlierThreshold->setSingleStep(0.01);
        doubleSpinBox_OdomFovisCliqueInlierThreshold->setValue(0.1);

        gridLayout_90->addWidget(doubleSpinBox_OdomFovisCliqueInlierThreshold, 1, 0, 1, 1);

        doubleSpinBox_OdomFovisMaxMeanReprojectionError = new QDoubleSpinBox(groupBox_23);
        doubleSpinBox_OdomFovisMaxMeanReprojectionError->setObjectName(QStringLiteral("doubleSpinBox_OdomFovisMaxMeanReprojectionError"));
        doubleSpinBox_OdomFovisMaxMeanReprojectionError->setDecimals(1);
        doubleSpinBox_OdomFovisMaxMeanReprojectionError->setMaximum(99);
        doubleSpinBox_OdomFovisMaxMeanReprojectionError->setSingleStep(0.1);
        doubleSpinBox_OdomFovisMaxMeanReprojectionError->setValue(10);

        gridLayout_90->addWidget(doubleSpinBox_OdomFovisMaxMeanReprojectionError, 3, 0, 1, 1);

        label_250 = new QLabel(groupBox_23);
        label_250->setObjectName(QStringLiteral("label_250"));
        label_250->setWordWrap(true);
        label_250->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_90->addWidget(label_250, 1, 1, 1, 1);

        label_376 = new QLabel(groupBox_23);
        label_376->setObjectName(QStringLiteral("label_376"));
        label_376->setWordWrap(true);
        label_376->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_90->addWidget(label_376, 4, 1, 1, 1);

        label_379 = new QLabel(groupBox_23);
        label_379->setObjectName(QStringLiteral("label_379"));
        label_379->setWordWrap(true);
        label_379->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_90->addWidget(label_379, 6, 1, 1, 1);

        gridLayout_90->setColumnStretch(1, 1);

        verticalLayout_118->addWidget(groupBox_23);

        groupBox_24 = new QGroupBox(groupBox_odomFovis);
        groupBox_24->setObjectName(QStringLiteral("groupBox_24"));
        gridLayout_38 = new QGridLayout(groupBox_24);
        gridLayout_38->setObjectName(QStringLiteral("gridLayout_38"));
        spinBox_OdomFovisFastThreshold = new QSpinBox(groupBox_24);
        spinBox_OdomFovisFastThreshold->setObjectName(QStringLiteral("spinBox_OdomFovisFastThreshold"));
        spinBox_OdomFovisFastThreshold->setMinimum(1);
        spinBox_OdomFovisFastThreshold->setMaximum(999);
        spinBox_OdomFovisFastThreshold->setValue(20);

        gridLayout_38->addWidget(spinBox_OdomFovisFastThreshold, 4, 0, 1, 1);

        checkBox_OdomFovisUseAdaptiveThreshold = new QCheckBox(groupBox_24);
        checkBox_OdomFovisUseAdaptiveThreshold->setObjectName(QStringLiteral("checkBox_OdomFovisUseAdaptiveThreshold"));
        checkBox_OdomFovisUseAdaptiveThreshold->setChecked(true);

        gridLayout_38->addWidget(checkBox_OdomFovisUseAdaptiveThreshold, 5, 0, 1, 1);

        doubleSpinBox_OdomFovisFastThresholdAdaptiveGain = new QDoubleSpinBox(groupBox_24);
        doubleSpinBox_OdomFovisFastThresholdAdaptiveGain->setObjectName(QStringLiteral("doubleSpinBox_OdomFovisFastThresholdAdaptiveGain"));
        doubleSpinBox_OdomFovisFastThresholdAdaptiveGain->setDecimals(3);
        doubleSpinBox_OdomFovisFastThresholdAdaptiveGain->setMinimum(0.001);
        doubleSpinBox_OdomFovisFastThresholdAdaptiveGain->setMaximum(1);
        doubleSpinBox_OdomFovisFastThresholdAdaptiveGain->setSingleStep(0.001);
        doubleSpinBox_OdomFovisFastThresholdAdaptiveGain->setValue(0.005);

        gridLayout_38->addWidget(doubleSpinBox_OdomFovisFastThresholdAdaptiveGain, 6, 0, 1, 1);

        label_382 = new QLabel(groupBox_24);
        label_382->setObjectName(QStringLiteral("label_382"));
        label_382->setWordWrap(true);
        label_382->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_38->addWidget(label_382, 4, 1, 1, 1);

        spinBox_OdomFovisMinPyramidLevel = new QSpinBox(groupBox_24);
        spinBox_OdomFovisMinPyramidLevel->setObjectName(QStringLiteral("spinBox_OdomFovisMinPyramidLevel"));
        spinBox_OdomFovisMinPyramidLevel->setMinimum(0);
        spinBox_OdomFovisMinPyramidLevel->setValue(0);

        gridLayout_38->addWidget(spinBox_OdomFovisMinPyramidLevel, 2, 0, 1, 1);

        spinBox_OdomFovisMaxPyramidLevel = new QSpinBox(groupBox_24);
        spinBox_OdomFovisMaxPyramidLevel->setObjectName(QStringLiteral("spinBox_OdomFovisMaxPyramidLevel"));
        spinBox_OdomFovisMaxPyramidLevel->setMinimum(1);
        spinBox_OdomFovisMaxPyramidLevel->setValue(3);

        gridLayout_38->addWidget(spinBox_OdomFovisMaxPyramidLevel, 1, 0, 1, 1);

        label_373 = new QLabel(groupBox_24);
        label_373->setObjectName(QStringLiteral("label_373"));
        label_373->setWordWrap(true);
        label_373->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_38->addWidget(label_373, 1, 1, 1, 1);

        spinBox_OdomFovisTargetPixelsPerFeature = new QSpinBox(groupBox_24);
        spinBox_OdomFovisTargetPixelsPerFeature->setObjectName(QStringLiteral("spinBox_OdomFovisTargetPixelsPerFeature"));
        spinBox_OdomFovisTargetPixelsPerFeature->setMinimum(1);
        spinBox_OdomFovisTargetPixelsPerFeature->setMaximum(999);
        spinBox_OdomFovisTargetPixelsPerFeature->setValue(250);

        gridLayout_38->addWidget(spinBox_OdomFovisTargetPixelsPerFeature, 3, 0, 1, 1);

        label_381 = new QLabel(groupBox_24);
        label_381->setObjectName(QStringLiteral("label_381"));
        label_381->setWordWrap(true);
        label_381->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_38->addWidget(label_381, 2, 1, 1, 1);

        spinBox_OdomFovisFeatureWindowSize = new QSpinBox(groupBox_24);
        spinBox_OdomFovisFeatureWindowSize->setObjectName(QStringLiteral("spinBox_OdomFovisFeatureWindowSize"));
        spinBox_OdomFovisFeatureWindowSize->setMinimum(1);
        spinBox_OdomFovisFeatureWindowSize->setValue(9);

        gridLayout_38->addWidget(spinBox_OdomFovisFeatureWindowSize, 0, 0, 1, 1);

        label_372 = new QLabel(groupBox_24);
        label_372->setObjectName(QStringLiteral("label_372"));
        label_372->setWordWrap(true);
        label_372->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_38->addWidget(label_372, 0, 1, 1, 1);

        label_378 = new QLabel(groupBox_24);
        label_378->setObjectName(QStringLiteral("label_378"));
        label_378->setWordWrap(true);
        label_378->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_38->addWidget(label_378, 3, 1, 1, 1);

        checkBox_OdomFovisUseHomographyInitialization = new QCheckBox(groupBox_24);
        checkBox_OdomFovisUseHomographyInitialization->setObjectName(QStringLiteral("checkBox_OdomFovisUseHomographyInitialization"));
        checkBox_OdomFovisUseHomographyInitialization->setChecked(true);

        gridLayout_38->addWidget(checkBox_OdomFovisUseHomographyInitialization, 7, 0, 1, 1);

        label_383 = new QLabel(groupBox_24);
        label_383->setObjectName(QStringLiteral("label_383"));
        label_383->setWordWrap(true);
        label_383->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_38->addWidget(label_383, 5, 1, 1, 1);

        label_400 = new QLabel(groupBox_24);
        label_400->setObjectName(QStringLiteral("label_400"));
        label_400->setWordWrap(true);
        label_400->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_38->addWidget(label_400, 6, 1, 1, 1);

        label_401 = new QLabel(groupBox_24);
        label_401->setObjectName(QStringLiteral("label_401"));
        label_401->setWordWrap(true);
        label_401->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_38->addWidget(label_401, 7, 1, 1, 1);

        gridLayout_38->setColumnStretch(1, 1);

        verticalLayout_118->addWidget(groupBox_24);

        verticalSpacer_60 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_118->addItem(verticalSpacer_60);

        groupBox_25 = new QGroupBox(groupBox_odomFovis);
        groupBox_25->setObjectName(QStringLiteral("groupBox_25"));
        gridLayout_91 = new QGridLayout(groupBox_25);
        gridLayout_91->setObjectName(QStringLiteral("gridLayout_91"));
        checkBox_OdomFovisUseBucketing = new QCheckBox(groupBox_25);
        checkBox_OdomFovisUseBucketing->setObjectName(QStringLiteral("checkBox_OdomFovisUseBucketing"));
        checkBox_OdomFovisUseBucketing->setChecked(true);

        gridLayout_91->addWidget(checkBox_OdomFovisUseBucketing, 0, 0, 1, 1);

        label_402 = new QLabel(groupBox_25);
        label_402->setObjectName(QStringLiteral("label_402"));
        label_402->setWordWrap(true);
        label_402->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_91->addWidget(label_402, 0, 1, 1, 1);

        spinBox_OdomFovisBucketHeight = new QSpinBox(groupBox_25);
        spinBox_OdomFovisBucketHeight->setObjectName(QStringLiteral("spinBox_OdomFovisBucketHeight"));
        spinBox_OdomFovisBucketHeight->setMinimum(1);
        spinBox_OdomFovisBucketHeight->setMaximum(999);
        spinBox_OdomFovisBucketHeight->setValue(80);

        gridLayout_91->addWidget(spinBox_OdomFovisBucketHeight, 2, 0, 1, 1);

        spinBox_OdomFovisMaxKeypointsPerBucket = new QSpinBox(groupBox_25);
        spinBox_OdomFovisMaxKeypointsPerBucket->setObjectName(QStringLiteral("spinBox_OdomFovisMaxKeypointsPerBucket"));
        spinBox_OdomFovisMaxKeypointsPerBucket->setMinimum(1);
        spinBox_OdomFovisMaxKeypointsPerBucket->setMaximum(999);
        spinBox_OdomFovisMaxKeypointsPerBucket->setValue(25);

        gridLayout_91->addWidget(spinBox_OdomFovisMaxKeypointsPerBucket, 3, 0, 1, 1);

        spinBox_OdomFovisBucketWidth = new QSpinBox(groupBox_25);
        spinBox_OdomFovisBucketWidth->setObjectName(QStringLiteral("spinBox_OdomFovisBucketWidth"));
        spinBox_OdomFovisBucketWidth->setMinimum(1);
        spinBox_OdomFovisBucketWidth->setMaximum(999);
        spinBox_OdomFovisBucketWidth->setValue(80);

        gridLayout_91->addWidget(spinBox_OdomFovisBucketWidth, 1, 0, 1, 1);

        checkBox_OdomFovisUseImageNormalization = new QCheckBox(groupBox_25);
        checkBox_OdomFovisUseImageNormalization->setObjectName(QStringLiteral("checkBox_OdomFovisUseImageNormalization"));
        checkBox_OdomFovisUseImageNormalization->setChecked(true);

        gridLayout_91->addWidget(checkBox_OdomFovisUseImageNormalization, 4, 0, 1, 1);

        label_403 = new QLabel(groupBox_25);
        label_403->setObjectName(QStringLiteral("label_403"));
        label_403->setWordWrap(true);
        label_403->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_91->addWidget(label_403, 1, 1, 1, 1);

        label_404 = new QLabel(groupBox_25);
        label_404->setObjectName(QStringLiteral("label_404"));
        label_404->setWordWrap(true);
        label_404->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_91->addWidget(label_404, 2, 1, 1, 1);

        label_405 = new QLabel(groupBox_25);
        label_405->setObjectName(QStringLiteral("label_405"));
        label_405->setWordWrap(true);
        label_405->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_91->addWidget(label_405, 3, 1, 1, 1);

        label_406 = new QLabel(groupBox_25);
        label_406->setObjectName(QStringLiteral("label_406"));
        label_406->setWordWrap(true);
        label_406->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_91->addWidget(label_406, 4, 1, 1, 1);

        gridLayout_91->setColumnStretch(1, 1);

        verticalLayout_118->addWidget(groupBox_25);

        groupBox_26 = new QGroupBox(groupBox_odomFovis);
        groupBox_26->setObjectName(QStringLiteral("groupBox_26"));
        gridLayout_92 = new QGridLayout(groupBox_26);
        gridLayout_92->setObjectName(QStringLiteral("gridLayout_92"));
        doubleSpinBox_OdomFovisStereoMaxDistEpipolarLine = new QDoubleSpinBox(groupBox_26);
        doubleSpinBox_OdomFovisStereoMaxDistEpipolarLine->setObjectName(QStringLiteral("doubleSpinBox_OdomFovisStereoMaxDistEpipolarLine"));
        doubleSpinBox_OdomFovisStereoMaxDistEpipolarLine->setDecimals(1);
        doubleSpinBox_OdomFovisStereoMaxDistEpipolarLine->setMinimum(0.1);
        doubleSpinBox_OdomFovisStereoMaxDistEpipolarLine->setMaximum(9.9);
        doubleSpinBox_OdomFovisStereoMaxDistEpipolarLine->setSingleStep(0.1);
        doubleSpinBox_OdomFovisStereoMaxDistEpipolarLine->setValue(1.5);

        gridLayout_92->addWidget(doubleSpinBox_OdomFovisStereoMaxDistEpipolarLine, 1, 0, 1, 1);

        doubleSpinBox_OdomFovisStereoMaxRefinementDisplacement = new QDoubleSpinBox(groupBox_26);
        doubleSpinBox_OdomFovisStereoMaxRefinementDisplacement->setObjectName(QStringLiteral("doubleSpinBox_OdomFovisStereoMaxRefinementDisplacement"));
        doubleSpinBox_OdomFovisStereoMaxRefinementDisplacement->setDecimals(1);
        doubleSpinBox_OdomFovisStereoMaxRefinementDisplacement->setMinimum(0.1);
        doubleSpinBox_OdomFovisStereoMaxRefinementDisplacement->setMaximum(9.9);
        doubleSpinBox_OdomFovisStereoMaxRefinementDisplacement->setSingleStep(0.1);
        doubleSpinBox_OdomFovisStereoMaxRefinementDisplacement->setValue(1);

        gridLayout_92->addWidget(doubleSpinBox_OdomFovisStereoMaxRefinementDisplacement, 2, 0, 1, 1);

        checkBox_OdomFovisStereoRequireMutualMatch = new QCheckBox(groupBox_26);
        checkBox_OdomFovisStereoRequireMutualMatch->setObjectName(QStringLiteral("checkBox_OdomFovisStereoRequireMutualMatch"));
        checkBox_OdomFovisStereoRequireMutualMatch->setChecked(true);

        gridLayout_92->addWidget(checkBox_OdomFovisStereoRequireMutualMatch, 0, 0, 1, 1);

        label_407 = new QLabel(groupBox_26);
        label_407->setObjectName(QStringLiteral("label_407"));
        label_407->setWordWrap(true);
        label_407->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_92->addWidget(label_407, 0, 1, 1, 1);

        spinBox_OdomFovisStereoMaxDisparity = new QSpinBox(groupBox_26);
        spinBox_OdomFovisStereoMaxDisparity->setObjectName(QStringLiteral("spinBox_OdomFovisStereoMaxDisparity"));
        spinBox_OdomFovisStereoMaxDisparity->setMinimum(1);
        spinBox_OdomFovisStereoMaxDisparity->setMaximum(999);
        spinBox_OdomFovisStereoMaxDisparity->setValue(128);

        gridLayout_92->addWidget(spinBox_OdomFovisStereoMaxDisparity, 3, 0, 1, 1);

        label_408 = new QLabel(groupBox_26);
        label_408->setObjectName(QStringLiteral("label_408"));
        label_408->setWordWrap(true);
        label_408->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_92->addWidget(label_408, 1, 1, 1, 1);

        label_409 = new QLabel(groupBox_26);
        label_409->setObjectName(QStringLiteral("label_409"));
        label_409->setWordWrap(true);
        label_409->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_92->addWidget(label_409, 2, 1, 1, 1);

        label_410 = new QLabel(groupBox_26);
        label_410->setObjectName(QStringLiteral("label_410"));
        label_410->setWordWrap(true);
        label_410->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_92->addWidget(label_410, 3, 1, 1, 1);

        gridLayout_92->setColumnStretch(1, 1);

        verticalLayout_118->addWidget(groupBox_26);


        verticalLayout_119->addWidget(groupBox_odomFovis);

        stackedWidget_odometryType->addWidget(page_67);
        page_68 = new QWidget();
        page_68->setObjectName(QStringLiteral("page_68"));
        verticalLayout_121 = new QVBoxLayout(page_68);
        verticalLayout_121->setObjectName(QStringLiteral("verticalLayout_121"));
        groupBox_odomViso2 = new QGroupBox(page_68);
        groupBox_odomViso2->setObjectName(QStringLiteral("groupBox_odomViso2"));
        verticalLayout_120 = new QVBoxLayout(groupBox_odomViso2);
        verticalLayout_120->setObjectName(QStringLiteral("verticalLayout_120"));
        label_380 = new QLabel(groupBox_odomViso2);
        label_380->setObjectName(QStringLiteral("label_380"));
        label_380->setWordWrap(true);
        label_380->setOpenExternalLinks(true);
        label_380->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_120->addWidget(label_380);

        gridLayout_86 = new QGridLayout();
        gridLayout_86->setObjectName(QStringLiteral("gridLayout_86"));
        label_386 = new QLabel(groupBox_odomViso2);
        label_386->setObjectName(QStringLiteral("label_386"));
        label_386->setWordWrap(true);
        label_386->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_86->addWidget(label_386, 1, 1, 1, 1);

        label_384 = new QLabel(groupBox_odomViso2);
        label_384->setObjectName(QStringLiteral("label_384"));
        label_384->setWordWrap(true);
        label_384->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_86->addWidget(label_384, 2, 1, 1, 1);

        label_385 = new QLabel(groupBox_odomViso2);
        label_385->setObjectName(QStringLiteral("label_385"));
        label_385->setWordWrap(true);
        label_385->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_86->addWidget(label_385, 0, 1, 1, 1);

        spinBox_OdomViso2RansacIters = new QSpinBox(groupBox_odomViso2);
        spinBox_OdomViso2RansacIters->setObjectName(QStringLiteral("spinBox_OdomViso2RansacIters"));
        spinBox_OdomViso2RansacIters->setMinimum(1);
        spinBox_OdomViso2RansacIters->setMaximum(999);
        spinBox_OdomViso2RansacIters->setSingleStep(10);
        spinBox_OdomViso2RansacIters->setValue(200);

        gridLayout_86->addWidget(spinBox_OdomViso2RansacIters, 0, 0, 1, 1);

        doubleSpinBox_OdomViso2InlierThreshold = new QDoubleSpinBox(groupBox_odomViso2);
        doubleSpinBox_OdomViso2InlierThreshold->setObjectName(QStringLiteral("doubleSpinBox_OdomViso2InlierThreshold"));
        doubleSpinBox_OdomViso2InlierThreshold->setDecimals(1);
        doubleSpinBox_OdomViso2InlierThreshold->setMinimum(0.1);
        doubleSpinBox_OdomViso2InlierThreshold->setMaximum(9.9);
        doubleSpinBox_OdomViso2InlierThreshold->setSingleStep(0.1);
        doubleSpinBox_OdomViso2InlierThreshold->setValue(2);

        gridLayout_86->addWidget(doubleSpinBox_OdomViso2InlierThreshold, 1, 0, 1, 1);

        checkBox_OdomViso2Reweighting = new QCheckBox(groupBox_odomViso2);
        checkBox_OdomViso2Reweighting->setObjectName(QStringLiteral("checkBox_OdomViso2Reweighting"));
        checkBox_OdomViso2Reweighting->setChecked(true);

        gridLayout_86->addWidget(checkBox_OdomViso2Reweighting, 2, 0, 1, 1);

        gridLayout_86->setColumnStretch(1, 1);

        verticalLayout_120->addLayout(gridLayout_86);

        groupBox = new QGroupBox(groupBox_odomViso2);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        gridLayout_87 = new QGridLayout(groupBox);
        gridLayout_87->setObjectName(QStringLiteral("gridLayout_87"));
        label_389 = new QLabel(groupBox);
        label_389->setObjectName(QStringLiteral("label_389"));
        label_389->setWordWrap(true);
        label_389->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_87->addWidget(label_389, 2, 1, 1, 1);

        label_388 = new QLabel(groupBox);
        label_388->setObjectName(QStringLiteral("label_388"));
        label_388->setWordWrap(true);
        label_388->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_87->addWidget(label_388, 1, 1, 1, 1);

        label_391 = new QLabel(groupBox);
        label_391->setObjectName(QStringLiteral("label_391"));
        label_391->setWordWrap(true);
        label_391->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_87->addWidget(label_391, 4, 1, 1, 1);

        spinBox_OdomViso2MatchNmsN = new QSpinBox(groupBox);
        spinBox_OdomViso2MatchNmsN->setObjectName(QStringLiteral("spinBox_OdomViso2MatchNmsN"));
        spinBox_OdomViso2MatchNmsN->setMinimum(1);
        spinBox_OdomViso2MatchNmsN->setMaximum(99);
        spinBox_OdomViso2MatchNmsN->setSingleStep(1);
        spinBox_OdomViso2MatchNmsN->setValue(3);

        gridLayout_87->addWidget(spinBox_OdomViso2MatchNmsN, 0, 0, 1, 1);

        label_387 = new QLabel(groupBox);
        label_387->setObjectName(QStringLiteral("label_387"));
        label_387->setWordWrap(true);
        label_387->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_87->addWidget(label_387, 0, 1, 1, 1);

        label_392 = new QLabel(groupBox);
        label_392->setObjectName(QStringLiteral("label_392"));
        label_392->setWordWrap(true);
        label_392->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_87->addWidget(label_392, 5, 1, 1, 1);

        label_395 = new QLabel(groupBox);
        label_395->setObjectName(QStringLiteral("label_395"));
        label_395->setWordWrap(true);
        label_395->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_87->addWidget(label_395, 8, 1, 1, 1);

        label_394 = new QLabel(groupBox);
        label_394->setObjectName(QStringLiteral("label_394"));
        label_394->setWordWrap(true);
        label_394->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_87->addWidget(label_394, 7, 1, 1, 1);

        label_393 = new QLabel(groupBox);
        label_393->setObjectName(QStringLiteral("label_393"));
        label_393->setWordWrap(true);
        label_393->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_87->addWidget(label_393, 6, 1, 1, 1);

        label_390 = new QLabel(groupBox);
        label_390->setObjectName(QStringLiteral("label_390"));
        label_390->setWordWrap(true);
        label_390->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_87->addWidget(label_390, 3, 1, 1, 1);

        label_396 = new QLabel(groupBox);
        label_396->setObjectName(QStringLiteral("label_396"));
        label_396->setWordWrap(true);
        label_396->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_87->addWidget(label_396, 9, 1, 1, 1);

        spinBox_OdomViso2MatchNmsTau = new QSpinBox(groupBox);
        spinBox_OdomViso2MatchNmsTau->setObjectName(QStringLiteral("spinBox_OdomViso2MatchNmsTau"));
        spinBox_OdomViso2MatchNmsTau->setMinimum(1);
        spinBox_OdomViso2MatchNmsTau->setMaximum(999);
        spinBox_OdomViso2MatchNmsTau->setSingleStep(1);
        spinBox_OdomViso2MatchNmsTau->setValue(50);

        gridLayout_87->addWidget(spinBox_OdomViso2MatchNmsTau, 1, 0, 1, 1);

        spinBox_OdomViso2MatchBinsize = new QSpinBox(groupBox);
        spinBox_OdomViso2MatchBinsize->setObjectName(QStringLiteral("spinBox_OdomViso2MatchBinsize"));
        spinBox_OdomViso2MatchBinsize->setMinimum(1);
        spinBox_OdomViso2MatchBinsize->setMaximum(999);
        spinBox_OdomViso2MatchBinsize->setSingleStep(1);
        spinBox_OdomViso2MatchBinsize->setValue(50);

        gridLayout_87->addWidget(spinBox_OdomViso2MatchBinsize, 2, 0, 1, 1);

        spinBox_OdomViso2MatchRadius = new QSpinBox(groupBox);
        spinBox_OdomViso2MatchRadius->setObjectName(QStringLiteral("spinBox_OdomViso2MatchRadius"));
        spinBox_OdomViso2MatchRadius->setMinimum(1);
        spinBox_OdomViso2MatchRadius->setMaximum(999);
        spinBox_OdomViso2MatchRadius->setSingleStep(10);
        spinBox_OdomViso2MatchRadius->setValue(200);

        gridLayout_87->addWidget(spinBox_OdomViso2MatchRadius, 3, 0, 1, 1);

        spinBox_OdomViso2MatchDispTolerance = new QSpinBox(groupBox);
        spinBox_OdomViso2MatchDispTolerance->setObjectName(QStringLiteral("spinBox_OdomViso2MatchDispTolerance"));
        spinBox_OdomViso2MatchDispTolerance->setMinimum(1);
        spinBox_OdomViso2MatchDispTolerance->setMaximum(999);
        spinBox_OdomViso2MatchDispTolerance->setSingleStep(1);
        spinBox_OdomViso2MatchDispTolerance->setValue(2);

        gridLayout_87->addWidget(spinBox_OdomViso2MatchDispTolerance, 4, 0, 1, 1);

        spinBox_OdomViso2MatchOutlierDispTolerance = new QSpinBox(groupBox);
        spinBox_OdomViso2MatchOutlierDispTolerance->setObjectName(QStringLiteral("spinBox_OdomViso2MatchOutlierDispTolerance"));
        spinBox_OdomViso2MatchOutlierDispTolerance->setMinimum(1);
        spinBox_OdomViso2MatchOutlierDispTolerance->setMaximum(999);
        spinBox_OdomViso2MatchOutlierDispTolerance->setSingleStep(1);
        spinBox_OdomViso2MatchOutlierDispTolerance->setValue(5);

        gridLayout_87->addWidget(spinBox_OdomViso2MatchOutlierDispTolerance, 5, 0, 1, 1);

        spinBox_OdomViso2MatchOutlierFlowTolerance = new QSpinBox(groupBox);
        spinBox_OdomViso2MatchOutlierFlowTolerance->setObjectName(QStringLiteral("spinBox_OdomViso2MatchOutlierFlowTolerance"));
        spinBox_OdomViso2MatchOutlierFlowTolerance->setMinimum(1);
        spinBox_OdomViso2MatchOutlierFlowTolerance->setMaximum(999);
        spinBox_OdomViso2MatchOutlierFlowTolerance->setSingleStep(1);
        spinBox_OdomViso2MatchOutlierFlowTolerance->setValue(5);

        gridLayout_87->addWidget(spinBox_OdomViso2MatchOutlierFlowTolerance, 6, 0, 1, 1);

        spinBox_OdomViso2MatchRefinement = new QSpinBox(groupBox);
        spinBox_OdomViso2MatchRefinement->setObjectName(QStringLiteral("spinBox_OdomViso2MatchRefinement"));
        spinBox_OdomViso2MatchRefinement->setMinimum(1);
        spinBox_OdomViso2MatchRefinement->setMaximum(2);
        spinBox_OdomViso2MatchRefinement->setSingleStep(1);
        spinBox_OdomViso2MatchRefinement->setValue(1);

        gridLayout_87->addWidget(spinBox_OdomViso2MatchRefinement, 9, 0, 1, 1);

        checkBox_OdomViso2MatchMultiStage = new QCheckBox(groupBox);
        checkBox_OdomViso2MatchMultiStage->setObjectName(QStringLiteral("checkBox_OdomViso2MatchMultiStage"));
        checkBox_OdomViso2MatchMultiStage->setChecked(true);

        gridLayout_87->addWidget(checkBox_OdomViso2MatchMultiStage, 7, 0, 1, 1);

        checkBox_OdomViso2MatchHalfResolution = new QCheckBox(groupBox);
        checkBox_OdomViso2MatchHalfResolution->setObjectName(QStringLiteral("checkBox_OdomViso2MatchHalfResolution"));
        checkBox_OdomViso2MatchHalfResolution->setChecked(true);

        gridLayout_87->addWidget(checkBox_OdomViso2MatchHalfResolution, 8, 0, 1, 1);

        gridLayout_87->setColumnStretch(1, 1);

        verticalLayout_120->addWidget(groupBox);

        groupBox_22 = new QGroupBox(groupBox_odomViso2);
        groupBox_22->setObjectName(QStringLiteral("groupBox_22"));
        gridLayout_89 = new QGridLayout(groupBox_22);
        gridLayout_89->setObjectName(QStringLiteral("gridLayout_89"));
        label_398 = new QLabel(groupBox_22);
        label_398->setObjectName(QStringLiteral("label_398"));
        label_398->setWordWrap(true);
        label_398->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_89->addWidget(label_398, 1, 1, 1, 1);

        label_397 = new QLabel(groupBox_22);
        label_397->setObjectName(QStringLiteral("label_397"));
        label_397->setWordWrap(true);
        label_397->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_89->addWidget(label_397, 0, 1, 1, 1);

        label_399 = new QLabel(groupBox_22);
        label_399->setObjectName(QStringLiteral("label_399"));
        label_399->setWordWrap(true);
        label_399->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_89->addWidget(label_399, 2, 1, 1, 1);

        doubleSpinBox_OdomViso2BucketWidth = new QDoubleSpinBox(groupBox_22);
        doubleSpinBox_OdomViso2BucketWidth->setObjectName(QStringLiteral("doubleSpinBox_OdomViso2BucketWidth"));
        doubleSpinBox_OdomViso2BucketWidth->setDecimals(0);
        doubleSpinBox_OdomViso2BucketWidth->setMinimum(1);
        doubleSpinBox_OdomViso2BucketWidth->setMaximum(999);
        doubleSpinBox_OdomViso2BucketWidth->setSingleStep(1);
        doubleSpinBox_OdomViso2BucketWidth->setValue(50);

        gridLayout_89->addWidget(doubleSpinBox_OdomViso2BucketWidth, 1, 0, 1, 1);

        doubleSpinBox_OdomViso2BucketHeight = new QDoubleSpinBox(groupBox_22);
        doubleSpinBox_OdomViso2BucketHeight->setObjectName(QStringLiteral("doubleSpinBox_OdomViso2BucketHeight"));
        doubleSpinBox_OdomViso2BucketHeight->setDecimals(0);
        doubleSpinBox_OdomViso2BucketHeight->setMinimum(1);
        doubleSpinBox_OdomViso2BucketHeight->setMaximum(999);
        doubleSpinBox_OdomViso2BucketHeight->setSingleStep(1);
        doubleSpinBox_OdomViso2BucketHeight->setValue(50);

        gridLayout_89->addWidget(doubleSpinBox_OdomViso2BucketHeight, 2, 0, 1, 1);

        spinBox_OdomViso2BucketMaxFeatures = new QSpinBox(groupBox_22);
        spinBox_OdomViso2BucketMaxFeatures->setObjectName(QStringLiteral("spinBox_OdomViso2BucketMaxFeatures"));
        spinBox_OdomViso2BucketMaxFeatures->setMinimum(1);
        spinBox_OdomViso2BucketMaxFeatures->setMaximum(99);
        spinBox_OdomViso2BucketMaxFeatures->setSingleStep(1);
        spinBox_OdomViso2BucketMaxFeatures->setValue(2);

        gridLayout_89->addWidget(spinBox_OdomViso2BucketMaxFeatures, 0, 0, 1, 1);

        gridLayout_89->setColumnStretch(1, 1);

        verticalLayout_120->addWidget(groupBox_22);

        verticalSpacer_61 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_120->addItem(verticalSpacer_61);


        verticalLayout_121->addWidget(groupBox_odomViso2);

        stackedWidget_odometryType->addWidget(page_68);
        page_69 = new QWidget();
        page_69->setObjectName(QStringLiteral("page_69"));
        verticalLayout_122 = new QVBoxLayout(page_69);
        verticalLayout_122->setObjectName(QStringLiteral("verticalLayout_122"));
        groupBox_odomDVO = new QGroupBox(page_69);
        groupBox_odomDVO->setObjectName(QStringLiteral("groupBox_odomDVO"));
        verticalLayout_123 = new QVBoxLayout(groupBox_odomDVO);
        verticalLayout_123->setObjectName(QStringLiteral("verticalLayout_123"));
        label_411 = new QLabel(groupBox_odomDVO);
        label_411->setObjectName(QStringLiteral("label_411"));
        label_411->setWordWrap(true);
        label_411->setOpenExternalLinks(true);
        label_411->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_123->addWidget(label_411);


        verticalLayout_122->addWidget(groupBox_odomDVO);

        verticalSpacer_63 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_122->addItem(verticalSpacer_63);

        stackedWidget_odometryType->addWidget(page_69);
        page_70 = new QWidget();
        page_70->setObjectName(QStringLiteral("page_70"));
        verticalLayout_125 = new QVBoxLayout(page_70);
        verticalLayout_125->setObjectName(QStringLiteral("verticalLayout_125"));
        groupBox_odomORBSLAM = new QGroupBox(page_70);
        groupBox_odomORBSLAM->setObjectName(QStringLiteral("groupBox_odomORBSLAM"));
        verticalLayout_124 = new QVBoxLayout(groupBox_odomORBSLAM);
        verticalLayout_124->setObjectName(QStringLiteral("verticalLayout_124"));
        label_412 = new QLabel(groupBox_odomORBSLAM);
        label_412->setObjectName(QStringLiteral("label_412"));
        label_412->setWordWrap(true);
        label_412->setOpenExternalLinks(true);
        label_412->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_124->addWidget(label_412);

        gridLayout_93 = new QGridLayout();
        gridLayout_93->setObjectName(QStringLiteral("gridLayout_93"));
        doubleSpinBox_OdomORBSLAMBf = new QDoubleSpinBox(groupBox_odomORBSLAM);
        doubleSpinBox_OdomORBSLAMBf->setObjectName(QStringLiteral("doubleSpinBox_OdomORBSLAMBf"));
        doubleSpinBox_OdomORBSLAMBf->setDecimals(4);
        doubleSpinBox_OdomORBSLAMBf->setMinimum(0);
        doubleSpinBox_OdomORBSLAMBf->setMaximum(9.9999);
        doubleSpinBox_OdomORBSLAMBf->setSingleStep(0.01);
        doubleSpinBox_OdomORBSLAMBf->setValue(0.076);

        gridLayout_93->addWidget(doubleSpinBox_OdomORBSLAMBf, 1, 1, 1, 1);

        label_414 = new QLabel(groupBox_odomORBSLAM);
        label_414->setObjectName(QStringLiteral("label_414"));
        label_414->setWordWrap(true);
        label_414->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_93->addWidget(label_414, 2, 2, 1, 1);

        doubleSpinBox_OdomORBSLAMThDepth = new QDoubleSpinBox(groupBox_odomORBSLAM);
        doubleSpinBox_OdomORBSLAMThDepth->setObjectName(QStringLiteral("doubleSpinBox_OdomORBSLAMThDepth"));
        doubleSpinBox_OdomORBSLAMThDepth->setDecimals(1);
        doubleSpinBox_OdomORBSLAMThDepth->setMinimum(0.1);
        doubleSpinBox_OdomORBSLAMThDepth->setMaximum(999);
        doubleSpinBox_OdomORBSLAMThDepth->setSingleStep(1);
        doubleSpinBox_OdomORBSLAMThDepth->setValue(40);

        gridLayout_93->addWidget(doubleSpinBox_OdomORBSLAMThDepth, 2, 1, 1, 1);

        label_413 = new QLabel(groupBox_odomORBSLAM);
        label_413->setObjectName(QStringLiteral("label_413"));
        label_413->setWordWrap(true);
        label_413->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_93->addWidget(label_413, 1, 2, 1, 1);

        lineEdit_OdomORBSLAMVocPath = new QLineEdit(groupBox_odomORBSLAM);
        lineEdit_OdomORBSLAMVocPath->setObjectName(QStringLiteral("lineEdit_OdomORBSLAMVocPath"));

        gridLayout_93->addWidget(lineEdit_OdomORBSLAMVocPath, 0, 1, 1, 1);

        label_415 = new QLabel(groupBox_odomORBSLAM);
        label_415->setObjectName(QStringLiteral("label_415"));
        label_415->setWordWrap(true);
        label_415->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_93->addWidget(label_415, 0, 2, 1, 1);

        toolButton_OdomORBSLAMVocPath = new QToolButton(groupBox_odomORBSLAM);
        toolButton_OdomORBSLAMVocPath->setObjectName(QStringLiteral("toolButton_OdomORBSLAMVocPath"));

        gridLayout_93->addWidget(toolButton_OdomORBSLAMVocPath, 0, 0, 1, 1);

        label_444 = new QLabel(groupBox_odomORBSLAM);
        label_444->setObjectName(QStringLiteral("label_444"));
        label_444->setWordWrap(true);
        label_444->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_93->addWidget(label_444, 3, 2, 1, 1);

        doubleSpinBox_OdomORBSLAMFps = new QDoubleSpinBox(groupBox_odomORBSLAM);
        doubleSpinBox_OdomORBSLAMFps->setObjectName(QStringLiteral("doubleSpinBox_OdomORBSLAMFps"));
        doubleSpinBox_OdomORBSLAMFps->setEnabled(false);
        doubleSpinBox_OdomORBSLAMFps->setDecimals(2);
        doubleSpinBox_OdomORBSLAMFps->setMinimum(0);
        doubleSpinBox_OdomORBSLAMFps->setMaximum(999);
        doubleSpinBox_OdomORBSLAMFps->setSingleStep(1);
        doubleSpinBox_OdomORBSLAMFps->setValue(0);

        gridLayout_93->addWidget(doubleSpinBox_OdomORBSLAMFps, 3, 1, 1, 1);

        label_445 = new QLabel(groupBox_odomORBSLAM);
        label_445->setObjectName(QStringLiteral("label_445"));
        label_445->setWordWrap(true);
        label_445->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_93->addWidget(label_445, 4, 2, 1, 1);

        spinBox_OdomORBSLAMMaxFeatures = new QSpinBox(groupBox_odomORBSLAM);
        spinBox_OdomORBSLAMMaxFeatures->setObjectName(QStringLiteral("spinBox_OdomORBSLAMMaxFeatures"));
        spinBox_OdomORBSLAMMaxFeatures->setMaximum(99999);
        spinBox_OdomORBSLAMMaxFeatures->setValue(1000);

        gridLayout_93->addWidget(spinBox_OdomORBSLAMMaxFeatures, 4, 1, 1, 1);

        label_449 = new QLabel(groupBox_odomORBSLAM);
        label_449->setObjectName(QStringLiteral("label_449"));
        label_449->setWordWrap(true);
        label_449->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_93->addWidget(label_449, 5, 2, 1, 1);

        spinBox_OdomORBSLAMMapSize = new QSpinBox(groupBox_odomORBSLAM);
        spinBox_OdomORBSLAMMapSize->setObjectName(QStringLiteral("spinBox_OdomORBSLAMMapSize"));
        spinBox_OdomORBSLAMMapSize->setMaximum(999999);
        spinBox_OdomORBSLAMMapSize->setValue(0);

        gridLayout_93->addWidget(spinBox_OdomORBSLAMMapSize, 5, 1, 1, 1);

        gridLayout_93->setColumnStretch(2, 1);

        verticalLayout_124->addLayout(gridLayout_93);


        verticalLayout_125->addWidget(groupBox_odomORBSLAM);

        verticalSpacer_64 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_125->addItem(verticalSpacer_64);

        stackedWidget_odometryType->addWidget(page_70);
        page_54 = new QWidget();
        page_54->setObjectName(QStringLiteral("page_54"));
        verticalLayout_85 = new QVBoxLayout(page_54);
        verticalLayout_85->setObjectName(QStringLiteral("verticalLayout_85"));
        groupBox_odomOKVIS = new QGroupBox(page_54);
        groupBox_odomOKVIS->setObjectName(QStringLiteral("groupBox_odomOKVIS"));
        verticalLayout_128 = new QVBoxLayout(groupBox_odomOKVIS);
        verticalLayout_128->setObjectName(QStringLiteral("verticalLayout_128"));
        label_466 = new QLabel(groupBox_odomOKVIS);
        label_466->setObjectName(QStringLiteral("label_466"));
        label_466->setWordWrap(true);
        label_466->setOpenExternalLinks(true);
        label_466->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_128->addWidget(label_466);

        gridLayout_98 = new QGridLayout();
        gridLayout_98->setObjectName(QStringLiteral("gridLayout_98"));
        lineEdit_OdomOkvisPath = new QLineEdit(groupBox_odomOKVIS);
        lineEdit_OdomOkvisPath->setObjectName(QStringLiteral("lineEdit_OdomOkvisPath"));

        gridLayout_98->addWidget(lineEdit_OdomOkvisPath, 0, 1, 1, 1);

        label_469 = new QLabel(groupBox_odomOKVIS);
        label_469->setObjectName(QStringLiteral("label_469"));
        label_469->setWordWrap(true);
        label_469->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_98->addWidget(label_469, 0, 2, 1, 1);

        toolButton_OdomOkvisPath = new QToolButton(groupBox_odomOKVIS);
        toolButton_OdomOkvisPath->setObjectName(QStringLiteral("toolButton_OdomOkvisPath"));

        gridLayout_98->addWidget(toolButton_OdomOkvisPath, 0, 0, 1, 1);

        gridLayout_98->setColumnStretch(2, 1);

        verticalLayout_128->addLayout(gridLayout_98);

        verticalLayout_128->setStretch(1, 1);

        verticalLayout_85->addWidget(groupBox_odomOKVIS);

        verticalSpacer_69 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_85->addItem(verticalSpacer_69);

        verticalLayout_85->setStretch(1, 1);
        stackedWidget_odometryType->addWidget(page_54);
        page_73 = new QWidget();
        page_73->setObjectName(QStringLiteral("page_73"));
        verticalLayout_130 = new QVBoxLayout(page_73);
        verticalLayout_130->setObjectName(QStringLiteral("verticalLayout_130"));
        groupBox_odomLOAM = new QGroupBox(page_73);
        groupBox_odomLOAM->setObjectName(QStringLiteral("groupBox_odomLOAM"));
        verticalLayout_129 = new QVBoxLayout(groupBox_odomLOAM);
        verticalLayout_129->setObjectName(QStringLiteral("verticalLayout_129"));
        label_472 = new QLabel(groupBox_odomLOAM);
        label_472->setObjectName(QStringLiteral("label_472"));
        label_472->setWordWrap(true);
        label_472->setOpenExternalLinks(true);
        label_472->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_129->addWidget(label_472);

        gridLayout_101 = new QGridLayout();
        gridLayout_101->setObjectName(QStringLiteral("gridLayout_101"));
        label_475 = new QLabel(groupBox_odomLOAM);
        label_475->setObjectName(QStringLiteral("label_475"));
        label_475->setWordWrap(true);
        label_475->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_101->addWidget(label_475, 2, 1, 1, 1);

        label_473 = new QLabel(groupBox_odomLOAM);
        label_473->setObjectName(QStringLiteral("label_473"));
        label_473->setWordWrap(true);
        label_473->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_101->addWidget(label_473, 0, 1, 1, 1);

        label_474 = new QLabel(groupBox_odomLOAM);
        label_474->setObjectName(QStringLiteral("label_474"));
        label_474->setWordWrap(true);
        label_474->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_101->addWidget(label_474, 1, 1, 1, 1);

        odom_loam_scan_period = new QDoubleSpinBox(groupBox_odomLOAM);
        odom_loam_scan_period->setObjectName(QStringLiteral("odom_loam_scan_period"));
        odom_loam_scan_period->setMinimum(0.01);
        odom_loam_scan_period->setMaximum(1);
        odom_loam_scan_period->setSingleStep(0.1);
        odom_loam_scan_period->setValue(0.5);

        gridLayout_101->addWidget(odom_loam_scan_period, 1, 0, 1, 1);

        odom_loam_sensor = new QComboBox(groupBox_odomLOAM);
        odom_loam_sensor->setObjectName(QStringLiteral("odom_loam_sensor"));
        odom_loam_sensor->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_101->addWidget(odom_loam_sensor, 0, 0, 1, 1);

        label_476 = new QLabel(groupBox_odomLOAM);
        label_476->setObjectName(QStringLiteral("label_476"));
        label_476->setWordWrap(true);
        label_476->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_101->addWidget(label_476, 3, 1, 1, 1);

        odom_loam_linvar = new QDoubleSpinBox(groupBox_odomLOAM);
        odom_loam_linvar->setObjectName(QStringLiteral("odom_loam_linvar"));
        odom_loam_linvar->setDecimals(4);
        odom_loam_linvar->setMinimum(0.0001);
        odom_loam_linvar->setMaximum(1);
        odom_loam_linvar->setSingleStep(0.0001);
        odom_loam_linvar->setValue(0.5);

        gridLayout_101->addWidget(odom_loam_linvar, 2, 0, 1, 1);

        odom_loam_angvar = new QDoubleSpinBox(groupBox_odomLOAM);
        odom_loam_angvar->setObjectName(QStringLiteral("odom_loam_angvar"));
        odom_loam_angvar->setDecimals(4);
        odom_loam_angvar->setMinimum(0.0001);
        odom_loam_angvar->setMaximum(1);
        odom_loam_angvar->setSingleStep(0.0001);
        odom_loam_angvar->setValue(0.5);

        gridLayout_101->addWidget(odom_loam_angvar, 3, 0, 1, 1);

        odom_loam_localMapping = new QCheckBox(groupBox_odomLOAM);
        odom_loam_localMapping->setObjectName(QStringLiteral("odom_loam_localMapping"));

        gridLayout_101->addWidget(odom_loam_localMapping, 4, 0, 1, 1);

        label_477 = new QLabel(groupBox_odomLOAM);
        label_477->setObjectName(QStringLiteral("label_477"));
        label_477->setWordWrap(true);
        label_477->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_101->addWidget(label_477, 4, 1, 1, 1);

        gridLayout_101->setColumnStretch(1, 1);

        verticalLayout_129->addLayout(gridLayout_101);

        verticalLayout_129->setStretch(1, 1);

        verticalLayout_130->addWidget(groupBox_odomLOAM);

        verticalSpacer_73 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_130->addItem(verticalSpacer_73);

        verticalLayout_130->setStretch(1, 1);
        stackedWidget_odometryType->addWidget(page_73);
        page_74 = new QWidget();
        page_74->setObjectName(QStringLiteral("page_74"));
        verticalLayout_132 = new QVBoxLayout(page_74);
        verticalLayout_132->setObjectName(QStringLiteral("verticalLayout_132"));
        groupBox_odomMSCKF = new QGroupBox(page_74);
        groupBox_odomMSCKF->setObjectName(QStringLiteral("groupBox_odomMSCKF"));
        verticalLayout_131 = new QVBoxLayout(groupBox_odomMSCKF);
        verticalLayout_131->setObjectName(QStringLiteral("verticalLayout_131"));
        label_480 = new QLabel(groupBox_odomMSCKF);
        label_480->setObjectName(QStringLiteral("label_480"));
        label_480->setWordWrap(true);
        label_480->setOpenExternalLinks(true);
        label_480->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_131->addWidget(label_480);

        groupBox_7 = new QGroupBox(groupBox_odomMSCKF);
        groupBox_7->setObjectName(QStringLiteral("groupBox_7"));
        gridLayout_103 = new QGridLayout(groupBox_7);
        gridLayout_103->setObjectName(QStringLiteral("gridLayout_103"));
        OdomMSCKFGridRow = new QSpinBox(groupBox_7);
        OdomMSCKFGridRow->setObjectName(QStringLiteral("OdomMSCKFGridRow"));
        OdomMSCKFGridRow->setMinimum(1);

        gridLayout_103->addWidget(OdomMSCKFGridRow, 0, 0, 1, 1);

        label_538 = new QLabel(groupBox_7);
        label_538->setObjectName(QStringLiteral("label_538"));
        label_538->setWordWrap(true);
        label_538->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_103->addWidget(label_538, 0, 1, 1, 1);

        OdomMSCKFGridCol = new QSpinBox(groupBox_7);
        OdomMSCKFGridCol->setObjectName(QStringLiteral("OdomMSCKFGridCol"));
        OdomMSCKFGridCol->setMinimum(1);

        gridLayout_103->addWidget(OdomMSCKFGridCol, 1, 0, 1, 1);

        label_537 = new QLabel(groupBox_7);
        label_537->setObjectName(QStringLiteral("label_537"));
        label_537->setWordWrap(true);
        label_537->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_103->addWidget(label_537, 1, 1, 1, 1);

        OdomMSCKFGridMinFeatureNum = new QSpinBox(groupBox_7);
        OdomMSCKFGridMinFeatureNum->setObjectName(QStringLiteral("OdomMSCKFGridMinFeatureNum"));

        gridLayout_103->addWidget(OdomMSCKFGridMinFeatureNum, 2, 0, 1, 1);

        label_536 = new QLabel(groupBox_7);
        label_536->setObjectName(QStringLiteral("label_536"));
        label_536->setWordWrap(true);
        label_536->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_103->addWidget(label_536, 2, 1, 1, 1);

        OdomMSCKFGridMaxFeatureNum = new QSpinBox(groupBox_7);
        OdomMSCKFGridMaxFeatureNum->setObjectName(QStringLiteral("OdomMSCKFGridMaxFeatureNum"));

        gridLayout_103->addWidget(OdomMSCKFGridMaxFeatureNum, 3, 0, 1, 1);

        label_535 = new QLabel(groupBox_7);
        label_535->setObjectName(QStringLiteral("label_535"));
        label_535->setWordWrap(true);
        label_535->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_103->addWidget(label_535, 3, 1, 1, 1);

        OdomMSCKFPyramidLevels = new QSpinBox(groupBox_7);
        OdomMSCKFPyramidLevels->setObjectName(QStringLiteral("OdomMSCKFPyramidLevels"));

        gridLayout_103->addWidget(OdomMSCKFPyramidLevels, 4, 0, 1, 1);

        label_534 = new QLabel(groupBox_7);
        label_534->setObjectName(QStringLiteral("label_534"));
        label_534->setWordWrap(true);
        label_534->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_103->addWidget(label_534, 4, 1, 1, 1);

        OdomMSCKFPatchSize = new QSpinBox(groupBox_7);
        OdomMSCKFPatchSize->setObjectName(QStringLiteral("OdomMSCKFPatchSize"));
        OdomMSCKFPatchSize->setMinimum(1);

        gridLayout_103->addWidget(OdomMSCKFPatchSize, 5, 0, 1, 1);

        label_533 = new QLabel(groupBox_7);
        label_533->setObjectName(QStringLiteral("label_533"));
        label_533->setWordWrap(true);
        label_533->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_103->addWidget(label_533, 5, 1, 1, 1);

        OdomMSCKFFastThreshold = new QSpinBox(groupBox_7);
        OdomMSCKFFastThreshold->setObjectName(QStringLiteral("OdomMSCKFFastThreshold"));

        gridLayout_103->addWidget(OdomMSCKFFastThreshold, 6, 0, 1, 1);

        label_532 = new QLabel(groupBox_7);
        label_532->setObjectName(QStringLiteral("label_532"));
        label_532->setWordWrap(true);
        label_532->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_103->addWidget(label_532, 6, 1, 1, 1);

        OdomMSCKFMaxIteration = new QSpinBox(groupBox_7);
        OdomMSCKFMaxIteration->setObjectName(QStringLiteral("OdomMSCKFMaxIteration"));

        gridLayout_103->addWidget(OdomMSCKFMaxIteration, 7, 0, 1, 1);

        label_531 = new QLabel(groupBox_7);
        label_531->setObjectName(QStringLiteral("label_531"));
        label_531->setWordWrap(true);
        label_531->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_103->addWidget(label_531, 7, 1, 1, 1);

        OdomMSCKFTrackPrecision = new QDoubleSpinBox(groupBox_7);
        OdomMSCKFTrackPrecision->setObjectName(QStringLiteral("OdomMSCKFTrackPrecision"));
        OdomMSCKFTrackPrecision->setMinimum(0);
        OdomMSCKFTrackPrecision->setMaximum(1);
        OdomMSCKFTrackPrecision->setSingleStep(0.1);
        OdomMSCKFTrackPrecision->setValue(0.5);

        gridLayout_103->addWidget(OdomMSCKFTrackPrecision, 8, 0, 1, 1);

        label_495 = new QLabel(groupBox_7);
        label_495->setObjectName(QStringLiteral("label_495"));
        label_495->setWordWrap(true);
        label_495->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_103->addWidget(label_495, 8, 1, 1, 1);

        OdomMSCKFRansacThreshold = new QDoubleSpinBox(groupBox_7);
        OdomMSCKFRansacThreshold->setObjectName(QStringLiteral("OdomMSCKFRansacThreshold"));
        OdomMSCKFRansacThreshold->setDecimals(1);
        OdomMSCKFRansacThreshold->setMinimum(0);
        OdomMSCKFRansacThreshold->setMaximum(99);
        OdomMSCKFRansacThreshold->setSingleStep(0.1);
        OdomMSCKFRansacThreshold->setValue(0.5);

        gridLayout_103->addWidget(OdomMSCKFRansacThreshold, 9, 0, 1, 1);

        label_496 = new QLabel(groupBox_7);
        label_496->setObjectName(QStringLiteral("label_496"));
        label_496->setWordWrap(true);
        label_496->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_103->addWidget(label_496, 9, 1, 1, 1);

        OdomMSCKFStereoThreshold = new QDoubleSpinBox(groupBox_7);
        OdomMSCKFStereoThreshold->setObjectName(QStringLiteral("OdomMSCKFStereoThreshold"));
        OdomMSCKFStereoThreshold->setDecimals(1);
        OdomMSCKFStereoThreshold->setMinimum(0);
        OdomMSCKFStereoThreshold->setMaximum(99);
        OdomMSCKFStereoThreshold->setSingleStep(0.1);
        OdomMSCKFStereoThreshold->setValue(0.5);

        gridLayout_103->addWidget(OdomMSCKFStereoThreshold, 10, 0, 1, 1);

        label_497 = new QLabel(groupBox_7);
        label_497->setObjectName(QStringLiteral("label_497"));
        label_497->setWordWrap(true);
        label_497->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_103->addWidget(label_497, 10, 1, 1, 1);

        gridLayout_103->setColumnStretch(1, 1);

        verticalLayout_131->addWidget(groupBox_7);

        groupBox_28 = new QGroupBox(groupBox_odomMSCKF);
        groupBox_28->setObjectName(QStringLiteral("groupBox_28"));
        gridLayout_102 = new QGridLayout(groupBox_28);
        gridLayout_102->setObjectName(QStringLiteral("gridLayout_102"));
        OdomMSCKFMaxCamStateSize = new QSpinBox(groupBox_28);
        OdomMSCKFMaxCamStateSize->setObjectName(QStringLiteral("OdomMSCKFMaxCamStateSize"));

        gridLayout_102->addWidget(OdomMSCKFMaxCamStateSize, 0, 0, 1, 1);

        label_539 = new QLabel(groupBox_28);
        label_539->setObjectName(QStringLiteral("label_539"));
        label_539->setWordWrap(true);
        label_539->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_539, 0, 1, 1, 1);

        OdomMSCKFPositionStdThreshold = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFPositionStdThreshold->setObjectName(QStringLiteral("OdomMSCKFPositionStdThreshold"));
        OdomMSCKFPositionStdThreshold->setDecimals(2);
        OdomMSCKFPositionStdThreshold->setMinimum(0);
        OdomMSCKFPositionStdThreshold->setMaximum(99);
        OdomMSCKFPositionStdThreshold->setSingleStep(0.1);
        OdomMSCKFPositionStdThreshold->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFPositionStdThreshold, 1, 0, 1, 1);

        label_486 = new QLabel(groupBox_28);
        label_486->setObjectName(QStringLiteral("label_486"));
        label_486->setWordWrap(true);
        label_486->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_486, 1, 1, 1, 1);

        OdomMSCKFRotationThreshold = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFRotationThreshold->setObjectName(QStringLiteral("OdomMSCKFRotationThreshold"));
        OdomMSCKFRotationThreshold->setDecimals(6);
        OdomMSCKFRotationThreshold->setMaximum(99);
        OdomMSCKFRotationThreshold->setSingleStep(0.1);
        OdomMSCKFRotationThreshold->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFRotationThreshold, 2, 0, 1, 1);

        label_487 = new QLabel(groupBox_28);
        label_487->setObjectName(QStringLiteral("label_487"));
        label_487->setWordWrap(true);
        label_487->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_487, 2, 1, 1, 1);

        OdomMSCKFTranslationThreshold = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFTranslationThreshold->setObjectName(QStringLiteral("OdomMSCKFTranslationThreshold"));
        OdomMSCKFTranslationThreshold->setMinimum(0);
        OdomMSCKFTranslationThreshold->setMaximum(99);
        OdomMSCKFTranslationThreshold->setSingleStep(0.1);
        OdomMSCKFTranslationThreshold->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFTranslationThreshold, 3, 0, 1, 1);

        label_488 = new QLabel(groupBox_28);
        label_488->setObjectName(QStringLiteral("label_488"));
        label_488->setWordWrap(true);
        label_488->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_488, 3, 1, 1, 1);

        OdomMSCKFTrackingRateThreshold = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFTrackingRateThreshold->setObjectName(QStringLiteral("OdomMSCKFTrackingRateThreshold"));
        OdomMSCKFTrackingRateThreshold->setMaximum(99);
        OdomMSCKFTrackingRateThreshold->setSingleStep(0.1);
        OdomMSCKFTrackingRateThreshold->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFTrackingRateThreshold, 4, 0, 1, 1);

        label_489 = new QLabel(groupBox_28);
        label_489->setObjectName(QStringLiteral("label_489"));
        label_489->setWordWrap(true);
        label_489->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_489, 4, 1, 1, 1);

        OdomMSCKFOptTranslationThreshold = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFOptTranslationThreshold->setObjectName(QStringLiteral("OdomMSCKFOptTranslationThreshold"));
        OdomMSCKFOptTranslationThreshold->setDecimals(3);
        OdomMSCKFOptTranslationThreshold->setMaximum(99);
        OdomMSCKFOptTranslationThreshold->setSingleStep(0.1);
        OdomMSCKFOptTranslationThreshold->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFOptTranslationThreshold, 5, 0, 1, 1);

        label_490 = new QLabel(groupBox_28);
        label_490->setObjectName(QStringLiteral("label_490"));
        label_490->setWordWrap(true);
        label_490->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_490, 5, 1, 1, 1);

        OdomMSCKFNoiseGyro = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFNoiseGyro->setObjectName(QStringLiteral("OdomMSCKFNoiseGyro"));
        OdomMSCKFNoiseGyro->setDecimals(6);
        OdomMSCKFNoiseGyro->setMaximum(1);
        OdomMSCKFNoiseGyro->setSingleStep(0.1);
        OdomMSCKFNoiseGyro->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFNoiseGyro, 6, 0, 1, 1);

        label_491 = new QLabel(groupBox_28);
        label_491->setObjectName(QStringLiteral("label_491"));
        label_491->setWordWrap(true);
        label_491->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_491, 6, 1, 1, 1);

        OdomMSCKFNoiseAcc = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFNoiseAcc->setObjectName(QStringLiteral("OdomMSCKFNoiseAcc"));
        OdomMSCKFNoiseAcc->setDecimals(6);
        OdomMSCKFNoiseAcc->setMaximum(1);
        OdomMSCKFNoiseAcc->setSingleStep(0.1);
        OdomMSCKFNoiseAcc->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFNoiseAcc, 7, 0, 1, 1);

        label_492 = new QLabel(groupBox_28);
        label_492->setObjectName(QStringLiteral("label_492"));
        label_492->setWordWrap(true);
        label_492->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_492, 7, 1, 1, 1);

        OdomMSCKFNoiseGyroBias = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFNoiseGyroBias->setObjectName(QStringLiteral("OdomMSCKFNoiseGyroBias"));
        OdomMSCKFNoiseGyroBias->setDecimals(6);
        OdomMSCKFNoiseGyroBias->setMaximum(1);
        OdomMSCKFNoiseGyroBias->setSingleStep(0.1);
        OdomMSCKFNoiseGyroBias->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFNoiseGyroBias, 8, 0, 1, 1);

        label_493 = new QLabel(groupBox_28);
        label_493->setObjectName(QStringLiteral("label_493"));
        label_493->setWordWrap(true);
        label_493->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_493, 8, 1, 1, 1);

        OdomMSCKFNoiseAccBias = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFNoiseAccBias->setObjectName(QStringLiteral("OdomMSCKFNoiseAccBias"));
        OdomMSCKFNoiseAccBias->setDecimals(6);
        OdomMSCKFNoiseAccBias->setMaximum(1);
        OdomMSCKFNoiseAccBias->setSingleStep(0.1);
        OdomMSCKFNoiseAccBias->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFNoiseAccBias, 9, 0, 1, 1);

        label_494 = new QLabel(groupBox_28);
        label_494->setObjectName(QStringLiteral("label_494"));
        label_494->setWordWrap(true);
        label_494->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_494, 9, 1, 1, 1);

        OdomMSCKFNoiseFeature = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFNoiseFeature->setObjectName(QStringLiteral("OdomMSCKFNoiseFeature"));
        OdomMSCKFNoiseFeature->setDecimals(6);
        OdomMSCKFNoiseFeature->setMaximum(1);
        OdomMSCKFNoiseFeature->setSingleStep(0.1);
        OdomMSCKFNoiseFeature->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFNoiseFeature, 10, 0, 1, 1);

        label_507 = new QLabel(groupBox_28);
        label_507->setObjectName(QStringLiteral("label_507"));
        label_507->setWordWrap(true);
        label_507->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_507, 10, 1, 1, 1);

        OdomMSCKFInitCovVel = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFInitCovVel->setObjectName(QStringLiteral("OdomMSCKFInitCovVel"));
        OdomMSCKFInitCovVel->setDecimals(4);
        OdomMSCKFInitCovVel->setMaximum(1);
        OdomMSCKFInitCovVel->setSingleStep(0.1);
        OdomMSCKFInitCovVel->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFInitCovVel, 11, 0, 1, 1);

        label_508 = new QLabel(groupBox_28);
        label_508->setObjectName(QStringLiteral("label_508"));
        label_508->setWordWrap(true);
        label_508->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_508, 11, 1, 1, 1);

        OdomMSCKFInitCovGyroBias = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFInitCovGyroBias->setObjectName(QStringLiteral("OdomMSCKFInitCovGyroBias"));
        OdomMSCKFInitCovGyroBias->setDecimals(5);
        OdomMSCKFInitCovGyroBias->setMaximum(1);
        OdomMSCKFInitCovGyroBias->setSingleStep(0.1);
        OdomMSCKFInitCovGyroBias->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFInitCovGyroBias, 12, 0, 1, 1);

        label_509 = new QLabel(groupBox_28);
        label_509->setObjectName(QStringLiteral("label_509"));
        label_509->setWordWrap(true);
        label_509->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_509, 12, 1, 1, 1);

        OdomMSCKFInitCovAccBias = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFInitCovAccBias->setObjectName(QStringLiteral("OdomMSCKFInitCovAccBias"));
        OdomMSCKFInitCovAccBias->setDecimals(3);
        OdomMSCKFInitCovAccBias->setMaximum(1);
        OdomMSCKFInitCovAccBias->setSingleStep(0.1);
        OdomMSCKFInitCovAccBias->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFInitCovAccBias, 13, 0, 1, 1);

        label_510 = new QLabel(groupBox_28);
        label_510->setObjectName(QStringLiteral("label_510"));
        label_510->setWordWrap(true);
        label_510->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_510, 13, 1, 1, 1);

        OdomMSCKFInitCovExRot = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFInitCovExRot->setObjectName(QStringLiteral("OdomMSCKFInitCovExRot"));
        OdomMSCKFInitCovExRot->setDecimals(8);
        OdomMSCKFInitCovExRot->setMaximum(1);
        OdomMSCKFInitCovExRot->setSingleStep(0.1);
        OdomMSCKFInitCovExRot->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFInitCovExRot, 14, 0, 1, 1);

        label_511 = new QLabel(groupBox_28);
        label_511->setObjectName(QStringLiteral("label_511"));
        label_511->setWordWrap(true);
        label_511->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_511, 14, 1, 1, 1);

        OdomMSCKFInitCovExTrans = new QDoubleSpinBox(groupBox_28);
        OdomMSCKFInitCovExTrans->setObjectName(QStringLiteral("OdomMSCKFInitCovExTrans"));
        OdomMSCKFInitCovExTrans->setDecimals(6);
        OdomMSCKFInitCovExTrans->setMaximum(1);
        OdomMSCKFInitCovExTrans->setSingleStep(0.1);
        OdomMSCKFInitCovExTrans->setValue(0.5);

        gridLayout_102->addWidget(OdomMSCKFInitCovExTrans, 15, 0, 1, 1);

        label_513 = new QLabel(groupBox_28);
        label_513->setObjectName(QStringLiteral("label_513"));
        label_513->setWordWrap(true);
        label_513->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_102->addWidget(label_513, 15, 1, 1, 1);

        gridLayout_102->setColumnStretch(1, 1);

        verticalLayout_131->addWidget(groupBox_28);


        verticalLayout_132->addWidget(groupBox_odomMSCKF);

        verticalSpacer_74 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_132->addItem(verticalSpacer_74);

        stackedWidget_odometryType->addWidget(page_74);
        page_79 = new QWidget();
        page_79->setObjectName(QStringLiteral("page_79"));
        verticalLayout_139 = new QVBoxLayout(page_79);
        verticalLayout_139->setObjectName(QStringLiteral("verticalLayout_139"));
        groupBox_odomVINS = new QGroupBox(page_79);
        groupBox_odomVINS->setObjectName(QStringLiteral("groupBox_odomVINS"));
        verticalLayout_138 = new QVBoxLayout(groupBox_odomVINS);
        verticalLayout_138->setObjectName(QStringLiteral("verticalLayout_138"));
        label_525 = new QLabel(groupBox_odomVINS);
        label_525->setObjectName(QStringLiteral("label_525"));
        label_525->setWordWrap(true);
        label_525->setOpenExternalLinks(true);
        label_525->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_138->addWidget(label_525);

        gridLayout_107 = new QGridLayout();
        gridLayout_107->setObjectName(QStringLiteral("gridLayout_107"));
        lineEdit_OdomVinsPath = new QLineEdit(groupBox_odomVINS);
        lineEdit_OdomVinsPath->setObjectName(QStringLiteral("lineEdit_OdomVinsPath"));

        gridLayout_107->addWidget(lineEdit_OdomVinsPath, 0, 1, 1, 1);

        label_526 = new QLabel(groupBox_odomVINS);
        label_526->setObjectName(QStringLiteral("label_526"));
        label_526->setWordWrap(true);
        label_526->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_107->addWidget(label_526, 0, 2, 1, 1);

        toolButton_OdomVinsPath = new QToolButton(groupBox_odomVINS);
        toolButton_OdomVinsPath->setObjectName(QStringLiteral("toolButton_OdomVinsPath"));

        gridLayout_107->addWidget(toolButton_OdomVinsPath, 0, 0, 1, 1);

        gridLayout_107->setColumnStretch(2, 1);

        verticalLayout_138->addLayout(gridLayout_107);


        verticalLayout_139->addWidget(groupBox_odomVINS);

        verticalSpacer_77 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_139->addItem(verticalSpacer_77);

        stackedWidget_odometryType->addWidget(page_79);
        page_26 = new QWidget();
        page_26->setObjectName(QStringLiteral("page_26"));
        verticalLayout_88 = new QVBoxLayout(page_26);
        verticalLayout_88->setObjectName(QStringLiteral("verticalLayout_88"));
        groupBox_odomMono = new QGroupBox(page_26);
        groupBox_odomMono->setObjectName(QStringLiteral("groupBox_odomMono"));
        verticalLayout_168 = new QVBoxLayout(groupBox_odomMono);
        verticalLayout_168->setObjectName(QStringLiteral("verticalLayout_168"));
        label_672 = new QLabel(groupBox_odomMono);
        label_672->setObjectName(QStringLiteral("label_672"));
        label_672->setWordWrap(true);
        label_672->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_168->addWidget(label_672);

        verticalSpacer_62 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_168->addItem(verticalSpacer_62);

        label_661 = new QLabel(groupBox_odomMono);
        label_661->setObjectName(QStringLiteral("label_661"));
        label_661->setWordWrap(true);
        label_661->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_168->addWidget(label_661);

        gridLayout_166 = new QGridLayout();
        gridLayout_166->setObjectName(QStringLiteral("gridLayout_166"));
        doubleSpinBox_minFlow = new QDoubleSpinBox(groupBox_odomMono);
        doubleSpinBox_minFlow->setObjectName(QStringLiteral("doubleSpinBox_minFlow"));
        doubleSpinBox_minFlow->setDecimals(0);
        doubleSpinBox_minFlow->setMaximum(1000);
        doubleSpinBox_minFlow->setSingleStep(10);
        doubleSpinBox_minFlow->setValue(100);

        gridLayout_166->addWidget(doubleSpinBox_minFlow, 0, 0, 1, 1);

        label_659 = new QLabel(groupBox_odomMono);
        label_659->setObjectName(QStringLiteral("label_659"));
        label_659->setWordWrap(true);
        label_659->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_166->addWidget(label_659, 0, 1, 1, 1);

        doubleSpinBox_minTranslation = new QDoubleSpinBox(groupBox_odomMono);
        doubleSpinBox_minTranslation->setObjectName(QStringLiteral("doubleSpinBox_minTranslation"));
        doubleSpinBox_minTranslation->setMinimum(0);
        doubleSpinBox_minTranslation->setSingleStep(0.01);
        doubleSpinBox_minTranslation->setValue(0.02);

        gridLayout_166->addWidget(doubleSpinBox_minTranslation, 2, 0, 1, 1);

        label_660 = new QLabel(groupBox_odomMono);
        label_660->setObjectName(QStringLiteral("label_660"));
        label_660->setWordWrap(true);
        label_660->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_166->addWidget(label_660, 2, 1, 1, 1);

        label_662 = new QLabel(groupBox_odomMono);
        label_662->setObjectName(QStringLiteral("label_662"));
        label_662->setWordWrap(true);
        label_662->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_166->addWidget(label_662, 1, 1, 1, 1);

        doubleSpinBox_minInitTranslation = new QDoubleSpinBox(groupBox_odomMono);
        doubleSpinBox_minInitTranslation->setObjectName(QStringLiteral("doubleSpinBox_minInitTranslation"));
        doubleSpinBox_minInitTranslation->setMinimum(0);
        doubleSpinBox_minInitTranslation->setSingleStep(0.01);
        doubleSpinBox_minInitTranslation->setValue(0.1);

        gridLayout_166->addWidget(doubleSpinBox_minInitTranslation, 1, 0, 1, 1);

        label_663 = new QLabel(groupBox_odomMono);
        label_663->setObjectName(QStringLiteral("label_663"));
        label_663->setWordWrap(true);
        label_663->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_166->addWidget(label_663, 3, 1, 1, 1);

        doubleSpinBox_maxVariance = new QDoubleSpinBox(groupBox_odomMono);
        doubleSpinBox_maxVariance->setObjectName(QStringLiteral("doubleSpinBox_maxVariance"));
        doubleSpinBox_maxVariance->setDecimals(3);
        doubleSpinBox_maxVariance->setMinimum(0);
        doubleSpinBox_maxVariance->setSingleStep(0.001);
        doubleSpinBox_maxVariance->setValue(0.01);

        gridLayout_166->addWidget(doubleSpinBox_maxVariance, 3, 0, 1, 1);

        gridLayout_166->setColumnStretch(1, 1);

        verticalLayout_168->addLayout(gridLayout_166);


        verticalLayout_88->addWidget(groupBox_odomMono);

        stackedWidget_odometryType->addWidget(page_26);

        verticalLayout_34->addWidget(stackedWidget_odometryType);

        stackedWidget_odometryFiltering = new QStackedWidget(groupBox_odometry1);
        stackedWidget_odometryFiltering->setObjectName(QStringLiteral("stackedWidget_odometryFiltering"));
        page_31 = new QWidget();
        page_31->setObjectName(QStringLiteral("page_31"));
        verticalLayout_84 = new QVBoxLayout(page_31);
        verticalLayout_84->setObjectName(QStringLiteral("verticalLayout_84"));
        stackedWidget_odometryFiltering->addWidget(page_31);
        page_57 = new QWidget();
        page_57->setObjectName(QStringLiteral("page_57"));
        verticalLayout_49 = new QVBoxLayout(page_57);
        verticalLayout_49->setObjectName(QStringLiteral("verticalLayout_49"));
        groupBox_odometryKalmanFilter2 = new QGroupBox(page_57);
        groupBox_odometryKalmanFilter2->setObjectName(QStringLiteral("groupBox_odometryKalmanFilter2"));
        verticalLayout_171 = new QVBoxLayout(groupBox_odometryKalmanFilter2);
        verticalLayout_171->setObjectName(QStringLiteral("verticalLayout_171"));
        label_673 = new QLabel(groupBox_odometryKalmanFilter2);
        label_673->setObjectName(QStringLiteral("label_673"));
        label_673->setWordWrap(true);
        label_673->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_171->addWidget(label_673);

        gridLayout_168 = new QGridLayout();
        gridLayout_168->setObjectName(QStringLiteral("gridLayout_168"));
        doubleSpinBox_kalmanMeasurementNoise = new QDoubleSpinBox(groupBox_odometryKalmanFilter2);
        doubleSpinBox_kalmanMeasurementNoise->setObjectName(QStringLiteral("doubleSpinBox_kalmanMeasurementNoise"));
        doubleSpinBox_kalmanMeasurementNoise->setDecimals(5);
        doubleSpinBox_kalmanMeasurementNoise->setMinimum(0);
        doubleSpinBox_kalmanMeasurementNoise->setMaximum(1);
        doubleSpinBox_kalmanMeasurementNoise->setSingleStep(0.01);
        doubleSpinBox_kalmanMeasurementNoise->setValue(0.01);

        gridLayout_168->addWidget(doubleSpinBox_kalmanMeasurementNoise, 1, 0, 1, 1);

        doubleSpinBox_kalmanProcessNoise = new QDoubleSpinBox(groupBox_odometryKalmanFilter2);
        doubleSpinBox_kalmanProcessNoise->setObjectName(QStringLiteral("doubleSpinBox_kalmanProcessNoise"));
        doubleSpinBox_kalmanProcessNoise->setDecimals(5);
        doubleSpinBox_kalmanProcessNoise->setMinimum(0);
        doubleSpinBox_kalmanProcessNoise->setMaximum(1);
        doubleSpinBox_kalmanProcessNoise->setSingleStep(0.001);
        doubleSpinBox_kalmanProcessNoise->setValue(0.001);

        gridLayout_168->addWidget(doubleSpinBox_kalmanProcessNoise, 0, 0, 1, 1);

        label_674 = new QLabel(groupBox_odometryKalmanFilter2);
        label_674->setObjectName(QStringLiteral("label_674"));
        label_674->setWordWrap(true);
        label_674->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_168->addWidget(label_674, 0, 1, 1, 1);

        label_675 = new QLabel(groupBox_odometryKalmanFilter2);
        label_675->setObjectName(QStringLiteral("label_675"));
        label_675->setWordWrap(true);
        label_675->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_168->addWidget(label_675, 1, 1, 1, 1);

        gridLayout_168->setColumnStretch(1, 1);

        verticalLayout_171->addLayout(gridLayout_168);

        verticalSpacer_31 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_171->addItem(verticalSpacer_31);


        verticalLayout_49->addWidget(groupBox_odometryKalmanFilter2);

        stackedWidget_odometryFiltering->addWidget(page_57);
        page_56 = new QWidget();
        page_56->setObjectName(QStringLiteral("page_56"));
        verticalLayout_87 = new QVBoxLayout(page_56);
        verticalLayout_87->setObjectName(QStringLiteral("verticalLayout_87"));
        groupBox_odometryParticleFilter2 = new QGroupBox(page_56);
        groupBox_odometryParticleFilter2->setObjectName(QStringLiteral("groupBox_odometryParticleFilter2"));
        verticalLayout_170 = new QVBoxLayout(groupBox_odometryParticleFilter2);
        verticalLayout_170->setObjectName(QStringLiteral("verticalLayout_170"));
        label_665 = new QLabel(groupBox_odometryParticleFilter2);
        label_665->setObjectName(QStringLiteral("label_665"));
        label_665->setWordWrap(true);
        label_665->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_170->addWidget(label_665);

        gridLayout_167 = new QGridLayout();
        gridLayout_167->setObjectName(QStringLiteral("gridLayout_167"));
        doubleSpinBox_particleLambdaT = new QDoubleSpinBox(groupBox_odometryParticleFilter2);
        doubleSpinBox_particleLambdaT->setObjectName(QStringLiteral("doubleSpinBox_particleLambdaT"));
        doubleSpinBox_particleLambdaT->setDecimals(1);
        doubleSpinBox_particleLambdaT->setMinimum(0);
        doubleSpinBox_particleLambdaT->setMaximum(1000);
        doubleSpinBox_particleLambdaT->setSingleStep(1);
        doubleSpinBox_particleLambdaT->setValue(15);

        gridLayout_167->addWidget(doubleSpinBox_particleLambdaT, 2, 0, 1, 1);

        doubleSpinBox_particleNoiseR = new QDoubleSpinBox(groupBox_odometryParticleFilter2);
        doubleSpinBox_particleNoiseR->setObjectName(QStringLiteral("doubleSpinBox_particleNoiseR"));
        doubleSpinBox_particleNoiseR->setDecimals(3);
        doubleSpinBox_particleNoiseR->setMinimum(0.001);
        doubleSpinBox_particleNoiseR->setMaximum(1);
        doubleSpinBox_particleNoiseR->setSingleStep(0.01);
        doubleSpinBox_particleNoiseR->setValue(0.005);

        gridLayout_167->addWidget(doubleSpinBox_particleNoiseR, 3, 0, 1, 1);

        doubleSpinBox_particleNoiseT = new QDoubleSpinBox(groupBox_odometryParticleFilter2);
        doubleSpinBox_particleNoiseT->setObjectName(QStringLiteral("doubleSpinBox_particleNoiseT"));
        doubleSpinBox_particleNoiseT->setDecimals(3);
        doubleSpinBox_particleNoiseT->setMinimum(0.001);
        doubleSpinBox_particleNoiseT->setMaximum(1);
        doubleSpinBox_particleNoiseT->setSingleStep(0.01);
        doubleSpinBox_particleNoiseT->setValue(0.05);

        gridLayout_167->addWidget(doubleSpinBox_particleNoiseT, 1, 0, 1, 1);

        label_666 = new QLabel(groupBox_odometryParticleFilter2);
        label_666->setObjectName(QStringLiteral("label_666"));
        label_666->setWordWrap(true);
        label_666->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_167->addWidget(label_666, 1, 1, 1, 1);

        label_667 = new QLabel(groupBox_odometryParticleFilter2);
        label_667->setObjectName(QStringLiteral("label_667"));
        label_667->setWordWrap(true);
        label_667->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_167->addWidget(label_667, 3, 1, 1, 1);

        label_668 = new QLabel(groupBox_odometryParticleFilter2);
        label_668->setObjectName(QStringLiteral("label_668"));
        label_668->setWordWrap(true);
        label_668->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_167->addWidget(label_668, 2, 1, 1, 1);

        label_669 = new QLabel(groupBox_odometryParticleFilter2);
        label_669->setObjectName(QStringLiteral("label_669"));
        label_669->setWordWrap(true);
        label_669->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_167->addWidget(label_669, 4, 1, 1, 1);

        doubleSpinBox_particleLambdaR = new QDoubleSpinBox(groupBox_odometryParticleFilter2);
        doubleSpinBox_particleLambdaR->setObjectName(QStringLiteral("doubleSpinBox_particleLambdaR"));
        doubleSpinBox_particleLambdaR->setDecimals(1);
        doubleSpinBox_particleLambdaR->setMinimum(0);
        doubleSpinBox_particleLambdaR->setMaximum(1000);
        doubleSpinBox_particleLambdaR->setSingleStep(1);
        doubleSpinBox_particleLambdaR->setValue(15);

        gridLayout_167->addWidget(doubleSpinBox_particleLambdaR, 4, 0, 1, 1);

        label_670 = new QLabel(groupBox_odometryParticleFilter2);
        label_670->setObjectName(QStringLiteral("label_670"));
        label_670->setWordWrap(true);
        label_670->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_167->addWidget(label_670, 0, 1, 1, 1);

        spinBox_particleSize = new QSpinBox(groupBox_odometryParticleFilter2);
        spinBox_particleSize->setObjectName(QStringLiteral("spinBox_particleSize"));
        spinBox_particleSize->setMinimum(1);
        spinBox_particleSize->setMaximum(10000);
        spinBox_particleSize->setValue(400);

        gridLayout_167->addWidget(spinBox_particleSize, 0, 0, 1, 1);

        gridLayout_167->setColumnStretch(1, 1);

        verticalLayout_170->addLayout(gridLayout_167);


        verticalLayout_87->addWidget(groupBox_odometryParticleFilter2);

        stackedWidget_odometryFiltering->addWidget(page_56);

        verticalLayout_34->addWidget(stackedWidget_odometryFiltering);

        label_162 = new QLabel(groupBox_odometry1);
        label_162->setObjectName(QStringLiteral("label_162"));
        label_162->setWordWrap(true);

        verticalLayout_34->addWidget(label_162);


        verticalLayout_32->addWidget(groupBox_odometry1);

        verticalSpacer_14 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_32->addItem(verticalSpacer_14);

        verticalLayout_32->setStretch(1, 1);
        stackedWidget->addWidget(page_12);
        page_46 = new QWidget();
        page_46->setObjectName(QStringLiteral("page_46"));
        verticalLayout_81 = new QVBoxLayout(page_46);
        verticalLayout_81->setObjectName(QStringLiteral("verticalLayout_81"));
        groupBox_motionEst1 = new QGroupBox(page_46);
        groupBox_motionEst1->setObjectName(QStringLiteral("groupBox_motionEst1"));
        gridLayout_53 = new QGridLayout(groupBox_motionEst1);
        gridLayout_53->setObjectName(QStringLiteral("gridLayout_53"));
        comboBox_registrationStrategy = new QComboBox(groupBox_motionEst1);
        comboBox_registrationStrategy->setObjectName(QStringLiteral("comboBox_registrationStrategy"));

        gridLayout_53->addWidget(comboBox_registrationStrategy, 0, 0, 1, 1);

        label_11 = new QLabel(groupBox_motionEst1);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setWordWrap(true);

        gridLayout_53->addWidget(label_11, 0, 1, 1, 1);

        loopClosure_bowForce2D = new QCheckBox(groupBox_motionEst1);
        loopClosure_bowForce2D->setObjectName(QStringLiteral("loopClosure_bowForce2D"));

        gridLayout_53->addWidget(loopClosure_bowForce2D, 1, 0, 1, 1);

        label_184 = new QLabel(groupBox_motionEst1);
        label_184->setObjectName(QStringLiteral("label_184"));
        label_184->setWordWrap(true);
        label_184->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_53->addWidget(label_184, 1, 1, 1, 1);

        label_370 = new QLabel(groupBox_motionEst1);
        label_370->setObjectName(QStringLiteral("label_370"));
        label_370->setWordWrap(true);
        label_370->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_53->addWidget(label_370, 2, 1, 1, 1);

        reg_repeatOnce = new QCheckBox(groupBox_motionEst1);
        reg_repeatOnce->setObjectName(QStringLiteral("reg_repeatOnce"));

        gridLayout_53->addWidget(reg_repeatOnce, 2, 0, 1, 1);

        gridLayout_53->setColumnStretch(1, 1);

        verticalLayout_81->addWidget(groupBox_motionEst1);

        verticalSpacer_41 = new QSpacerItem(20, 1221, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_81->addItem(verticalSpacer_41);

        stackedWidget->addWidget(page_46);
        page_21 = new QWidget();
        page_21->setObjectName(QStringLiteral("page_21"));
        verticalLayout_50 = new QVBoxLayout(page_21);
        verticalLayout_50->setObjectName(QStringLiteral("verticalLayout_50"));
        groupBox_visualTransform2 = new QGroupBox(page_21);
        groupBox_visualTransform2->setObjectName(QStringLiteral("groupBox_visualTransform2"));
        verticalLayout_41 = new QVBoxLayout(groupBox_visualTransform2);
        verticalLayout_41->setObjectName(QStringLiteral("verticalLayout_41"));
        gridLayout_23 = new QGridLayout();
        gridLayout_23->setObjectName(QStringLiteral("gridLayout_23"));
        loopClosure_estimationType = new QComboBox(groupBox_visualTransform2);
        loopClosure_estimationType->setObjectName(QStringLiteral("loopClosure_estimationType"));
        loopClosure_estimationType->setSizeAdjustPolicy(QComboBox::AdjustToContentsOnFirstShow);

        gridLayout_23->addWidget(loopClosure_estimationType, 0, 0, 1, 1);

        loopClosure_bowIterations = new QSpinBox(groupBox_visualTransform2);
        loopClosure_bowIterations->setObjectName(QStringLiteral("loopClosure_bowIterations"));
        loopClosure_bowIterations->setMinimum(1);
        loopClosure_bowIterations->setMaximum(10000);
        loopClosure_bowIterations->setSingleStep(1);
        loopClosure_bowIterations->setValue(100);

        gridLayout_23->addWidget(loopClosure_bowIterations, 4, 0, 1, 1);

        label_15 = new QLabel(groupBox_visualTransform2);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_23->addWidget(label_15, 4, 1, 1, 1);

        label_298 = new QLabel(groupBox_visualTransform2);
        label_298->setObjectName(QStringLiteral("label_298"));
        label_298->setWordWrap(true);
        label_298->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_23->addWidget(label_298, 5, 1, 1, 1);

        loopClosure_forwardEst = new QCheckBox(groupBox_visualTransform2);
        loopClosure_forwardEst->setObjectName(QStringLiteral("loopClosure_forwardEst"));

        gridLayout_23->addWidget(loopClosure_forwardEst, 5, 0, 1, 1);

        loopClosure_bowMinInliers = new QSpinBox(groupBox_visualTransform2);
        loopClosure_bowMinInliers->setObjectName(QStringLiteral("loopClosure_bowMinInliers"));
        loopClosure_bowMinInliers->setMinimum(6);
        loopClosure_bowMinInliers->setMaximum(1000);
        loopClosure_bowMinInliers->setValue(10);

        gridLayout_23->addWidget(loopClosure_bowMinInliers, 1, 0, 1, 1);

        label_146 = new QLabel(groupBox_visualTransform2);
        label_146->setObjectName(QStringLiteral("label_146"));
        label_146->setWordWrap(true);
        label_146->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_23->addWidget(label_146, 0, 1, 1, 1);

        label_2 = new QLabel(groupBox_visualTransform2);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setWordWrap(true);
        label_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_23->addWidget(label_2, 1, 1, 1, 1);

        loopClosure_bundle = new QComboBox(groupBox_visualTransform2);
        loopClosure_bundle->setObjectName(QStringLiteral("loopClosure_bundle"));
        loopClosure_bundle->setSizeAdjustPolicy(QComboBox::AdjustToContentsOnFirstShow);

        gridLayout_23->addWidget(loopClosure_bundle, 6, 0, 1, 1);

        label_346 = new QLabel(groupBox_visualTransform2);
        label_346->setObjectName(QStringLiteral("label_346"));
        label_346->setWordWrap(true);
        label_346->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_23->addWidget(label_346, 6, 1, 1, 1);

        label_554 = new QLabel(groupBox_visualTransform2);
        label_554->setObjectName(QStringLiteral("label_554"));
        label_554->setWordWrap(true);
        label_554->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_23->addWidget(label_554, 2, 1, 1, 1);

        label_555 = new QLabel(groupBox_visualTransform2);
        label_555->setObjectName(QStringLiteral("label_555"));
        label_555->setWordWrap(true);
        label_555->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_23->addWidget(label_555, 3, 1, 1, 1);

        visMinDistribution = new QDoubleSpinBox(groupBox_visualTransform2);
        visMinDistribution->setObjectName(QStringLiteral("visMinDistribution"));
        visMinDistribution->setDecimals(4);
        visMinDistribution->setMaximum(0.5);
        visMinDistribution->setSingleStep(0.01);

        gridLayout_23->addWidget(visMinDistribution, 3, 0, 1, 1);

        visMeanDistance = new QDoubleSpinBox(groupBox_visualTransform2);
        visMeanDistance->setObjectName(QStringLiteral("visMeanDistance"));
        visMeanDistance->setMaximum(9999);

        gridLayout_23->addWidget(visMeanDistance, 2, 0, 1, 1);

        gridLayout_23->setColumnStretch(1, 1);

        verticalLayout_41->addLayout(gridLayout_23);

        groupBox_4 = new QGroupBox(groupBox_visualTransform2);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        verticalLayout_150 = new QVBoxLayout(groupBox_4);
        verticalLayout_150->setObjectName(QStringLiteral("verticalLayout_150"));
        gridLayout_30 = new QGridLayout();
        gridLayout_30->setObjectName(QStringLiteral("gridLayout_30"));
        reextract_nn = new QComboBox(groupBox_4);
        reextract_nn->setObjectName(QStringLiteral("reextract_nn"));
        reextract_nn->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_30->addWidget(reextract_nn, 0, 0, 1, 1);

        label_179 = new QLabel(groupBox_4);
        label_179->setObjectName(QStringLiteral("label_179"));
        label_179->setWordWrap(true);
        label_179->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_30->addWidget(label_179, 0, 1, 1, 1);

        reextract_nndrRatio = new QDoubleSpinBox(groupBox_4);
        reextract_nndrRatio->setObjectName(QStringLiteral("reextract_nndrRatio"));
        reextract_nndrRatio->setDecimals(2);
        reextract_nndrRatio->setMinimum(0.01);
        reextract_nndrRatio->setMaximum(1);
        reextract_nndrRatio->setSingleStep(0.1);
        reextract_nndrRatio->setValue(0.7);

        gridLayout_30->addWidget(reextract_nndrRatio, 1, 0, 1, 1);

        label_181 = new QLabel(groupBox_4);
        label_181->setObjectName(QStringLiteral("label_181"));
        label_181->setWordWrap(true);
        label_181->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_30->addWidget(label_181, 1, 1, 1, 1);

        spinBox_visCorGuessWinSize = new QSpinBox(groupBox_4);
        spinBox_visCorGuessWinSize->setObjectName(QStringLiteral("spinBox_visCorGuessWinSize"));
        spinBox_visCorGuessWinSize->setMinimum(0);
        spinBox_visCorGuessWinSize->setMaximum(1000);
        spinBox_visCorGuessWinSize->setValue(16);

        gridLayout_30->addWidget(spinBox_visCorGuessWinSize, 2, 0, 1, 1);

        label_303 = new QLabel(groupBox_4);
        label_303->setObjectName(QStringLiteral("label_303"));
        label_303->setWordWrap(true);
        label_303->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_30->addWidget(label_303, 2, 1, 1, 1);

        checkBox__visCorGuessMatchToProjection = new QCheckBox(groupBox_4);
        checkBox__visCorGuessMatchToProjection->setObjectName(QStringLiteral("checkBox__visCorGuessMatchToProjection"));

        gridLayout_30->addWidget(checkBox__visCorGuessMatchToProjection, 3, 0, 1, 1);

        label_448 = new QLabel(groupBox_4);
        label_448->setObjectName(QStringLiteral("label_448"));
        label_448->setWordWrap(true);
        label_448->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_30->addWidget(label_448, 3, 1, 1, 1);

        gridLayout_30->setColumnStretch(1, 1);

        verticalLayout_150->addLayout(gridLayout_30);

        verticalSpacer_44 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_150->addItem(verticalSpacer_44);

        groupBox_pymatcher = new QGroupBox(groupBox_4);
        groupBox_pymatcher->setObjectName(QStringLiteral("groupBox_pymatcher"));
        verticalLayout_149 = new QVBoxLayout(groupBox_pymatcher);
        verticalLayout_149->setObjectName(QStringLiteral("verticalLayout_149"));
        label_586 = new QLabel(groupBox_pymatcher);
        label_586->setObjectName(QStringLiteral("label_586"));
        label_586->setWordWrap(true);
        label_586->setOpenExternalLinks(true);
        label_586->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_149->addWidget(label_586);

        gridLayout_116 = new QGridLayout();
        gridLayout_116->setObjectName(QStringLiteral("gridLayout_116"));
        toolButton_pymatcher_path = new QToolButton(groupBox_pymatcher);
        toolButton_pymatcher_path->setObjectName(QStringLiteral("toolButton_pymatcher_path"));

        gridLayout_116->addWidget(toolButton_pymatcher_path, 0, 0, 1, 1);

        lineEdit_pymatcher_path = new QLineEdit(groupBox_pymatcher);
        lineEdit_pymatcher_path->setObjectName(QStringLiteral("lineEdit_pymatcher_path"));

        gridLayout_116->addWidget(lineEdit_pymatcher_path, 0, 1, 1, 1);

        label_585 = new QLabel(groupBox_pymatcher);
        label_585->setObjectName(QStringLiteral("label_585"));
        label_585->setWordWrap(true);
        label_585->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_116->addWidget(label_585, 2, 2, 1, 1);

        pymatcher_matchThreshold = new QDoubleSpinBox(groupBox_pymatcher);
        pymatcher_matchThreshold->setObjectName(QStringLiteral("pymatcher_matchThreshold"));
        pymatcher_matchThreshold->setDecimals(3);
        pymatcher_matchThreshold->setMinimum(0.001);
        pymatcher_matchThreshold->setMaximum(1);
        pymatcher_matchThreshold->setSingleStep(0.01);
        pymatcher_matchThreshold->setValue(0.2);

        gridLayout_116->addWidget(pymatcher_matchThreshold, 2, 1, 1, 1);

        label_584 = new QLabel(groupBox_pymatcher);
        label_584->setObjectName(QStringLiteral("label_584"));
        label_584->setWordWrap(true);
        label_584->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_116->addWidget(label_584, 1, 2, 1, 1);

        label_583 = new QLabel(groupBox_pymatcher);
        label_583->setObjectName(QStringLiteral("label_583"));
        label_583->setWordWrap(true);
        label_583->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_116->addWidget(label_583, 0, 2, 1, 1);

        pymatcher_iterations = new QSpinBox(groupBox_pymatcher);
        pymatcher_iterations->setObjectName(QStringLiteral("pymatcher_iterations"));
        pymatcher_iterations->setMinimum(0);
        pymatcher_iterations->setMaximum(999999);
        pymatcher_iterations->setSingleStep(1);
        pymatcher_iterations->setValue(20);

        gridLayout_116->addWidget(pymatcher_iterations, 1, 1, 1, 1);

        label_587 = new QLabel(groupBox_pymatcher);
        label_587->setObjectName(QStringLiteral("label_587"));
        label_587->setWordWrap(true);
        label_587->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_116->addWidget(label_587, 3, 2, 1, 1);

        checkBox_pymatcher_cuda = new QCheckBox(groupBox_pymatcher);
        checkBox_pymatcher_cuda->setObjectName(QStringLiteral("checkBox_pymatcher_cuda"));

        gridLayout_116->addWidget(checkBox_pymatcher_cuda, 3, 1, 1, 1);

        label_590 = new QLabel(groupBox_pymatcher);
        label_590->setObjectName(QStringLiteral("label_590"));
        label_590->setWordWrap(true);
        label_590->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_116->addWidget(label_590, 4, 2, 1, 1);

        lineEdit_pymatcher_model = new QLineEdit(groupBox_pymatcher);
        lineEdit_pymatcher_model->setObjectName(QStringLiteral("lineEdit_pymatcher_model"));

        gridLayout_116->addWidget(lineEdit_pymatcher_model, 4, 1, 1, 1);

        toolButton_pymatcher_model = new QToolButton(groupBox_pymatcher);
        toolButton_pymatcher_model->setObjectName(QStringLiteral("toolButton_pymatcher_model"));

        gridLayout_116->addWidget(toolButton_pymatcher_model, 4, 0, 1, 1);

        gridLayout_116->setColumnStretch(2, 1);

        verticalLayout_149->addLayout(gridLayout_116);

        verticalSpacer_86 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_149->addItem(verticalSpacer_86);


        verticalLayout_150->addWidget(groupBox_pymatcher);

        groupBox_gms = new QGroupBox(groupBox_4);
        groupBox_gms->setObjectName(QStringLiteral("groupBox_gms"));
        verticalLayout_151 = new QVBoxLayout(groupBox_gms);
        verticalLayout_151->setObjectName(QStringLiteral("verticalLayout_151"));
        label_588 = new QLabel(groupBox_gms);
        label_588->setObjectName(QStringLiteral("label_588"));
        label_588->setWordWrap(true);
        label_588->setOpenExternalLinks(true);
        label_588->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_151->addWidget(label_588);

        gridLayout_117 = new QGridLayout();
        gridLayout_117->setObjectName(QStringLiteral("gridLayout_117"));
        label_592 = new QLabel(groupBox_gms);
        label_592->setObjectName(QStringLiteral("label_592"));
        label_592->setWordWrap(true);
        label_592->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_117->addWidget(label_592, 0, 1, 1, 1);

        checkBox_gms_withRotation = new QCheckBox(groupBox_gms);
        checkBox_gms_withRotation->setObjectName(QStringLiteral("checkBox_gms_withRotation"));

        gridLayout_117->addWidget(checkBox_gms_withRotation, 0, 0, 1, 1);

        label_589 = new QLabel(groupBox_gms);
        label_589->setObjectName(QStringLiteral("label_589"));
        label_589->setWordWrap(true);
        label_589->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_117->addWidget(label_589, 2, 1, 1, 1);

        gms_thresholdFactor = new QDoubleSpinBox(groupBox_gms);
        gms_thresholdFactor->setObjectName(QStringLiteral("gms_thresholdFactor"));
        gms_thresholdFactor->setDecimals(1);
        gms_thresholdFactor->setMinimum(0);
        gms_thresholdFactor->setMaximum(1000);
        gms_thresholdFactor->setSingleStep(0.01);
        gms_thresholdFactor->setValue(0.2);

        gridLayout_117->addWidget(gms_thresholdFactor, 2, 0, 1, 1);

        label_593 = new QLabel(groupBox_gms);
        label_593->setObjectName(QStringLiteral("label_593"));
        label_593->setWordWrap(true);
        label_593->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_117->addWidget(label_593, 1, 1, 1, 1);

        checkBox_gms_withScale = new QCheckBox(groupBox_gms);
        checkBox_gms_withScale->setObjectName(QStringLiteral("checkBox_gms_withScale"));

        gridLayout_117->addWidget(checkBox_gms_withScale, 1, 0, 1, 1);

        gridLayout_117->setColumnStretch(1, 1);

        verticalLayout_151->addLayout(gridLayout_117);

        verticalSpacer_87 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_151->addItem(verticalSpacer_87);


        verticalLayout_150->addWidget(groupBox_gms);


        verticalLayout_41->addWidget(groupBox_4);

        stackedWidget_loopClosureEstimation = new QStackedWidget(groupBox_visualTransform2);
        stackedWidget_loopClosureEstimation->setObjectName(QStringLiteral("stackedWidget_loopClosureEstimation"));
        page_38 = new QWidget();
        page_38->setObjectName(QStringLiteral("page_38"));
        verticalLayout_68 = new QVBoxLayout(page_38);
        verticalLayout_68->setSpacing(0);
        verticalLayout_68->setObjectName(QStringLiteral("verticalLayout_68"));
        verticalLayout_68->setContentsMargins(0, 0, 0, 0);
        groupBox_3 = new QGroupBox(page_38);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        gridLayout_6 = new QGridLayout(groupBox_3);
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        loopClosure_bowInlierDistance = new QDoubleSpinBox(groupBox_3);
        loopClosure_bowInlierDistance->setObjectName(QStringLiteral("loopClosure_bowInlierDistance"));
        loopClosure_bowInlierDistance->setDecimals(3);
        loopClosure_bowInlierDistance->setMinimum(0.001);
        loopClosure_bowInlierDistance->setSingleStep(0.01);
        loopClosure_bowInlierDistance->setValue(0.02);

        gridLayout_6->addWidget(loopClosure_bowInlierDistance, 0, 0, 1, 1);

        label_14 = new QLabel(groupBox_3);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setWordWrap(true);
        label_14->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_6->addWidget(label_14, 0, 1, 1, 1);

        loopClosure_bowRefineIterations = new QSpinBox(groupBox_3);
        loopClosure_bowRefineIterations->setObjectName(QStringLiteral("loopClosure_bowRefineIterations"));
        loopClosure_bowRefineIterations->setMinimum(0);
        loopClosure_bowRefineIterations->setMaximum(10000);
        loopClosure_bowRefineIterations->setSingleStep(1);
        loopClosure_bowRefineIterations->setValue(10);

        gridLayout_6->addWidget(loopClosure_bowRefineIterations, 1, 0, 1, 1);

        label_38 = new QLabel(groupBox_3);
        label_38->setObjectName(QStringLiteral("label_38"));
        label_38->setWordWrap(true);
        label_38->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_6->addWidget(label_38, 1, 1, 1, 1);

        gridLayout_6->setColumnStretch(1, 1);

        verticalLayout_68->addWidget(groupBox_3);

        stackedWidget_loopClosureEstimation->addWidget(page_38);
        page_39 = new QWidget();
        page_39->setObjectName(QStringLiteral("page_39"));
        verticalLayout_69 = new QVBoxLayout(page_39);
        verticalLayout_69->setSpacing(0);
        verticalLayout_69->setObjectName(QStringLiteral("verticalLayout_69"));
        verticalLayout_69->setContentsMargins(0, 0, 0, 0);
        groupBox_13 = new QGroupBox(page_39);
        groupBox_13->setObjectName(QStringLiteral("groupBox_13"));
        gridLayout_59 = new QGridLayout(groupBox_13);
        gridLayout_59->setObjectName(QStringLiteral("gridLayout_59"));
        loopClosure_pnpReprojError = new QDoubleSpinBox(groupBox_13);
        loopClosure_pnpReprojError->setObjectName(QStringLiteral("loopClosure_pnpReprojError"));
        loopClosure_pnpReprojError->setDecimals(1);
        loopClosure_pnpReprojError->setMinimum(0.1);
        loopClosure_pnpReprojError->setSingleStep(1);
        loopClosure_pnpReprojError->setValue(8);

        gridLayout_59->addWidget(loopClosure_pnpReprojError, 0, 0, 1, 1);

        label_236 = new QLabel(groupBox_13);
        label_236->setObjectName(QStringLiteral("label_236"));
        label_236->setWordWrap(true);
        label_236->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_59->addWidget(label_236, 0, 1, 1, 1);

        loopClosure_pnpFlags = new QComboBox(groupBox_13);
        loopClosure_pnpFlags->setObjectName(QStringLiteral("loopClosure_pnpFlags"));

        gridLayout_59->addWidget(loopClosure_pnpFlags, 1, 0, 1, 1);

        label_235 = new QLabel(groupBox_13);
        label_235->setObjectName(QStringLiteral("label_235"));
        label_235->setWordWrap(true);
        label_235->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_59->addWidget(label_235, 1, 1, 1, 1);

        label_loopClosure_pnpOpenCV2 = new QLabel(groupBox_13);
        label_loopClosure_pnpOpenCV2->setObjectName(QStringLiteral("label_loopClosure_pnpOpenCV2"));
        label_loopClosure_pnpOpenCV2->setWordWrap(true);
        label_loopClosure_pnpOpenCV2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_59->addWidget(label_loopClosure_pnpOpenCV2, 2, 1, 1, 1);

        loopClosure_pnpRefineIterations = new QSpinBox(groupBox_13);
        loopClosure_pnpRefineIterations->setObjectName(QStringLiteral("loopClosure_pnpRefineIterations"));
        loopClosure_pnpRefineIterations->setMinimum(0);
        loopClosure_pnpRefineIterations->setMaximum(999999);
        loopClosure_pnpRefineIterations->setSingleStep(1);
        loopClosure_pnpRefineIterations->setValue(1);

        gridLayout_59->addWidget(loopClosure_pnpRefineIterations, 2, 0, 1, 1);

        gridLayout_59->setColumnStretch(1, 1);

        verticalLayout_69->addWidget(groupBox_13);

        stackedWidget_loopClosureEstimation->addWidget(page_39);
        page_40 = new QWidget();
        page_40->setObjectName(QStringLiteral("page_40"));
        verticalLayout_72 = new QVBoxLayout(page_40);
        verticalLayout_72->setSpacing(0);
        verticalLayout_72->setObjectName(QStringLiteral("verticalLayout_72"));
        verticalLayout_72->setContentsMargins(0, 0, 0, 0);
        groupBox_14 = new QGroupBox(page_40);
        groupBox_14->setObjectName(QStringLiteral("groupBox_14"));
        verticalLayout_73 = new QVBoxLayout(groupBox_14);
        verticalLayout_73->setObjectName(QStringLiteral("verticalLayout_73"));
        label_671 = new QLabel(groupBox_14);
        label_671->setObjectName(QStringLiteral("label_671"));
        label_671->setWordWrap(true);
        label_671->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_73->addWidget(label_671);

        gridLayout_60 = new QGridLayout();
        gridLayout_60->setObjectName(QStringLiteral("gridLayout_60"));
        loopClosure_bowEpipolarGeometryVar = new QDoubleSpinBox(groupBox_14);
        loopClosure_bowEpipolarGeometryVar->setObjectName(QStringLiteral("loopClosure_bowEpipolarGeometryVar"));
        loopClosure_bowEpipolarGeometryVar->setDecimals(3);
        loopClosure_bowEpipolarGeometryVar->setMinimum(0);
        loopClosure_bowEpipolarGeometryVar->setSingleStep(0.001);
        loopClosure_bowEpipolarGeometryVar->setValue(0.02);

        gridLayout_60->addWidget(loopClosure_bowEpipolarGeometryVar, 0, 0, 1, 1);

        label_664 = new QLabel(groupBox_14);
        label_664->setObjectName(QStringLiteral("label_664"));
        label_664->setWordWrap(true);
        label_664->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_60->addWidget(label_664, 0, 1, 1, 1);

        gridLayout_60->setColumnStretch(1, 1);

        verticalLayout_73->addLayout(gridLayout_60);

        verticalSpacer_52 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_73->addItem(verticalSpacer_52);


        verticalLayout_72->addWidget(groupBox_14);

        stackedWidget_loopClosureEstimation->addWidget(page_40);

        verticalLayout_41->addWidget(stackedWidget_loopClosureEstimation);

        groupBox_VisExtract = new QGroupBox(groupBox_visualTransform2);
        groupBox_VisExtract->setObjectName(QStringLiteral("groupBox_VisExtract"));
        groupBox_VisExtract->setCheckable(false);
        groupBox_VisExtract->setChecked(false);
        verticalLayout_46 = new QVBoxLayout(groupBox_VisExtract);
        verticalLayout_46->setObjectName(QStringLiteral("verticalLayout_46"));
        label_182 = new QLabel(groupBox_VisExtract);
        label_182->setObjectName(QStringLiteral("label_182"));
        label_182->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_46->addWidget(label_182);

        gridLayout_25 = new QGridLayout();
        gridLayout_25->setObjectName(QStringLiteral("gridLayout_25"));
        label_237 = new QLabel(groupBox_VisExtract);
        label_237->setObjectName(QStringLiteral("label_237"));
        label_237->setWordWrap(true);
        label_237->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_25->addWidget(label_237, 3, 1, 1, 1);

        label_261 = new QLabel(groupBox_VisExtract);
        label_261->setObjectName(QStringLiteral("label_261"));
        label_261->setWordWrap(true);
        label_261->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_25->addWidget(label_261, 6, 1, 1, 1);

        loopClosure_roi = new QLineEdit(groupBox_VisExtract);
        loopClosure_roi->setObjectName(QStringLiteral("loopClosure_roi"));

        gridLayout_25->addWidget(loopClosure_roi, 6, 0, 1, 1);

        reextract_maxFeatures = new QSpinBox(groupBox_VisExtract);
        reextract_maxFeatures->setObjectName(QStringLiteral("reextract_maxFeatures"));
        reextract_maxFeatures->setMaximum(999999);

        gridLayout_25->addWidget(reextract_maxFeatures, 2, 0, 1, 1);

        label_180 = new QLabel(groupBox_VisExtract);
        label_180->setObjectName(QStringLiteral("label_180"));
        label_180->setWordWrap(true);
        label_180->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_25->addWidget(label_180, 2, 1, 1, 1);

        label_275 = new QLabel(groupBox_VisExtract);
        label_275->setObjectName(QStringLiteral("label_275"));
        label_275->setWordWrap(true);
        label_275->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_25->addWidget(label_275, 4, 1, 1, 1);

        vis_feature_detector = new QComboBox(groupBox_VisExtract);
        vis_feature_detector->setObjectName(QStringLiteral("vis_feature_detector"));
        vis_feature_detector->setSizeAdjustPolicy(QComboBox::AdjustToContents);

        gridLayout_25->addWidget(vis_feature_detector, 1, 0, 1, 1);

        loopClosure_bowMaxDepth = new QDoubleSpinBox(groupBox_VisExtract);
        loopClosure_bowMaxDepth->setObjectName(QStringLiteral("loopClosure_bowMaxDepth"));
        loopClosure_bowMaxDepth->setMinimum(0);
        loopClosure_bowMaxDepth->setValue(5);

        gridLayout_25->addWidget(loopClosure_bowMaxDepth, 3, 0, 1, 1);

        label_178 = new QLabel(groupBox_VisExtract);
        label_178->setObjectName(QStringLiteral("label_178"));
        label_178->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_25->addWidget(label_178, 1, 1, 1, 1);

        loopClosure_bowMinDepth = new QDoubleSpinBox(groupBox_VisExtract);
        loopClosure_bowMinDepth->setObjectName(QStringLiteral("loopClosure_bowMinDepth"));
        loopClosure_bowMinDepth->setMinimum(0);
        loopClosure_bowMinDepth->setValue(5);

        gridLayout_25->addWidget(loopClosure_bowMinDepth, 4, 0, 1, 1);

        reextract_gridrows = new QSpinBox(groupBox_VisExtract);
        reextract_gridrows->setObjectName(QStringLiteral("reextract_gridrows"));
        reextract_gridrows->setMinimum(1);
        reextract_gridrows->setMaximum(99);
        reextract_gridrows->setValue(1);

        gridLayout_25->addWidget(reextract_gridrows, 7, 0, 1, 1);

        label_438 = new QLabel(groupBox_VisExtract);
        label_438->setObjectName(QStringLiteral("label_438"));
        label_438->setWordWrap(true);
        label_438->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_25->addWidget(label_438, 7, 1, 1, 1);

        label_439 = new QLabel(groupBox_VisExtract);
        label_439->setObjectName(QStringLiteral("label_439"));
        label_439->setWordWrap(true);
        label_439->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_25->addWidget(label_439, 8, 1, 1, 1);

        reextract_gridcols = new QSpinBox(groupBox_VisExtract);
        reextract_gridcols->setObjectName(QStringLiteral("reextract_gridcols"));
        reextract_gridcols->setMinimum(1);
        reextract_gridcols->setMaximum(99);
        reextract_gridcols->setValue(1);

        gridLayout_25->addWidget(reextract_gridcols, 8, 0, 1, 1);

        label_450 = new QLabel(groupBox_VisExtract);
        label_450->setObjectName(QStringLiteral("label_450"));
        label_450->setWordWrap(true);
        label_450->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_25->addWidget(label_450, 5, 1, 1, 1);

        checkBox_visDepthAsMask = new QCheckBox(groupBox_VisExtract);
        checkBox_visDepthAsMask->setObjectName(QStringLiteral("checkBox_visDepthAsMask"));

        gridLayout_25->addWidget(checkBox_visDepthAsMask, 5, 0, 1, 1);

        gridLayout_25->setColumnStretch(1, 1);

        verticalLayout_46->addLayout(gridLayout_25);

        groupBox_11 = new QGroupBox(groupBox_VisExtract);
        groupBox_11->setObjectName(QStringLiteral("groupBox_11"));
        verticalLayout_59 = new QVBoxLayout(groupBox_11);
        verticalLayout_59->setObjectName(QStringLiteral("verticalLayout_59"));
        label_35 = new QLabel(groupBox_11);
        label_35->setObjectName(QStringLiteral("label_35"));
        label_35->setWordWrap(true);
        label_35->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_59->addWidget(label_35);

        gridLayout_36 = new QGridLayout();
        gridLayout_36->setObjectName(QStringLiteral("gridLayout_36"));
        subpix_winSize = new QSpinBox(groupBox_11);
        subpix_winSize->setObjectName(QStringLiteral("subpix_winSize"));
        subpix_winSize->setMinimum(0);
        subpix_winSize->setMaximum(999999);
        subpix_winSize->setSingleStep(1);
        subpix_winSize->setValue(5);

        gridLayout_36->addWidget(subpix_winSize, 0, 0, 1, 1);

        label_200 = new QLabel(groupBox_11);
        label_200->setObjectName(QStringLiteral("label_200"));
        label_200->setWordWrap(true);
        label_200->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_36->addWidget(label_200, 0, 1, 1, 1);

        subpix_iterations = new QSpinBox(groupBox_11);
        subpix_iterations->setObjectName(QStringLiteral("subpix_iterations"));
        subpix_iterations->setMinimum(0);
        subpix_iterations->setMaximum(999999);
        subpix_iterations->setSingleStep(1);
        subpix_iterations->setValue(20);

        gridLayout_36->addWidget(subpix_iterations, 1, 0, 1, 1);

        label_203 = new QLabel(groupBox_11);
        label_203->setObjectName(QStringLiteral("label_203"));
        label_203->setWordWrap(true);
        label_203->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_36->addWidget(label_203, 1, 1, 1, 1);

        subpix_eps = new QDoubleSpinBox(groupBox_11);
        subpix_eps->setObjectName(QStringLiteral("subpix_eps"));
        subpix_eps->setDecimals(3);
        subpix_eps->setMinimum(0.001);
        subpix_eps->setMaximum(0.1);
        subpix_eps->setSingleStep(0.01);
        subpix_eps->setValue(0.03);

        gridLayout_36->addWidget(subpix_eps, 2, 0, 1, 1);

        label_204 = new QLabel(groupBox_11);
        label_204->setObjectName(QStringLiteral("label_204"));
        label_204->setWordWrap(true);
        label_204->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_36->addWidget(label_204, 2, 1, 1, 1);

        gridLayout_36->setColumnStretch(1, 1);

        verticalLayout_59->addLayout(gridLayout_36);

        verticalSpacer_37 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_59->addItem(verticalSpacer_37);


        verticalLayout_46->addWidget(groupBox_11);

        verticalLayout_46->setStretch(2, 1);

        verticalLayout_41->addWidget(groupBox_VisExtract);

        verticalSpacer_66 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_41->addItem(verticalSpacer_66);

        verticalLayout_41->setStretch(3, 1);

        verticalLayout_50->addWidget(groupBox_visualTransform2);

        verticalSpacer_20 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_50->addItem(verticalSpacer_20);

        verticalLayout_50->setStretch(1, 1);
        stackedWidget->addWidget(page_21);
        page_27 = new QWidget();
        page_27->setObjectName(QStringLiteral("page_27"));
        verticalLayout_52 = new QVBoxLayout(page_27);
        verticalLayout_52->setObjectName(QStringLiteral("verticalLayout_52"));
        groupBox_icp2 = new QGroupBox(page_27);
        groupBox_icp2->setObjectName(QStringLiteral("groupBox_icp2"));
        verticalLayout_20 = new QVBoxLayout(groupBox_icp2);
        verticalLayout_20->setObjectName(QStringLiteral("verticalLayout_20"));
        label_7 = new QLabel(groupBox_icp2);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_20->addWidget(label_7);

        gridLayout_48 = new QGridLayout();
        gridLayout_48->setObjectName(QStringLiteral("gridLayout_48"));
        label_139 = new QLabel(groupBox_icp2);
        label_139->setObjectName(QStringLiteral("label_139"));
        label_139->setWordWrap(true);
        label_139->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_139, 3, 1, 1, 1);

        loopClosure_icpRangeMax = new QDoubleSpinBox(groupBox_icp2);
        loopClosure_icpRangeMax->setObjectName(QStringLiteral("loopClosure_icpRangeMax"));
        loopClosure_icpRangeMax->setDecimals(2);
        loopClosure_icpRangeMax->setMinimum(0);
        loopClosure_icpRangeMax->setMaximum(999);
        loopClosure_icpRangeMax->setSingleStep(1);
        loopClosure_icpRangeMax->setValue(0);

        gridLayout_48->addWidget(loopClosure_icpRangeMax, 6, 0, 1, 1);

        loopClosure_icpDownsamplingStep = new QSpinBox(groupBox_icp2);
        loopClosure_icpDownsamplingStep->setObjectName(QStringLiteral("loopClosure_icpDownsamplingStep"));
        loopClosure_icpDownsamplingStep->setMinimum(1);
        loopClosure_icpDownsamplingStep->setMaximum(999999);
        loopClosure_icpDownsamplingStep->setSingleStep(1);
        loopClosure_icpDownsamplingStep->setValue(1);

        gridLayout_48->addWidget(loopClosure_icpDownsamplingStep, 3, 0, 1, 1);

        label_55 = new QLabel(groupBox_icp2);
        label_55->setObjectName(QStringLiteral("label_55"));
        label_55->setWordWrap(true);
        label_55->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_55, 2, 1, 1, 1);

        loopClosure_icpRatio = new QDoubleSpinBox(groupBox_icp2);
        loopClosure_icpRatio->setObjectName(QStringLiteral("loopClosure_icpRatio"));
        loopClosure_icpRatio->setMinimum(0);
        loopClosure_icpRatio->setMaximum(1);
        loopClosure_icpRatio->setSingleStep(0.01);
        loopClosure_icpRatio->setValue(0.7);

        gridLayout_48->addWidget(loopClosure_icpRatio, 10, 0, 1, 1);

        label_136 = new QLabel(groupBox_icp2);
        label_136->setObjectName(QStringLiteral("label_136"));
        label_136->setWordWrap(true);
        label_136->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_136, 9, 1, 1, 1);

        loopClosure_icpIterations = new QSpinBox(groupBox_icp2);
        loopClosure_icpIterations->setObjectName(QStringLiteral("loopClosure_icpIterations"));
        loopClosure_icpIterations->setMinimum(1);
        loopClosure_icpIterations->setMaximum(1000);
        loopClosure_icpIterations->setValue(10);

        gridLayout_48->addWidget(loopClosure_icpIterations, 8, 0, 1, 1);

        loopClosure_icpRangeMin = new QDoubleSpinBox(groupBox_icp2);
        loopClosure_icpRangeMin->setObjectName(QStringLiteral("loopClosure_icpRangeMin"));
        loopClosure_icpRangeMin->setDecimals(2);
        loopClosure_icpRangeMin->setMinimum(0);
        loopClosure_icpRangeMin->setSingleStep(1);
        loopClosure_icpRangeMin->setValue(0);

        gridLayout_48->addWidget(loopClosure_icpRangeMin, 5, 0, 1, 1);

        comboBox_icpStrategy = new QComboBox(groupBox_icp2);
        comboBox_icpStrategy->setObjectName(QStringLiteral("comboBox_icpStrategy"));

        gridLayout_48->addWidget(comboBox_icpStrategy, 0, 0, 1, 1);

        loopClosure_icpMaxCorrespondenceDistance = new QDoubleSpinBox(groupBox_icp2);
        loopClosure_icpMaxCorrespondenceDistance->setObjectName(QStringLiteral("loopClosure_icpMaxCorrespondenceDistance"));
        loopClosure_icpMaxCorrespondenceDistance->setDecimals(3);
        loopClosure_icpMaxCorrespondenceDistance->setMinimum(0.001);
        loopClosure_icpMaxCorrespondenceDistance->setMaximum(9999.99);
        loopClosure_icpMaxCorrespondenceDistance->setSingleStep(0.01);
        loopClosure_icpMaxCorrespondenceDistance->setValue(0.1);

        gridLayout_48->addWidget(loopClosure_icpMaxCorrespondenceDistance, 7, 0, 1, 1);

        loopClosure_icpPointToPlane = new QCheckBox(groupBox_icp2);
        loopClosure_icpPointToPlane->setObjectName(QStringLiteral("loopClosure_icpPointToPlane"));

        gridLayout_48->addWidget(loopClosure_icpPointToPlane, 13, 0, 1, 1);

        loopClosure_icpForce4DoF = new QCheckBox(groupBox_icp2);
        loopClosure_icpForce4DoF->setObjectName(QStringLiteral("loopClosure_icpForce4DoF"));

        gridLayout_48->addWidget(loopClosure_icpForce4DoF, 11, 0, 1, 1);

        loopClosure_icpPointToPlaneLowComplexityStrategy = new QComboBox(groupBox_icp2);
        loopClosure_icpPointToPlaneLowComplexityStrategy->setObjectName(QStringLiteral("loopClosure_icpPointToPlaneLowComplexityStrategy"));

        gridLayout_48->addWidget(loopClosure_icpPointToPlaneLowComplexityStrategy, 19, 0, 1, 1);

        label_51 = new QLabel(groupBox_icp2);
        label_51->setObjectName(QStringLiteral("label_51"));
        label_51->setWordWrap(true);
        label_51->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_51, 1, 1, 1, 1);

        label_559 = new QLabel(groupBox_icp2);
        label_559->setObjectName(QStringLiteral("label_559"));
        label_559->setWordWrap(true);
        label_559->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_559, 5, 1, 1, 1);

        loopClosure_icpVoxelSize = new QDoubleSpinBox(groupBox_icp2);
        loopClosure_icpVoxelSize->setObjectName(QStringLiteral("loopClosure_icpVoxelSize"));
        loopClosure_icpVoxelSize->setDecimals(3);
        loopClosure_icpVoxelSize->setMinimum(0);
        loopClosure_icpVoxelSize->setSingleStep(0.01);
        loopClosure_icpVoxelSize->setValue(0.005);

        gridLayout_48->addWidget(loopClosure_icpVoxelSize, 4, 0, 1, 1);

        label_426 = new QLabel(groupBox_icp2);
        label_426->setObjectName(QStringLiteral("label_426"));
        label_426->setWordWrap(true);
        label_426->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_426, 16, 1, 1, 1);

        label_133 = new QLabel(groupBox_icp2);
        label_133->setObjectName(QStringLiteral("label_133"));
        label_133->setWordWrap(true);
        label_133->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_133, 10, 1, 1, 1);

        loopClosure_icpPointToPlaneNormalsRadius = new QDoubleSpinBox(groupBox_icp2);
        loopClosure_icpPointToPlaneNormalsRadius->setObjectName(QStringLiteral("loopClosure_icpPointToPlaneNormalsRadius"));
        loopClosure_icpPointToPlaneNormalsRadius->setSingleStep(0.01);

        gridLayout_48->addWidget(loopClosure_icpPointToPlaneNormalsRadius, 16, 0, 1, 1);

        label_124 = new QLabel(groupBox_icp2);
        label_124->setObjectName(QStringLiteral("label_124"));
        label_124->setWordWrap(true);
        label_124->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_124, 8, 1, 1, 1);

        label_212 = new QLabel(groupBox_icp2);
        label_212->setObjectName(QStringLiteral("label_212"));
        label_212->setWordWrap(true);
        label_212->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_212, 14, 1, 1, 1);

        label_622 = new QLabel(groupBox_icp2);
        label_622->setObjectName(QStringLiteral("label_622"));
        label_622->setWordWrap(true);
        label_622->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_622, 11, 1, 1, 1);

        label_122 = new QLabel(groupBox_icp2);
        label_122->setObjectName(QStringLiteral("label_122"));
        label_122->setWordWrap(true);
        label_122->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_122, 7, 1, 1, 1);

        label_144 = new QLabel(groupBox_icp2);
        label_144->setObjectName(QStringLiteral("label_144"));
        label_144->setWordWrap(true);
        label_144->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_144, 13, 1, 1, 1);

        label_608 = new QLabel(groupBox_icp2);
        label_608->setObjectName(QStringLiteral("label_608"));
        label_608->setWordWrap(true);
        label_608->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_608, 17, 1, 1, 1);

        globalDetection_icpMaxRotation = new QDoubleSpinBox(groupBox_icp2);
        globalDetection_icpMaxRotation->setObjectName(QStringLiteral("globalDetection_icpMaxRotation"));
        globalDetection_icpMaxRotation->setDecimals(2);
        globalDetection_icpMaxRotation->setMaximum(3.14);
        globalDetection_icpMaxRotation->setSingleStep(0.01);
        globalDetection_icpMaxRotation->setValue(0.78);

        gridLayout_48->addWidget(globalDetection_icpMaxRotation, 2, 0, 1, 1);

        loopClosure_icpPointToPlaneNormals = new QSpinBox(groupBox_icp2);
        loopClosure_icpPointToPlaneNormals->setObjectName(QStringLiteral("loopClosure_icpPointToPlaneNormals"));
        loopClosure_icpPointToPlaneNormals->setMinimum(1);
        loopClosure_icpPointToPlaneNormals->setMaximum(1000);
        loopClosure_icpPointToPlaneNormals->setValue(20);

        gridLayout_48->addWidget(loopClosure_icpPointToPlaneNormals, 14, 0, 1, 1);

        globalDetection_icpMaxTranslation = new QDoubleSpinBox(groupBox_icp2);
        globalDetection_icpMaxTranslation->setObjectName(QStringLiteral("globalDetection_icpMaxTranslation"));
        globalDetection_icpMaxTranslation->setDecimals(2);
        globalDetection_icpMaxTranslation->setMaximum(10);
        globalDetection_icpMaxTranslation->setSingleStep(0.01);
        globalDetection_icpMaxTranslation->setValue(0.2);

        gridLayout_48->addWidget(globalDetection_icpMaxTranslation, 1, 0, 1, 1);

        loopClosure_icpEpsilon = new QDoubleSpinBox(groupBox_icp2);
        loopClosure_icpEpsilon->setObjectName(QStringLiteral("loopClosure_icpEpsilon"));
        loopClosure_icpEpsilon->setDecimals(6);
        loopClosure_icpEpsilon->setMinimum(0);
        loopClosure_icpEpsilon->setMaximum(1);
        loopClosure_icpEpsilon->setSingleStep(0.01);
        loopClosure_icpEpsilon->setValue(0);

        gridLayout_48->addWidget(loopClosure_icpEpsilon, 9, 0, 1, 1);

        label_617 = new QLabel(groupBox_icp2);
        label_617->setObjectName(QStringLiteral("label_617"));
        label_617->setWordWrap(true);
        label_617->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_617, 0, 1, 1, 1);

        label_125 = new QLabel(groupBox_icp2);
        label_125->setObjectName(QStringLiteral("label_125"));
        label_125->setWordWrap(true);
        label_125->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_125, 4, 1, 1, 1);

        label_607 = new QLabel(groupBox_icp2);
        label_607->setObjectName(QStringLiteral("label_607"));
        label_607->setWordWrap(true);
        label_607->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_607, 19, 1, 1, 1);

        label_560 = new QLabel(groupBox_icp2);
        label_560->setObjectName(QStringLiteral("label_560"));
        label_560->setWordWrap(true);
        label_560->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_560, 6, 1, 1, 1);

        label_429 = new QLabel(groupBox_icp2);
        label_429->setObjectName(QStringLiteral("label_429"));
        label_429->setWordWrap(true);
        label_429->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_429, 18, 1, 1, 1);

        loopClosure_icpPointToPlaneGroundNormalsUp = new QDoubleSpinBox(groupBox_icp2);
        loopClosure_icpPointToPlaneGroundNormalsUp->setObjectName(QStringLiteral("loopClosure_icpPointToPlaneGroundNormalsUp"));
        loopClosure_icpPointToPlaneGroundNormalsUp->setDecimals(2);
        loopClosure_icpPointToPlaneGroundNormalsUp->setMaximum(1);
        loopClosure_icpPointToPlaneGroundNormalsUp->setSingleStep(0.01);

        gridLayout_48->addWidget(loopClosure_icpPointToPlaneGroundNormalsUp, 17, 0, 1, 1);

        loopClosure_icpPointToPlaneNormalsMinComplexity = new QDoubleSpinBox(groupBox_icp2);
        loopClosure_icpPointToPlaneNormalsMinComplexity->setObjectName(QStringLiteral("loopClosure_icpPointToPlaneNormalsMinComplexity"));
        loopClosure_icpPointToPlaneNormalsMinComplexity->setDecimals(3);
        loopClosure_icpPointToPlaneNormalsMinComplexity->setMaximum(1);
        loopClosure_icpPointToPlaneNormalsMinComplexity->setSingleStep(0.01);

        gridLayout_48->addWidget(loopClosure_icpPointToPlaneNormalsMinComplexity, 18, 0, 1, 1);

        label_446 = new QLabel(groupBox_icp2);
        label_446->setObjectName(QStringLiteral("label_446"));
        label_446->setWordWrap(true);
        label_446->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_48->addWidget(label_446, 12, 1, 1, 1);

        doubleSpinBox_icpOutlierRatio = new QDoubleSpinBox(groupBox_icp2);
        doubleSpinBox_icpOutlierRatio->setObjectName(QStringLiteral("doubleSpinBox_icpOutlierRatio"));
        doubleSpinBox_icpOutlierRatio->setDecimals(2);
        doubleSpinBox_icpOutlierRatio->setMinimum(0.01);
        doubleSpinBox_icpOutlierRatio->setMaximum(1);
        doubleSpinBox_icpOutlierRatio->setSingleStep(0.1);
        doubleSpinBox_icpOutlierRatio->setValue(0.85);

        gridLayout_48->addWidget(doubleSpinBox_icpOutlierRatio, 12, 0, 1, 1);

        gridLayout_48->setColumnStretch(1, 1);

        verticalLayout_20->addLayout(gridLayout_48);

        stackedWidget_icpStrategy = new QStackedWidget(groupBox_icp2);
        stackedWidget_icpStrategy->setObjectName(QStringLiteral("stackedWidget_icpStrategy"));
        page_90 = new QWidget();
        page_90->setObjectName(QStringLiteral("page_90"));
        verticalLayout_156 = new QVBoxLayout(page_90);
        verticalLayout_156->setObjectName(QStringLiteral("verticalLayout_156"));
        groupBox_33 = new QGroupBox(page_90);
        groupBox_33->setObjectName(QStringLiteral("groupBox_33"));
        verticalLayout_160 = new QVBoxLayout(groupBox_33);
        verticalLayout_160->setObjectName(QStringLiteral("verticalLayout_160"));
        label_628 = new QLabel(groupBox_33);
        label_628->setObjectName(QStringLiteral("label_628"));
        label_628->setWordWrap(true);
        label_628->setOpenExternalLinks(true);
        label_628->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_160->addWidget(label_628);

        verticalSpacer_82 = new QSpacerItem(20, 380, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_160->addItem(verticalSpacer_82);


        verticalLayout_156->addWidget(groupBox_33);

        stackedWidget_icpStrategy->addWidget(page_90);
        page_91 = new QWidget();
        page_91->setObjectName(QStringLiteral("page_91"));
        verticalLayout_157 = new QVBoxLayout(page_91);
        verticalLayout_157->setObjectName(QStringLiteral("verticalLayout_157"));
        groupBox_libpointmatcher = new QGroupBox(page_91);
        groupBox_libpointmatcher->setObjectName(QStringLiteral("groupBox_libpointmatcher"));
        groupBox_libpointmatcher->setCheckable(false);
        verticalLayout_126 = new QVBoxLayout(groupBox_libpointmatcher);
        verticalLayout_126->setObjectName(QStringLiteral("verticalLayout_126"));
        label_422 = new QLabel(groupBox_libpointmatcher);
        label_422->setObjectName(QStringLiteral("label_422"));
        label_422->setWordWrap(true);
        label_422->setOpenExternalLinks(true);
        label_422->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_126->addWidget(label_422);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setObjectName(QStringLiteral("horizontalLayout_11"));
        toolButton_IcpConfigPath = new QToolButton(groupBox_libpointmatcher);
        toolButton_IcpConfigPath->setObjectName(QStringLiteral("toolButton_IcpConfigPath"));

        horizontalLayout_11->addWidget(toolButton_IcpConfigPath);

        lineEdit_IcpPMConfigPath = new QLineEdit(groupBox_libpointmatcher);
        lineEdit_IcpPMConfigPath->setObjectName(QStringLiteral("lineEdit_IcpPMConfigPath"));

        horizontalLayout_11->addWidget(lineEdit_IcpPMConfigPath);

        label_423 = new QLabel(groupBox_libpointmatcher);
        label_423->setObjectName(QStringLiteral("label_423"));
        label_423->setWordWrap(true);
        label_423->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        horizontalLayout_11->addWidget(label_423);

        horizontalLayout_11->setStretch(2, 1);

        verticalLayout_126->addLayout(horizontalLayout_11);

        gridLayout_97 = new QGridLayout();
        gridLayout_97->setObjectName(QStringLiteral("gridLayout_97"));
        spinBox_icpPMMatcherKnn = new QSpinBox(groupBox_libpointmatcher);
        spinBox_icpPMMatcherKnn->setObjectName(QStringLiteral("spinBox_icpPMMatcherKnn"));
        spinBox_icpPMMatcherKnn->setMinimum(1);

        gridLayout_97->addWidget(spinBox_icpPMMatcherKnn, 1, 0, 1, 1);

        label_424 = new QLabel(groupBox_libpointmatcher);
        label_424->setObjectName(QStringLiteral("label_424"));
        label_424->setWordWrap(true);
        label_424->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_97->addWidget(label_424, 1, 1, 1, 1);

        loopClosure_icpPMMatcherIntensity = new QCheckBox(groupBox_libpointmatcher);
        loopClosure_icpPMMatcherIntensity->setObjectName(QStringLiteral("loopClosure_icpPMMatcherIntensity"));

        gridLayout_97->addWidget(loopClosure_icpPMMatcherIntensity, 3, 0, 1, 1);

        doubleSpinBox_icpPMMatcherEpsilon = new QDoubleSpinBox(groupBox_libpointmatcher);
        doubleSpinBox_icpPMMatcherEpsilon->setObjectName(QStringLiteral("doubleSpinBox_icpPMMatcherEpsilon"));
        doubleSpinBox_icpPMMatcherEpsilon->setDecimals(2);
        doubleSpinBox_icpPMMatcherEpsilon->setMinimum(0);
        doubleSpinBox_icpPMMatcherEpsilon->setMaximum(99);
        doubleSpinBox_icpPMMatcherEpsilon->setSingleStep(1);
        doubleSpinBox_icpPMMatcherEpsilon->setValue(0);

        gridLayout_97->addWidget(doubleSpinBox_icpPMMatcherEpsilon, 2, 0, 1, 1);

        label_447 = new QLabel(groupBox_libpointmatcher);
        label_447->setObjectName(QStringLiteral("label_447"));
        label_447->setWordWrap(true);
        label_447->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_97->addWidget(label_447, 2, 1, 1, 1);

        label_609 = new QLabel(groupBox_libpointmatcher);
        label_609->setObjectName(QStringLiteral("label_609"));
        label_609->setWordWrap(true);
        label_609->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_97->addWidget(label_609, 3, 1, 1, 1);

        gridLayout_97->setColumnStretch(1, 1);

        verticalLayout_126->addLayout(gridLayout_97);


        verticalLayout_157->addWidget(groupBox_libpointmatcher);

        stackedWidget_icpStrategy->addWidget(page_91);
        page_92 = new QWidget();
        page_92->setObjectName(QStringLiteral("page_92"));
        verticalLayout_158 = new QVBoxLayout(page_92);
        verticalLayout_158->setObjectName(QStringLiteral("verticalLayout_158"));
        groupBox_32 = new QGroupBox(page_92);
        groupBox_32->setObjectName(QStringLiteral("groupBox_32"));
        verticalLayout_159 = new QVBoxLayout(groupBox_32);
        verticalLayout_159->setObjectName(QStringLiteral("verticalLayout_159"));
        label_623 = new QLabel(groupBox_32);
        label_623->setObjectName(QStringLiteral("label_623"));
        label_623->setWordWrap(true);
        label_623->setOpenExternalLinks(true);
        label_623->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_159->addWidget(label_623);

        gridLayout_121 = new QGridLayout();
        gridLayout_121->setObjectName(QStringLiteral("gridLayout_121"));
        doubleSpinBox_icpCCMaxFinalRMS = new QDoubleSpinBox(groupBox_32);
        doubleSpinBox_icpCCMaxFinalRMS->setObjectName(QStringLiteral("doubleSpinBox_icpCCMaxFinalRMS"));
        doubleSpinBox_icpCCMaxFinalRMS->setDecimals(6);
        doubleSpinBox_icpCCMaxFinalRMS->setMinimum(1e-6);
        doubleSpinBox_icpCCMaxFinalRMS->setMaximum(1);
        doubleSpinBox_icpCCMaxFinalRMS->setSingleStep(1);
        doubleSpinBox_icpCCMaxFinalRMS->setValue(0.1);

        gridLayout_121->addWidget(doubleSpinBox_icpCCMaxFinalRMS, 3, 0, 1, 1);

        label_630 = new QLabel(groupBox_32);
        label_630->setObjectName(QStringLiteral("label_630"));
        label_630->setWordWrap(true);
        label_630->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_121->addWidget(label_630, 2, 1, 1, 1);

        spinBox_icpCCSamplingLimit = new QSpinBox(groupBox_32);
        spinBox_icpCCSamplingLimit->setObjectName(QStringLiteral("spinBox_icpCCSamplingLimit"));
        spinBox_icpCCSamplingLimit->setMinimum(100);
        spinBox_icpCCSamplingLimit->setMaximum(9999999);
        spinBox_icpCCSamplingLimit->setValue(50000);

        gridLayout_121->addWidget(spinBox_icpCCSamplingLimit, 1, 0, 1, 1);

        label_631 = new QLabel(groupBox_32);
        label_631->setObjectName(QStringLiteral("label_631"));
        label_631->setWordWrap(true);
        label_631->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_121->addWidget(label_631, 3, 1, 1, 1);

        label_629 = new QLabel(groupBox_32);
        label_629->setObjectName(QStringLiteral("label_629"));
        label_629->setWordWrap(true);
        label_629->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_121->addWidget(label_629, 1, 1, 1, 1);

        checkBox_icpCCFilterOutFarthestPoints = new QCheckBox(groupBox_32);
        checkBox_icpCCFilterOutFarthestPoints->setObjectName(QStringLiteral("checkBox_icpCCFilterOutFarthestPoints"));

        gridLayout_121->addWidget(checkBox_icpCCFilterOutFarthestPoints, 2, 0, 1, 1);

        gridLayout_121->setColumnStretch(1, 1);

        verticalLayout_159->addLayout(gridLayout_121);

        verticalSpacer_75 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_159->addItem(verticalSpacer_75);


        verticalLayout_158->addWidget(groupBox_32);

        stackedWidget_icpStrategy->addWidget(page_92);

        verticalLayout_20->addWidget(stackedWidget_icpStrategy);


        verticalLayout_52->addWidget(groupBox_icp2);

        verticalSpacer_21 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_52->addItem(verticalSpacer_21);

        verticalLayout_52->setStretch(1, 1);
        stackedWidget->addWidget(page_27);
        page_65 = new QWidget();
        page_65->setObjectName(QStringLiteral("page_65"));
        verticalLayout_114 = new QVBoxLayout(page_65);
        verticalLayout_114->setObjectName(QStringLiteral("verticalLayout_114"));
        groupBox_optimizer1 = new QGroupBox(page_65);
        groupBox_optimizer1->setObjectName(QStringLiteral("groupBox_optimizer1"));
        verticalLayout_113 = new QVBoxLayout(groupBox_optimizer1);
        verticalLayout_113->setObjectName(QStringLiteral("verticalLayout_113"));
        gridLayout_80 = new QGridLayout();
        gridLayout_80->setObjectName(QStringLiteral("gridLayout_80"));
        graphOptimization_iterations = new QSpinBox(groupBox_optimizer1);
        graphOptimization_iterations->setObjectName(QStringLiteral("graphOptimization_iterations"));
        graphOptimization_iterations->setMinimum(0);
        graphOptimization_iterations->setMaximum(10000);
        graphOptimization_iterations->setValue(100);

        gridLayout_80->addWidget(graphOptimization_iterations, 0, 0, 1, 1);

        graphOptimization_stopEpsilon = new QDoubleSpinBox(groupBox_optimizer1);
        graphOptimization_stopEpsilon->setObjectName(QStringLiteral("graphOptimization_stopEpsilon"));
        graphOptimization_stopEpsilon->setDecimals(5);
        graphOptimization_stopEpsilon->setMinimum(0);
        graphOptimization_stopEpsilon->setMaximum(1);
        graphOptimization_stopEpsilon->setSingleStep(0.001);
        graphOptimization_stopEpsilon->setValue(0.001);

        gridLayout_80->addWidget(graphOptimization_stopEpsilon, 1, 0, 1, 1);

        label_141 = new QLabel(groupBox_optimizer1);
        label_141->setObjectName(QStringLiteral("label_141"));
        label_141->setWordWrap(true);
        label_141->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_80->addWidget(label_141, 2, 1, 1, 1);

        label_64 = new QLabel(groupBox_optimizer1);
        label_64->setObjectName(QStringLiteral("label_64"));
        label_64->setWordWrap(true);
        label_64->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_80->addWidget(label_64, 1, 1, 1, 1);

        graphOptimization_covarianceIgnored = new QCheckBox(groupBox_optimizer1);
        graphOptimization_covarianceIgnored->setObjectName(QStringLiteral("graphOptimization_covarianceIgnored"));

        gridLayout_80->addWidget(graphOptimization_covarianceIgnored, 2, 0, 1, 1);

        label_31 = new QLabel(groupBox_optimizer1);
        label_31->setObjectName(QStringLiteral("label_31"));
        label_31->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_80->addWidget(label_31, 0, 1, 1, 1);

        gridLayout_80->setColumnStretch(1, 1);

        verticalLayout_113->addLayout(gridLayout_80);

        groupBoxx_g2o = new QGroupBox(groupBox_optimizer1);
        groupBoxx_g2o->setObjectName(QStringLiteral("groupBoxx_g2o"));
        verticalLayout_117 = new QVBoxLayout(groupBoxx_g2o);
        verticalLayout_117->setObjectName(QStringLiteral("verticalLayout_117"));
        gridLayout_70 = new QGridLayout();
        gridLayout_70->setObjectName(QStringLiteral("gridLayout_70"));
        comboBox_g2o_solver = new QComboBox(groupBoxx_g2o);
        comboBox_g2o_solver->setObjectName(QStringLiteral("comboBox_g2o_solver"));

        gridLayout_70->addWidget(comboBox_g2o_solver, 0, 0, 1, 1);

        label_107 = new QLabel(groupBoxx_g2o);
        label_107->setObjectName(QStringLiteral("label_107"));
        label_107->setWordWrap(true);

        gridLayout_70->addWidget(label_107, 0, 1, 1, 1);

        comboBox_g2o_optimizer = new QComboBox(groupBoxx_g2o);
        comboBox_g2o_optimizer->setObjectName(QStringLiteral("comboBox_g2o_optimizer"));

        gridLayout_70->addWidget(comboBox_g2o_optimizer, 1, 0, 1, 1);

        label_138 = new QLabel(groupBoxx_g2o);
        label_138->setObjectName(QStringLiteral("label_138"));
        label_138->setWordWrap(true);

        gridLayout_70->addWidget(label_138, 1, 1, 1, 1);

        gridLayout_70->setColumnStretch(1, 1);

        verticalLayout_117->addLayout(gridLayout_70);

        groupBox_20 = new QGroupBox(groupBoxx_g2o);
        groupBox_20->setObjectName(QStringLiteral("groupBox_20"));
        gridLayout_84 = new QGridLayout(groupBox_20);
        gridLayout_84->setObjectName(QStringLiteral("gridLayout_84"));
        doubleSpinBox_g2o_pixelVariance = new QDoubleSpinBox(groupBox_20);
        doubleSpinBox_g2o_pixelVariance->setObjectName(QStringLiteral("doubleSpinBox_g2o_pixelVariance"));
        doubleSpinBox_g2o_pixelVariance->setDecimals(2);
        doubleSpinBox_g2o_pixelVariance->setMinimum(0.01);
        doubleSpinBox_g2o_pixelVariance->setMaximum(999);
        doubleSpinBox_g2o_pixelVariance->setSingleStep(0.001);
        doubleSpinBox_g2o_pixelVariance->setValue(0.01);

        gridLayout_84->addWidget(doubleSpinBox_g2o_pixelVariance, 0, 0, 1, 1);

        label_148 = new QLabel(groupBox_20);
        label_148->setObjectName(QStringLiteral("label_148"));
        label_148->setWordWrap(true);

        gridLayout_84->addWidget(label_148, 0, 1, 1, 1);

        doubleSpinBox_g2o_robustKernelDelta = new QDoubleSpinBox(groupBox_20);
        doubleSpinBox_g2o_robustKernelDelta->setObjectName(QStringLiteral("doubleSpinBox_g2o_robustKernelDelta"));
        doubleSpinBox_g2o_robustKernelDelta->setDecimals(3);
        doubleSpinBox_g2o_robustKernelDelta->setMinimum(0);
        doubleSpinBox_g2o_robustKernelDelta->setMaximum(999);
        doubleSpinBox_g2o_robustKernelDelta->setSingleStep(1);
        doubleSpinBox_g2o_robustKernelDelta->setValue(5.991);

        gridLayout_84->addWidget(doubleSpinBox_g2o_robustKernelDelta, 1, 0, 1, 1);

        label_350 = new QLabel(groupBox_20);
        label_350->setObjectName(QStringLiteral("label_350"));
        label_350->setWordWrap(true);

        gridLayout_84->addWidget(label_350, 1, 1, 1, 1);

        doubleSpinBox_g2o_baseline = new QDoubleSpinBox(groupBox_20);
        doubleSpinBox_g2o_baseline->setObjectName(QStringLiteral("doubleSpinBox_g2o_baseline"));
        doubleSpinBox_g2o_baseline->setDecimals(4);
        doubleSpinBox_g2o_baseline->setMinimum(0);
        doubleSpinBox_g2o_baseline->setMaximum(9);
        doubleSpinBox_g2o_baseline->setSingleStep(0.01);
        doubleSpinBox_g2o_baseline->setValue(0.075);

        gridLayout_84->addWidget(doubleSpinBox_g2o_baseline, 2, 0, 1, 1);

        label_352 = new QLabel(groupBox_20);
        label_352->setObjectName(QStringLiteral("label_352"));
        label_352->setWordWrap(true);

        gridLayout_84->addWidget(label_352, 2, 1, 1, 1);

        gridLayout_84->setColumnStretch(1, 1);

        verticalLayout_117->addWidget(groupBox_20);


        verticalLayout_113->addWidget(groupBoxx_g2o);

        groupBox_19 = new QGroupBox(groupBox_optimizer1);
        groupBox_19->setObjectName(QStringLiteral("groupBox_19"));
        gridLayout_83 = new QGridLayout(groupBox_19);
        gridLayout_83->setObjectName(QStringLiteral("gridLayout_83"));
        comboBox_gtsam_optimizer = new QComboBox(groupBox_19);
        comboBox_gtsam_optimizer->setObjectName(QStringLiteral("comboBox_gtsam_optimizer"));

        gridLayout_83->addWidget(comboBox_gtsam_optimizer, 0, 0, 1, 1);

        label_364 = new QLabel(groupBox_19);
        label_364->setObjectName(QStringLiteral("label_364"));
        label_364->setWordWrap(true);

        gridLayout_83->addWidget(label_364, 0, 1, 1, 1);

        gridLayout_83->setColumnStretch(1, 1);

        verticalLayout_113->addWidget(groupBox_19);


        verticalLayout_114->addWidget(groupBox_optimizer1);

        verticalSpacer_58 = new QSpacerItem(20, 2185, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_114->addItem(verticalSpacer_58);

        stackedWidget->addWidget(page_65);
        page_29 = new QWidget();
        page_29->setObjectName(QStringLiteral("page_29"));
        verticalLayout_57 = new QVBoxLayout(page_29);
        verticalLayout_57->setObjectName(QStringLiteral("verticalLayout_57"));
        groupBox_stereo1 = new QGroupBox(page_29);
        groupBox_stereo1->setObjectName(QStringLiteral("groupBox_stereo1"));
        verticalLayout_56 = new QVBoxLayout(groupBox_stereo1);
        verticalLayout_56->setObjectName(QStringLiteral("verticalLayout_56"));
        label_187 = new QLabel(groupBox_stereo1);
        label_187->setObjectName(QStringLiteral("label_187"));
        label_187->setWordWrap(true);
        label_187->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_56->addWidget(label_187);

        groupBox_10 = new QGroupBox(groupBox_stereo1);
        groupBox_10->setObjectName(QStringLiteral("groupBox_10"));
        gridLayout_33 = new QGridLayout(groupBox_10);
        gridLayout_33->setObjectName(QStringLiteral("gridLayout_33"));
        label_284 = new QLabel(groupBox_10);
        label_284->setObjectName(QStringLiteral("label_284"));
        label_284->setWordWrap(true);
        label_284->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_33->addWidget(label_284, 2, 1, 1, 1);

        label_285 = new QLabel(groupBox_10);
        label_285->setObjectName(QStringLiteral("label_285"));
        label_285->setWordWrap(true);
        label_285->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_33->addWidget(label_285, 5, 1, 1, 1);

        stereo_winWidth = new QSpinBox(groupBox_10);
        stereo_winWidth->setObjectName(QStringLiteral("stereo_winWidth"));
        stereo_winWidth->setMinimum(3);
        stereo_winWidth->setMaximum(999999);
        stereo_winWidth->setSingleStep(1);
        stereo_winWidth->setValue(21);

        gridLayout_33->addWidget(stereo_winWidth, 1, 0, 1, 1);

        label_205 = new QLabel(groupBox_10);
        label_205->setObjectName(QStringLiteral("label_205"));
        label_205->setWordWrap(true);
        label_205->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_33->addWidget(label_205, 1, 1, 1, 1);

        label_206 = new QLabel(groupBox_10);
        label_206->setObjectName(QStringLiteral("label_206"));
        label_206->setWordWrap(true);
        label_206->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_33->addWidget(label_206, 4, 1, 1, 1);

        stereo_ssd = new QCheckBox(groupBox_10);
        stereo_ssd->setObjectName(QStringLiteral("stereo_ssd"));

        gridLayout_33->addWidget(stereo_ssd, 8, 0, 1, 1);

        stereo_winHeight = new QSpinBox(groupBox_10);
        stereo_winHeight->setObjectName(QStringLiteral("stereo_winHeight"));
        stereo_winHeight->setMinimum(3);
        stereo_winHeight->setMaximum(999999);
        stereo_winHeight->setSingleStep(1);
        stereo_winHeight->setValue(21);

        gridLayout_33->addWidget(stereo_winHeight, 2, 0, 1, 1);

        stereo_iterations = new QSpinBox(groupBox_10);
        stereo_iterations->setObjectName(QStringLiteral("stereo_iterations"));
        stereo_iterations->setMinimum(1);
        stereo_iterations->setMaximum(999999);
        stereo_iterations->setSingleStep(1);
        stereo_iterations->setValue(30);

        gridLayout_33->addWidget(stereo_iterations, 4, 0, 1, 1);

        stereo_flow_eps = new QDoubleSpinBox(groupBox_10);
        stereo_flow_eps->setObjectName(QStringLiteral("stereo_flow_eps"));
        stereo_flow_eps->setDecimals(4);
        stereo_flow_eps->setMinimum(0.0001);
        stereo_flow_eps->setMaximum(0.1);
        stereo_flow_eps->setSingleStep(0.01);
        stereo_flow_eps->setValue(0.01);

        gridLayout_33->addWidget(stereo_flow_eps, 9, 0, 1, 1);

        label_207 = new QLabel(groupBox_10);
        label_207->setObjectName(QStringLiteral("label_207"));
        label_207->setWordWrap(true);
        label_207->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_33->addWidget(label_207, 9, 1, 1, 1);

        label_208 = new QLabel(groupBox_10);
        label_208->setObjectName(QStringLiteral("label_208"));
        label_208->setWordWrap(true);
        label_208->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_33->addWidget(label_208, 3, 1, 1, 1);

        stereo_maxLevel = new QSpinBox(groupBox_10);
        stereo_maxLevel->setObjectName(QStringLiteral("stereo_maxLevel"));
        stereo_maxLevel->setMinimum(0);
        stereo_maxLevel->setMaximum(999999);
        stereo_maxLevel->setSingleStep(1);
        stereo_maxLevel->setValue(3);

        gridLayout_33->addWidget(stereo_maxLevel, 3, 0, 1, 1);

        label_287 = new QLabel(groupBox_10);
        label_287->setObjectName(QStringLiteral("label_287"));
        label_287->setWordWrap(true);
        label_287->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_33->addWidget(label_287, 8, 1, 1, 1);

        label_214 = new QLabel(groupBox_10);
        label_214->setObjectName(QStringLiteral("label_214"));
        label_214->setWordWrap(true);
        label_214->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_33->addWidget(label_214, 7, 1, 1, 1);

        label_286 = new QLabel(groupBox_10);
        label_286->setObjectName(QStringLiteral("label_286"));
        label_286->setWordWrap(true);
        label_286->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_33->addWidget(label_286, 6, 1, 1, 1);

        stereo_opticalFlow = new QCheckBox(groupBox_10);
        stereo_opticalFlow->setObjectName(QStringLiteral("stereo_opticalFlow"));

        gridLayout_33->addWidget(stereo_opticalFlow, 7, 0, 1, 1);

        stereo_minDisparity = new QDoubleSpinBox(groupBox_10);
        stereo_minDisparity->setObjectName(QStringLiteral("stereo_minDisparity"));
        stereo_minDisparity->setMaximum(9999);

        gridLayout_33->addWidget(stereo_minDisparity, 5, 0, 1, 1);

        stereo_maxDisparity = new QDoubleSpinBox(groupBox_10);
        stereo_maxDisparity->setObjectName(QStringLiteral("stereo_maxDisparity"));
        stereo_maxDisparity->setMaximum(9999);

        gridLayout_33->addWidget(stereo_maxDisparity, 6, 0, 1, 1);

        gridLayout_33->setColumnStretch(1, 1);

        verticalLayout_56->addWidget(groupBox_10);


        verticalLayout_57->addWidget(groupBox_stereo1);

        verticalSpacer_29 = new QSpacerItem(20, 464, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_57->addItem(verticalSpacer_29);

        stackedWidget->addWidget(page_29);
        page_51 = new QWidget();
        page_51->setObjectName(QStringLiteral("page_51"));
        verticalLayout_83 = new QVBoxLayout(page_51);
        verticalLayout_83->setObjectName(QStringLiteral("verticalLayout_83"));
        groupBox_stereoDense2 = new QGroupBox(page_51);
        groupBox_stereoDense2->setObjectName(QStringLiteral("groupBox_stereoDense2"));
        verticalLayout_82 = new QVBoxLayout(groupBox_stereoDense2);
        verticalLayout_82->setObjectName(QStringLiteral("verticalLayout_82"));
        horizontalLayout_13 = new QHBoxLayout();
        horizontalLayout_13->setObjectName(QStringLiteral("horizontalLayout_13"));
        comboBox_stereoDense_strategy = new QComboBox(groupBox_stereoDense2);
        comboBox_stereoDense_strategy->setObjectName(QStringLiteral("comboBox_stereoDense_strategy"));

        horizontalLayout_13->addWidget(comboBox_stereoDense_strategy);

        label_515 = new QLabel(groupBox_stereoDense2);
        label_515->setObjectName(QStringLiteral("label_515"));
        label_515->setWordWrap(true);
        label_515->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        horizontalLayout_13->addWidget(label_515);

        horizontalLayout_13->setStretch(1, 1);

        verticalLayout_82->addLayout(horizontalLayout_13);

        stackedWidget_stereoDense = new QStackedWidget(groupBox_stereoDense2);
        stackedWidget_stereoDense->setObjectName(QStringLiteral("stackedWidget_stereoDense"));
        page_75 = new QWidget();
        page_75->setObjectName(QStringLiteral("page_75"));
        verticalLayout_133 = new QVBoxLayout(page_75);
        verticalLayout_133->setObjectName(QStringLiteral("verticalLayout_133"));
        label_197 = new QLabel(page_75);
        label_197->setObjectName(QStringLiteral("label_197"));
        label_197->setWordWrap(true);
        label_197->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_133->addWidget(label_197);

        groupBox_16 = new QGroupBox(page_75);
        groupBox_16->setObjectName(QStringLiteral("groupBox_16"));
        gridLayout_34 = new QGridLayout(groupBox_16);
        gridLayout_34->setObjectName(QStringLiteral("gridLayout_34"));
        stereobm_uniquessRatio = new QSpinBox(groupBox_16);
        stereobm_uniquessRatio->setObjectName(QStringLiteral("stereobm_uniquessRatio"));
        stereobm_uniquessRatio->setMinimum(1);
        stereobm_uniquessRatio->setMaximum(999999);
        stereobm_uniquessRatio->setSingleStep(1);
        stereobm_uniquessRatio->setValue(21);

        gridLayout_34->addWidget(stereobm_uniquessRatio, 6, 0, 1, 1);

        stereobm_numDisparities = new QSpinBox(groupBox_16);
        stereobm_numDisparities->setObjectName(QStringLiteral("stereobm_numDisparities"));
        stereobm_numDisparities->setMinimum(0);
        stereobm_numDisparities->setMaximum(999999);
        stereobm_numDisparities->setSingleStep(1);
        stereobm_numDisparities->setValue(21);

        gridLayout_34->addWidget(stereobm_numDisparities, 3, 0, 1, 1);

        stereobm_preFilterCap = new QSpinBox(groupBox_16);
        stereobm_preFilterCap->setObjectName(QStringLiteral("stereobm_preFilterCap"));
        stereobm_preFilterCap->setMinimum(1);
        stereobm_preFilterCap->setMaximum(999999);
        stereobm_preFilterCap->setSingleStep(1);
        stereobm_preFilterCap->setValue(21);

        gridLayout_34->addWidget(stereobm_preFilterCap, 5, 0, 1, 1);

        label_278 = new QLabel(groupBox_16);
        label_278->setObjectName(QStringLiteral("label_278"));
        label_278->setWordWrap(true);
        label_278->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_34->addWidget(label_278, 4, 1, 1, 1);

        label_276 = new QLabel(groupBox_16);
        label_276->setObjectName(QStringLiteral("label_276"));
        label_276->setWordWrap(true);
        label_276->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_34->addWidget(label_276, 2, 1, 1, 1);

        label_281 = new QLabel(groupBox_16);
        label_281->setObjectName(QStringLiteral("label_281"));
        label_281->setWordWrap(true);
        label_281->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_34->addWidget(label_281, 7, 1, 1, 1);

        label_279 = new QLabel(groupBox_16);
        label_279->setObjectName(QStringLiteral("label_279"));
        label_279->setWordWrap(true);
        label_279->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_34->addWidget(label_279, 5, 1, 1, 1);

        stereobm_speckleWinSize = new QSpinBox(groupBox_16);
        stereobm_speckleWinSize->setObjectName(QStringLiteral("stereobm_speckleWinSize"));
        stereobm_speckleWinSize->setMinimum(0);
        stereobm_speckleWinSize->setMaximum(999999);
        stereobm_speckleWinSize->setSingleStep(1);
        stereobm_speckleWinSize->setValue(21);

        gridLayout_34->addWidget(stereobm_speckleWinSize, 8, 0, 1, 1);

        label_277 = new QLabel(groupBox_16);
        label_277->setObjectName(QStringLiteral("label_277"));
        label_277->setWordWrap(true);
        label_277->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_34->addWidget(label_277, 3, 1, 1, 1);

        label_216 = new QLabel(groupBox_16);
        label_216->setObjectName(QStringLiteral("label_216"));
        label_216->setWordWrap(true);
        label_216->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_34->addWidget(label_216, 1, 1, 1, 1);

        stereobm_minDisparity = new QSpinBox(groupBox_16);
        stereobm_minDisparity->setObjectName(QStringLiteral("stereobm_minDisparity"));
        stereobm_minDisparity->setMinimum(0);
        stereobm_minDisparity->setMaximum(999999);
        stereobm_minDisparity->setSingleStep(1);
        stereobm_minDisparity->setValue(21);

        gridLayout_34->addWidget(stereobm_minDisparity, 2, 0, 1, 1);

        stereobm_blockSize = new QSpinBox(groupBox_16);
        stereobm_blockSize->setObjectName(QStringLiteral("stereobm_blockSize"));
        stereobm_blockSize->setMinimum(3);
        stereobm_blockSize->setMaximum(999999);
        stereobm_blockSize->setSingleStep(1);
        stereobm_blockSize->setValue(21);

        gridLayout_34->addWidget(stereobm_blockSize, 1, 0, 1, 1);

        stereobm_preFilterSize = new QSpinBox(groupBox_16);
        stereobm_preFilterSize->setObjectName(QStringLiteral("stereobm_preFilterSize"));
        stereobm_preFilterSize->setMinimum(1);
        stereobm_preFilterSize->setMaximum(999999);
        stereobm_preFilterSize->setSingleStep(1);
        stereobm_preFilterSize->setValue(21);

        gridLayout_34->addWidget(stereobm_preFilterSize, 4, 0, 1, 1);

        label_280 = new QLabel(groupBox_16);
        label_280->setObjectName(QStringLiteral("label_280"));
        label_280->setWordWrap(true);
        label_280->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_34->addWidget(label_280, 6, 1, 1, 1);

        stereobm_tetureThreshold = new QSpinBox(groupBox_16);
        stereobm_tetureThreshold->setObjectName(QStringLiteral("stereobm_tetureThreshold"));
        stereobm_tetureThreshold->setMinimum(1);
        stereobm_tetureThreshold->setMaximum(999999);
        stereobm_tetureThreshold->setSingleStep(1);
        stereobm_tetureThreshold->setValue(21);

        gridLayout_34->addWidget(stereobm_tetureThreshold, 7, 0, 1, 1);

        stereobm_speckleRange = new QSpinBox(groupBox_16);
        stereobm_speckleRange->setObjectName(QStringLiteral("stereobm_speckleRange"));
        stereobm_speckleRange->setMinimum(0);
        stereobm_speckleRange->setMaximum(999999);
        stereobm_speckleRange->setSingleStep(1);
        stereobm_speckleRange->setValue(21);

        gridLayout_34->addWidget(stereobm_speckleRange, 9, 0, 1, 1);

        label_282 = new QLabel(groupBox_16);
        label_282->setObjectName(QStringLiteral("label_282"));
        label_282->setWordWrap(true);
        label_282->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_34->addWidget(label_282, 8, 1, 1, 1);

        label_283 = new QLabel(groupBox_16);
        label_283->setObjectName(QStringLiteral("label_283"));
        label_283->setWordWrap(true);
        label_283->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_34->addWidget(label_283, 9, 1, 1, 1);

        label_501 = new QLabel(groupBox_16);
        label_501->setObjectName(QStringLiteral("label_501"));
        label_501->setWordWrap(true);
        label_501->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_34->addWidget(label_501, 10, 1, 1, 1);

        stereobm_disp12MaxDiff = new QSpinBox(groupBox_16);
        stereobm_disp12MaxDiff->setObjectName(QStringLiteral("stereobm_disp12MaxDiff"));
        stereobm_disp12MaxDiff->setMinimum(-1);
        stereobm_disp12MaxDiff->setMaximum(999999);
        stereobm_disp12MaxDiff->setSingleStep(1);
        stereobm_disp12MaxDiff->setValue(21);

        gridLayout_34->addWidget(stereobm_disp12MaxDiff, 10, 0, 1, 1);

        gridLayout_34->setColumnStretch(1, 1);

        verticalLayout_133->addWidget(groupBox_16);

        stackedWidget_stereoDense->addWidget(page_75);
        page_76 = new QWidget();
        page_76->setObjectName(QStringLiteral("page_76"));
        verticalLayout_134 = new QVBoxLayout(page_76);
        verticalLayout_134->setObjectName(QStringLiteral("verticalLayout_134"));
        label_514 = new QLabel(page_76);
        label_514->setObjectName(QStringLiteral("label_514"));
        label_514->setWordWrap(true);
        label_514->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_134->addWidget(label_514);

        groupBox_30 = new QGroupBox(page_76);
        groupBox_30->setObjectName(QStringLiteral("groupBox_30"));
        gridLayout_105 = new QGridLayout(groupBox_30);
        gridLayout_105->setObjectName(QStringLiteral("gridLayout_105"));
        label_517 = new QLabel(groupBox_30);
        label_517->setObjectName(QStringLiteral("label_517"));
        label_517->setWordWrap(true);
        label_517->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_105->addWidget(label_517, 9, 1, 1, 1);

        stereosgbm_disp12MaxDiff = new QSpinBox(groupBox_30);
        stereosgbm_disp12MaxDiff->setObjectName(QStringLiteral("stereosgbm_disp12MaxDiff"));
        stereosgbm_disp12MaxDiff->setMinimum(0);
        stereosgbm_disp12MaxDiff->setMaximum(999999);
        stereosgbm_disp12MaxDiff->setSingleStep(1);
        stereosgbm_disp12MaxDiff->setValue(21);

        gridLayout_105->addWidget(stereosgbm_disp12MaxDiff, 8, 0, 1, 1);

        label_516 = new QLabel(groupBox_30);
        label_516->setObjectName(QStringLiteral("label_516"));
        label_516->setWordWrap(true);
        label_516->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_105->addWidget(label_516, 8, 1, 1, 1);

        stereosgbm_numDisparities = new QSpinBox(groupBox_30);
        stereosgbm_numDisparities->setObjectName(QStringLiteral("stereosgbm_numDisparities"));
        stereosgbm_numDisparities->setMinimum(0);
        stereosgbm_numDisparities->setMaximum(999999);
        stereosgbm_numDisparities->setSingleStep(1);
        stereosgbm_numDisparities->setValue(21);

        gridLayout_105->addWidget(stereosgbm_numDisparities, 3, 0, 1, 1);

        stereosgbm_uniquessRatio = new QSpinBox(groupBox_30);
        stereosgbm_uniquessRatio->setObjectName(QStringLiteral("stereosgbm_uniquessRatio"));
        stereosgbm_uniquessRatio->setMinimum(-1);
        stereosgbm_uniquessRatio->setMaximum(999999);
        stereosgbm_uniquessRatio->setSingleStep(1);
        stereosgbm_uniquessRatio->setValue(21);

        gridLayout_105->addWidget(stereosgbm_uniquessRatio, 5, 0, 1, 1);

        label_499 = new QLabel(groupBox_30);
        label_499->setObjectName(QStringLiteral("label_499"));
        label_499->setWordWrap(true);
        label_499->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_105->addWidget(label_499, 4, 1, 1, 1);

        label_500 = new QLabel(groupBox_30);
        label_500->setObjectName(QStringLiteral("label_500"));
        label_500->setWordWrap(true);
        label_500->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_105->addWidget(label_500, 2, 1, 1, 1);

        stereosgbm_speckleWinSize = new QSpinBox(groupBox_30);
        stereosgbm_speckleWinSize->setObjectName(QStringLiteral("stereosgbm_speckleWinSize"));
        stereosgbm_speckleWinSize->setMinimum(0);
        stereosgbm_speckleWinSize->setMaximum(999999);
        stereosgbm_speckleWinSize->setSingleStep(1);
        stereosgbm_speckleWinSize->setValue(21);

        gridLayout_105->addWidget(stereosgbm_speckleWinSize, 6, 0, 1, 1);

        label_504 = new QLabel(groupBox_30);
        label_504->setObjectName(QStringLiteral("label_504"));
        label_504->setWordWrap(true);
        label_504->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_105->addWidget(label_504, 1, 1, 1, 1);

        label_503 = new QLabel(groupBox_30);
        label_503->setObjectName(QStringLiteral("label_503"));
        label_503->setWordWrap(true);
        label_503->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_105->addWidget(label_503, 3, 1, 1, 1);

        stereosgbm_minDisparity = new QSpinBox(groupBox_30);
        stereosgbm_minDisparity->setObjectName(QStringLiteral("stereosgbm_minDisparity"));
        stereosgbm_minDisparity->setMinimum(0);
        stereosgbm_minDisparity->setMaximum(999999);
        stereosgbm_minDisparity->setSingleStep(1);
        stereosgbm_minDisparity->setValue(21);

        gridLayout_105->addWidget(stereosgbm_minDisparity, 2, 0, 1, 1);

        stereosgbm_preFilterCap = new QSpinBox(groupBox_30);
        stereosgbm_preFilterCap->setObjectName(QStringLiteral("stereosgbm_preFilterCap"));
        stereosgbm_preFilterCap->setMinimum(0);
        stereosgbm_preFilterCap->setMaximum(999999);
        stereosgbm_preFilterCap->setSingleStep(1);
        stereosgbm_preFilterCap->setValue(21);

        gridLayout_105->addWidget(stereosgbm_preFilterCap, 4, 0, 1, 1);

        stereosgbm_blockSize = new QSpinBox(groupBox_30);
        stereosgbm_blockSize->setObjectName(QStringLiteral("stereosgbm_blockSize"));
        stereosgbm_blockSize->setMinimum(3);
        stereosgbm_blockSize->setMaximum(999999);
        stereosgbm_blockSize->setSingleStep(1);
        stereosgbm_blockSize->setValue(21);

        gridLayout_105->addWidget(stereosgbm_blockSize, 1, 0, 1, 1);

        label_505 = new QLabel(groupBox_30);
        label_505->setObjectName(QStringLiteral("label_505"));
        label_505->setWordWrap(true);
        label_505->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_105->addWidget(label_505, 5, 1, 1, 1);

        label_506 = new QLabel(groupBox_30);
        label_506->setObjectName(QStringLiteral("label_506"));
        label_506->setWordWrap(true);
        label_506->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_105->addWidget(label_506, 6, 1, 1, 1);

        label_512 = new QLabel(groupBox_30);
        label_512->setObjectName(QStringLiteral("label_512"));
        label_512->setWordWrap(true);
        label_512->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_105->addWidget(label_512, 7, 1, 1, 1);

        stereosgbm_speckleRange = new QSpinBox(groupBox_30);
        stereosgbm_speckleRange->setObjectName(QStringLiteral("stereosgbm_speckleRange"));
        stereosgbm_speckleRange->setMinimum(0);
        stereosgbm_speckleRange->setMaximum(999999);
        stereosgbm_speckleRange->setSingleStep(1);
        stereosgbm_speckleRange->setValue(21);

        gridLayout_105->addWidget(stereosgbm_speckleRange, 7, 0, 1, 1);

        stereosgbm_p2 = new QSpinBox(groupBox_30);
        stereosgbm_p2->setObjectName(QStringLiteral("stereosgbm_p2"));
        stereosgbm_p2->setMinimum(0);
        stereosgbm_p2->setMaximum(999999);
        stereosgbm_p2->setSingleStep(1);
        stereosgbm_p2->setValue(21);

        gridLayout_105->addWidget(stereosgbm_p2, 10, 0, 1, 1);

        label_518 = new QLabel(groupBox_30);
        label_518->setObjectName(QStringLiteral("label_518"));
        label_518->setWordWrap(true);
        label_518->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_105->addWidget(label_518, 10, 1, 1, 1);

        stereosgbm_p1 = new QSpinBox(groupBox_30);
        stereosgbm_p1->setObjectName(QStringLiteral("stereosgbm_p1"));
        stereosgbm_p1->setMinimum(0);
        stereosgbm_p1->setMaximum(999999);
        stereosgbm_p1->setSingleStep(1);
        stereosgbm_p1->setValue(21);

        gridLayout_105->addWidget(stereosgbm_p1, 9, 0, 1, 1);

        label_519 = new QLabel(groupBox_30);
        label_519->setObjectName(QStringLiteral("label_519"));
        label_519->setWordWrap(true);
        label_519->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_105->addWidget(label_519, 11, 1, 1, 1);

        stereosgbm_mode = new QComboBox(groupBox_30);
        stereosgbm_mode->setObjectName(QStringLiteral("stereosgbm_mode"));

        gridLayout_105->addWidget(stereosgbm_mode, 11, 0, 1, 1);

        gridLayout_105->setColumnStretch(1, 1);

        verticalLayout_134->addWidget(groupBox_30);

        stackedWidget_stereoDense->addWidget(page_76);

        verticalLayout_82->addWidget(stackedWidget_stereoDense);

        verticalSpacer_53 = new QSpacerItem(20, 2367, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_82->addItem(verticalSpacer_53);

        verticalLayout_82->setStretch(2, 1);

        verticalLayout_83->addWidget(groupBox_stereoDense2);

        stackedWidget->addWidget(page_51);
        page_45 = new QWidget();
        page_45->setObjectName(QStringLiteral("page_45"));
        verticalLayout_74 = new QVBoxLayout(page_45);
        verticalLayout_74->setObjectName(QStringLiteral("verticalLayout_74"));
        groupBox_feature1 = new QGroupBox(page_45);
        groupBox_feature1->setObjectName(QStringLiteral("groupBox_feature1"));
        verticalLayout_75 = new QVBoxLayout(groupBox_feature1);
        verticalLayout_75->setObjectName(QStringLiteral("verticalLayout_75"));
        label_10 = new QLabel(groupBox_feature1);
        label_10->setObjectName(QStringLiteral("label_10"));

        verticalLayout_75->addWidget(label_10);


        verticalLayout_74->addWidget(groupBox_feature1);

        verticalSpacer_36 = new QSpacerItem(20, 1197, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_74->addItem(verticalSpacer_36);

        stackedWidget->addWidget(page_45);
        page_13 = new QWidget();
        page_13->setObjectName(QStringLiteral("page_13"));
        verticalLayout_38 = new QVBoxLayout(page_13);
        verticalLayout_38->setObjectName(QStringLiteral("verticalLayout_38"));
        groupBox_detector_surf2 = new QGroupBox(page_13);
        groupBox_detector_surf2->setObjectName(QStringLiteral("groupBox_detector_surf2"));
        gridLayout_15 = new QGridLayout(groupBox_detector_surf2);
        gridLayout_15->setObjectName(QStringLiteral("gridLayout_15"));
        surf_checkBox_upright = new QCheckBox(groupBox_detector_surf2);
        surf_checkBox_upright->setObjectName(QStringLiteral("surf_checkBox_upright"));
        surf_checkBox_upright->setEnabled(true);

        gridLayout_15->addWidget(surf_checkBox_upright, 4, 0, 1, 1);

        label_41 = new QLabel(groupBox_detector_surf2);
        label_41->setObjectName(QStringLiteral("label_41"));
        label_41->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_15->addWidget(label_41, 2, 1, 1, 1);

        label_8 = new QLabel(groupBox_detector_surf2);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setWordWrap(true);
        label_8->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_15->addWidget(label_8, 3, 1, 1, 1);

        label_surf_checkBox_gpuKeypointsRatio = new QLabel(groupBox_detector_surf2);
        label_surf_checkBox_gpuKeypointsRatio->setObjectName(QStringLiteral("label_surf_checkBox_gpuKeypointsRatio"));
        label_surf_checkBox_gpuKeypointsRatio->setEnabled(true);
        label_surf_checkBox_gpuKeypointsRatio->setWordWrap(true);
        label_surf_checkBox_gpuKeypointsRatio->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_15->addWidget(label_surf_checkBox_gpuKeypointsRatio, 6, 1, 1, 1);

        label_surf_checkBox_gpuVersion = new QLabel(groupBox_detector_surf2);
        label_surf_checkBox_gpuVersion->setObjectName(QStringLiteral("label_surf_checkBox_gpuVersion"));
        label_surf_checkBox_gpuVersion->setEnabled(true);
        label_surf_checkBox_gpuVersion->setWordWrap(true);
        label_surf_checkBox_gpuVersion->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_15->addWidget(label_surf_checkBox_gpuVersion, 5, 1, 1, 1);

        surf_spinBox_octaves = new QSpinBox(groupBox_detector_surf2);
        surf_spinBox_octaves->setObjectName(QStringLiteral("surf_spinBox_octaves"));
        surf_spinBox_octaves->setMinimum(1);
        surf_spinBox_octaves->setMaximum(100000);
        surf_spinBox_octaves->setValue(4);

        gridLayout_15->addWidget(surf_spinBox_octaves, 1, 0, 1, 1);

        surf_checkBox_gpuVersion = new QCheckBox(groupBox_detector_surf2);
        surf_checkBox_gpuVersion->setObjectName(QStringLiteral("surf_checkBox_gpuVersion"));
        surf_checkBox_gpuVersion->setEnabled(true);

        gridLayout_15->addWidget(surf_checkBox_gpuVersion, 5, 0, 1, 1);

        label_49 = new QLabel(groupBox_detector_surf2);
        label_49->setObjectName(QStringLiteral("label_49"));
        label_49->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_15->addWidget(label_49, 4, 1, 1, 1);

        surf_doubleSpinBox_gpuKeypointsRatio = new QDoubleSpinBox(groupBox_detector_surf2);
        surf_doubleSpinBox_gpuKeypointsRatio->setObjectName(QStringLiteral("surf_doubleSpinBox_gpuKeypointsRatio"));
        surf_doubleSpinBox_gpuKeypointsRatio->setValue(0.01);

        gridLayout_15->addWidget(surf_doubleSpinBox_gpuKeypointsRatio, 6, 0, 1, 1);

        surf_doubleSpinBox_hessianThr = new QDoubleSpinBox(groupBox_detector_surf2);
        surf_doubleSpinBox_hessianThr->setObjectName(QStringLiteral("surf_doubleSpinBox_hessianThr"));
        surf_doubleSpinBox_hessianThr->setDecimals(0);
        surf_doubleSpinBox_hessianThr->setMaximum(50000);
        surf_doubleSpinBox_hessianThr->setValue(500);

        gridLayout_15->addWidget(surf_doubleSpinBox_hessianThr, 0, 0, 1, 1);

        checkBox_surfExtended = new QCheckBox(groupBox_detector_surf2);
        checkBox_surfExtended->setObjectName(QStringLiteral("checkBox_surfExtended"));

        gridLayout_15->addWidget(checkBox_surfExtended, 3, 0, 1, 1);

        surf_spinBox_octaveLayers = new QSpinBox(groupBox_detector_surf2);
        surf_spinBox_octaveLayers->setObjectName(QStringLiteral("surf_spinBox_octaveLayers"));
        surf_spinBox_octaveLayers->setMinimum(1);
        surf_spinBox_octaveLayers->setMaximum(100000);
        surf_spinBox_octaveLayers->setValue(2);

        gridLayout_15->addWidget(surf_spinBox_octaveLayers, 2, 0, 1, 1);

        label_26 = new QLabel(groupBox_detector_surf2);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_15->addWidget(label_26, 1, 1, 1, 1);

        label_4 = new QLabel(groupBox_detector_surf2);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_15->addWidget(label_4, 0, 1, 1, 1);

        gridLayout_15->setColumnStretch(1, 1);

        verticalLayout_38->addWidget(groupBox_detector_surf2);

        verticalSpacer_17 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_38->addItem(verticalSpacer_17);

        stackedWidget->addWidget(page_13);
        page_14 = new QWidget();
        page_14->setObjectName(QStringLiteral("page_14"));
        verticalLayout_39 = new QVBoxLayout(page_14);
        verticalLayout_39->setObjectName(QStringLiteral("verticalLayout_39"));
        groupBox_detector_sift2 = new QGroupBox(page_14);
        groupBox_detector_sift2->setObjectName(QStringLiteral("groupBox_detector_sift2"));
        gridLayout_19 = new QGridLayout(groupBox_detector_sift2);
        gridLayout_19->setObjectName(QStringLiteral("gridLayout_19"));
        sift_spinBox_nFeatures = new QSpinBox(groupBox_detector_sift2);
        sift_spinBox_nFeatures->setObjectName(QStringLiteral("sift_spinBox_nFeatures"));

        gridLayout_19->addWidget(sift_spinBox_nFeatures, 0, 0, 1, 1);

        label_129 = new QLabel(groupBox_detector_sift2);
        label_129->setObjectName(QStringLiteral("label_129"));
        label_129->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_19->addWidget(label_129, 4, 1, 1, 1);

        sift_doubleSpinBox_contrastThr = new QDoubleSpinBox(groupBox_detector_sift2);
        sift_doubleSpinBox_contrastThr->setObjectName(QStringLiteral("sift_doubleSpinBox_contrastThr"));
        sift_doubleSpinBox_contrastThr->setDecimals(6);
        sift_doubleSpinBox_contrastThr->setMinimum(0);
        sift_doubleSpinBox_contrastThr->setMaximum(50000);
        sift_doubleSpinBox_contrastThr->setSingleStep(0.001);
        sift_doubleSpinBox_contrastThr->setValue(0.006667);

        gridLayout_19->addWidget(sift_doubleSpinBox_contrastThr, 2, 0, 1, 1);

        label_5 = new QLabel(groupBox_detector_sift2);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_19->addWidget(label_5, 2, 1, 1, 1);

        label_30 = new QLabel(groupBox_detector_sift2);
        label_30->setObjectName(QStringLiteral("label_30"));
        label_30->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_19->addWidget(label_30, 3, 1, 1, 1);

        sift_doubleSpinBox_edgeThr = new QDoubleSpinBox(groupBox_detector_sift2);
        sift_doubleSpinBox_edgeThr->setObjectName(QStringLiteral("sift_doubleSpinBox_edgeThr"));
        sift_doubleSpinBox_edgeThr->setSingleStep(0.1);
        sift_doubleSpinBox_edgeThr->setValue(10);

        gridLayout_19->addWidget(sift_doubleSpinBox_edgeThr, 3, 0, 1, 1);

        sift_doubleSpinBox_sigma = new QDoubleSpinBox(groupBox_detector_sift2);
        sift_doubleSpinBox_sigma->setObjectName(QStringLiteral("sift_doubleSpinBox_sigma"));
        sift_doubleSpinBox_sigma->setSingleStep(0.1);
        sift_doubleSpinBox_sigma->setValue(10);

        gridLayout_19->addWidget(sift_doubleSpinBox_sigma, 4, 0, 1, 1);

        sift_spinBox_nOctaveLayers = new QSpinBox(groupBox_detector_sift2);
        sift_spinBox_nOctaveLayers->setObjectName(QStringLiteral("sift_spinBox_nOctaveLayers"));

        gridLayout_19->addWidget(sift_spinBox_nOctaveLayers, 1, 0, 1, 1);

        label_130 = new QLabel(groupBox_detector_sift2);
        label_130->setObjectName(QStringLiteral("label_130"));
        label_130->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_19->addWidget(label_130, 1, 1, 1, 1);

        label_131 = new QLabel(groupBox_detector_sift2);
        label_131->setObjectName(QStringLiteral("label_131"));
        label_131->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_19->addWidget(label_131, 0, 1, 1, 1);

        label_591 = new QLabel(groupBox_detector_sift2);
        label_591->setObjectName(QStringLiteral("label_591"));
        label_591->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_19->addWidget(label_591, 5, 1, 1, 1);

        sift_checkBox_rootsift = new QCheckBox(groupBox_detector_sift2);
        sift_checkBox_rootsift->setObjectName(QStringLiteral("sift_checkBox_rootsift"));

        gridLayout_19->addWidget(sift_checkBox_rootsift, 5, 0, 1, 1);

        gridLayout_19->setColumnStretch(1, 1);

        verticalLayout_39->addWidget(groupBox_detector_sift2);

        verticalSpacer_19 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_39->addItem(verticalSpacer_19);

        stackedWidget->addWidget(page_14);
        page_17 = new QWidget();
        page_17->setObjectName(QStringLiteral("page_17"));
        verticalLayout_14 = new QVBoxLayout(page_17);
        verticalLayout_14->setObjectName(QStringLiteral("verticalLayout_14"));
        groupBox_detector_fast2 = new QGroupBox(page_17);
        groupBox_detector_fast2->setObjectName(QStringLiteral("groupBox_detector_fast2"));
        groupBox_detector_fast2->setCheckable(false);
        verticalLayout = new QVBoxLayout(groupBox_detector_fast2);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        gridLayout_94 = new QGridLayout();
        gridLayout_94->setObjectName(QStringLiteral("gridLayout_94"));
        fastThreshold = new QSpinBox(groupBox_detector_fast2);
        fastThreshold->setObjectName(QStringLiteral("fastThreshold"));
        fastThreshold->setMinimum(1);
        fastThreshold->setMaximum(9999);
        fastThreshold->setValue(50);

        gridLayout_94->addWidget(fastThreshold, 0, 0, 1, 1);

        label_56 = new QLabel(groupBox_detector_fast2);
        label_56->setObjectName(QStringLiteral("label_56"));
        label_56->setWordWrap(true);
        label_56->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_94->addWidget(label_56, 0, 1, 1, 1);

        fastThresholdMin = new QSpinBox(groupBox_detector_fast2);
        fastThresholdMin->setObjectName(QStringLiteral("fastThresholdMin"));
        fastThresholdMin->setMinimum(1);
        fastThresholdMin->setMaximum(9999);
        fastThresholdMin->setValue(50);

        gridLayout_94->addWidget(fastThresholdMin, 1, 0, 1, 1);

        label_111 = new QLabel(groupBox_detector_fast2);
        label_111->setObjectName(QStringLiteral("label_111"));
        label_111->setWordWrap(true);
        label_111->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_94->addWidget(label_111, 1, 1, 1, 1);

        fastSuppressNonMax = new QCheckBox(groupBox_detector_fast2);
        fastSuppressNonMax->setObjectName(QStringLiteral("fastSuppressNonMax"));
        fastSuppressNonMax->setChecked(true);

        gridLayout_94->addWidget(fastSuppressNonMax, 2, 0, 1, 1);

        label_69 = new QLabel(groupBox_detector_fast2);
        label_69->setObjectName(QStringLiteral("label_69"));
        label_69->setWordWrap(true);
        label_69->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_94->addWidget(label_69, 2, 1, 1, 1);

        label_561 = new QLabel(groupBox_detector_fast2);
        label_561->setObjectName(QStringLiteral("label_561"));
        label_561->setWordWrap(true);
        label_561->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_94->addWidget(label_561, 3, 1, 1, 1);

        fastCV = new QSpinBox(groupBox_detector_fast2);
        fastCV->setObjectName(QStringLiteral("fastCV"));
        fastCV->setMinimum(0);
        fastCV->setMaximum(10);
        fastCV->setValue(0);

        gridLayout_94->addWidget(fastCV, 3, 0, 1, 1);

        gridLayout_94->setColumnStretch(1, 1);

        verticalLayout->addLayout(gridLayout_94);

        groupBox_fast_opencv2 = new QGroupBox(groupBox_detector_fast2);
        groupBox_fast_opencv2->setObjectName(QStringLiteral("groupBox_fast_opencv2"));
        gridLayout_71 = new QGridLayout(groupBox_fast_opencv2);
        gridLayout_71->setObjectName(QStringLiteral("gridLayout_71"));
        fastThresholdMax = new QSpinBox(groupBox_fast_opencv2);
        fastThresholdMax->setObjectName(QStringLiteral("fastThresholdMax"));
        fastThresholdMax->setMinimum(1);
        fastThresholdMax->setMaximum(9999);
        fastThresholdMax->setValue(50);

        gridLayout_71->addWidget(fastThresholdMax, 1, 0, 1, 1);

        label_115 = new QLabel(groupBox_fast_opencv2);
        label_115->setObjectName(QStringLiteral("label_115"));
        label_115->setWordWrap(true);
        label_115->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_71->addWidget(label_115, 1, 1, 1, 1);

        fastGridRows = new QSpinBox(groupBox_fast_opencv2);
        fastGridRows->setObjectName(QStringLiteral("fastGridRows"));
        fastGridRows->setMinimum(0);
        fastGridRows->setMaximum(9999);
        fastGridRows->setValue(0);

        gridLayout_71->addWidget(fastGridRows, 2, 0, 1, 1);

        label_104 = new QLabel(groupBox_fast_opencv2);
        label_104->setObjectName(QStringLiteral("label_104"));
        label_104->setWordWrap(true);
        label_104->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_71->addWidget(label_104, 2, 1, 1, 1);

        fastGridCols = new QSpinBox(groupBox_fast_opencv2);
        fastGridCols->setObjectName(QStringLiteral("fastGridCols"));
        fastGridCols->setMinimum(0);
        fastGridCols->setMaximum(9999);
        fastGridCols->setValue(0);

        gridLayout_71->addWidget(fastGridCols, 3, 0, 1, 1);

        label_109 = new QLabel(groupBox_fast_opencv2);
        label_109->setObjectName(QStringLiteral("label_109"));
        label_109->setWordWrap(true);
        label_109->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_71->addWidget(label_109, 3, 1, 1, 1);

        fastGpu = new QCheckBox(groupBox_fast_opencv2);
        fastGpu->setObjectName(QStringLiteral("fastGpu"));
        fastGpu->setChecked(true);

        gridLayout_71->addWidget(fastGpu, 4, 0, 1, 1);

        label_fastGPU = new QLabel(groupBox_fast_opencv2);
        label_fastGPU->setObjectName(QStringLiteral("label_fastGPU"));
        label_fastGPU->setWordWrap(true);
        label_fastGPU->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_71->addWidget(label_fastGPU, 4, 1, 1, 1);

        fastKeypointRatio = new QDoubleSpinBox(groupBox_fast_opencv2);
        fastKeypointRatio->setObjectName(QStringLiteral("fastKeypointRatio"));
        fastKeypointRatio->setMaximum(1);
        fastKeypointRatio->setSingleStep(0.01);
        fastKeypointRatio->setValue(0.05);

        gridLayout_71->addWidget(fastKeypointRatio, 5, 0, 1, 1);

        label_fastGPUKptRatio = new QLabel(groupBox_fast_opencv2);
        label_fastGPUKptRatio->setObjectName(QStringLiteral("label_fastGPUKptRatio"));
        label_fastGPUKptRatio->setWordWrap(true);
        label_fastGPUKptRatio->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_71->addWidget(label_fastGPUKptRatio, 5, 1, 1, 1);

        gridLayout_71->setColumnStretch(1, 1);

        verticalLayout->addWidget(groupBox_fast_opencv2);

        verticalSpacer_16 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_16);


        verticalLayout_14->addWidget(groupBox_detector_fast2);

        stackedWidget->addWidget(page_17);
        page_15 = new QWidget();
        page_15->setObjectName(QStringLiteral("page_15"));
        verticalLayout_40 = new QVBoxLayout(page_15);
        verticalLayout_40->setObjectName(QStringLiteral("verticalLayout_40"));
        groupBox_detector_brief2 = new QGroupBox(page_15);
        groupBox_detector_brief2->setObjectName(QStringLiteral("groupBox_detector_brief2"));
        gridLayout_16 = new QGridLayout(groupBox_detector_brief2);
        gridLayout_16->setObjectName(QStringLiteral("gridLayout_16"));
        briefBytes = new QSpinBox(groupBox_detector_brief2);
        briefBytes->setObjectName(QStringLiteral("briefBytes"));
        briefBytes->setMinimum(1);
        briefBytes->setMaximum(1000);
        briefBytes->setValue(32);

        gridLayout_16->addWidget(briefBytes, 0, 0, 1, 1);

        label_52 = new QLabel(groupBox_detector_brief2);
        label_52->setObjectName(QStringLiteral("label_52"));
        label_52->setWordWrap(true);
        label_52->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_16->addWidget(label_52, 0, 1, 1, 1);

        gridLayout_16->setColumnStretch(1, 1);

        verticalLayout_40->addWidget(groupBox_detector_brief2);

        verticalSpacer_22 = new QSpacerItem(20, 660, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_40->addItem(verticalSpacer_22);

        stackedWidget->addWidget(page_15);
        page_18 = new QWidget();
        page_18->setObjectName(QStringLiteral("page_18"));
        verticalLayout_35 = new QVBoxLayout(page_18);
        verticalLayout_35->setObjectName(QStringLiteral("verticalLayout_35"));
        groupBox_detector_orb2 = new QGroupBox(page_18);
        groupBox_detector_orb2->setObjectName(QStringLiteral("groupBox_detector_orb2"));
        gridLayout_14 = new QGridLayout(groupBox_detector_orb2);
        gridLayout_14->setObjectName(QStringLiteral("gridLayout_14"));
        doubleSpinBox_ORBScaleFactor = new QDoubleSpinBox(groupBox_detector_orb2);
        doubleSpinBox_ORBScaleFactor->setObjectName(QStringLiteral("doubleSpinBox_ORBScaleFactor"));
        doubleSpinBox_ORBScaleFactor->setDecimals(1);
        doubleSpinBox_ORBScaleFactor->setValue(1.2);

        gridLayout_14->addWidget(doubleSpinBox_ORBScaleFactor, 0, 0, 1, 1);

        label_32 = new QLabel(groupBox_detector_orb2);
        label_32->setObjectName(QStringLiteral("label_32"));
        label_32->setWordWrap(true);
        label_32->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_14->addWidget(label_32, 0, 1, 1, 1);

        spinBox_ORBNLevels = new QSpinBox(groupBox_detector_orb2);
        spinBox_ORBNLevels->setObjectName(QStringLiteral("spinBox_ORBNLevels"));
        spinBox_ORBNLevels->setValue(8);

        gridLayout_14->addWidget(spinBox_ORBNLevels, 1, 0, 1, 1);

        label_43 = new QLabel(groupBox_detector_orb2);
        label_43->setObjectName(QStringLiteral("label_43"));
        label_43->setWordWrap(true);
        label_43->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_14->addWidget(label_43, 1, 1, 1, 1);

        spinBox_ORBEdgeThreshold = new QSpinBox(groupBox_detector_orb2);
        spinBox_ORBEdgeThreshold->setObjectName(QStringLiteral("spinBox_ORBEdgeThreshold"));
        spinBox_ORBEdgeThreshold->setValue(31);

        gridLayout_14->addWidget(spinBox_ORBEdgeThreshold, 2, 0, 1, 1);

        label_95 = new QLabel(groupBox_detector_orb2);
        label_95->setObjectName(QStringLiteral("label_95"));
        label_95->setWordWrap(true);
        label_95->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_14->addWidget(label_95, 2, 1, 1, 1);

        spinBox_ORBFirstLevel = new QSpinBox(groupBox_detector_orb2);
        spinBox_ORBFirstLevel->setObjectName(QStringLiteral("spinBox_ORBFirstLevel"));
        spinBox_ORBFirstLevel->setMaximum(0);

        gridLayout_14->addWidget(spinBox_ORBFirstLevel, 3, 0, 1, 1);

        label_127 = new QLabel(groupBox_detector_orb2);
        label_127->setObjectName(QStringLiteral("label_127"));
        label_127->setWordWrap(true);
        label_127->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_14->addWidget(label_127, 3, 1, 1, 1);

        spinBox_ORBWTA_K = new QSpinBox(groupBox_detector_orb2);
        spinBox_ORBWTA_K->setObjectName(QStringLiteral("spinBox_ORBWTA_K"));
        spinBox_ORBWTA_K->setValue(2);

        gridLayout_14->addWidget(spinBox_ORBWTA_K, 4, 0, 1, 1);

        label_128 = new QLabel(groupBox_detector_orb2);
        label_128->setObjectName(QStringLiteral("label_128"));
        label_128->setWordWrap(true);
        label_128->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_14->addWidget(label_128, 4, 1, 1, 1);

        spinBox_ORBScoreType = new QSpinBox(groupBox_detector_orb2);
        spinBox_ORBScoreType->setObjectName(QStringLiteral("spinBox_ORBScoreType"));

        gridLayout_14->addWidget(spinBox_ORBScoreType, 5, 0, 1, 1);

        label_140 = new QLabel(groupBox_detector_orb2);
        label_140->setObjectName(QStringLiteral("label_140"));
        label_140->setWordWrap(true);
        label_140->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_14->addWidget(label_140, 5, 1, 1, 1);

        spinBox_ORBPatchSize = new QSpinBox(groupBox_detector_orb2);
        spinBox_ORBPatchSize->setObjectName(QStringLiteral("spinBox_ORBPatchSize"));
        spinBox_ORBPatchSize->setValue(31);

        gridLayout_14->addWidget(spinBox_ORBPatchSize, 6, 0, 1, 1);

        label_147 = new QLabel(groupBox_detector_orb2);
        label_147->setObjectName(QStringLiteral("label_147"));
        label_147->setWordWrap(true);
        label_147->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_14->addWidget(label_147, 6, 1, 1, 1);

        checkBox_ORBGpu = new QCheckBox(groupBox_detector_orb2);
        checkBox_ORBGpu->setObjectName(QStringLiteral("checkBox_ORBGpu"));

        gridLayout_14->addWidget(checkBox_ORBGpu, 7, 0, 1, 1);

        label_orbGpu = new QLabel(groupBox_detector_orb2);
        label_orbGpu->setObjectName(QStringLiteral("label_orbGpu"));
        label_orbGpu->setWordWrap(true);
        label_orbGpu->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_14->addWidget(label_orbGpu, 7, 1, 1, 1);

        gridLayout_14->setColumnStretch(1, 1);

        verticalLayout_35->addWidget(groupBox_detector_orb2);

        verticalSpacer_18 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_35->addItem(verticalSpacer_18);

        stackedWidget->addWidget(page_18);
        page_11 = new QWidget();
        page_11->setObjectName(QStringLiteral("page_11"));
        verticalLayout_36 = new QVBoxLayout(page_11);
        verticalLayout_36->setObjectName(QStringLiteral("verticalLayout_36"));
        groupBox_detector_freak2 = new QGroupBox(page_11);
        groupBox_detector_freak2->setObjectName(QStringLiteral("groupBox_detector_freak2"));
        gridLayout_13 = new QGridLayout(groupBox_detector_freak2);
        gridLayout_13->setObjectName(QStringLiteral("gridLayout_13"));
        checkBox_FREAKOrientationNormalized = new QCheckBox(groupBox_detector_freak2);
        checkBox_FREAKOrientationNormalized->setObjectName(QStringLiteral("checkBox_FREAKOrientationNormalized"));
        checkBox_FREAKOrientationNormalized->setChecked(true);

        gridLayout_13->addWidget(checkBox_FREAKOrientationNormalized, 0, 0, 1, 1);

        label_6 = new QLabel(groupBox_detector_freak2);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setWordWrap(true);
        label_6->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_13->addWidget(label_6, 0, 1, 1, 1);

        checkBox_FREAKScaleNormalized = new QCheckBox(groupBox_detector_freak2);
        checkBox_FREAKScaleNormalized->setObjectName(QStringLiteral("checkBox_FREAKScaleNormalized"));
        checkBox_FREAKScaleNormalized->setChecked(true);

        gridLayout_13->addWidget(checkBox_FREAKScaleNormalized, 1, 0, 1, 1);

        label_9 = new QLabel(groupBox_detector_freak2);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setWordWrap(true);
        label_9->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_13->addWidget(label_9, 1, 1, 1, 1);

        doubleSpinBox_FREAKPatternScale = new QDoubleSpinBox(groupBox_detector_freak2);
        doubleSpinBox_FREAKPatternScale->setObjectName(QStringLiteral("doubleSpinBox_FREAKPatternScale"));
        doubleSpinBox_FREAKPatternScale->setValue(22);

        gridLayout_13->addWidget(doubleSpinBox_FREAKPatternScale, 2, 0, 1, 1);

        label_13 = new QLabel(groupBox_detector_freak2);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setWordWrap(true);
        label_13->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_13->addWidget(label_13, 2, 1, 1, 1);

        spinBox_FREAKNOctaves = new QSpinBox(groupBox_detector_freak2);
        spinBox_FREAKNOctaves->setObjectName(QStringLiteral("spinBox_FREAKNOctaves"));
        spinBox_FREAKNOctaves->setValue(4);

        gridLayout_13->addWidget(spinBox_FREAKNOctaves, 3, 0, 1, 1);

        label_23 = new QLabel(groupBox_detector_freak2);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setWordWrap(true);
        label_23->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_13->addWidget(label_23, 3, 1, 1, 1);

        gridLayout_13->setColumnStretch(1, 1);

        verticalLayout_36->addWidget(groupBox_detector_freak2);

        verticalSpacer_13 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_36->addItem(verticalSpacer_13);

        stackedWidget->addWidget(page_11);
        page_16 = new QWidget();
        page_16->setObjectName(QStringLiteral("page_16"));
        verticalLayout_9 = new QVBoxLayout(page_16);
        verticalLayout_9->setObjectName(QStringLiteral("verticalLayout_9"));
        groupBox_detector_gftt2 = new QGroupBox(page_16);
        groupBox_detector_gftt2->setObjectName(QStringLiteral("groupBox_detector_gftt2"));
        gridLayout_21 = new QGridLayout(groupBox_detector_gftt2);
        gridLayout_21->setObjectName(QStringLiteral("gridLayout_21"));
        doubleSpinBox_GFTT_qualityLevel = new QDoubleSpinBox(groupBox_detector_gftt2);
        doubleSpinBox_GFTT_qualityLevel->setObjectName(QStringLiteral("doubleSpinBox_GFTT_qualityLevel"));
        doubleSpinBox_GFTT_qualityLevel->setDecimals(4);
        doubleSpinBox_GFTT_qualityLevel->setMinimum(0.0001);
        doubleSpinBox_GFTT_qualityLevel->setMaximum(1);
        doubleSpinBox_GFTT_qualityLevel->setSingleStep(0.01);
        doubleSpinBox_GFTT_qualityLevel->setValue(0.01);

        gridLayout_21->addWidget(doubleSpinBox_GFTT_qualityLevel, 0, 0, 1, 1);

        doubleSpinBox_GFTT_minDistance = new QDoubleSpinBox(groupBox_detector_gftt2);
        doubleSpinBox_GFTT_minDistance->setObjectName(QStringLiteral("doubleSpinBox_GFTT_minDistance"));
        doubleSpinBox_GFTT_minDistance->setDecimals(1);
        doubleSpinBox_GFTT_minDistance->setValue(1);

        gridLayout_21->addWidget(doubleSpinBox_GFTT_minDistance, 1, 0, 1, 1);

        checkBox_GFTT_useHarrisDetector = new QCheckBox(groupBox_detector_gftt2);
        checkBox_GFTT_useHarrisDetector->setObjectName(QStringLiteral("checkBox_GFTT_useHarrisDetector"));
        checkBox_GFTT_useHarrisDetector->setChecked(true);

        gridLayout_21->addWidget(checkBox_GFTT_useHarrisDetector, 3, 0, 1, 1);

        label_172 = new QLabel(groupBox_detector_gftt2);
        label_172->setObjectName(QStringLiteral("label_172"));
        label_172->setWordWrap(true);
        label_172->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_21->addWidget(label_172, 3, 1, 1, 1);

        doubleSpinBox_GFTT_k = new QDoubleSpinBox(groupBox_detector_gftt2);
        doubleSpinBox_GFTT_k->setObjectName(QStringLiteral("doubleSpinBox_GFTT_k"));
        doubleSpinBox_GFTT_k->setMaximum(1);
        doubleSpinBox_GFTT_k->setSingleStep(0.01);
        doubleSpinBox_GFTT_k->setValue(0.04);

        gridLayout_21->addWidget(doubleSpinBox_GFTT_k, 4, 0, 1, 1);

        label_173 = new QLabel(groupBox_detector_gftt2);
        label_173->setObjectName(QStringLiteral("label_173"));
        label_173->setWordWrap(true);
        label_173->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_21->addWidget(label_173, 4, 1, 1, 1);

        spinBox_GFTT_blockSize = new QSpinBox(groupBox_detector_gftt2);
        spinBox_GFTT_blockSize->setObjectName(QStringLiteral("spinBox_GFTT_blockSize"));
        spinBox_GFTT_blockSize->setValue(3);

        gridLayout_21->addWidget(spinBox_GFTT_blockSize, 2, 0, 1, 1);

        label_175 = new QLabel(groupBox_detector_gftt2);
        label_175->setObjectName(QStringLiteral("label_175"));
        label_175->setWordWrap(true);
        label_175->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_21->addWidget(label_175, 0, 1, 1, 1);

        label_176 = new QLabel(groupBox_detector_gftt2);
        label_176->setObjectName(QStringLiteral("label_176"));
        label_176->setWordWrap(true);
        label_176->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_21->addWidget(label_176, 1, 1, 1, 1);

        label_177 = new QLabel(groupBox_detector_gftt2);
        label_177->setObjectName(QStringLiteral("label_177"));
        label_177->setWordWrap(true);
        label_177->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_21->addWidget(label_177, 2, 1, 1, 1);

        gridLayout_21->setColumnStretch(1, 1);

        verticalLayout_9->addWidget(groupBox_detector_gftt2);

        verticalSpacer_25 = new QSpacerItem(20, 644, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_9->addItem(verticalSpacer_25);

        stackedWidget->addWidget(page_16);
        page_25 = new QWidget();
        page_25->setObjectName(QStringLiteral("page_25"));
        verticalLayout_47 = new QVBoxLayout(page_25);
        verticalLayout_47->setObjectName(QStringLiteral("verticalLayout_47"));
        groupBox_detector_brisk2 = new QGroupBox(page_25);
        groupBox_detector_brisk2->setObjectName(QStringLiteral("groupBox_detector_brisk2"));
        gridLayout_26 = new QGridLayout(groupBox_detector_brisk2);
        gridLayout_26->setObjectName(QStringLiteral("gridLayout_26"));
        label_191 = new QLabel(groupBox_detector_brisk2);
        label_191->setObjectName(QStringLiteral("label_191"));
        label_191->setWordWrap(true);
        label_191->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_26->addWidget(label_191, 0, 1, 1, 1);

        doubleSpinBox_BRISK_patterScale = new QDoubleSpinBox(groupBox_detector_brisk2);
        doubleSpinBox_BRISK_patterScale->setObjectName(QStringLiteral("doubleSpinBox_BRISK_patterScale"));
        doubleSpinBox_BRISK_patterScale->setMaximum(10);
        doubleSpinBox_BRISK_patterScale->setSingleStep(0.1);
        doubleSpinBox_BRISK_patterScale->setValue(1);

        gridLayout_26->addWidget(doubleSpinBox_BRISK_patterScale, 2, 0, 1, 1);

        label_188 = new QLabel(groupBox_detector_brisk2);
        label_188->setObjectName(QStringLiteral("label_188"));
        label_188->setWordWrap(true);
        label_188->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_26->addWidget(label_188, 1, 1, 1, 1);

        spinBox_BRISK_octaves = new QSpinBox(groupBox_detector_brisk2);
        spinBox_BRISK_octaves->setObjectName(QStringLiteral("spinBox_BRISK_octaves"));
        spinBox_BRISK_octaves->setMaximum(10);
        spinBox_BRISK_octaves->setValue(3);

        gridLayout_26->addWidget(spinBox_BRISK_octaves, 1, 0, 1, 1);

        label_189 = new QLabel(groupBox_detector_brisk2);
        label_189->setObjectName(QStringLiteral("label_189"));
        label_189->setWordWrap(true);
        label_189->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_26->addWidget(label_189, 2, 1, 1, 1);

        spinBox_BRISK_thresh = new QSpinBox(groupBox_detector_brisk2);
        spinBox_BRISK_thresh->setObjectName(QStringLiteral("spinBox_BRISK_thresh"));
        spinBox_BRISK_thresh->setMaximum(9999);
        spinBox_BRISK_thresh->setValue(30);

        gridLayout_26->addWidget(spinBox_BRISK_thresh, 0, 0, 1, 1);

        gridLayout_26->setColumnStretch(1, 1);

        verticalLayout_47->addWidget(groupBox_detector_brisk2);

        verticalSpacer_26 = new QSpacerItem(20, 742, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_47->addItem(verticalSpacer_26);

        stackedWidget->addWidget(page_25);
        page_71 = new QWidget();
        page_71->setObjectName(QStringLiteral("page_71"));
        verticalLayout_3 = new QVBoxLayout(page_71);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        groupBox_detector_kaze2 = new QGroupBox(page_71);
        groupBox_detector_kaze2->setObjectName(QStringLiteral("groupBox_detector_kaze2"));
        gridLayout_95 = new QGridLayout(groupBox_detector_kaze2);
        gridLayout_95->setObjectName(QStringLiteral("gridLayout_95"));
        checkBox_kaze_extended = new QCheckBox(groupBox_detector_kaze2);
        checkBox_kaze_extended->setObjectName(QStringLiteral("checkBox_kaze_extended"));

        gridLayout_95->addWidget(checkBox_kaze_extended, 0, 0, 1, 1);

        label_420 = new QLabel(groupBox_detector_kaze2);
        label_420->setObjectName(QStringLiteral("label_420"));
        label_420->setWordWrap(true);
        label_420->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_95->addWidget(label_420, 1, 1, 1, 1);

        label_416 = new QLabel(groupBox_detector_kaze2);
        label_416->setObjectName(QStringLiteral("label_416"));
        label_416->setWordWrap(true);
        label_416->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_95->addWidget(label_416, 3, 1, 1, 1);

        checkBox_kaze_upright = new QCheckBox(groupBox_detector_kaze2);
        checkBox_kaze_upright->setObjectName(QStringLiteral("checkBox_kaze_upright"));

        gridLayout_95->addWidget(checkBox_kaze_upright, 1, 0, 1, 1);

        label_417 = new QLabel(groupBox_detector_kaze2);
        label_417->setObjectName(QStringLiteral("label_417"));
        label_417->setWordWrap(true);
        label_417->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_95->addWidget(label_417, 4, 1, 1, 1);

        spinBox_kaze_octavelayers = new QSpinBox(groupBox_detector_kaze2);
        spinBox_kaze_octavelayers->setObjectName(QStringLiteral("spinBox_kaze_octavelayers"));
        spinBox_kaze_octavelayers->setMinimum(1);
        spinBox_kaze_octavelayers->setMaximum(16);
        spinBox_kaze_octavelayers->setValue(3);

        gridLayout_95->addWidget(spinBox_kaze_octavelayers, 4, 0, 1, 1);

        spinBox_kaze_octaves = new QSpinBox(groupBox_detector_kaze2);
        spinBox_kaze_octaves->setObjectName(QStringLiteral("spinBox_kaze_octaves"));
        spinBox_kaze_octaves->setMinimum(1);
        spinBox_kaze_octaves->setMaximum(16);
        spinBox_kaze_octaves->setValue(16);

        gridLayout_95->addWidget(spinBox_kaze_octaves, 3, 0, 1, 1);

        label_419 = new QLabel(groupBox_detector_kaze2);
        label_419->setObjectName(QStringLiteral("label_419"));
        label_419->setWordWrap(true);
        label_419->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_95->addWidget(label_419, 0, 1, 1, 1);

        doubleSpinBox_kaze_threshold = new QDoubleSpinBox(groupBox_detector_kaze2);
        doubleSpinBox_kaze_threshold->setObjectName(QStringLiteral("doubleSpinBox_kaze_threshold"));
        doubleSpinBox_kaze_threshold->setDecimals(4);
        doubleSpinBox_kaze_threshold->setMinimum(0.0001);
        doubleSpinBox_kaze_threshold->setMaximum(9999);
        doubleSpinBox_kaze_threshold->setSingleStep(0.001);
        doubleSpinBox_kaze_threshold->setValue(0.001);

        gridLayout_95->addWidget(doubleSpinBox_kaze_threshold, 2, 0, 1, 1);

        label_418 = new QLabel(groupBox_detector_kaze2);
        label_418->setObjectName(QStringLiteral("label_418"));
        label_418->setWordWrap(true);
        label_418->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_95->addWidget(label_418, 2, 1, 1, 1);

        label_421 = new QLabel(groupBox_detector_kaze2);
        label_421->setObjectName(QStringLiteral("label_421"));
        label_421->setWordWrap(true);
        label_421->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_95->addWidget(label_421, 5, 1, 1, 1);

        spinBox_kaze_diffusivity = new QSpinBox(groupBox_detector_kaze2);
        spinBox_kaze_diffusivity->setObjectName(QStringLiteral("spinBox_kaze_diffusivity"));
        spinBox_kaze_diffusivity->setMaximum(3);
        spinBox_kaze_diffusivity->setValue(3);

        gridLayout_95->addWidget(spinBox_kaze_diffusivity, 5, 0, 1, 1);

        gridLayout_95->setColumnStretch(1, 1);

        verticalLayout_3->addWidget(groupBox_detector_kaze2);

        verticalSpacer_65 = new QSpacerItem(20, 2487, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_65);

        stackedWidget->addWidget(page_71);
        page_85 = new QWidget();
        page_85->setObjectName(QStringLiteral("page_85"));
        verticalLayout_146 = new QVBoxLayout(page_85);
        verticalLayout_146->setObjectName(QStringLiteral("verticalLayout_146"));
        groupBox_detector_superpoint_torch2 = new QGroupBox(page_85);
        groupBox_detector_superpoint_torch2->setObjectName(QStringLiteral("groupBox_detector_superpoint_torch2"));
        verticalLayout_147 = new QVBoxLayout(groupBox_detector_superpoint_torch2);
        verticalLayout_147->setObjectName(QStringLiteral("verticalLayout_147"));
        label_575 = new QLabel(groupBox_detector_superpoint_torch2);
        label_575->setObjectName(QStringLiteral("label_575"));
        label_575->setWordWrap(true);
        label_575->setOpenExternalLinks(true);
        label_575->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_147->addWidget(label_575);

        gridLayout_114 = new QGridLayout();
        gridLayout_114->setObjectName(QStringLiteral("gridLayout_114"));
        toolButton_sptorch_path = new QToolButton(groupBox_detector_superpoint_torch2);
        toolButton_sptorch_path->setObjectName(QStringLiteral("toolButton_sptorch_path"));

        gridLayout_114->addWidget(toolButton_sptorch_path, 0, 0, 1, 1);

        lineEdit_sptorch_path = new QLineEdit(groupBox_detector_superpoint_torch2);
        lineEdit_sptorch_path->setObjectName(QStringLiteral("lineEdit_sptorch_path"));

        gridLayout_114->addWidget(lineEdit_sptorch_path, 0, 1, 1, 1);

        label_579 = new QLabel(groupBox_detector_superpoint_torch2);
        label_579->setObjectName(QStringLiteral("label_579"));
        label_579->setWordWrap(true);
        label_579->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_114->addWidget(label_579, 0, 2, 1, 1);

        doubleSpinBox_sptorch_threshold = new QDoubleSpinBox(groupBox_detector_superpoint_torch2);
        doubleSpinBox_sptorch_threshold->setObjectName(QStringLiteral("doubleSpinBox_sptorch_threshold"));
        doubleSpinBox_sptorch_threshold->setDecimals(4);
        doubleSpinBox_sptorch_threshold->setMinimum(0.0001);
        doubleSpinBox_sptorch_threshold->setMaximum(1);
        doubleSpinBox_sptorch_threshold->setSingleStep(0.001);
        doubleSpinBox_sptorch_threshold->setValue(0.001);

        gridLayout_114->addWidget(doubleSpinBox_sptorch_threshold, 1, 1, 1, 1);

        label_578 = new QLabel(groupBox_detector_superpoint_torch2);
        label_578->setObjectName(QStringLiteral("label_578"));
        label_578->setWordWrap(true);
        label_578->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_114->addWidget(label_578, 1, 2, 1, 1);

        checkBox_sptorch_nms = new QCheckBox(groupBox_detector_superpoint_torch2);
        checkBox_sptorch_nms->setObjectName(QStringLiteral("checkBox_sptorch_nms"));

        gridLayout_114->addWidget(checkBox_sptorch_nms, 2, 1, 1, 1);

        label_577 = new QLabel(groupBox_detector_superpoint_torch2);
        label_577->setObjectName(QStringLiteral("label_577"));
        label_577->setWordWrap(true);
        label_577->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_114->addWidget(label_577, 2, 2, 1, 1);

        checkBox_sptorch_cuda = new QCheckBox(groupBox_detector_superpoint_torch2);
        checkBox_sptorch_cuda->setObjectName(QStringLiteral("checkBox_sptorch_cuda"));

        gridLayout_114->addWidget(checkBox_sptorch_cuda, 4, 1, 1, 1);

        label_574 = new QLabel(groupBox_detector_superpoint_torch2);
        label_574->setObjectName(QStringLiteral("label_574"));
        label_574->setWordWrap(true);
        label_574->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_114->addWidget(label_574, 4, 2, 1, 1);

        label_580 = new QLabel(groupBox_detector_superpoint_torch2);
        label_580->setObjectName(QStringLiteral("label_580"));
        label_580->setWordWrap(true);
        label_580->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_114->addWidget(label_580, 3, 2, 1, 1);

        spinBox_sptorch_minDistance = new QSpinBox(groupBox_detector_superpoint_torch2);
        spinBox_sptorch_minDistance->setObjectName(QStringLiteral("spinBox_sptorch_minDistance"));

        gridLayout_114->addWidget(spinBox_sptorch_minDistance, 3, 1, 1, 1);

        gridLayout_114->setColumnStretch(2, 1);

        verticalLayout_147->addLayout(gridLayout_114);


        verticalLayout_146->addWidget(groupBox_detector_superpoint_torch2);

        verticalSpacer_84 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_146->addItem(verticalSpacer_84);

        stackedWidget->addWidget(page_85);
        page_87 = new QWidget();
        page_87->setObjectName(QStringLiteral("page_87"));
        verticalLayout_153 = new QVBoxLayout(page_87);
        verticalLayout_153->setObjectName(QStringLiteral("verticalLayout_153"));
        groupBox_pydetector2 = new QGroupBox(page_87);
        groupBox_pydetector2->setObjectName(QStringLiteral("groupBox_pydetector2"));
        verticalLayout_152 = new QVBoxLayout(groupBox_pydetector2);
        verticalLayout_152->setObjectName(QStringLiteral("verticalLayout_152"));
        label_614 = new QLabel(groupBox_pydetector2);
        label_614->setObjectName(QStringLiteral("label_614"));
        label_614->setWordWrap(true);
        label_614->setOpenExternalLinks(true);
        label_614->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        verticalLayout_152->addWidget(label_614);

        gridLayout_118 = new QGridLayout();
        gridLayout_118->setObjectName(QStringLiteral("gridLayout_118"));
        label_615 = new QLabel(groupBox_pydetector2);
        label_615->setObjectName(QStringLiteral("label_615"));
        label_615->setWordWrap(true);
        label_615->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_118->addWidget(label_615, 0, 2, 1, 1);

        lineEdit_pydetector_path = new QLineEdit(groupBox_pydetector2);
        lineEdit_pydetector_path->setObjectName(QStringLiteral("lineEdit_pydetector_path"));

        gridLayout_118->addWidget(lineEdit_pydetector_path, 0, 1, 1, 1);

        toolButton_pydetector_path = new QToolButton(groupBox_pydetector2);
        toolButton_pydetector_path->setObjectName(QStringLiteral("toolButton_pydetector_path"));

        gridLayout_118->addWidget(toolButton_pydetector_path, 0, 0, 1, 1);

        checkBox_pydetector_cuda = new QCheckBox(groupBox_pydetector2);
        checkBox_pydetector_cuda->setObjectName(QStringLiteral("checkBox_pydetector_cuda"));

        gridLayout_118->addWidget(checkBox_pydetector_cuda, 1, 1, 1, 1);

        label_618 = new QLabel(groupBox_pydetector2);
        label_618->setObjectName(QStringLiteral("label_618"));
        label_618->setWordWrap(true);
        label_618->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_118->addWidget(label_618, 1, 2, 1, 1);

        gridLayout_118->setColumnStretch(2, 1);

        verticalLayout_152->addLayout(gridLayout_118);


        verticalLayout_153->addWidget(groupBox_pydetector2);

        verticalSpacer_88 = new QSpacerItem(20, 3128, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_153->addItem(verticalSpacer_88);

        stackedWidget->addWidget(page_87);

        verticalLayout_24->addWidget(stackedWidget);


        verticalLayout_16->addLayout(verticalLayout_24);

        scrollArea->setWidget(scrollAreaWidgetContents);

        verticalLayout_17->addWidget(scrollArea);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        horizontalLayout->setContentsMargins(12, -1, 12, -1);
        buttonBox_local = new QDialogButtonBox(preferencesDialog);
        buttonBox_local->setObjectName(QStringLiteral("buttonBox_local"));
        buttonBox_local->setOrientation(Qt::Horizontal);
        buttonBox_local->setStandardButtons(QDialogButtonBox::Apply|QDialogButtonBox::RestoreDefaults);

        horizontalLayout->addWidget(buttonBox_local);


        verticalLayout_17->addLayout(horizontalLayout);


        horizontalLayout_2->addLayout(verticalLayout_17);

        horizontalLayout_2->setStretch(1, 1);

        gridLayout_49->addLayout(horizontalLayout_2, 0, 0, 1, 1);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        radioButton_basic = new QRadioButton(preferencesDialog);
        radioButton_basic->setObjectName(QStringLiteral("radioButton_basic"));
        radioButton_basic->setChecked(true);

        horizontalLayout_3->addWidget(radioButton_basic);

        radioButton_advanced = new QRadioButton(preferencesDialog);
        radioButton_advanced->setObjectName(QStringLiteral("radioButton_advanced"));

        horizontalLayout_3->addWidget(radioButton_advanced);

        buttonBox_global = new QDialogButtonBox(preferencesDialog);
        buttonBox_global->setObjectName(QStringLiteral("buttonBox_global"));
        buttonBox_global->setOrientation(Qt::Horizontal);
        buttonBox_global->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        horizontalLayout_3->addWidget(buttonBox_global);


        gridLayout_49->addLayout(horizontalLayout_3, 1, 0, 1, 1);


        retranslateUi(preferencesDialog);

        stackedWidget->setCurrentIndex(22);
        comboBox_loggerLevel->setCurrentIndex(2);
        comboBox_loggerEventLevel->setCurrentIndex(3);
        comboBox_loggerPauseLevel->setCurrentIndex(3);
        comboBox_loggerType->setCurrentIndex(1);
        comboBox_loggerFilter->setCurrentIndex(-1);
        stackedWidget_src->setCurrentIndex(1);
        stackedWidget_rgbd->setCurrentIndex(9);
        stackedWidget_stereo->setCurrentIndex(10);
        comboBox_depthai_resolution->setCurrentIndex(1);
        source_comboBox_image_type->setCurrentIndex(0);
        stackedWidget_image->setCurrentIndex(0);
        stackedWidget_imuFilter->setCurrentIndex(2);
        comboBox_dbSynchronous->setCurrentIndex(2);
        odom_registration->setCurrentIndex(3);
        stackedWidget_odometryType->setCurrentIndex(5);
        stackedWidget_odometryFiltering->setCurrentIndex(1);
        reextract_nn->setCurrentIndex(0);
        stackedWidget_loopClosureEstimation->setCurrentIndex(2);
        vis_feature_detector->setCurrentIndex(0);
        stackedWidget_icpStrategy->setCurrentIndex(2);
        stackedWidget_stereoDense->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(preferencesDialog);
    } // setupUi

    void retranslateUi(QDialog *preferencesDialog)
    {
        preferencesDialog->setWindowTitle(QApplication::translate("preferencesDialog", "Preferences", Q_NULLPTR));
        groupBox_generalSettingsGui0->setTitle(QApplication::translate("preferencesDialog", "General Settings (GUI)", Q_NULLPTR));
        checkBox_beep->setText(QString());
        checkBox_notifyWhenNewGlobalPathIsReceived->setText(QString());
        label_247->setText(QApplication::translate("preferencesDialog", "Use time for figures' x-axis. Otherwise, sensor data IDs are used for camera/odometry info and for all mapping statistics, node IDs are used.", Q_NULLPTR));
        label_313->setText(QApplication::translate("preferencesDialog", "Insert created clouds in the GUI cache to avoid regenerating clouds on export.", Q_NULLPTR));
        label_102->setText(QApplication::translate("preferencesDialog", "Notify when a new global path is received.", Q_NULLPTR));
        general_checkBox_imagesKept->setText(QString());
        general_checkBox_cloudsKept->setText(QString());
        checkBox_stamps->setText(QString());
        label_342->setText(QApplication::translate("preferencesDialog", "Cache statistics.", Q_NULLPTR));
        label_88->setText(QApplication::translate("preferencesDialog", "Beep! on special events (finished processing the data set, an error has occured, ...).", Q_NULLPTR));
        label_59->setText(QApplication::translate("preferencesDialog", "Insert new data received in the GUI cache. Used to show the loop closure image and the 3D Map.", Q_NULLPTR));
        checkbox_odomDisabled->setText(QString());
        label_239->setText(QApplication::translate("preferencesDialog", "Disable odometry. This option only makes sense in localization mode with a pre-built map and that odometry cannot be computed from the source selected (e.g. RGB source).", Q_NULLPTR));
        checkBox_cacheStatistics->setText(QString());
        label_347->setText(QApplication::translate("preferencesDialog", "When a ground truth is provided, align it with the map.", Q_NULLPTR));
        checkbox_groundTruthAlign->setText(QString());
        groupBox_2->setTitle(QApplication::translate("preferencesDialog", "Save/Load Settings", Q_NULLPTR));
        pushButton_loadConfig->setText(QApplication::translate("preferencesDialog", "Load settings (*.ini) ...", Q_NULLPTR));
        pushButton_saveConfig->setText(QApplication::translate("preferencesDialog", "Save settings (*.ini) ...", Q_NULLPTR));
        pushButton_resetConfig->setText(QApplication::translate("preferencesDialog", "Reset all settings", Q_NULLPTR));
        groupBox_5->setTitle(QApplication::translate("preferencesDialog", "Loop Closure Detection", Q_NULLPTR));
        checkBox_imageRejectedShown->setText(QString());
        label_105->setText(QApplication::translate("preferencesDialog", "Show the image of the highest hypothesis.", Q_NULLPTR));
        checkBox_imageHighestHypShown->setText(QString());
        label_73->setText(QApplication::translate("preferencesDialog", "Show image when an hypothesis is rejected.", Q_NULLPTR));
        label_70->setText(QApplication::translate("preferencesDialog", "Vertical layout.", Q_NULLPTR));
        checkBox_verticalLayoutUsed->setText(QString());
        groupBox_29->setTitle(QApplication::translate("preferencesDialog", "Graph View", Q_NULLPTR));
        label_222->setText(QApplication::translate("preferencesDialog", "Show loop closure hypotheses on the graph view. Nodes are colorized from blue to red depending on the highest hypothesis (which is red). Posterior hypotheses should be published. ", Q_NULLPTR));
        radioButton_wordsGraphView->setText(QString());
        radioButton_posteriorGraphView->setText(QString());
        label_478->setText(QApplication::translate("preferencesDialog", "Show 3D visual words count on the graph view. Nodes are colorized from blue to red depending on the maximum visual features extracted (which is red).", Q_NULLPTR));
        radioButton_nochangeGraphView->setText(QString());
        label_479->setText(QApplication::translate("preferencesDialog", "No change of graph color.", Q_NULLPTR));
        label_481->setText(QApplication::translate("preferencesDialog", "Show nodes on which loop closures or localizations happened in red.", Q_NULLPTR));
        radioButton_localizationsGraphView->setText(QString());
        groupBox_21->setTitle(QApplication::translate("preferencesDialog", "Odometry", Q_NULLPTR));
        label_369->setText(QApplication::translate("preferencesDialog", "Show only inliers.", Q_NULLPTR));
        checkBox_odom_onlyInliersShown->setText(QString());
        label_159->setText(QApplication::translate("preferencesDialog", "Warning theshold:\n"
"Show a yellow background when the number of odometry inliers goes under this threshold. If 0, it is ignored. You can see the current odometry inliers count under Statistics view -> General -> Odom inliers.", Q_NULLPTR));
        groupBox_cloudRendering1->setTitle(QApplication::translate("preferencesDialog", "3D Rendering", Q_NULLPTR));
        groupBox_12->setTitle(QApplication::translate("preferencesDialog", "Point Cloud", Q_NULLPTR));
        label_355->setText(QApplication::translate("preferencesDialog", "Floor filtering height (0=Disabled). This is done in /map frame.", Q_NULLPTR));
        label_169->setText(QApplication::translate("preferencesDialog", "Noise filtering min neighbors.", Q_NULLPTR));
        label_459->setText(QApplication::translate("preferencesDialog", "Default color scheme key.", Q_NULLPTR));
        doubleSpinBox_maxDepth_odom->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_154->setText(QApplication::translate("preferencesDialog", "Map", Q_NULLPTR));
        label_137->setText(QApplication::translate("preferencesDialog", "Odometry", Q_NULLPTR));
        checkBox_showClouds->setText(QString());
        checkBox_showOdomClouds->setText(QString());
        label_119->setText(QApplication::translate("preferencesDialog", "Show 3D clouds.", Q_NULLPTR));
        label_108->setText(QApplication::translate("preferencesDialog", "Decimation (1-2-4-8-...). Negative decimation is done from RGB size instead of depth size (if depth is smaller than RGB, it may be interpolated depending of the decimation value).", Q_NULLPTR));
        doubleSpinBox_opacity->setSuffix(QString());
        label_155->setText(QApplication::translate("preferencesDialog", "Opacity.", Q_NULLPTR));
        label_157->setText(QApplication::translate("preferencesDialog", "Point size (1..64).", Q_NULLPTR));
        doubleSpinBox_noiseRadius->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        doubleSpinBox_maxDepth->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_132->setText(QApplication::translate("preferencesDialog", "Maximum depth (0 means no limit).", Q_NULLPTR));
        doubleSpinBox_minDepth->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        doubleSpinBox_minDepth_odom->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_150->setText(QApplication::translate("preferencesDialog", "Minimum depth.", Q_NULLPTR));
        label_353->setText(QApplication::translate("preferencesDialog", "ROI ratios [left right top bottom] between 0 and 1.", Q_NULLPTR));
        doubleSpinBox_floorFilterHeight->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        doubleSpinBox_voxel->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_168->setText(QApplication::translate("preferencesDialog", "Noise filtering radius (0=disabled). Done after voxel filtering.", Q_NULLPTR));
        label_338->setText(QApplication::translate("preferencesDialog", "Ceiling filtering height (0=Disabled). This is done in /map frame.", Q_NULLPTR));
        label_171->setText(QApplication::translate("preferencesDialog", "Voxel filtering size (0=disabled).", Q_NULLPTR));
        doubleSpinBox_ceilingFilterHeight->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_210->setText(QApplication::translate("preferencesDialog", "Normal K search. If not 0, normals will be computed and added to created cloud for visualization (keys  7, 8 and 9).", Q_NULLPTR));
        label_427->setText(QApplication::translate("preferencesDialog", "Normal radius search. If not 0, normals will be computed and added to created cloud for visualization (keys  7, 8 and 9).", Q_NULLPTR));
        doubleSpinBox_opacity_odom->setSuffix(QString());
        groupBox_organized->setTitle(QApplication::translate("preferencesDialog", "Organized Online Meshing", Q_NULLPTR));
        label_85->setText(QApplication::translate("preferencesDialog", "Angle tolerance", Q_NULLPTR));
        label_quad->setText(QApplication::translate("preferencesDialog", "Quad", Q_NULLPTR));
        spinBox_mesh_triangleSize->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        label_87->setText(QApplication::translate("preferencesDialog", "Triangle size", Q_NULLPTR));
        checkBox_mesh_quad->setText(QString());
        doubleSpinBox_mesh_angleTolerance->setSuffix(QApplication::translate("preferencesDialog", " deg", Q_NULLPTR));
        label_337->setText(QApplication::translate("preferencesDialog", "Texture", Q_NULLPTR));
        checkBox_mesh_texture->setText(QString());
        groupBox_17->setTitle(QApplication::translate("preferencesDialog", "Laser Scan", Q_NULLPTR));
        doubleSpinBox_opacity_odom_scan->setSuffix(QString());
        label_273->setText(QApplication::translate("preferencesDialog", "Scan downsampling step size.", Q_NULLPTR));
        label_110->setText(QApplication::translate("preferencesDialog", "Show scans.", Q_NULLPTR));
        label_271->setText(QApplication::translate("preferencesDialog", "Scan voxel size.", Q_NULLPTR));
        label_356->setText(QApplication::translate("preferencesDialog", "Odometry", Q_NULLPTR));
        label_359->setText(QApplication::translate("preferencesDialog", "Map", Q_NULLPTR));
        doubleSpinBox_ceilingFilterHeight_scan->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_241->setText(QApplication::translate("preferencesDialog", "Normal K search. If not 0, normals will be computed and added to created cloud for visualization (keys  7, 8 and 9).", Q_NULLPTR));
        label_458->setText(QApplication::translate("preferencesDialog", "Default color scheme key.", Q_NULLPTR));
        label_460->setText(QApplication::translate("preferencesDialog", "Maximum range (0 means no limit).", Q_NULLPTR));
        doubleSpinBox_maxRange->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_428->setText(QApplication::translate("preferencesDialog", "Normal radius search. If not 0, normals will be computed and added to created cloud for visualization (keys  7, 8 and 9).", Q_NULLPTR));
        label_158->setText(QApplication::translate("preferencesDialog", "Scan point size (1..64).", Q_NULLPTR));
        doubleSpinBox_voxelSizeScan_odom->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        checkBox_showScans->setText(QString());
        label_156->setText(QApplication::translate("preferencesDialog", "Scan opacity.", Q_NULLPTR));
        checkBox_showOdomScans->setText(QString());
        doubleSpinBox_opacity_scan->setSuffix(QString());
        doubleSpinBox_voxelSizeScan->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_367->setText(QApplication::translate("preferencesDialog", "Ceiling filtering height (0=Disabled). This is done in /map frame.", Q_NULLPTR));
        label_368->setText(QApplication::translate("preferencesDialog", "Floor filtering height (0=Disabled). This is done in /map frame.", Q_NULLPTR));
        doubleSpinBox_floorFilterHeight_scan->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_461->setText(QApplication::translate("preferencesDialog", "Minimum range.", Q_NULLPTR));
        doubleSpinBox_minRange->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        doubleSpinBox_maxRange_odom->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        doubleSpinBox_minRange_odom->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        groupBox_18->setTitle(QApplication::translate("preferencesDialog", "Miscellaneous", Q_NULLPTR));
        label_213->setText(QApplication::translate("preferencesDialog", "Show graphs.", Q_NULLPTR));
        checkBox_showFeatures->setText(QString());
        checkBox_showGraphs->setText(QString());
        checkBox_showLabels->setText(QString());
        checkBox_showOdomFeatures->setText(QString());
        label_123->setText(QApplication::translate("preferencesDialog", "Show 3D features.", Q_NULLPTR));
        label_243->setText(QApplication::translate("preferencesDialog", "Show labels.", Q_NULLPTR));
        label_360->setText(QApplication::translate("preferencesDialog", "Map", Q_NULLPTR));
        label_361->setText(QApplication::translate("preferencesDialog", "Odometry", Q_NULLPTR));
        label_166->setText(QApplication::translate("preferencesDialog", "Feature point size.", Q_NULLPTR));
        label_362->setText(QApplication::translate("preferencesDialog", "Show frustums.", Q_NULLPTR));
        checkBox_showFrustums->setText(QString());
        checkBox_showOdomFrustums->setText(QString());
        label_301->setText(QApplication::translate("preferencesDialog", "Show landmarks.", Q_NULLPTR));
        checkBox_showLandmarks->setText(QString());
        label_528->setText(QApplication::translate("preferencesDialog", "Landmark size. If zero, marker's length parameter is used.", Q_NULLPTR));
        doubleSpinBox_landmarkSize->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_567->setText(QApplication::translate("preferencesDialog", "Show IMU filtered gravity.", Q_NULLPTR));
        label_568->setText(QApplication::translate("preferencesDialog", "Show IMU acceleration.", Q_NULLPTR));
        checkBox_showIMUAcc->setText(QString());
        checkBox_showIMUGravity->setText(QString());
        checkBox_showIMUGravity_odom->setText(QString());
        doubleSpinBox_lengthIMUGravity->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        doubleSpinBox_lengthIMUGravity_odom->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_569->setText(QApplication::translate("preferencesDialog", "IMU filtered gravity length.", Q_NULLPTR));
        label_572->setText(QApplication::translate("preferencesDialog", "Show frames.", Q_NULLPTR));
        checkBox_showFrames->setText(QString());
        groupBox_filtering2->setTitle(QApplication::translate("preferencesDialog", "Node Filtering", Q_NULLPTR));
        label_160->setText(QApplication::translate("preferencesDialog", "For visualization purpose, superposed clouds can be filtered by one of the approaches below.", Q_NULLPTR));
        label_71->setText(QApplication::translate("preferencesDialog", "No filtering.", Q_NULLPTR));
        radioButton_noFiltering->setText(QString());
        radioButton_subtractFiltering->setText(QString());
        doubleSpinBox_cloudFilterRadius->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_37->setText(QApplication::translate("preferencesDialog", "Radius.", Q_NULLPTR));
        doubleSpinBox_cloudFilterAngle->setSuffix(QApplication::translate("preferencesDialog", " deg", Q_NULLPTR));
        label_48->setText(QApplication::translate("preferencesDialog", "Angle.", Q_NULLPTR));
        label_161->setText(QApplication::translate("preferencesDialog", "Cloud subtraction filtering. When a new cloud is added to the map, its points are subtracted from the previous cloud.", Q_NULLPTR));
        label_33->setText(QApplication::translate("preferencesDialog", "Pose filtering. By comparing poses in the same area, only one cloud in a fixed radius and angle is shown.", Q_NULLPTR));
        radioButton_nodeFiltering->setText(QString());
        doubleSpinBox_subtractFilteringRadius->setSuffix(QString());
        label_192->setText(QApplication::translate("preferencesDialog", "Point radius used to find close points between the current cloud and the previous one.", Q_NULLPTR));
        doubleSpinBox_subtractFilteringAngle->setSuffix(QApplication::translate("preferencesDialog", " deg", Q_NULLPTR));
        label_193->setText(QApplication::translate("preferencesDialog", "Point angle used to keep only points with similar normal between the current cloud and the previous one.", Q_NULLPTR));
        label_174->setText(QApplication::translate("preferencesDialog", "Minimum points of the previous cloud in the fixed point radius in order to substract the point in the new cloud. Increasing this value reduces the black contours between clouds.", Q_NULLPTR));
        groupBox_gridMap2->setTitle(QApplication::translate("preferencesDialog", "Grid Map Assembling", Q_NULLPTR));
        label_167->setText(QApplication::translate("preferencesDialog", "When the Graph View is visible or if the \"Show in 3D map view\" below is checked, the grid map is assembled using occupancy grids saved in each node. See Occupancy Grid (advanced) panel for parameters used to create the local occupancy grids. If nodes don't have local occupancy grid created, use the Database Viewer tool to generate them.", Q_NULLPTR));
        checkBox_map_shown->setText(QString());
        label_map_shown->setText(QApplication::translate("preferencesDialog", "Show in 3D map view.", Q_NULLPTR));
        label_170->setText(QApplication::translate("preferencesDialog", "Opacity.", Q_NULLPTR));
        doubleSpinBox_map_opacity->setSuffix(QString());
        groupBox_octomap->setTitle(QApplication::translate("preferencesDialog", "OctoMap (3D Occupancy Grid Map)", Q_NULLPTR));
        checkBox_octomap_show3dMap->setText(QString());
        label_octomap_treeDepth->setText(QApplication::translate("preferencesDialog", "Octomap maximum tree depth (max 16). The highest depth means the smallest resolution of the map (cell size). At smallest resolution the octomap shows RGB colors. Other resolutions produce z-axis gradient colored octomap.", Q_NULLPTR));
        label_octomap_treeDepth_5->setText(QApplication::translate("preferencesDialog", "OctoMap rendering type. Cube rendering requires significant more GPU power.", Q_NULLPTR));
        checkBox_octomap_2dgrid->setText(QString());
        label_octomap_treeDepth_3->setText(QApplication::translate("preferencesDialog", "Show in 3D map view.", Q_NULLPTR));
        label_octomap_treeDepth_6->setText(QApplication::translate("preferencesDialog", "Point size. When cube rendering is disabled.", Q_NULLPTR));
        label_octomap_treeDepth_8->setText(QApplication::translate("preferencesDialog", "2D grid map created from OctoMap.", Q_NULLPTR));
        comboBox_octomap_renderingType->clear();
        comboBox_octomap_renderingType->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Point Cloud", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Cube", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Volume", Q_NULLPTR)
        );
        groupBox_logging1->setTitle(QApplication::translate("preferencesDialog", "Logging", Q_NULLPTR));
        label_60->setText(QApplication::translate("preferencesDialog", "Logger level.", Q_NULLPTR));
        comboBox_loggerLevel->clear();
        comboBox_loggerLevel->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "DEBUG", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "INFO", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "WARNING", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "ERROR", Q_NULLPTR)
        );
        comboBox_loggerEventLevel->clear();
        comboBox_loggerEventLevel->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "DEBUG", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "INFO", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "WARNING", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "ERROR", Q_NULLPTR)
        );
        label_61->setText(QApplication::translate("preferencesDialog", "Logger event sent level (fatal messages are always sent).", Q_NULLPTR));
        comboBox_loggerPauseLevel->clear();
        comboBox_loggerPauseLevel->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "DEBUG", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "INFO", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "WARNING", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "ERROR", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "FATAL", Q_NULLPTR)
        );
        label_62->setText(QApplication::translate("preferencesDialog", "Logger level on which the process will automatically pause, showing the log console. A level set under the logger event level will do nothing.", Q_NULLPTR));
        checkBox_logger_printTime->setText(QString());
        label_67->setText(QApplication::translate("preferencesDialog", "Logger print time.", Q_NULLPTR));
        comboBox_loggerType->clear();
        comboBox_loggerType->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "No log", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Console", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "File", Q_NULLPTR)
        );
        label_68->setText(QApplication::translate("preferencesDialog", "Logger type: \n"
"when using the file type, logs are saved in LogRtabmap.txt (located in the working directory).", Q_NULLPTR));
        label_118->setText(QApplication::translate("preferencesDialog", "Logger print thread ID.", Q_NULLPTR));
        checkBox_logger_printThreadId->setText(QString());
        label_164->setText(QApplication::translate("preferencesDialog", "Logger thread filter. If some threads are selected, only messages from these threads will be printed. If no thread is selected, then all messages are logged. ", Q_NULLPTR));
        groupBox_source0->setTitle(QApplication::translate("preferencesDialog", "Source", Q_NULLPTR));
        label_3->setText(QApplication::translate("preferencesDialog", "Input rate (0 means as fast as possible).", Q_NULLPTR));
        label_42->setText(QApplication::translate("preferencesDialog", "Local transform from /base_link to /camera_link. Mouse over the box to show formats.", Q_NULLPTR));
        label_19->setText(QApplication::translate("preferencesDialog", "Source type. Select specific driver below.", Q_NULLPTR));
        label_39->setText(QApplication::translate("preferencesDialog", "ID of the device, which might be a serial number, bus@address or the index of the device. If empty, the first device found is taken.", Q_NULLPTR));
        label_244->setText(QApplication::translate("preferencesDialog", "Create a calibration file with known intrinsics (fx, fy, cx, cy). Useful if you already know the intrinsics of the source of images.", Q_NULLPTR));
        toolButton_source_path_calibration->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        lineEdit_calibrationFile->setText(QString());
        general_doubleSpinBox_imgRate->setSuffix(QApplication::translate("preferencesDialog", " Hz", Q_NULLPTR));
        label_17->setText(QApplication::translate("preferencesDialog", "Mirroring mode (flip image horizontally). It has no effect on database source.", Q_NULLPTR));
        source_mirroring->setText(QString());
#ifndef QT_NO_TOOLTIP
        lineEdit_sourceLocalTransform->setToolTip(QApplication::translate("preferencesDialog", "<html><head/><body><p>Format (3 values): x y z<br/>Format (6 values): x y z roll pitch yaw<br/>Format (7 values): x y z qx qy qz qw<br/>Format (9 values, 3x3 rotation): r11 r12 r13 r21 r22 r23 r31 r32 r33<br/>Format (12 values, 3x4 transform): r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz</p><p>KITTI: /base_link to /gray_camera = 0 0 1 -1 0 0 0 -1 0<br/>KITTI: /base_link to /color_camera = 0 0 1 0 -1 0 0 -0.06 0 -1 0 0<br/>KITTI: /base_footprint to /gray_camera = 0 0 1 0 -1 0 0 0 0 -1 0 1.67<br/>KITTI: /base_footprint to /color_camera = 0 0 1 0 -1 0 0 -0.06 0 -1 0 1.67</p><p>EuRoC MAV: /base_link to /cam0 = T_BS*T_SC0 = -0.0257742 0.00375623 0.999661 0.00981073 -0.999557 -0.0149672 -0.0257155 0.064677 0.0148655 -0.999881 0.00414038 -0.0216401</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        lineEdit_sourceLocalTransform->setText(QApplication::translate("preferencesDialog", "0 0 1 -1 0 0 0 -1 0", Q_NULLPTR));
        comboBox_sourceType->clear();
        comboBox_sourceType->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "RGB-D", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Stereo", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "RGB", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Database", Q_NULLPTR)
        );
        pushButton_test_camera->setText(QApplication::translate("preferencesDialog", "Test", Q_NULLPTR));
        pushButton_calibrate->setText(QApplication::translate("preferencesDialog", "Calibrate", Q_NULLPTR));
        label_24->setText(QApplication::translate("preferencesDialog", "Calibration files are saved in \"camera_info\" folder of the working directory.", Q_NULLPTR));
        lineEdit_sourceDevice->setText(QString());
        pushButton_calibrate_simple->setText(QApplication::translate("preferencesDialog", "Create Calibration", Q_NULLPTR));
        label_18->setText(QApplication::translate("preferencesDialog", "Calibration file path (*.yaml). If empty, the GUID of the camera is used (for those having one). OpenNI and Freenect drivers use factory calibration by default (so they ignore this parameter). A calibrated camera is required for RGB-D SLAM mode.", Q_NULLPTR));
        label_36->setText(QApplication::translate("preferencesDialog", "Image decimation. RGB/Mono and depth images will be resized according to this value (size*1/decimation). Note that if depth images are captured, decimation should be a multiple of the depth image size. If depth images are smaller than RGB images, the decimation is first applied on RGB, if the resulting RGB image is still bigger than depth image, the depth is not decimated.", Q_NULLPTR));
        label_229->setText(QApplication::translate("preferencesDialog", "Only RGB images (for RGB-D cameras) or left images (for stereo cameras) are published.", Q_NULLPTR));
        checkbox_rgbd_colorOnly->setText(QString());
        groupBox_sourceRGBD->setTitle(QApplication::translate("preferencesDialog", "RGB-D", Q_NULLPTR));
        label_40->setText(QApplication::translate("preferencesDialog", "Grabber for RGB-D devices (i.e., Primesense PSDK, Microsoft Kinect, Asus XTion Pro/Live).", Q_NULLPTR));
        comboBox_cameraRGBD->clear();
        comboBox_cameraRGBD->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "OpenNI-PCL", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Freenect", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "OpenNI-CV", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "OpenNI-CV-ASUS", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "OpenNI2", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Freenect2", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "RealSense", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Images", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Kinect for Windows SDK v2", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "RealSense2", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Kinect for Azure", Q_NULLPTR)
        );
        label_228->setText(QApplication::translate("preferencesDialog", "Driver.", Q_NULLPTR));
        groupBox_openni->setTitle(QApplication::translate("preferencesDialog", "OpenNI", Q_NULLPTR));
        lineEdit_openniOniPath->setText(QString());
        label_245->setText(QApplication::translate("preferencesDialog", "Path to a *.ONI file.", Q_NULLPTR));
        toolButton_openniOniPath->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        groupBox_openni2->setTitle(QApplication::translate("preferencesDialog", "OpenNI 2", Q_NULLPTR));
        label_435->setText(QApplication::translate("preferencesDialog", "IR-Depth horizontal shift. Only used when custom calibration is done.", Q_NULLPTR));
        openni2_autoWhiteBalance->setText(QString());
        label_264->setText(QApplication::translate("preferencesDialog", "Use timestamps and frame IDs from OpenNI2.", Q_NULLPTR));
        openni2_stampsIdsUsed->setText(QString());
        label_231->setText(QApplication::translate("preferencesDialog", "Path to a *.ONI file.", Q_NULLPTR));
        label_220->setText(QApplication::translate("preferencesDialog", "Gain.", Q_NULLPTR));
        lineEdit_openni2OniPath->setText(QString());
        toolButton_openni2OniPath->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        label_217->setText(QApplication::translate("preferencesDialog", "Auto white balance.", Q_NULLPTR));
        label_223->setText(QApplication::translate("preferencesDialog", "Mirroring.", Q_NULLPTR));
        openni2_mirroring->setText(QString());
        openni2_autoExposure->setText(QString());
        label_218->setText(QApplication::translate("preferencesDialog", "Auto exposure.", Q_NULLPTR));
        label_219->setText(QApplication::translate("preferencesDialog", "Exposure.", Q_NULLPTR));
        label_436->setText(QApplication::translate("preferencesDialog", "IR-Depth vertical shift. Only used when custom calibration is done.", Q_NULLPTR));
        groupBox_freenect2->setTitle(QApplication::translate("preferencesDialog", "Freenect2", Q_NULLPTR));
        label_269->setText(QApplication::translate("preferencesDialog", "Depth edge aware filtering.", Q_NULLPTR));
        comboBox_freenect2Format->clear();
        comboBox_freenect2Format->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Registered Color to Depth SD", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Registered Depth to Color SD (old way)", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Registered Depth to Color HD", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Registered Depth to Color HD (old way)", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "IR + Depth", Q_NULLPTR)
        );
        checkBox_freenect2BilateralFiltering->setText(QString());
        label_268->setText(QApplication::translate("preferencesDialog", "Depth bilateral filtering.", Q_NULLPTR));
        checkBox_freenect2EdgeAwareFiltering->setText(QString());
        label_266->setText(QApplication::translate("preferencesDialog", "Min depth.", Q_NULLPTR));
        label_230->setText(QApplication::translate("preferencesDialog", "Format.", Q_NULLPTR));
        label_270->setText(QApplication::translate("preferencesDialog", "Depth noise filtering.", Q_NULLPTR));
        label_267->setText(QApplication::translate("preferencesDialog", "Max depth.", Q_NULLPTR));
        checkBox_freenect2NoiseFiltering->setText(QString());
        label_434->setText(QApplication::translate("preferencesDialog", "Pipeline, any one of \"gl cl clkde cuda cudakde cpu\". If empty, default one is used. Depending on how libfreenect2 was built, the selected pipeline may not be available.", Q_NULLPTR));
        lineEdit_freenect2Pipeline->setText(QString());
        groupBox_realsense->setTitle(QApplication::translate("preferencesDialog", "RealSense", Q_NULLPTR));
        comboBox_realsensePresetRGB->clear();
        comboBox_realsensePresetRGB->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Best Quality", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Largest Image", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Highest Framerate", Q_NULLPTR)
        );
        label_318->setText(QApplication::translate("preferencesDialog", "Preset for depth stream.", Q_NULLPTR));
        label_317->setText(QApplication::translate("preferencesDialog", "Preset for RGB stream.", Q_NULLPTR));
        comboBox_realsensePresetDepth->clear();
        comboBox_realsensePresetDepth->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Best Quality", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Largest Image", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Highest Framerate", Q_NULLPTR)
        );
        label_realsenseOdom->setText(QApplication::translate("preferencesDialog", "Use visual inertial odometry for ZR300.", Q_NULLPTR));
        checkbox_realsenseOdom->setText(QString());
        checkbox_realsenseDepthScaledToRGBSize->setText(QString());
        label_realsenseOdom_2->setText(QApplication::translate("preferencesDialog", "Depth image scaled to RGB image size.", Q_NULLPTR));
        comboBox_realsenseRGBSource->clear();
        comboBox_realsenseRGBSource->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Color", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Infrared", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Fisheye", Q_NULLPTR)
        );
        label_realsenseOdom_3->setText(QApplication::translate("preferencesDialog", "RGB camera source. For fisheye has depth available only if it is calibrated.", Q_NULLPTR));
        groupBox_cameraRGBDImages->setTitle(QApplication::translate("preferencesDialog", "RGB-D Images", Q_NULLPTR));
        label_252->setText(QApplication::translate("preferencesDialog", "Path to directory containing RGB images.", Q_NULLPTR));
        toolButton_cameraRGBDImages_path_rgb->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        label_257->setText(QApplication::translate("preferencesDialog", "Depth scale factor (depth = pixel value / factor).", Q_NULLPTR));
        toolButton_cameraRGBDImages_path_depth->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        lineEdit_cameraRGBDImages_path_rgb->setText(QString());
        lineEdit_cameraRGBDImages_path_depth->setText(QString());
        label_254->setText(QApplication::translate("preferencesDialog", "Path to directory containing depth images. The directory should have the same size has the RGB directory. The depth images should be already registered to RGB images. Assume that UINT16 images are in mm and FLOAT32 images are in m.", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        doubleSpinBox_cameraRGBDImages_scale->setToolTip(QApplication::translate("preferencesDialog", "For RGBD-SLAM datasets, it is 5.", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_441->setText(QApplication::translate("preferencesDialog", "Start position (index).", Q_NULLPTR));
        label_540->setText(QApplication::translate("preferencesDialog", "Maximum frames after start position. 0 means publishing all frames.", Q_NULLPTR));
        groupBox_k4w2->setTitle(QApplication::translate("preferencesDialog", "Kinect for Windows SDK v2", Q_NULLPTR));
        comboBox_k4w2Format->clear();
        comboBox_k4w2Format->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Registered Color to Depth SD", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Registered Depth to Color SD", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Registered Depth to Color HD", Q_NULLPTR)
        );
        label_440->setText(QApplication::translate("preferencesDialog", "Format.", Q_NULLPTR));
        groupBox_realsense2->setTitle(QApplication::translate("preferencesDialog", "RealSense2", Q_NULLPTR));
        spinBox_rs2_width_depth->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        spinBox_rs2_height_depth->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        label_565->setText(QApplication::translate("preferencesDialog", "Dual Mode (D400/L500+T265): Odometry is computed by T265 and RGB-D frames are from D400/L500.", Q_NULLPTR));
        label_549->setText(QApplication::translate("preferencesDialog", "RGB/IR stream width. Set 1280 for L515.", Q_NULLPTR));
        spinBox_rs2_height->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        spinBox_rs2_rate->setSuffix(QApplication::translate("preferencesDialog", " Hz", Q_NULLPTR));
        checkbox_rs2_emitter->setText(QString());
        label_468->setText(QApplication::translate("preferencesDialog", "IR emitter enabled", Q_NULLPTR));
        checkbox_rs2_irMode->setText(QString());
        label_471->setText(QApplication::translate("preferencesDialog", "IR mode: use IR camera instead of RGB camera. Tracking will be more accurate (field-of-view of the IR camera is larger with less motion blur). Make sure to disable IR emitter. ", Q_NULLPTR));
        spinBox_rs2_width->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        label_550->setText(QApplication::translate("preferencesDialog", "RGB/IR stream height. Set 720 for L515.", Q_NULLPTR));
        label_551->setText(QApplication::translate("preferencesDialog", "RGB/IR stream rate. Set 30 for L515.", Q_NULLPTR));
        label_562->setText(QApplication::translate("preferencesDialog", "Use depth image in IR mode (instead of right image).", Q_NULLPTR));
        checkbox_rs2_irDepth->setText(QString());
        checkbox_rs2_dualMode->setText(QString());
#ifndef QT_NO_TOOLTIP
        lineEdit_rs2_dualModeExtrinsics->setToolTip(QApplication::translate("preferencesDialog", "<html><head/><body><p>Format (3 values): x y z<br/>Format (6 values): x y z roll pitch yaw<br/>Format (7 values): x y z qx qy qz qw<br/>Format (9 values, 3x3 rotation): r11 r12 r13 r21 r22 r23 r31 r32 r33<br/>Format (12 values, 3x4 transform): r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz</p><p>KITTI: /base_link to /gray_camera = 0 0 1 -1 0 0 0 -1 0<br/>KITTI: /base_link to /color_camera = 0 0 1 0 -1 0 0 -0.06 0 -1 0 0<br/>KITTI: /base_footprint to /gray_camera = 0 0 1 0 -1 0 0 0 0 -1 0 1.67<br/>KITTI: /base_footprint to /color_camera = 0 0 1 0 -1 0 0 -0.06 0 -1 0 1.67</p><p>EuRoC MAV: /base_link to /cam0 = T_BS*T_SC0 = -0.0257742 0.00375623 0.999661 0.00981073 -0.999557 -0.0149672 -0.0257155 0.064677 0.0148655 -0.999881 0.00414038 -0.0216401</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        lineEdit_rs2_dualModeExtrinsics->setText(QApplication::translate("preferencesDialog", "0.009 0.021 0.027 0.000 -0.018 0.005", Q_NULLPTR));
        label_571->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>D400 Series Visual Presets. See this <a href=\"https://github.com/IntelRealSense/librealsense/wiki/D400-Series-Visual-Presets\"><span style=\" text-decoration: underline; color:#0000ff;\">page</span></a>.</p></body></html>", Q_NULLPTR));
        toolButton_rs2_jsonFile->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        label_606->setText(QApplication::translate("preferencesDialog", "Global time sync. This will make sure IMU data is interpolated at image timestamp, otherwise latest received IMU is synchronized to current frame. Set to off if your firmware of your camera cannot synchronize timestamps.", Q_NULLPTR));
        checkbox_rs2_globalTimeStync->setText(QString());
        label_611->setText(QApplication::translate("preferencesDialog", "(L515) Depth stream width.", Q_NULLPTR));
        spinBox_rs2_rate_depth->setSuffix(QApplication::translate("preferencesDialog", " Hz", Q_NULLPTR));
        label_612->setText(QApplication::translate("preferencesDialog", "(L515) Depth stream height.", Q_NULLPTR));
        label_613->setText(QApplication::translate("preferencesDialog", "(L515) Depth stream rate.", Q_NULLPTR));
        label_566->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>Dual Mode extrinsics (T265's pose frame to D400's left IR camera). Default extrinsics match the 3D printed bracket <a href=\" https://www.intelrealsense.com/depth-and-tracking-combined-get-started/\"><span style=\" text-decoration: underline; color:#0000ff;\">here</span></a> (<a href=\"https://github.com/IntelRealSense/realsense-ros/blob/occupancy-mapping/realsense2_camera/meshes/mount_t265_d435.stl\"><span style=\" text-decoration: underline; color:#0000ff;\">stl</span></a>).</p></body></html>", Q_NULLPTR));
        groupBox_k4a->setTitle(QApplication::translate("preferencesDialog", "Kinect for Azure", Q_NULLPTR));
        checkbox_k4a_irDepth->setText(QString());
        source_checkBox_useMKVStamps->setText(QString());
        label_604->setText(QApplication::translate("preferencesDialog", "Use MKV file stamps as input rate.", Q_NULLPTR));
        label_601->setText(QApplication::translate("preferencesDialog", "Frames per second", Q_NULLPTR));
        toolButton_k4a_mkv->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        lineEdit_k4a_mkv->setText(QString());
        comboBox_k4a_framerate->clear();
        comboBox_k4a_framerate->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "5", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "15", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "30", Q_NULLPTR)
        );
        comboBox_k4a_rgb_resolution->clear();
        comboBox_k4a_rgb_resolution->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "720p", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "1080p", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "1440p", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "1536p", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "2160p", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "3072p", Q_NULLPTR)
        );
        label_603->setText(QApplication::translate("preferencesDialog", "Path to a *.MKV file.", Q_NULLPTR));
        label_558->setText(QApplication::translate("preferencesDialog", "Use IR for RGB image (may work better in dark areas).", Q_NULLPTR));
        comboBox_k4a_depth_resolution->clear();
        comboBox_k4a_depth_resolution->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "320x288 (NFOV 2x2BINNED)", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "640x576 (NFOV UNBINNED)", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "512x512 (WFOV 2x2BINNED)", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "1024x1024 (WFOV UNBINNED)", Q_NULLPTR)
        );
        label_602->setText(QApplication::translate("preferencesDialog", "Depth camera resolution. 2x2 binning mode extends the Z-range in comparison to the corresponding unbinned mode. Binning is done at the cost of lowering image resolution.", Q_NULLPTR));
        label_600->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>RGB camera resolution. Note that when color camera is used (IR mode is not checked below), to <span style=\" font-weight:600;\">avoid black borders in point clouds</span> generated, set ROI ratios under 3D Rendering settings to &quot;0.05 0.05 0.05 0.05&quot; under Map and Odom columns.</p></body></html>", Q_NULLPTR));
        groupBox_sourceStereo->setTitle(QApplication::translate("preferencesDialog", "Stereo", Q_NULLPTR));
        label_46->setText(QApplication::translate("preferencesDialog", "Grabber for stereo devices (i.e., Bumblebee2, Zed camera).", Q_NULLPTR));
        comboBox_cameraStereo->clear();
        comboBox_cameraStereo->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "DC1394", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "FlyCapture2", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Images", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Video", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "ZED sdk", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Usb camera", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Tara Camera", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "RealSense2", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "MyntEyeS", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "ZED Open Capture", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "DepthAI", Q_NULLPTR)
        );
        label_240->setText(QApplication::translate("preferencesDialog", "Driver.", Q_NULLPTR));
        checkbox_stereo_depthGenerated->setText(QString());
        label_stereo_depthGenerated->setText(QApplication::translate("preferencesDialog", "Generate disparity image and convert it to depth. The resulting output is a RGB-D image instead of stereo images. Dense disparity parameters can be found under StereoBM tab.", Q_NULLPTR));
        label_stereo_rectify->setText(QApplication::translate("preferencesDialog", "Rectify images. If checked, the images will be rectified using the calibration file (if its name is set above). If not checked, we assume that images are already rectified.", Q_NULLPTR));
        checkBox_stereo_rectify->setText(QString());
        label_stereo_rectify_2->setText(QApplication::translate("preferencesDialog", "Auto exposure compensation between left and right images.", Q_NULLPTR));
        checkBox_stereo_exposureCompensation->setText(QString());
        groupBox_cameraStereoImages->setTitle(QApplication::translate("preferencesDialog", "Stereo Images", Q_NULLPTR));
        toolButton_cameraStereoImages_path_left->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        label_249->setText(QApplication::translate("preferencesDialog", "Path to directory containing left images.", Q_NULLPTR));
        lineEdit_cameraStereoImages_path_left->setText(QString());
        label_253->setText(QApplication::translate("preferencesDialog", "Path to directory containing right images. The directory should have the same size has the left images directory. ", Q_NULLPTR));
        lineEdit_cameraStereoImages_path_right->setText(QString());
        toolButton_cameraStereoImages_path_right->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        label_442->setText(QApplication::translate("preferencesDialog", "Start position (index).", Q_NULLPTR));
        label_530->setText(QApplication::translate("preferencesDialog", "Maximum frames after start position. 0 means publishing all frames.", Q_NULLPTR));
        groupBox_cameraStereoVideo->setTitle(QApplication::translate("preferencesDialog", "Stereo Video (*.avi, *.mp4)", Q_NULLPTR));
        toolButton_cameraStereoVideo_path->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        lineEdit_cameraStereoVideo_path->setText(QString());
        label_339->setText(QApplication::translate("preferencesDialog", "Side-by-Side (SBS) video or left video.", Q_NULLPTR));
        label_340->setText(QApplication::translate("preferencesDialog", "Right video. Should be empty if a SBS video is set above.", Q_NULLPTR));
        lineEdit_cameraStereoVideo_path_2->setText(QString());
        toolButton_cameraStereoVideo_path_2->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        groupBox_cameraStereoZed->setTitle(QApplication::translate("preferencesDialog", "Zed sdk", Q_NULLPTR));
        label_305->setText(QApplication::translate("preferencesDialog", "Path to a *.SVO file.", Q_NULLPTR));
        comboBox_stereoZed_resolution->clear();
        comboBox_stereoZed_resolution->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "HD2K", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "HD1080", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "HD720", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "VGA", Q_NULLPTR)
        );
        label_291->setText(QApplication::translate("preferencesDialog", "Sensing mode.", Q_NULLPTR));
        comboBox_stereoZed_sensingMode->clear();
        comboBox_stereoZed_sensingMode->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "STANDARD", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "FILL", Q_NULLPTR)
        );
        comboBox_stereoZed_quality->clear();
        comboBox_stereoZed_quality->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "NONE", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "PERFORMANCE", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "QUALITY", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "ULTRA", Q_NULLPTR)
        );
        label_306->setText(QApplication::translate("preferencesDialog", "Depth confidence. Filtering value for the disparity map (and by extension the depth map). A lower value means more confidence and precision (but less density), an upper value reduces the filtering (more density, less certainty).", Q_NULLPTR));
        label_274->setText(QApplication::translate("preferencesDialog", "Resolution. Not used when a SVO file is used.", Q_NULLPTR));
        toolButton_zedSvoPath->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        lineEdit_zedSvoPath->setText(QString());
        label_290->setText(QApplication::translate("preferencesDialog", "Quality. If NONE, the disparity is not computed on the GPU, but on CPU using OpenCV (see StereoBM panel for parameters).", Q_NULLPTR));
        checkbox_stereoZed_odom->setText(QString());
        label_316->setText(QApplication::translate("preferencesDialog", "Use ZED visual odometry.", Q_NULLPTR));
        label_343->setText(QApplication::translate("preferencesDialog", "Self-Calibration.", Q_NULLPTR));
        checkbox_stereoZed_selfCalibration->setText(QString());
        label_570->setText(QApplication::translate("preferencesDialog", "[SDK3] Texture confidence. Threshold to reject depth values based on their textureness confidence. A lower value means more confidence and precision (but less density). An upper value reduces filtering (more density, less certainty.).", Q_NULLPTR));
        groupBox_cameraStereoUsb->setTitle(QApplication::translate("preferencesDialog", "Usb Camera", Q_NULLPTR));
        label_470->setText(QApplication::translate("preferencesDialog", "Optional right device ID. It should be -1 if the stereo camera streams side-by-side images.", Q_NULLPTR));
        label_620->setText(QApplication::translate("preferencesDialog", "Stream width (0 for default resolution).", Q_NULLPTR));
        label_619->setText(QApplication::translate("preferencesDialog", "Stream height (0 for default resolution).", Q_NULLPTR));
        groupBox_cameraStereoRealSense2->setTitle(QApplication::translate("preferencesDialog", "RealSense2", Q_NULLPTR));
        checkbox_stereoRealSense2_odom->setText(QString());
        label_546->setText(QApplication::translate("preferencesDialog", "Use RealSense2 visual inertial odometry.", Q_NULLPTR));
        groupBox_cameraMyntEye->setTitle(QApplication::translate("preferencesDialog", "MYNT EYE S", Q_NULLPTR));
        checkbox_stereoMyntEye_depth->setText(QString());
        label_576->setText(QApplication::translate("preferencesDialog", "Compute depth image with MYNT EYE S SDK. Ignored if images are not rectified from MYNT EYE S SDK.", Q_NULLPTR));
        label_581->setText(QApplication::translate("preferencesDialog", "Rectify images using MYNT EYE S SDK.", Q_NULLPTR));
        checkbox_stereoMyntEye_rectify->setText(QString());
        checkbox_stereoMyntEye_autoExposure->setText(QString());
        label_595->setText(QApplication::translate("preferencesDialog", "Image gain [0-48], valid if manual-exposure.", Q_NULLPTR));
        label_596->setText(QApplication::translate("preferencesDialog", "Image brightness [0-240], valid if manual-exposure.", Q_NULLPTR));
        label_594->setText(QApplication::translate("preferencesDialog", "Auto-exposure.", Q_NULLPTR));
        label_597->setText(QApplication::translate("preferencesDialog", "Image contrast [0-254], valid if manual-exposure.", Q_NULLPTR));
        label_599->setText(QApplication::translate("preferencesDialog", "IR control [0-160]. 0 means that the IR pattern is not projected.", Q_NULLPTR));
        groupBox_cameraStereoZedOC->setTitle(QApplication::translate("preferencesDialog", "Zed Open Capture", Q_NULLPTR));
        comboBox_stereoZedOC_resolution->clear();
        comboBox_stereoZedOC_resolution->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "HD2K", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "HD1080", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "HD720", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "VGA", Q_NULLPTR)
        );
        label_624->setText(QApplication::translate("preferencesDialog", "Resolution.", Q_NULLPTR));
        groupBox_cameraDepthAI->setTitle(QApplication::translate("preferencesDialog", "DepthAI", Q_NULLPTR));
        label_626->setText(QApplication::translate("preferencesDialog", "Resolution.", Q_NULLPTR));
        label_625->setText(QApplication::translate("preferencesDialog", "Output depth.", Q_NULLPTR));
        checkBox_depthai_depth->setText(QString());
        comboBox_depthai_resolution->clear();
        comboBox_depthai_resolution->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "720p", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "800p", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "400p", Q_NULLPTR)
        );
        label_627->setText(QApplication::translate("preferencesDialog", "Depth confidence.", Q_NULLPTR));
        groupBox_sourceRGB->setTitle(QApplication::translate("preferencesDialog", "RGB", Q_NULLPTR));
        label_20->setText(QApplication::translate("preferencesDialog", "Source type.", Q_NULLPTR));
        source_comboBox_image_type->clear();
        source_comboBox_image_type->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Usb camera", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Images", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Video file", Q_NULLPTR)
        );
        label_242->setText(QApplication::translate("preferencesDialog", "Rectify images. If checked, the images will be rectified using the calibration file (if its name is set above). If not checked, we assume that images are already rectified.", Q_NULLPTR));
        checkBox_rgb_rectify->setText(QString());
        source_groupBox_usbcam->setTitle(QApplication::translate("preferencesDialog", "USB Camera", Q_NULLPTR));
        label_553->setText(QApplication::translate("preferencesDialog", "Stream height (0 for default resolution).", Q_NULLPTR));
        label_552->setText(QApplication::translate("preferencesDialog", "Stream width (0 for default resolution).", Q_NULLPTR));
        source_groupBox_images->setTitle(QApplication::translate("preferencesDialog", "Images Dataset", Q_NULLPTR));
        label_21->setText(QApplication::translate("preferencesDialog", "Start position (index).", Q_NULLPTR));
        source_images_toolButton_selectSource->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        label_34->setText(QApplication::translate("preferencesDialog", "Maximum frames after start position. 0 means publishing all frames.", Q_NULLPTR));
        source_groupBox_video->setTitle(QApplication::translate("preferencesDialog", "Video (AVI)", Q_NULLPTR));
        source_video_toolButton_selectSource->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        groupBox_sourceDatabase->setTitle(QApplication::translate("preferencesDialog", "Database", Q_NULLPTR));
        label_263->setText(QApplication::translate("preferencesDialog", "Ignore goals saved in the database.", Q_NULLPTR));
        source_checkBox_ignoreGoals->setText(QString());
        source_checkBox_ignoreOdometry->setText(QString());
#ifndef QT_NO_TOOLTIP
        toolButton_dbViewer->setToolTip(QApplication::translate("preferencesDialog", "Open database viewer", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        toolButton_dbViewer->setText(QString());
        source_database_toolButton_selectSource->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        label_58->setText(QApplication::translate("preferencesDialog", "Start position (node ID).", Q_NULLPTR));
        label_72->setText(QApplication::translate("preferencesDialog", "Ignore odometry saved in the database, so if RGB-D SLAM is activated, odometry will be recomputed.", Q_NULLPTR));
        label_80->setText(QApplication::translate("preferencesDialog", "Ignore goal delay.", Q_NULLPTR));
        source_checkBox_ignoreGoalDelay->setText(QString());
        label_90->setText(QApplication::translate("preferencesDialog", "Use database stamps as input rate.", Q_NULLPTR));
        source_checkBox_useDbStamps->setText(QString());
        label_315->setText(QApplication::translate("preferencesDialog", "Camera index. If the database contains multi-camera data, you can choose which camera to use. -1 means that all cameras are streamed.", Q_NULLPTR));
        label_529->setText(QApplication::translate("preferencesDialog", "Stop position (node ID) is the last node to process. If 0, all nodes after start position are published.", Q_NULLPTR));
        label_557->setText(QApplication::translate("preferencesDialog", "If the database contains stereo data, generate disparity image and convert it to depth. The resulting output is a RGB-D image instead of stereo images. Dense disparity parameters can be found under StereoBM tab.", Q_NULLPTR));
        source_checkBox_stereoToDepthDB->setText(QString());
        groupBox_sourceImages_optional->setTitle(QApplication::translate("preferencesDialog", "Directory of images (optional settings)", Q_NULLPTR));
        label_288->setText(QApplication::translate("preferencesDialog", "Ground truth file. Select the correct format below.", Q_NULLPTR));
        label_289->setText(QApplication::translate("preferencesDialog", "Ground truth format. See tool tip for more details on formats. Note that formats without stamps should have the same number of values than the source images.", Q_NULLPTR));
        label_265->setText(QApplication::translate("preferencesDialog", "Bayer mode. For convenience, if the images are bayered.", Q_NULLPTR));
        toolButton_cameraImages_gt->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        lineEdit_cameraImages_odom->setText(QString());
        label_293->setText(QApplication::translate("preferencesDialog", "Path to directory containing optional laser scans (*.pcd, *.ply, *.bin [KITTI format]). The directory should have the same size has the images directory. ", Q_NULLPTR));
        label_348->setText(QApplication::translate("preferencesDialog", "Odometry file. Select the correct format below.", Q_NULLPTR));
        label_255->setText(QApplication::translate("preferencesDialog", "Use file names as timestamps. Format is epoch time. Example: \"1305031102.175304.png\".", Q_NULLPTR));
        toolButton_cameraImages_timestamps->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        lineEdit_cameraImages_timestamps->setText(QString());
        lineEdit_cameraImages_gt->setText(QString());
        checkBox_cameraImages_syncTimeStamps->setText(QString());
        checkBox_cameraImages_timestamps->setText(QString());
        label_292->setText(QApplication::translate("preferencesDialog", "Maximum laser scan points.", Q_NULLPTR));
        label_256->setText(QApplication::translate("preferencesDialog", "Synchronize capture rate with timestamps.", Q_NULLPTR));
        lineEdit_cameraImages_path_scans->setText(QString());
        label_251->setText(QApplication::translate("preferencesDialog", "Timestamps file (*.txt). The file should contain one column. The number of rows should be the same than the number of images in the folder. Not used if \"Use file names as timestamps\" above is checked. ", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        spinBox_cameraImages_max_scan_pts->setToolTip(QApplication::translate("preferencesDialog", "<html><head/><body><p>KITTI: 130 000 points</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        comboBox_cameraImages_gtFormat->clear();
        comboBox_cameraImages_gtFormat->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Raw", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "RGBD-SLAM (motion capture)", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "KITTI", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "TORO", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "g2o", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "NewCollege", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Malaga Urban", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "St Lucia Stereo", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Karlsruhe", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "EuRoC MAV", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "RGBD-SLAM", Q_NULLPTR)
        );
#ifndef QT_NO_TOOLTIP
        comboBox_cameraImages_gtFormat->setToolTip(QApplication::translate("preferencesDialog", "<html><head/><body><p>Raw Format (3 values): x y z<br/>Raw Format (6 values): x y z roll pitch yaw<br/>Raw Format (7 values): x y z qx qy qz qw<br/>Raw Format (9 values, 3x3 rotation): r11 r12 r13 r21 r22 r23 r31 r32 r33<br/>Raw Format (12 values, 3x4 transform): r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz</p><p>RGBD-SLAM (stamp tx ty tz qx qy qz qw)<br/>KITTI (stamp + 12 values transform)<br/>TORO<br/>g2o<br/>NewCollege (stamp x y)<br/>Malaga Urban (GPS)<br/>St Lucia Stereo (INS)<br/>EuRoC MAV (stamp,tx,ty,tz,qw,qx,qy,qz...)</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        toolButton_cameraImages_path_scans->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        lineEdit_cameraImages_laser_transform->setToolTip(QApplication::translate("preferencesDialog", "<html><head/><body><p>Format (3 values): x y z<br/>Format (6 values): x y z roll pitch yaw<br/>Format (7 values): x y z qx qy qz qw<br/>Format (9 values, 3x3 rotation): r11 r12 r13 r21 r22 r23 r31 r32 r33<br/>Format (12 values, 3x4 transform): r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz</p><p>KITTI: /base_link to /scan = -0.27 0 0.08 0 0 0<br/>KITTI: /base_footprint to /scan = -0.27 0 1.75 0 0 0</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        lineEdit_cameraImages_laser_transform->setText(QApplication::translate("preferencesDialog", "0 0 0 0 0 0", Q_NULLPTR));
        label_294->setText(QApplication::translate("preferencesDialog", "Local transform from /base_link to /scan_link. Mouse over the box to show formats.", Q_NULLPTR));
        comboBox_cameraImages_bayerMode->clear();
        comboBox_cameraImages_bayerMode->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Disabled", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "BG", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "GB", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "RG", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "GR", Q_NULLPTR)
        );
        label_349->setText(QApplication::translate("preferencesDialog", "Odometry format. See tool tip for more details on formats. Note that formats without stamps should have the same number of values than the source images.", Q_NULLPTR));
        comboBox_cameraImages_odomFormat->clear();
        comboBox_cameraImages_odomFormat->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Raw", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "RGBD-SLAM (motion capture)", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "KITTI", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "TORO", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "g2o", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "NewCollege", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Malaga Urban", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "St Lucia Stereo", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Karlsruhe", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "EuRoC MAV", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "RGBD-SLAM", Q_NULLPTR)
        );
#ifndef QT_NO_TOOLTIP
        comboBox_cameraImages_odomFormat->setToolTip(QApplication::translate("preferencesDialog", "<html><head/><body><p>Raw Format (3 values): x y z<br/>Raw Format (6 values): x y z roll pitch yaw<br/>Raw Format (7 values): x y z qx qy qz qw<br/>Raw Format (9 values, 3x3 rotation): r11 r12 r13 r21 r22 r23 r31 r32 r33<br/>Raw Format (12 values, 3x4 transform): r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz</p><p>RGBD-SLAM (stamp tx ty tz qx qy qz qw)<br/>KITTI (stamp + 12 values transform)<br/>TORO<br/>g2o<br/>NewCollege (stamp x y)<br/>Malaga Urban (GPS)<br/>St Lucia Stereo (INS)<br/>EuRoC MAV (stamp,tx,ty,tz,qw,qx,qy,qz...)</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        toolButton_cameraImages_odom->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        label_443->setText(QApplication::translate("preferencesDialog", "Max time difference between data and corresponding pose for format with stamps. If delay is over this threshold, the pose won't be set on data loaded. This is used when odometry and/or ground truth files are set.", Q_NULLPTR));
        doubleSpinBox_maxPoseTimeDiff->setSuffix(QApplication::translate("preferencesDialog", " s", Q_NULLPTR));
        label_464->setText(QApplication::translate("preferencesDialog", "Local transform from /base_link to /imu_link. Mouse over the box to show formats.", Q_NULLPTR));
        label_463->setText(QApplication::translate("preferencesDialog", "Path to file containing optional IMU data (*.csv [EuRoC format]).", Q_NULLPTR));
        toolButton_cameraImages_path_imu->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        lineEdit_cameraImages_path_imu->setText(QString());
#ifndef QT_NO_TOOLTIP
        lineEdit_cameraImages_imu_transform->setToolTip(QApplication::translate("preferencesDialog", "<html><head/><body><p>Format (3 values): x y z<br/>Format (6 values): x y z roll pitch yaw<br/>Format (7 values): x y z qx qy qz qw<br/>Format (9 values, 3x3 rotation): r11 r12 r13 r21 r22 r23 r31 r32 r33<br/>Format (12 values, 3x4 transform): r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz</p><p>EuRoC: /base_link to /imu = 0 0 1 0 -1 0 1 0 0</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        lineEdit_cameraImages_imu_transform->setText(QApplication::translate("preferencesDialog", "0 0 0 0 0 0", Q_NULLPTR));
        label_465->setText(QApplication::translate("preferencesDialog", "IMU Rate. To synchronize capture rate with IMU timestamps, set to 0. This can be set a little over the actual IMU rate to keep up with camera capture rate if images are dropped by odometry.", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        spinBox_cameraImages_max_imu_rate->setToolTip(QApplication::translate("preferencesDialog", "EuRoC: 200 Hz -> 250 Hz", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_605->setText(QApplication::translate("preferencesDialog", "Load config file for each frame (intrinsics, pose and stamp). Config files should be in the same directory than RGB frames and they should have the same name than the corresponding frame file. Currently supporting only 3DScannerApp for iOS export config format (JSON).", Q_NULLPTR));
        checkBox_cameraImages_configForEachFrame->setText(QString());
        groupBox_depthFromScan->setTitle(QApplication::translate("preferencesDialog", "Generate depth image from laser scan", Q_NULLPTR));
        groupBox_depthFromScan_fillHoles->setTitle(QApplication::translate("preferencesDialog", "Interpolate depth values to fill holes", Q_NULLPTR));
        radioButton_depthFromScan_vertical->setText(QApplication::translate("preferencesDialog", "Vertically", Q_NULLPTR));
        radioButton_depthFromScan_horizontal->setText(QApplication::translate("preferencesDialog", "Horizontally", Q_NULLPTR));
        checkBox_depthFromScan_fillBorders->setText(QApplication::translate("preferencesDialog", "Fill holes from the image border.", Q_NULLPTR));
        groupBox_depthImageFiltering->setTitle(QApplication::translate("preferencesDialog", "Depth Image Filtering", Q_NULLPTR));
        toolButton_source_distortionModel->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        lineEdit_source_distortionModel->setText(QString());
#ifndef QT_NO_TOOLTIP
        toolButton_distortionModel->setToolTip(QApplication::translate("preferencesDialog", "Open database viewer", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        toolButton_distortionModel->setText(QString());
        label_341->setText(QApplication::translate("preferencesDialog", "Distortion model (output from depth calibration).", Q_NULLPTR));
        groupBox_bilateral->setTitle(QApplication::translate("preferencesDialog", "Bilateral Filtering of the Depth Image", Q_NULLPTR));
        doubleSpinBox_bilateral_sigmaR->setSuffix(QString());
        label_344->setText(QApplication::translate("preferencesDialog", "Standard deviation of the Gaussian for the intensity difference. Set the standard deviation of the Gaussian used to control how much an adjacent pixel is downweighted because of the intensity difference (depth in our case).", Q_NULLPTR));
        doubleSpinBox_bilateral_sigmaS->setSuffix(QString());
        label_345->setText(QApplication::translate("preferencesDialog", "Size of the Gaussian bilateral filter window to use. Set the standard deviation of the Gaussian used by the bilateral filter for the spatial neighborhood/window.", Q_NULLPTR));
        groupBox_imuFiltering->setTitle(QApplication::translate("preferencesDialog", "IMU Filtering", Q_NULLPTR));
        comboBox_imuFilter_strategy->clear();
        comboBox_imuFilter_strategy->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "None", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Madgwick Filter", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Complementary Filter", Q_NULLPTR)
        );
        label_541->setText(QApplication::translate("preferencesDialog", "IMU filtering strategy used to estimate the orientation.", Q_NULLPTR));
        checkbox_publishInterIMU->setText(QString());
        label_563->setText(QApplication::translate("preferencesDialog", "Publish inter IMU messages from the camera. IMU received between images will be published as separate topic. IMU filtering strategy will be ignored.", Q_NULLPTR));
        label_621->setText(QApplication::translate("preferencesDialog", "Convert IMU in base frame before filtering. This can help to initialize correctly the yaw.", Q_NULLPTR));
        checkBox_imuFilter_baseFrameConversion->setText(QString());
        groupBox_madgwickfilter->setTitle(QApplication::translate("preferencesDialog", "Madgwick Filter", Q_NULLPTR));
        doubleSpinBox_imuFilterMadgwickGain->setSuffix(QString());
        label_542->setText(QApplication::translate("preferencesDialog", "Gain.", Q_NULLPTR));
        doubleSpinBox_imuFilterMadgwickZeta->setSuffix(QString());
        label_543->setText(QApplication::translate("preferencesDialog", "Zeta.", Q_NULLPTR));
        groupBox_complementaryfilter->setTitle(QApplication::translate("preferencesDialog", "Complementary Filter", Q_NULLPTR));
        label_547->setText(QApplication::translate("preferencesDialog", "Do bias estimation.", Q_NULLPTR));
        label_544->setText(QApplication::translate("preferencesDialog", "Gain acc.", Q_NULLPTR));
        checkBox_imuFilterComplementaryDoBiasEstimation->setText(QString());
        doubleSpinBox_imuFilterComplementaryGainAcc->setSuffix(QString());
        label_545->setText(QApplication::translate("preferencesDialog", "Bias alpha.", Q_NULLPTR));
        doubleSpinBox_imuFilterComplementaryBiasAlpha->setSuffix(QString());
        checkBox_imuFilterComplementaryDoAdaptiveGain->setText(QString());
        label_548->setText(QApplication::translate("preferencesDialog", "Do adaptive gain.", Q_NULLPTR));
        groupBox_scan->setTitle(QApplication::translate("preferencesDialog", "Laser scans", Q_NULLPTR));
        label_16->setText(QApplication::translate("preferencesDialog", "If you want to use ICP registration, the sensor data should have laser scan. Laser scans can be created from the depth images (see option below) or loaded from the source selected. These parameters can be used to reduce the point cloud size directly in the capturing thread.", Q_NULLPTR));
        doubleSpinBox_source_scanRangeMin->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        spinBox_source_scanNormalsK->setToolTip(QApplication::translate("preferencesDialog", "<html><head/><body><p>KITTI: 130 000 points</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_300->setText(QApplication::translate("preferencesDialog", "Downsample step size for laser scans. If your laser scans are created from depth images, the step is used as decimation of the depth images.", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        doubleSpinBox_source_scanVoxelSize->setToolTip(QApplication::translate("preferencesDialog", "<html><head/><body><p>KITTI: 130 000 points</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        doubleSpinBox_source_scanVoxelSize->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        spinBox_source_scanDownsampleStep->setToolTip(QApplication::translate("preferencesDialog", "<html><head/><body><p>KITTI: 130 000 points</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_302->setText(QApplication::translate("preferencesDialog", "Voxel size for uniform sampling. Note that it is done after downsampling.", Q_NULLPTR));
        label_304->setText(QApplication::translate("preferencesDialog", "K nearest neighbors for normals computation (0=disabled, 20 can be a good default value). Useful if the ICP registration approach is point to plane.", Q_NULLPTR));
        label_425->setText(QApplication::translate("preferencesDialog", "Search radius for normals computation (0=disabled). Useful if the ICP registration approach is point to plane.", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        doubleSpinBox_source_scanNormalsRadius->setToolTip(QApplication::translate("preferencesDialog", "<html><head/><body><p>KITTI: 130 000 points</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_453->setText(QApplication::translate("preferencesDialog", "Force ground normals to be all upward (e.g., 0.8). Useful to make sure Velodyne's scans on ground have all normals upward. 0 means disabled.", Q_NULLPTR));
        doubleSpinBox_source_scanRangeMax->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_522->setText(QApplication::translate("preferencesDialog", "Minimum range, filter points below that range. 0 means disabled.", Q_NULLPTR));
        label_523->setText(QApplication::translate("preferencesDialog", "Maximum range, filter points above that range. 0 means disabled.", Q_NULLPTR));
        checkBox_source_scanFromDepth->setText(QString());
        label_source_scanFromDepth->setText(QApplication::translate("preferencesDialog", "Generate laser scan from depth image.", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        doubleSpinBox_source_scanNormalsForceGroundUp->setToolTip(QApplication::translate("preferencesDialog", "<html><head/><body><p>KITTI: 130 000 points</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        groupBox_rtabmap_basic0->setTitle(QApplication::translate("preferencesDialog", "RTAB-Map Settings", Q_NULLPTR));
        general_doubleSpinBox_detectionRate_2->setSuffix(QApplication::translate("preferencesDialog", " Hz", Q_NULLPTR));
        general_checkBox_SLAM_mode_2->setText(QString());
        label_79->setText(QApplication::translate("preferencesDialog", "SLAM mode.\n"
" If set to false, global localization is performed, so without adding new data to the map.", Q_NULLPTR));
        general_checkBox_activateRGBD_2->setText(QString());
        label_activateRGBD_2->setText(QApplication::translate("preferencesDialog", "Activate metric RGB-D SLAM.\n"
"If set to false, classic RTAB-Map loop closure detection is done using only images and without any metric information.", Q_NULLPTR));
        general_checkBox_publishStats_2->setText(QString());
        label_publishStat->setText(QApplication::translate("preferencesDialog", "Publish statistics.", Q_NULLPTR));
        label_134->setText(QApplication::translate("preferencesDialog", "Detection rate (0 means inf). RTAB-Map will filter input images to satisfy this rate.", Q_NULLPTR));
        label_112->setText(QApplication::translate("preferencesDialog", "Images buffer size (0 means inf).", Q_NULLPTR));
        general_doubleSpinBox_timeThr_2->setSuffix(QApplication::translate("preferencesDialog", " ms", Q_NULLPTR));
        label_timeLimit_2->setText(QApplication::translate("preferencesDialog", "T_time : Max time (ms) allowed (0 means inf).", Q_NULLPTR));
        label_121->setText(QApplication::translate("preferencesDialog", "T_loop : Loop closure threshold.", Q_NULLPTR));
        label_similarity_2->setText(QApplication::translate("preferencesDialog", "T_similarity : Similarity threshold (for Rehearsal/Weight Update).", Q_NULLPTR));
        label_126->setText(QApplication::translate("preferencesDialog", "STM size : Short-term memory size.", Q_NULLPTR));
        groupBox_rtabmap_advanced0->setTitle(QApplication::translate("preferencesDialog", "RTAB-Map Settings", Q_NULLPTR));
        general_doubleSpinBox_detectionRate->setSuffix(QApplication::translate("preferencesDialog", " Hz", Q_NULLPTR));
        general_checkBox_SLAM_mode->setText(QString());
        label_92->setText(QApplication::translate("preferencesDialog", "SLAM mode.\n"
" If set to false, global localization is performed, so without adding new data to the map.", Q_NULLPTR));
        general_checkBox_startNewMapOnLoopClosure->setText(QString());
        label_135->setText(QApplication::translate("preferencesDialog", "Detection rate (0 means inf). RTAB-Map will filter input data to satisfy this rate. If you want to process all data, consider set \"Detection rate\" to 0 and \"Data buffer size\" to 0.", Q_NULLPTR));
        general_checkBox_createIntermediateNodes->setText(QString());
        label_185->setText(QApplication::translate("preferencesDialog", "Start a new map only if there is a global loop closure detected first with a previous map. If there is no map in memory, a new map is still created.", Q_NULLPTR));
        label_84->setText(QApplication::translate("preferencesDialog", "Data buffer size (0 means inf).", Q_NULLPTR));
        label_165->setText(QApplication::translate("preferencesDialog", "Create intermediate nodes if odometry is faster than the detection rate.", Q_NULLPTR));
        label_467->setText(QApplication::translate("preferencesDialog", "Images are already rectified. By default RTAB-Map assumes that received images are rectified. If they are not, they can be rectified by RTAB-Map if this parameter is false.", Q_NULLPTR));
        general_checkBox_imagesAlreadyRectified->setText(QString());
        label_329->setText(QApplication::translate("preferencesDialog", "Start a new map only if the first signature is not bad (i.e., has enough features, see Vocabulary->Bad signature ratio).", Q_NULLPTR));
        general_checkBox_startNewMapOnGoodSignature->setText(QString());
        label_498->setText(QApplication::translate("preferencesDialog", "Rectify only features. If images are not already rectified (see parameter above) and this parameter is true, the whole RGB image will not be rectified, only the features. Warning: As projection of RGB-D image to point cloud is assuming that images are rectified, the generated point cloud map will have wrong colors if this parameter is true.", Q_NULLPTR));
        general_checkBox_rectifyOnlyFeatures->setText(QString());
        label_573->setText(QApplication::translate("preferencesDialog", "Save localization data during localization session (when SLAM mode is false). When enabled, the database will then also grow in localization mode. This mode would be used only for debugging purpose.", Q_NULLPTR));
        general_checkBox_saveLocalizationData->setText(QString());
        groupBox_6->setTitle(QApplication::translate("preferencesDialog", "Thresholds", Q_NULLPTR));
        general_doubleSpinBox_timeThr->setSuffix(QApplication::translate("preferencesDialog", " ms", Q_NULLPTR));
        label_timeLimit->setText(QApplication::translate("preferencesDialog", "T_time : Max time (ms) allowed (0 means inf).", Q_NULLPTR));
        label_maxWmSize->setText(QApplication::translate("preferencesDialog", "Maximum signatures allowed in Working Memory (0 means inf).", Q_NULLPTR));
        label_93->setText(QApplication::translate("preferencesDialog", "T_loop : Loop closure threshold. Setting to 0 means that the new place hypothesis is used as threshold.", Q_NULLPTR));
        label_96->setText(QApplication::translate("preferencesDialog", "T_ratio : The loop closure hypothesis must be over T_ratio x lastHypothesisValue.", Q_NULLPTR));
        groupBox_publishing->setTitle(QApplication::translate("preferencesDialog", "Publish statistics (stuff to be shown in the GUI) and save them in database", Q_NULLPTR));
        general_checkBox_publishPdf->setText(QString());
        label_106->setText(QApplication::translate("preferencesDialog", "Publish loop closure hypotheses (pdf).", Q_NULLPTR));
        general_checkBox_publishLikelihood->setText(QString());
        label_462->setText(QApplication::translate("preferencesDialog", "Save working memory state after each update.", Q_NULLPTR));
        general_checkBox_publishRawData->setText(QString());
        label_91->setText(QApplication::translate("preferencesDialog", "Publish signature data.", Q_NULLPTR));
        label_116->setText(QApplication::translate("preferencesDialog", "Publish loop closure likelihood.", Q_NULLPTR));
        label_452->setText(QApplication::translate("preferencesDialog", "Publish RAM usage.", Q_NULLPTR));
        general_checkBox_publishRAM->setText(QString());
        label_437->setText(QApplication::translate("preferencesDialog", "Compute RMSE when ground truth is available.", Q_NULLPTR));
        general_checkBox_publishRMSE->setText(QString());
        general_checkBox_saveWMState->setText(QString());
        groupBox_statistics->setTitle(QApplication::translate("preferencesDialog", "Log statistics (LogI.txt and LogF.txt) in the working directory.", Q_NULLPTR));
        general_checkBox_statisticLogsBufferedInRAM->setText(QString());
        label_143->setText(QApplication::translate("preferencesDialog", "Statistics logs buffered in RAM and written to hard drive on exit (otherwise logs are written at each iteration on hard drive).", Q_NULLPTR));
        general_checkBox_statisticLoggedHeaders->setText(QString());
        label_209->setText(QApplication::translate("preferencesDialog", "Add headers (column names) to log files.", Q_NULLPTR));
        label_workingDirectory->setText(QApplication::translate("preferencesDialog", "Working directory (where to put logs/dump stuff).", Q_NULLPTR));
        toolButton_workingDirectory->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        groupBox_memory1->setTitle(QApplication::translate("preferencesDialog", "Memory", Q_NULLPTR));
        label_retrieved_10->setText(QApplication::translate("preferencesDialog", "Reduce graph. Merge nodes when loop closures are added (ignoring those with user data set).", Q_NULLPTR));
        label_retrieved_4->setText(QApplication::translate("preferencesDialog", "Initialize the Woking Memory with all nodes from Long-Term memory, instead of only nodes of the last session. This may be useful in localization mode, where less processing time is required than in SLAM mode, so more nodes can be kept in Working Memory.", Q_NULLPTR));
        general_checkBox_initWMWithAllNodes->setText(QString());
        general_checkBox_keepRawData->setText(QString());
        general_checkBox_reduceGraph->setText(QString());
        label_retrieved_3->setText(QApplication::translate("preferencesDialog", "Bad signatures are ignored.", Q_NULLPTR));
        general_checkBox_createMapLabels->setText(QString());
        label_retrieved_5->setText(QApplication::translate("preferencesDialog", "Keep raw sensor data. Only useful to save loop closure computation time when features re-extraction is enabled. Disable to save RAM memory.", Q_NULLPTR));
        label_retrieved_11->setText(QApplication::translate("preferencesDialog", "Create map labels. The first node of a map will be labelled as \"map#\" where # is the map ID.", Q_NULLPTR));
        general_checkBox_keepDescriptors->setText(QString());
        label_retrieved_12->setText(QApplication::translate("preferencesDialog", "Raw descriptors kept in memory.", Q_NULLPTR));
        label_retrieved_13->setText(QApplication::translate("preferencesDialog", "Image pre decimation. This option can be used to reduce image size before features extraction.", Q_NULLPTR));
        label_retrieved_2->setText(QApplication::translate("preferencesDialog", "True=Generate location Ids, False=use input image ids.", Q_NULLPTR));
        label_retrieved->setText(QApplication::translate("preferencesDialog", "Maximum locations retrieved at the same time from LTM to WM.", Q_NULLPTR));
        general_checkBox_generateIds->setText(QString());
        general_checkBox_badSignaturesIgnored->setText(QString());
        general_checkBox_transferSortingByWeightId->setText(QString());
        label_retrieved_9->setText(QApplication::translate("preferencesDialog", "Save depth image into 16 bits format to reduce memory used. Warning: values over ~65 meters are ignored (maximum 65535 millimeters).", Q_NULLPTR));
        label_29->setText(QApplication::translate("preferencesDialog", "STM size : Short-term memory size.", Q_NULLPTR));
        general_checkBox_saveDepth16bits->setText(QString());
        label_ratioRecent->setText(QApplication::translate("preferencesDialog", "T_recent : Ratio of locations after the last loop closure in WM that cannot be transferred.", Q_NULLPTR));
        label_retrieved_6->setText(QApplication::translate("preferencesDialog", "Image post decimation. This option can be used to save images in lower resolution (size/decimation). It is done on the original image.", Q_NULLPTR));
        label_retrieved_7->setText(QApplication::translate("preferencesDialog", "On transfer, signatures are sorted by weight->ID (i.e. the oldest of the lowest weighted signatures are transferred first). If false, the signatures are sorted by weight->Age (i.e. the oldest inserted in WM of the lowest weighted signatures are transferred first). Note that retrieval updates the age, not the ID.", Q_NULLPTR));
        general_checkBox_compressionParallelized->setText(QString());
        general_checkBox_saveIntermediateNodeData->setText(QString());
        label_retrieved_15->setText(QApplication::translate("preferencesDialog", "Multi-threaded compression.", Q_NULLPTR));
        label_retrieved_16->setText(QApplication::translate("preferencesDialog", "Save intermediate node data.", Q_NULLPTR));
        groupBox_27->setTitle(QApplication::translate("preferencesDialog", "Laser scan filtering", Q_NULLPTR));
        label_retrieved_8->setText(QApplication::translate("preferencesDialog", "Downsampling step. If > 1, downsample the laser scans when creating a location. This feature can be used to save laser scans already downsampled.", Q_NULLPTR));
        general_doubleSpinBox_laserScanNormalRadius->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_retrieved_14->setText(QApplication::translate("preferencesDialog", "Normal K. If > 0 and laser scans don't have normals, normals will be computed with K search neighbors when creating a signature.", Q_NULLPTR));
        label_retrieved_17->setText(QApplication::translate("preferencesDialog", "Normal Radius. If > 0 and laser scans don't have normals, normals will be computed with radius search when creating a signature.", Q_NULLPTR));
        label_retrieved_18->setText(QApplication::translate("preferencesDialog", "Voxel size. If > 0 m, voxel filtering is done on laser scans when creating a signature. If the laser scan had normals, they will be removed. To recompute the normals, make sure to use Normal K or Normal Radius parameters below.", Q_NULLPTR));
        general_doubleSpinBox_laserScanVoxelSize->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        groupBox_rehearsal2->setTitle(QApplication::translate("preferencesDialog", "Rehearsal / Weight Update", Q_NULLPTR));
        general_checkBox_RehearsalIdUpdatedToNewOne->setText(QString());
        label_similarity->setText(QApplication::translate("preferencesDialog", "T_similarity : Similarity threshold. Values must be >=0.0 and <=1.0.", Q_NULLPTR));
        label_rehearsalIdUpdate->setText(QApplication::translate("preferencesDialog", "On merging, update to new id.", Q_NULLPTR));
        label_rehearsalIdUpdate_2->setText(QApplication::translate("preferencesDialog", "Ignore Weight Update when the robot is moving.", Q_NULLPTR));
        rgdb_rehearsalWeightIgnoredWhileMoving->setText(QString());
        groupBox_signature_keypoint2->setTitle(QApplication::translate("preferencesDialog", "Vocabulary", Q_NULLPTR));
        checkBox_useOdomFeatures->setText(QString());
        label_145->setText(QApplication::translate("preferencesDialog", "Use odometry features. Disable to select different feature type. Note that if no features are received, they will be re-extracted (if visual odometry is used, you may want to match the feature type below with the one used for odometry).", Q_NULLPTR));
        groupBox_8->setTitle(QApplication::translate("preferencesDialog", "Features Extraction", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_22->setToolTip(QApplication::translate("preferencesDialog", "0 means that the response (hessian) threshold \n"
"used for the detector will not be adapted. \n"
"Otherwise, the threshold is modified to \n"
"generate the number of words requested.", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_22->setText(QApplication::translate("preferencesDialog", "Maximum words per image (0=no maximum). Setting to -1 will disable features extraction, so disabling loop closure detection indirectly.", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_81->setToolTip(QApplication::translate("preferencesDialog", "0 means that the response (hessian) threshold \n"
"used for the detector will not be adapted. \n"
"Otherwise, the threshold is modified to \n"
"generate the number of words requested.", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_81->setText(QApplication::translate("preferencesDialog", "Bad signature ratio (less than Ratio x AverageWordsPerImage = bad).", Q_NULLPTR));
        label_99->setText(QApplication::translate("preferencesDialog", "Top ROI ratio (0 = no change).", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_57->setToolTip(QApplication::translate("preferencesDialog", "0 means that the response (hessian) threshold \n"
"used for the detector will not be adapted. \n"
"Otherwise, the threshold is modified to \n"
"generate the number of words requested.", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_57->setText(QApplication::translate("preferencesDialog", "Minimum words depth. Only used when a depth image is provided. Applied before \"Maximum words per image\".", Q_NULLPTR));
        comboBox_detector_strategy->clear();
        comboBox_detector_strategy->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "SURF", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "SIFT", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "ORB", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "FAST+FREAK", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "FAST+BRIEF", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "GFTT+FREAK", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "GFTT+BRIEF", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "BRISK", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "GFTT+ORB", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "KAZE", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "ORB OCTREE", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "SuperPoint Torch", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "SURF+FREAK", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "GFTT+DAISY", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "SURF+DAISY", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "PyDetector", Q_NULLPTR)
        );
        label_101->setText(QApplication::translate("preferencesDialog", "ROI ratios [left, right, top, bottom] between 0 and 1.", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_53->setToolTip(QApplication::translate("preferencesDialog", "0 means that the response (hessian) threshold \n"
"used for the detector will not be adapted. \n"
"Otherwise, the threshold is modified to \n"
"generate the number of words requested.", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_53->setText(QApplication::translate("preferencesDialog", "Maximum words depth (0 means inf). Only used when a depth image is provided. Applied before \"Maximum words per image\".", Q_NULLPTR));
        doubleSpinBox_kp_roi1->setSuffix(QApplication::translate("preferencesDialog", " %", Q_NULLPTR));
        label_117->setText(QApplication::translate("preferencesDialog", "Visual word type.", Q_NULLPTR));
        surf_doubleSpinBox_maxDepth->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        doubleSpinBox_kp_roi0->setSuffix(QApplication::translate("preferencesDialog", " %", Q_NULLPTR));
        label_97->setText(QApplication::translate("preferencesDialog", "Left ROI ratio (0 = no change).", Q_NULLPTR));
        label_98->setText(QApplication::translate("preferencesDialog", "Right ROI ratio (0 = no change).", Q_NULLPTR));
        doubleSpinBox_kp_roi2->setSuffix(QApplication::translate("preferencesDialog", " %", Q_NULLPTR));
        doubleSpinBox_kp_roi3->setSuffix(QApplication::translate("preferencesDialog", " %", Q_NULLPTR));
        label_100->setText(QApplication::translate("preferencesDialog", "Bottom ROI ratio (0 = no change).", Q_NULLPTR));
        surf_doubleSpinBox_minDepth->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_50->setToolTip(QApplication::translate("preferencesDialog", "0 means that the response (hessian) threshold \n"
"used for the detector will not be adapted. \n"
"Otherwise, the threshold is modified to \n"
"generate the number of words requested.", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_50->setText(QApplication::translate("preferencesDialog", "Number of rows of the grid used to extract uniformly \"max words / grid cells\" features from each cell.", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_86->setToolTip(QApplication::translate("preferencesDialog", "0 means that the response (hessian) threshold \n"
"used for the detector will not be adapted. \n"
"Otherwise, the threshold is modified to \n"
"generate the number of words requested.", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_86->setText(QApplication::translate("preferencesDialog", "Number of columns of the grid used to extract uniformly \"max words / grid cells\" features from each cell.", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_262->setToolTip(QApplication::translate("preferencesDialog", "0 means that the response (hessian) threshold \n"
"used for the detector will not be adapted. \n"
"Otherwise, the threshold is modified to \n"
"generate the number of words requested.", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_262->setText(QApplication::translate("preferencesDialog", "Use depth image as mask when extracting features.", Q_NULLPTR));
        checkBox_memDepthAsMask->setText(QString());
#ifndef QT_NO_TOOLTIP
        label_582->setToolTip(QApplication::translate("preferencesDialog", "0 means that the response (hessian) threshold \n"
"used for the detector will not be adapted. \n"
"Otherwise, the threshold is modified to \n"
"generate the number of words requested.", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_582->setText(QApplication::translate("preferencesDialog", "Triangulate features without depth using stereo from motion (odometry). It would be ignored if depth as mask is checked and the feature detector used supports masking.", Q_NULLPTR));
        checkBox_memStereoFromMotion->setText(QString());
        groupBox_15->setTitle(QApplication::translate("preferencesDialog", "Sub Pixel Refining", Q_NULLPTR));
        label_44->setText(QApplication::translate("preferencesDialog", "Refining features to sub pixel. Sub pixel corners may not be needed for features like SURF/SIFT, which are already sub pixel.", Q_NULLPTR));
        label_202->setText(QApplication::translate("preferencesDialog", "Window size.", Q_NULLPTR));
        label_225->setText(QApplication::translate("preferencesDialog", "Iterations. 0 disables sub pixel refining.", Q_NULLPTR));
        label_227->setText(QApplication::translate("preferencesDialog", "Epsilon.", Q_NULLPTR));
        groupBox_dictionary_strategy2->setTitle(QApplication::translate("preferencesDialog", "Features Quantization", Q_NULLPTR));
        toolButton_dictionaryPath->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        label_78->setText(QApplication::translate("preferencesDialog", "NNDR ratio\n"
"(A matching pair is accepted, if its distance is closer than X times the distance of the second nearest neighbor)\n"
"Lower the ratio -> higher the precision.", Q_NULLPTR));
        label_dictionaryPath->setText(QApplication::translate("preferencesDialog", "Path to a pre-computed dictionary (when \"Use an incremental vocabulary\" is not set).", Q_NULLPTR));
        checkBox_dictionary_incremental->setText(QString());
        label_12->setText(QApplication::translate("preferencesDialog", "Nearest neighbor strategy.", Q_NULLPTR));
        comboBox_dictionary_strategy->clear();
        comboBox_dictionary_strategy->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "FLANN Linear", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "FLANN KdTree ", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "FLANN LSH", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Brute Force", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Brute Force GPU", Q_NULLPTR)
        );
        label_incrementalDict->setText(QApplication::translate("preferencesDialog", "Use an incremental vocabulary.\n"
"When set to false, no new words are added to dictionary, so no more updates are required after each detection, which greatly increases time performance at the cost of lower adaptation to new environments.", Q_NULLPTR));
        label_94->setText(QApplication::translate("preferencesDialog", "If the dictionary update and signature creation were parallelized.", Q_NULLPTR));
        checkBox_kp_parallelized->setText(QString());
        checkBox_kp_newWordsComparedTogether->setText(QString());
        label_186->setText(QApplication::translate("preferencesDialog", "When adding new words to dictionary, they are compared also with each other (to detect same words in the same signature).", Q_NULLPTR));
        label_260->setText(QApplication::translate("preferencesDialog", "When using a FLANN-based nearest neighbor strategy, add/remove points to its index without always rebuilding the index (the index is built only when the dictionary increases of the factor below in size).", Q_NULLPTR));
        checkBox_kp_incrementalFlann->setText(QString());
        label_451->setText(QApplication::translate("preferencesDialog", "Factor used when rebuilding the incremental FLANN index. Set 1 to disable.", Q_NULLPTR));
        label_556->setText(QApplication::translate("preferencesDialog", "For FLANN KdTree, binary descriptors are converted to float by converting each byte to float instead of converting each bit to float. When converting bytes instead of bits, less memory is used and search is faster at the cost of slightly less accurate matching.", Q_NULLPTR));
        checkBox_kp_byteToFloat->setText(QString());
        groupBox_database2->setTitle(QApplication::translate("preferencesDialog", "Database", Q_NULLPTR));
        general_checkBox_keepBinaryData->setText(QString());
        label_keepRawData->setText(QApplication::translate("preferencesDialog", "Keep sensor compressed data.", Q_NULLPTR));
        checkBox_dbInMemory->setText(QString());
        comboBox_dbJournalMode->clear();
        comboBox_dbJournalMode->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "DELETE", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "TRUNCATE", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "PERSIST", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "MEMORY", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "OFF", Q_NULLPTR)
        );
        label_74->setText(QApplication::translate("preferencesDialog", "Using database in the memory instead of a file on the hard disk (this greatly improve database access performance but it requires more RAM memory).", Q_NULLPTR));
        label_120->setText(QApplication::translate("preferencesDialog", "Sqlite3 temp store,\n"
"see Sqlite3 doc 'PRAGMA temp_store'.", Q_NULLPTR));
        comboBox_dbTempStore->clear();
        comboBox_dbTempStore->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "DEFAULT", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "FILE", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "MEMORY", Q_NULLPTR)
        );
        general_checkBox_keepNotLinkedNodes->setText(QString());
        label_75->setText(QApplication::translate("preferencesDialog", "Sqlite3 cache size,\n"
"see Sqlite3 doc 'PRAGMA cache_size'.", Q_NULLPTR));
        label_77->setText(QApplication::translate("preferencesDialog", "Sqlite3 journal mode,\n"
"see Sqlite3 doc 'PRAGMA journal_mode'.", Q_NULLPTR));
        label_89->setText(QApplication::translate("preferencesDialog", "Sqlite3 synchronous,\n"
"see Sqlite3 doc 'PRAGMA synchronous'.", Q_NULLPTR));
        comboBox_dbSynchronous->clear();
        comboBox_dbSynchronous->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "OFF", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "NORMAL", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "FULL", Q_NULLPTR)
        );
        label_keepRehearsed->setText(QApplication::translate("preferencesDialog", "Keep not linked nodes in db (rehearsed nodes and deleted nodes are saved).", Q_NULLPTR));
        label_299->setText(QApplication::translate("preferencesDialog", "RGB image compression format (should be \".jpg\" or \".png\").", Q_NULLPTR));
        label_610->setText(QApplication::translate("preferencesDialog", "Target database version for backward compatibility purpose. Only Major and minor versions are used and should be set (e.g., \"0.19\" vs \"0.20\" or \"1.0\" vs \"2.0\"). Patch version is ignored (e.g., \"0.20.1\" and \"0.20.3\" will generate a \"0.20\" database).", Q_NULLPTR));
        lineEdit_targetDatabaseVersion->setPlaceholderText(QApplication::translate("preferencesDialog", "major.minor", Q_NULLPTR));
        groupBox_bayes1->setTitle(QApplication::translate("preferencesDialog", "Loop Closure Detection", Q_NULLPTR));
        label->setText(QApplication::translate("preferencesDialog", "Prediction probabilities for each loop closure event:", Q_NULLPTR));
        label_76->setText(QApplication::translate("preferencesDialog", "We can edit how will like the prediction used in the Bayes filter when there is a loop closure hypothesis. Format is [VP LC N1 N2 N3...]. LC is the loop closure event and N# are its neighbors. VP (virtual place) is the probability to be in a new place when a loop closure was found on the last iteration.", Q_NULLPTR));
        label_65->setText(QApplication::translate("preferencesDialog", "Prediction", Q_NULLPTR));
        label_66->setText(QApplication::translate("preferencesDialog", "Sum", Q_NULLPTR));
        label_prediction_sum->setText(QString());
        label_63->setText(QApplication::translate("preferencesDialog", "Preview", Q_NULLPTR));
        checkBox_bayes_fullPredictionUpdate->setText(QString());
        label_113->setText(QApplication::translate("preferencesDialog", "The VP here is the probability to be in a new place since no loop closure was found on the last iteration.", Q_NULLPTR));
        label_142->setText(QApplication::translate("preferencesDialog", "Regenerate all the prediction matrix on each iteration (otherwise only removed/added ids are updated).", Q_NULLPTR));
        label_82->setText(QApplication::translate("preferencesDialog", "Use the tf-idf method to compute the likelihood. Otherwise, images are compared with each other.", Q_NULLPTR));
        checkBox_kp_tfIdfLikelihoodUsed->setText(QString());
        label_502->setText(QApplication::translate("preferencesDialog", "Use GPS to filter likelihood (if GPS is recorded). Only locations inside the Local Radius (see RGBD-SLAM panel) of the current GPS location are considered for loop closure detection.", Q_NULLPTR));
        checkBox_rtabmap_loopGPS->setText(QString());
        groupBox_vh_epipolar2->setTitle(QApplication::translate("preferencesDialog", "Hypothesis Verification", Q_NULLPTR));
        label_25->setText(QApplication::translate("preferencesDialog", "Minimum match count to accept a loop closure.", Q_NULLPTR));
        label_27->setText(QApplication::translate("preferencesDialog", "Fundamental Matrix : Distance (pixels) for inliers.", Q_NULLPTR));
        label_28->setText(QApplication::translate("preferencesDialog", "Fundamental Matrix : Ransac performance.", Q_NULLPTR));
        groupBox_slam1->setTitle(QApplication::translate("preferencesDialog", "RGB-D SLAM", Q_NULLPTR));
        general_checkBox_activateRGBD->setText(QString());
        label_activateRGBD->setText(QApplication::translate("preferencesDialog", "Activate metric RGB-D SLAM. This module enables 6d metric SLAM directly in RTAB-Map.\n"
"If set to false, classic RTAB-Map loop closure detection is done using only images and without any metric information.", Q_NULLPTR));
        label_83->setText(QApplication::translate("preferencesDialog", "Rigid transformations between nodes are saved on the neighbor links of the RTAB-Map's graph. On loop closures, a new constraint is added to graph and the graph optimizer optimizes the graph.", Q_NULLPTR));
        groupBox_slam_update->setTitle(QApplication::translate("preferencesDialog", "Map Update", Q_NULLPTR));
        label_432->setText(QApplication::translate("preferencesDialog", "Maximum linear speed to update the map (0 means not limit).", Q_NULLPTR));
        label_scanMatching->setText(QApplication::translate("preferencesDialog", "Neighbor link refining. When a new node is added to the graph, the transformation of its neighbor link (odometry) with the previous node is refined using ICP registration approach (laser scans required).", Q_NULLPTR));
        label_152->setText(QApplication::translate("preferencesDialog", "Angular update: Minimum angular displacement to update the map. Note that Weight Update is done prior to this, so weights are still updated.", Q_NULLPTR));
        loopClosure_reextract->setText(QString());
        rgdb_newMapOdomChange->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_scanMatching_5->setText(QApplication::translate("preferencesDialog", "Ratio of working memory for which local nodes are immunized from transfer.", Q_NULLPTR));
        rgbd_savedLocalizationIgnored->setText(QString());
        label_scanMatching_9->setText(QApplication::translate("preferencesDialog", "Re-extract visual features when computing loop closure transformations.", Q_NULLPTR));
        rgdb_linearSpeedUpdate->setSuffix(QApplication::translate("preferencesDialog", " m/s", Q_NULLPTR));
        odomScanHistory->setText(QString());
        localDetection_radius->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        rgdb_localImmunizationRatio->setSuffix(QString());
        memCovOffDiagIgnored->setText(QString());
        label_163->setText(QApplication::translate("preferencesDialog", "Odometry change detected that triggers a new map (0 means whatever the odometry change, the detector will still link the new pose in the current map). Also by default, when an odometry with Identity transformation is detected, a new map is automatically created. ", Q_NULLPTR));
        label_433->setText(QApplication::translate("preferencesDialog", "Maximum angular speed to update the map (0 means not limit).", Q_NULLPTR));
        loopClosure_bunlde->setText(QString());
        label_scanMatching_10->setText(QApplication::translate("preferencesDialog", "Do local bundle adjustment with neighborhood of the loop closure.", Q_NULLPTR));
        label_scanMatching_11->setText(QApplication::translate("preferencesDialog", "Ignore off diagonal values of the odometry covariance matrix.", Q_NULLPTR));
        rgdb_linearUpdate->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_153->setText(QApplication::translate("preferencesDialog", "Linear update: Minimum linear displacement to update the map. Note that Weight Update is done prior to this, so weights are still updated.", Q_NULLPTR));
        rgbd_loopCovLimited->setText(QString());
        rgdb_angularUpdate->setSuffix(QApplication::translate("preferencesDialog", " rad", Q_NULLPTR));
        label_scanMatching_2->setText(QApplication::translate("preferencesDialog", "If true, rtabmap will assume the robot is restarting from origin of the map. If false, rtabmap will assume the robot is restarting from the last saved localization pose from previous session (the place where it shut down previously). Used only in localization mode.", Q_NULLPTR));
        label_space2->setText(QApplication::translate("preferencesDialog", "Local radius for nodes selection in the local map. This parameter is used in some approaches of the sub-panels.", Q_NULLPTR));
        label_scanMatching_3->setText(QApplication::translate("preferencesDialog", "Maximum local locations retrieved (0=disabled) near the current pose in the local map or on the current planned path (those on the planned path have priority).", Q_NULLPTR));
        rgdb_angularSpeedUpdate->setSuffix(QApplication::translate("preferencesDialog", " rad/s", Q_NULLPTR));
        label_scanMatching_12->setText(QApplication::translate("preferencesDialog", "Limit covariance of non-neighbor links to minimum covariance of neighbor links. In other words, if covariance of a loop closure link is smaller than the minimum covariance of odometry links, its covariance is set to minimum covariance of odometry links.", Q_NULLPTR));
        odomGravity->setText(QString());
        label_space2_5->setText(QApplication::translate("preferencesDialog", "Maximum odometry cache size. Used only in localization mode and when maximum graph error parameter is not null. This is used to verify localization transforms to make sure we don't teleport to a location very similar to one we previously localized on. When the cache is full, the whole cache is cleared and the next localization is automatically accepted without verification. Set 0 to disable caching.", Q_NULLPTR));
        label_scanMatching_7->setText(QApplication::translate("preferencesDialog", "Use odometry instead of IMU orientation to add gravity links to new nodes created. We assume that odometry is already aligned with gravity (e.g., we are using a VIO approach). Gravity constraints are used by graph optimization only if \"Optimizer/GravitySigma\" is not zero.", Q_NULLPTR));
        label_space2_12->setText(QApplication::translate("preferencesDialog", "Reject loop closures/localizations if the distance from the map is over this distance (0=disabled).", Q_NULLPTR));
        maxLocalizationDistance->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_scanMatching_14->setText(QApplication::translate("preferencesDialog", "Use Identity matrix as guess when computing loop closure transform, otherwise no guess is used (assuming that registration strategy can deal with transformation estimation without guess).", Q_NULLPTR));
        loopClosure_identityGuess->setText(QString());
        groupBox_proximityDetection2->setTitle(QApplication::translate("preferencesDialog", "Proximity Detection", Q_NULLPTR));
        groupBox_localDetection_time->setTitle(QApplication::translate("preferencesDialog", "Proximity Detection in Time", Q_NULLPTR));
        label_54->setText(QApplication::translate("preferencesDialog", "Activate proximity detection over all locations in STM. The Bayes filter is not used here: If there are enough correspondences between the current image and others in STM, transformations are computed. This generates more constraints in the map's graph, so more time is required to optimize the graph.", Q_NULLPTR));
        groupBox_localDetection_space->setTitle(QApplication::translate("preferencesDialog", "Proximity Detection in Space", Q_NULLPTR));
        label_space1_2->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>Activate proximity detection over nodes (in Working Memory) near in space. The new nodes are compared to those inside the local radius of the current estimated position of the robot. There are 2 steps: </p><p>1) do a <span style=\" font-weight:600;\">one-to-one</span> proximity detection, meaning the current node is compared to closest node on each close path based current localization,</p><p>2) then if the laser scans are used, a <span style=\" font-weight:600;\">one-to-many</span> detection is done by comparing the current laser scan to a point cloud assembled with laser scans for each close path. It is useful when the robot is visiting backward (camera not in the same direction) already visited locations.</p></body></html>", Q_NULLPTR));
        localDetection_pathFilteringRadius->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_scanMatching_4->setText(QApplication::translate("preferencesDialog", "When comparing to a local path for one-to-many proximity detection, merge the scans using the odometry poses (with neighbor link optimizations) instead of the ones in the optimized local graph.", Q_NULLPTR));
        label_space3_3->setText(QApplication::translate("preferencesDialog", "Path filtering radius to reduce the number of nodes to compare in a path in one-to-many proximity detection. The nearest node in a path should be inside that radius to be considered for one-to-one proximity detection.", Q_NULLPTR));
        checkBox_localSpacePathOdomPosesUsed->setText(QString());
        label_space3_2->setText(QApplication::translate("preferencesDialog", "Maximum graph depth between the current/last loop closure location and the proximity hypotheses. Set 0 to ignore.", Q_NULLPTR));
        label_scanMatching_6->setText(QApplication::translate("preferencesDialog", "Save scan matching IDs from one-to-many proximity detection in link's user data.", Q_NULLPTR));
        checkBox_localSpaceScanMatchingIDsSaved->setText(QString());
        label_scanMatching_8->setText(QApplication::translate("preferencesDialog", "Maximum angle (degrees) for one-to-one proximity detection.", Q_NULLPTR));
        localDetection_angle->setSuffix(QApplication::translate("preferencesDialog", " deg", Q_NULLPTR));
        label_space3_4->setText(QApplication::translate("preferencesDialog", "Maximum paths compared (from the most recent) for proximity detection. 0 means no limit.", Q_NULLPTR));
        label_space3_8->setText(QApplication::translate("preferencesDialog", "Maximum neighbor nodes compared on each path for one-to-many proximity detection. Set to 0 to disable one-to-many proximity detection (by merging the laser scans).", Q_NULLPTR));
        label_scanMatching_13->setText(QApplication::translate("preferencesDialog", "Use odometry as motion guess for one-to-one proximity detection.", Q_NULLPTR));
        checkBox_localSpaceOdomGuess->setText(QString());
        groupBox_graphOptimization2->setTitle(QApplication::translate("preferencesDialog", "Graph Optimization", Q_NULLPTR));
        graphOptimization_type->clear();
        graphOptimization_type->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "TORO", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "g2o", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "GTSAM", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Ceres", Q_NULLPTR)
        );
        label_45->setText(QApplication::translate("preferencesDialog", "Graph optimization algorithm. See Optimizer panel.", Q_NULLPTR));
        graphOptimization_robust->setText(QString());
        label_258->setText(QApplication::translate("preferencesDialog", "Robust graph optimization using Vertigo (only for g2o and GTSAM optimization strategies). This approach can filter wrong loop closure detections.", Q_NULLPTR));
        graphOptimization_maxError->setSuffix(QString());
        label_259->setText(QApplication::translate("preferencesDialog", "Reject loop closures if optimization error ratio is greater than this value (0=disabled). Ratio is computed as absolute error over standard deviation of each link. This will help to detect when a wrong loop closure is added to the graph. It should not be used at the same time than Vertigo above.", Q_NULLPTR));
        graphOptimization_fromGraphEnd->setText(QString());
        label_151->setText(QApplication::translate("preferencesDialog", "Optimize graph from the newest node.", Q_NULLPTR));
        label_211->setText(QApplication::translate("preferencesDialog", "-If true, there is no odometry correction computed. All previous poses in the map are corrected instead, not the last one (which corresponds to latest odometry value). So, the transform between frames /map to /odom will be always Identity even on loop closures.", Q_NULLPTR));
        label_183->setText(QApplication::translate("preferencesDialog", "-If false, the graph is optimized from the oldest node of the current graph. It can be useful to preserve the map referential from the oldest node. An odometry correction between frames /map to /odom is computed. Warning: when some nodes are transferred, the first referential of the local map may change, resulting in momentary changes in robot/map position (which are annoying in teleoperation).", Q_NULLPTR));
        label_431->setText(QApplication::translate("preferencesDialog", "Ignore pose priors.", Q_NULLPTR));
        graphOptimization_priorsIgnored->setText(QString());
        graphOptimization_landmarksIgnored->setText(QString());
        label_521->setText(QApplication::translate("preferencesDialog", "Ignore landmarks.", Q_NULLPTR));
        label_527->setText(QApplication::translate("preferencesDialog", "Gravity sigma value (>=0, typically between 0.1 and 0.3). Optimization is done while preserving gravity orientation of the poses. This should be used only with visual/lidar inertial odometry approaches, for which we assume that all odometry poses are aligned with gravity. Set to 0 to disable gravity constraints. Currently supported only with g2o and GTSAM optimization strategies.", Q_NULLPTR));
        graphOptimization_gravitySigma->setSuffix(QString());
        groupBox_occupancy2->setTitle(QApplication::translate("preferencesDialog", "Local Occupancy Grid", Q_NULLPTR));
        doubleSpinBox_grid_noiseRadius->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        doubleSpinBox_grid_minGroundHeight->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_335->setText(QApplication::translate("preferencesDialog", "Footprint filtering height (0=disabled). Footprint length and width should be set.", Q_NULLPTR));
        doubleSpinBox_grid_maxObstacleHeight->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        doubleSpinBox_grid_maxGroundHeight->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_321->setText(QApplication::translate("preferencesDialog", "Noise filtering min neighbors.", Q_NULLPTR));
        label_310->setText(QApplication::translate("preferencesDialog", "Maximum ground height (0=disabled). Should be set if Normals Segmentation Approach is checked below.", Q_NULLPTR));
        label_309->setText(QApplication::translate("preferencesDialog", "Maximum obstacles height (0=disabled).", Q_NULLPTR));
        doubleSpinBox_grid_resolution->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_331->setText(QApplication::translate("preferencesDialog", "Laser scan decimation.", Q_NULLPTR));
        label_333->setText(QApplication::translate("preferencesDialog", "Footprint filtering length (0=disabled).", Q_NULLPTR));
        label_334->setText(QApplication::translate("preferencesDialog", "Footprint filtering width (0=disabled). Footprint length should be set.", Q_NULLPTR));
        doubleSpinBox_grid_footprintWidth->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        doubleSpinBox_grid_footprintHeight->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_327->setText(QApplication::translate("preferencesDialog", "Minimum ground height (0=disabled).", Q_NULLPTR));
        label_326->setText(QApplication::translate("preferencesDialog", "Resolution (cell size).", Q_NULLPTR));
        checkbox_rgbd_createOccupancyGrid->setText(QString());
        label_325->setText(QApplication::translate("preferencesDialog", "Noise filtering radius (0=disabled). Done after segmentation.", Q_NULLPTR));
        doubleSpinBox_grid_footprintLength->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        checkBox_grid_projMapFrame->setText(QString());
        label_365->setText(QApplication::translate("preferencesDialog", "Create local occupancy grid maps. This should be enabled to generate a global occupancy grid map. A cloud will be created from the depth image (see option below) or the 3D laser scan. This cloud will be segmented into ground and obstacle cells. For 2D laser scan, the occupancy grid is created using 2D ray casting (only Resolution and 2D Laser Scan related parameters are used).", Q_NULLPTR));
        label_311->setText(QApplication::translate("preferencesDialog", "Projection in map frame. On a 3D terrain and a fixed local camera transform (the cloud is created relative to ground), you may want to disable this to do the projection in robot frame instead.", Q_NULLPTR));
        label_323->setText(QApplication::translate("preferencesDialog", "Minimum range from the sensor.", Q_NULLPTR));
        doubleSpinBox_grid_minDepth->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_320->setText(QApplication::translate("preferencesDialog", "Maximum range from the sensor (0 means no limit).", Q_NULLPTR));
        doubleSpinBox_grid_maxDepth->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_456->setText(QApplication::translate("preferencesDialog", "Input cloud is downsampled by voxel filter (voxel size is cell size) before doing segmentation of obstacles and ground.", Q_NULLPTR));
        checkBox_grid_preVoxelFiltering->setText(QString());
        groupBox_grid_fromDepthImage->setTitle(QApplication::translate("preferencesDialog", "From Depth Image", Q_NULLPTR));
        label_octomap_groundObstacle_2->setText(QApplication::translate("preferencesDialog", "Use depth image for cloud projection instead of laser scan.", Q_NULLPTR));
        label_322->setText(QApplication::translate("preferencesDialog", "Image Decimation (1-2-4-8-...). Negative decimation is done from RGB size instead of depth size (if depth is smaller than RGB, it may be interpolated depending of the decimation value).", Q_NULLPTR));
        label_324->setText(QApplication::translate("preferencesDialog", "ROI ratios [left, right, top, bottom] between 0 and 1.", Q_NULLPTR));
        label_363->setText(QApplication::translate("preferencesDialog", "Ray tracing is done for each obstacle, filling unknown space between the sensor and obstacles. If RTAB-Map is not built with OctoMap and 3D is checked below, 3D ray tracing cannot be done.", Q_NULLPTR));
        checkBox_grid_projRayTracing->setText(QString());
        groupBox_grid_normalsSegmentation->setTitle(QApplication::translate("preferencesDialog", "Normals Segmentation Approach", Q_NULLPTR));
        label_312->setText(QApplication::translate("preferencesDialog", "In this approach, the ground of the 3D cloud is segmented using the point normals instead of a passthrough filter.", Q_NULLPTR));
        doubleSpinBox_grid_maxGroundAngle->setSuffix(QApplication::translate("preferencesDialog", " deg", Q_NULLPTR));
        label_307->setText(QApplication::translate("preferencesDialog", "Maximum angle between point's normal to ground's normal to label it as ground. Points with higher angle difference are considered as obstacles.", Q_NULLPTR));
        label_308->setText(QApplication::translate("preferencesDialog", "Minimum cluster size to project the points. The distance between clusters is defined by 2*Resolution.", Q_NULLPTR));
        label_272->setText(QApplication::translate("preferencesDialog", "Flat obstacles detected.", Q_NULLPTR));
        checkBox_grid_flatObstaclesDetected->setText(QString());
        label_330->setText(QApplication::translate("preferencesDialog", " K neighbors to compute normals.", Q_NULLPTR));
        label_336->setText(QApplication::translate("preferencesDialog", "Cluster radius.", Q_NULLPTR));
        doubleSpinBox_grid_clusterRadius->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        groupBox_grid_3d->setTitle(QApplication::translate("preferencesDialog", "3D", Q_NULLPTR));
        label_octomap_groundObstacle_3->setText(QApplication::translate("preferencesDialog", "A 3D occupancy grid is required if you want an Octomap. Set to false if you want only a 2D map, the cloud will be projected on xy plane. A 2D map can be still generated if checked, but it requires more memory and time to generate it.", Q_NULLPTR));
        label_octomap_groundObstacle->setText(QApplication::translate("preferencesDialog", "Ground segmentation (Normals Segmentation Approach) is ignored, all points are obstacles. Use this only if you want an Octomap with ground identified as an obstacle (e.g., with an UAV).", Q_NULLPTR));
        checkBox_grid_groundObstacle->setText(QString());
        groupBox_9->setTitle(QApplication::translate("preferencesDialog", "2D Laser Scan Ray Tracing", Q_NULLPTR));
        label_octomap_groundObstacle_5->setText(QApplication::translate("preferencesDialog", "These parameters only affect 2D occupancy grid created from 2D laser scan (when \"From Depth image\" is not checked and laser scan is 2D).", Q_NULLPTR));
        checkBox_grid_unknownSpaceFilled->setText(QString());
        label_328->setText(QApplication::translate("preferencesDialog", "Unknown space filled.", Q_NULLPTR));
        groupBox_occupancyGlobal2->setTitle(QApplication::translate("preferencesDialog", "Global Occupancy Grid", Q_NULLPTR));
        label_484->setText(QApplication::translate("preferencesDialog", "Probability clamping threshold minimum.", Q_NULLPTR));
        checkBox_grid_fullUpdate->setText(QString());
        label_332->setText(QApplication::translate("preferencesDialog", "Full update. When the graph is changed, the whole map will be reconstructed instead of moving individually each cells of the map. Data added to cache won't be released after updating the map. This process is longer but more robust to drift that would erase some parts of the map when it should not.", Q_NULLPTR));
        label_224->setText(QApplication::translate("preferencesDialog", "Erode obstacle cells.", Q_NULLPTR));
        label_482->setText(QApplication::translate("preferencesDialog", "Probability of a hit.", Q_NULLPTR));
        label_319->setText(QApplication::translate("preferencesDialog", "Footprint radius used to clear all obstacles under the graph.", Q_NULLPTR));
        checkBox_grid_erode->setText(QString());
        doubleSpinBox_grid_footprintRadius->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_366->setText(QApplication::translate("preferencesDialog", "Minimum map size.", Q_NULLPTR));
        doubleSpinBox_grid_minMapSize->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_455->setText(QApplication::translate("preferencesDialog", "Maximum nodes assembled in the map starting from the last node (0=unlimited).", Q_NULLPTR));
        label_454->setText(QApplication::translate("preferencesDialog", "Graph changed detection error. Update map only if poses in new optimized graph have moved more than this value.", Q_NULLPTR));
        doubleSpinBox_grid_updateError->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_457->setText(QApplication::translate("preferencesDialog", "Occupancy threshold.", Q_NULLPTR));
        doubleSpinBox_grid_occThr->setSuffix(QString());
        label_483->setText(QApplication::translate("preferencesDialog", "Probability of a miss.", Q_NULLPTR));
        doubleSpinBox_grid_probHit->setSuffix(QString());
        doubleSpinBox_grid_probMiss->setSuffix(QString());
        label_485->setText(QApplication::translate("preferencesDialog", "Probability clamping threshold maximum.", Q_NULLPTR));
        doubleSpinBox_grid_clampingMin->setSuffix(QString());
        doubleSpinBox_grid_clampingMax->setSuffix(QString());
        label_616->setText(QApplication::translate("preferencesDialog", "Altitude delta. Assemble only nodes that have the same altitude of +-delta meters of the current pose (0=disabled). This is used to generate 2D occupancy grid based on the current altitude (e.g., multi-floor building).", Q_NULLPTR));
        doubleSpinBox_grid_altitudeDelta->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        groupBox_pathPlanning2->setTitle(QApplication::translate("preferencesDialog", "Path Planning", Q_NULLPTR));
        label_space2_3->setText(QApplication::translate("preferencesDialog", " If the goal is too far (>local radius) from the graph, the plan is aborted. The next goal in the graph can't be farther than the local radius around the current position. Anticipated nodes on the path are retrieved in the local radius up to \"Maximum local locations retrieved\". A virtual link is added between the current location and the nearest one on the path up to local radius.", Q_NULLPTR));
        graphPlan_goalReachedRadius->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_space2_2->setText(QApplication::translate("preferencesDialog", "Goal reached radius.", Q_NULLPTR));
        label_space3_5->setText(QApplication::translate("preferencesDialog", "When a goal is received and processed with success, it is saved in user data of the location with this format: \"GOAL:#\".", Q_NULLPTR));
        graphPlan_goalsSavedInUserData->setText(QString());
        label_space2_4->setText(QApplication::translate("preferencesDialog", "Mark the current goal node on the path as unreachable if it a new goal is not updated after X iterations (0=disabled). If all upcoming nodes on the path are unreachabled, the plan fails.", Q_NULLPTR));
        graphPlan_linearVelocity->setSuffix(QApplication::translate("preferencesDialog", " m/s", Q_NULLPTR));
        graphPlan_angularVelocity->setSuffix(QApplication::translate("preferencesDialog", " rad/s", Q_NULLPTR));
        label_space3_6->setText(QApplication::translate("preferencesDialog", "Linear velocity used to compute path weights based on time instead of distance (0=disabled).", Q_NULLPTR));
        label_space3_7->setText(QApplication::translate("preferencesDialog", "Angular velocity used to compute path weights based on time instead of distance. Paths with poses in the same direction of the robot will have lower costs (0=disabled).", Q_NULLPTR));
        groupBox_markerDetection2->setTitle(QApplication::translate("preferencesDialog", "Marker Detection", Q_NULLPTR));
        label_space2_9->setText(QApplication::translate("preferencesDialog", "Angular variance to set on marker detections. Set to >=9999 to use only position (xyz) constraint in graph optimization.", Q_NULLPTR));
        ArucoVarianceLinear->setSuffix(QString());
        label_markerDetection->setText(QApplication::translate("preferencesDialog", "Detect static markers to be added as landmarks for graph optimization. If input data have already landmarks, this will be ignored.", Q_NULLPTR));
        label_space2_6->setText(QApplication::translate("preferencesDialog", "Linear variance to set on marker detections.", Q_NULLPTR));
        label_space2_7->setText(QApplication::translate("preferencesDialog", "Marker length. The length (m) of the markers' side. 0 means automatic marker length estimation using the depth image (the camera should look at the marker perpendicularly for initialization).", Q_NULLPTR));
        RGBDMarkerDetection->setText(QString());
        ArucoVarianceAngular->setSuffix(QString());
        ArucoMarkerLength->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_space2_8->setText(QApplication::translate("preferencesDialog", "Maximum depth error between all corners of a marker when estimating the marker length (when marker length above is 0). The smaller it is, the more perpendicular the camera should be toward the marker to initialize the length.", Q_NULLPTR));
        ArucoMaxDepthError->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_space2_13->setText(QApplication::translate("preferencesDialog", "Minimum detection range (0=disabled).", Q_NULLPTR));
        label_space2_14->setText(QApplication::translate("preferencesDialog", "Maximum detection range (0=unlimited).", Q_NULLPTR));
        ArucoMarkerRangeMin->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        ArucoMarkerRangeMax->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        groupBox_31->setTitle(QApplication::translate("preferencesDialog", "ArUco", Q_NULLPTR));
        ArucoDictionary->clear();
        ArucoDictionary->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "4X4_50", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "4X4_100", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "4X4_250", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "4X4_1000", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "5X5_50", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "5X5_100", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "5X5_250", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "5X5_1000", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "6X6_50", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "6X6_100", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "6X6_250", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "6X6_1000", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "7X7_50", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "7X7_100", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "7X7_250", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "7X7_1000", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "ARUCO_ORIGINAL", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "APRILTAG_16h5", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "APRILTAG_25h9", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "APRILTAG_36h10", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "APRILTAG_36h11", Q_NULLPTR)
        );
        label_space2_10->setText(QApplication::translate("preferencesDialog", "Dictionary to use.", Q_NULLPTR));
        ArucoCornerRefinementMethod->clear();
        ArucoCornerRefinementMethod->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "None", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Subpixel", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Contour", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "AprilTag 2", Q_NULLPTR)
        );
        label_space2_11->setText(QApplication::translate("preferencesDialog", "Corner refinement method.", Q_NULLPTR));
        groupBox_odometry1->setTitle(QApplication::translate("preferencesDialog", "Odometry", Q_NULLPTR));
        label_114->setText(QApplication::translate("preferencesDialog", "Odometry is computed as fast as possible up to \"Source->Input rate\" parameter. Odometry is not computed if already provided by the source.", Q_NULLPTR));
        label_198->setText(QApplication::translate("preferencesDialog", "Guess based on previous motion.", Q_NULLPTR));
        label_196->setText(QApplication::translate("preferencesDialog", "[Visual] Create a new keyframe when the number of inliers drops under this ratio of features in last frame. Setting the value to 0 means that a keyframe is created for each processed frame.", Q_NULLPTR));
        label_246->setText(QApplication::translate("preferencesDialog", "[Geometry] Create a new keyframe when the number of inliers drops under this threshold. Setting value to 0 means that a keyframe is created for each processed frame.", Q_NULLPTR));
        label_103->setText(QApplication::translate("preferencesDialog", "Odometry strategy.", Q_NULLPTR));
        odom_strategy->clear();
        odom_strategy->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Frame to Map", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Frame to Frame", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Fovis", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Viso2", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "DVO", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "ORB SLAM", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "OKVIS", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "LOAM", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "MSCKF_VIO", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "VINS-Fusion", Q_NULLPTR)
        );
        label_314->setText(QApplication::translate("preferencesDialog", "Align odometry with ground on initialization.", Q_NULLPTR));
        odom_alignWithGround->setText(QString());
        label_238->setText(QApplication::translate("preferencesDialog", "If the robot is holonomic (strafing commands can be issued). If not, y value will be estimated from x and yaw values (y=x*tan(yaw)).", Q_NULLPTR));
        odom_flow_guessMotion->setText(QString());
        pushButton_testOdometry->setText(QApplication::translate("preferencesDialog", "Test odometry", Q_NULLPTR));
        label_221->setText(QApplication::translate("preferencesDialog", "Fill info with data (inliers/outliers features to be shown in Odometry view).", Q_NULLPTR));
        label_232->setText(QApplication::translate("preferencesDialog", "Data buffer size (0 means inf).", Q_NULLPTR));
        label_47->setText(QApplication::translate("preferencesDialog", "Automatically reset odometry after X consecutive images on which odometry cannot be computed (value=0 disables auto-reset). When reset, the odometry starts from the last pose computed.", Q_NULLPTR));
        label_248->setText(QApplication::translate("preferencesDialog", "Decimation of images before features extraction. Negative decimation is done from RGB size instead of depth size (if depth is smaller than RGB, it may be interpolated depending of the decimation value).", Q_NULLPTR));
        odom_filteringStrategy->clear();
        odom_filteringStrategy->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "No filtering", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Kalman filtering", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Particle filtering", Q_NULLPTR)
        );
        odom_fillInfoData->setText(QString());
        odom_holonomic->setText(QString());
        label_233->setText(QApplication::translate("preferencesDialog", "Pose estimation filtering strategy.", Q_NULLPTR));
        label_351->setText(QApplication::translate("preferencesDialog", "Registration approach. Default means the same registration approach used in Motion Estimation panel (same used by the loop closure detector).", Q_NULLPTR));
        odom_registration->clear();
        odom_registration->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Visual", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Geometry (ICP)", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Visual + Geometry", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "[Default]", Q_NULLPTR)
        );
        label_354->setText(QApplication::translate("preferencesDialog", "[Visual] Create a new keyframe when the number of inliers drops under this threshold. Setting the value to 0 means that a keyframe is created for each processed frame.", Q_NULLPTR));
        label_520->setText(QApplication::translate("preferencesDialog", "Guess smoothing delay (s). Estimated velocity is averaged based on last transforms up to this maximum delay. This can help to get smoother velocity prediction. Last velocity computed is used directly if a filtering strategy is set or the delay is below the odometry rate.", Q_NULLPTR));
        odom_guess_smoothing_delay->setSuffix(QApplication::translate("preferencesDialog", " s", Q_NULLPTR));
        groupBox_odomF2M->setTitle(QApplication::translate("preferencesDialog", "Frame to Map", Q_NULLPTR));
        label_226->setText(QApplication::translate("preferencesDialog", "The approach maintains a local map of features to match to. It adds incrementally new points to local map up to maximum map size, then removes old points as new points are added. See \"Motion Estimation\" panel for registration parameters.", Q_NULLPTR));
        doubleSpinBox_odom_f2m_scanRadius->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_215->setText(QApplication::translate("preferencesDialog", "[Geometry] Radius used to filter points of a new added scan to local map. This could match the voxel size of the laser scans.", Q_NULLPTR));
        label_195->setText(QApplication::translate("preferencesDialog", "[Geometry] Maximum local scan map size (points).", Q_NULLPTR));
        label_190->setText(QApplication::translate("preferencesDialog", "[Visual] Maximum map size: If > 0 (example 5000), the odometry will maintain a local map of X maximum features. This will decrease odometry drifting when the camera is not moving.", Q_NULLPTR));
        label_194->setText(QApplication::translate("preferencesDialog", "[Visual] Maximum features (sorted by keypoint response) added to local map from a new key-frame. 0 means no limit.", Q_NULLPTR));
        label_357->setText(QApplication::translate("preferencesDialog", "[Visual] Local bundle adjustment. See Optimizer panel. This will not work if Optical Flow correspondences strategy is selected in Visual Registration panel.", Q_NULLPTR));
        label_358->setText(QApplication::translate("preferencesDialog", "[Visual] Maximum frames used for bundle adjustment (0=inf or all current frames in the local map).", Q_NULLPTR));
        odom_f2m_bundleStrategy->clear();
        odom_f2m_bundleStrategy->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Disabled", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "g2o", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "cvsba", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Ceres", Q_NULLPTR)
        );
        label_430->setText(QApplication::translate("preferencesDialog", "[Geometry] Max angle (degrees) used to filter points of a new added scan to local map (when Radius above is >0). 0 means any angle.", Q_NULLPTR));
        doubleSpinBox_odom_f2m_scanAngle->setSuffix(QApplication::translate("preferencesDialog", " deg", Q_NULLPTR));
        label_524->setText(QApplication::translate("preferencesDialog", "[Visual] If a new frame has points without valid depth, they are added to local feature map only if points with valid depth on total points is over this ratio. Setting to 1 means no points without valid depth are added to local feature map.", Q_NULLPTR));
        odom_f2m_validDepthRatio->setSuffix(QString());
        label_564->setText(QApplication::translate("preferencesDialog", "[Geometry] Distance Range used to filter points of local map (when > 0). 0 means local map is updated using time and not range.", Q_NULLPTR));
        doubleSpinBox_odom_f2m_scanRange->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_598->setText(QApplication::translate("preferencesDialog", "[Visual] Gravity sigma used for bundle adjustment (<0, use same value than Optimizer/GravitySigma parameter)", Q_NULLPTR));
        odom_f2m_gravitySigma->setSuffix(QString());
        groupBox_odomF2F->setTitle(QApplication::translate("preferencesDialog", "Frame to Frame", Q_NULLPTR));
        label_199->setText(QApplication::translate("preferencesDialog", "Motion is computed with a frame to key-frame approach. See Motion Estimation panel for registration parameters.", Q_NULLPTR));
        comboBox_odomf2f_corType->clear();
        comboBox_odomf2f_corType->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Features Matching", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Optical Flow", Q_NULLPTR)
        );
        label_149->setText(QApplication::translate("preferencesDialog", "[Visual] Correspondences computation approach.", Q_NULLPTR));
        groupBox_odomf2f_corType->setTitle(QApplication::translate("preferencesDialog", "[Visual] Correspondences computation: Optical Flow", Q_NULLPTR));
        label_201->setText(QApplication::translate("preferencesDialog", "Window size.", Q_NULLPTR));
        label_295->setText(QApplication::translate("preferencesDialog", "Iterations.", Q_NULLPTR));
        label_296->setText(QApplication::translate("preferencesDialog", "Epsilon.", Q_NULLPTR));
        label_297->setText(QApplication::translate("preferencesDialog", "Max level.", Q_NULLPTR));
        groupBox_odomFovis->setTitle(QApplication::translate("preferencesDialog", "Fovis", Q_NULLPTR));
        label_234->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>Fast Odometry from VISion: <a href=\"https://github.com/srv/libfovis\"><span style=\" text-decoration: underline; color:#0000ff;\">https://github.com/srv/libfovis</span></a></p></body></html>", Q_NULLPTR));
        groupBox_23->setTitle(QApplication::translate("preferencesDialog", "Motion Estimator", Q_NULLPTR));
        spinBox_OdomFovisFeatureSearchWindow->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        label_371->setText(QApplication::translate("preferencesDialog", "The maximum image-space reprojection error (in pixels) a feature match is allowed to have and still be considered an inlier in the set of features used for motion estimation.", Q_NULLPTR));
        label_377->setText(QApplication::translate("preferencesDialog", "Specifies the size of the search window to apply when searching for feature matches across time frames.  The search is conducted around the feature location predicted by the initial rotation estimate.", Q_NULLPTR));
        checkBox_OdomFovisUpdateTargetFeaturesWithRefined->setText(QString());
        label_374->setText(QApplication::translate("preferencesDialog", "Minimum number of features in the inlier set for the motion estimate to be considered valid.", Q_NULLPTR));
        doubleSpinBox_OdomFovisInlierMaxReprojectionError->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        checkBox_OdomFovisUseSubpixelRefinement->setText(QString());
        label_375->setText(QApplication::translate("preferencesDialog", "Maximum mean reprojection error over the inlier feature matches for the motion estimate to be considered valid.", Q_NULLPTR));
        doubleSpinBox_OdomFovisCliqueInlierThreshold->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        doubleSpinBox_OdomFovisMaxMeanReprojectionError->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        label_250->setText(QApplication::translate("preferencesDialog", "See Howard's greedy max-clique algorithm for determining the maximum set of mutually consisten feature matches. This specifies the compatibility threshold, in meters.", Q_NULLPTR));
        label_376->setText(QApplication::translate("preferencesDialog", "Specifies whether or not to refine feature matches to subpixel resolution.", Q_NULLPTR));
        label_379->setText(QApplication::translate("preferencesDialog", "When subpixel refinement is enabled, the refined feature locations can be saved over the original feature locations.  This has a slightly negative impact on frame-to-frame visual odometry, but is likely better when using this library as part of a visual SLAM algorithm.", Q_NULLPTR));
        groupBox_24->setTitle(QApplication::translate("preferencesDialog", "Features", Q_NULLPTR));
        spinBox_OdomFovisFastThreshold->setSuffix(QString());
        checkBox_OdomFovisUseAdaptiveThreshold->setText(QString());
        doubleSpinBox_OdomFovisFastThresholdAdaptiveGain->setSuffix(QString());
        label_382->setText(QApplication::translate("preferencesDialog", "FAST threshold.", Q_NULLPTR));
        label_373->setText(QApplication::translate("preferencesDialog", "The maximum Gaussian pyramid level to process the image at. Pyramid level 1 corresponds to the original image.", Q_NULLPTR));
        spinBox_OdomFovisTargetPixelsPerFeature->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        label_381->setText(QApplication::translate("preferencesDialog", "The minimum pyramid level.", Q_NULLPTR));
        spinBox_OdomFovisFeatureWindowSize->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        label_372->setText(QApplication::translate("preferencesDialog", "The size of the n x n image patch surrounding each feature, used for keypoint matching.", Q_NULLPTR));
        label_378->setText(QApplication::translate("preferencesDialog", "Specifies the desired feature density as a ratio of input image pixels per feature detected.  This number is used to control the adaptive feature thresholding.", Q_NULLPTR));
        checkBox_OdomFovisUseHomographyInitialization->setText(QString());
        label_383->setText(QApplication::translate("preferencesDialog", "Use FAST adaptive threshold.", Q_NULLPTR));
        label_400->setText(QApplication::translate("preferencesDialog", "FAST threshold adaptive gain.", Q_NULLPTR));
        label_401->setText(QApplication::translate("preferencesDialog", "Use homography initialization.", Q_NULLPTR));
        groupBox_25->setTitle(QApplication::translate("preferencesDialog", "Bucket", Q_NULLPTR));
        checkBox_OdomFovisUseBucketing->setText(QString());
        label_402->setText(QApplication::translate("preferencesDialog", "Use bucketing.", Q_NULLPTR));
        spinBox_OdomFovisBucketHeight->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        spinBox_OdomFovisMaxKeypointsPerBucket->setSuffix(QString());
        spinBox_OdomFovisBucketWidth->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        checkBox_OdomFovisUseImageNormalization->setText(QString());
        label_403->setText(QApplication::translate("preferencesDialog", "Bucket width.", Q_NULLPTR));
        label_404->setText(QApplication::translate("preferencesDialog", "Bucket height.", Q_NULLPTR));
        label_405->setText(QApplication::translate("preferencesDialog", "Max Keypoints per bucket.", Q_NULLPTR));
        label_406->setText(QApplication::translate("preferencesDialog", "Use image normalization.", Q_NULLPTR));
        groupBox_26->setTitle(QApplication::translate("preferencesDialog", "Stereo", Q_NULLPTR));
        doubleSpinBox_OdomFovisStereoMaxDistEpipolarLine->setSuffix(QString());
        doubleSpinBox_OdomFovisStereoMaxRefinementDisplacement->setSuffix(QString());
        checkBox_OdomFovisStereoRequireMutualMatch->setText(QString());
        label_407->setText(QApplication::translate("preferencesDialog", "Require mutual match.", Q_NULLPTR));
        spinBox_OdomFovisStereoMaxDisparity->setSuffix(QString());
        label_408->setText(QApplication::translate("preferencesDialog", "Maximum distance to epipolar line.", Q_NULLPTR));
        label_409->setText(QApplication::translate("preferencesDialog", "Max refinement displacement.", Q_NULLPTR));
        label_410->setText(QApplication::translate("preferencesDialog", "Max disparity.", Q_NULLPTR));
        groupBox_odomViso2->setTitle(QApplication::translate("preferencesDialog", "Viso2", Q_NULLPTR));
        label_380->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>Library for Visual Odometry 2: <a href=\"http://www.cvlibs.net/software/libviso/\"><span style=\" text-decoration: underline; color:#0000ff;\">http://www.cvlibs.net/software/libviso</span></a></p><p><span style=\" font-weight:600;\">Only stereo is supported.</span><br/></p></body></html>", Q_NULLPTR));
        label_386->setText(QApplication::translate("preferencesDialog", "Fundamental matrix inlier threshold.", Q_NULLPTR));
        label_384->setText(QApplication::translate("preferencesDialog", "Lower border weights (more robust to calibration errors).", Q_NULLPTR));
        label_385->setText(QApplication::translate("preferencesDialog", "Number of RANSAC iterations.", Q_NULLPTR));
        doubleSpinBox_OdomViso2InlierThreshold->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        checkBox_OdomViso2Reweighting->setText(QString());
        groupBox->setTitle(QApplication::translate("preferencesDialog", "Match", Q_NULLPTR));
        label_389->setText(QApplication::translate("preferencesDialog", "Matching bin width/height (affects efficiency only).", Q_NULLPTR));
        label_388->setText(QApplication::translate("preferencesDialog", "Non-max-suppression: interest point peakiness threshold.", Q_NULLPTR));
        label_391->setText(QApplication::translate("preferencesDialog", "Disparity tolerance for stereo matches (in pixels).", Q_NULLPTR));
        spinBox_OdomViso2MatchNmsN->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        label_387->setText(QApplication::translate("preferencesDialog", "Non-max-suppression: min. distance between maxima (in pixels).", Q_NULLPTR));
        label_392->setText(QApplication::translate("preferencesDialog", "Outlier removal: disparity tolerance (in pixels).", Q_NULLPTR));
        label_395->setText(QApplication::translate("preferencesDialog", "Match at half resolution, refine at full resolution.", Q_NULLPTR));
        label_394->setText(QApplication::translate("preferencesDialog", "Multistage matching (denser and faster).", Q_NULLPTR));
        label_393->setText(QApplication::translate("preferencesDialog", "Outlier removal: flow tolerance (in pixels).", Q_NULLPTR));
        label_390->setText(QApplication::translate("preferencesDialog", "Matching radius (du/dv in pixels).", Q_NULLPTR));
        label_396->setText(QApplication::translate("preferencesDialog", "Refinement (0=none,1=pixel,2=subpixel).", Q_NULLPTR));
        spinBox_OdomViso2MatchBinsize->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        spinBox_OdomViso2MatchRadius->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        spinBox_OdomViso2MatchDispTolerance->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        checkBox_OdomViso2MatchMultiStage->setText(QString());
        checkBox_OdomViso2MatchHalfResolution->setText(QString());
        groupBox_22->setTitle(QApplication::translate("preferencesDialog", "Bucket", Q_NULLPTR));
        label_398->setText(QApplication::translate("preferencesDialog", "Width of bucket.", Q_NULLPTR));
        label_397->setText(QApplication::translate("preferencesDialog", "Maximal number of features per bucket.", Q_NULLPTR));
        label_399->setText(QApplication::translate("preferencesDialog", "Height of bucket.", Q_NULLPTR));
        doubleSpinBox_OdomViso2BucketWidth->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        doubleSpinBox_OdomViso2BucketHeight->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        groupBox_odomDVO->setTitle(QApplication::translate("preferencesDialog", "DVO", Q_NULLPTR));
        label_411->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>Dense Visual Odometry: <a href=\"https://github.com/tum-vision/dvo_slam\"><span style=\" text-decoration: underline; color:#0000ff;\">https://github.com/tum-vision/dvo_slam</span></a></p><p><span style=\" font-weight:600;\">Only RGB-D is supported.</span><br/></p></body></html>", Q_NULLPTR));
        groupBox_odomORBSLAM->setTitle(QApplication::translate("preferencesDialog", "ORB SLAM", Q_NULLPTR));
        label_412->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>ORB SLAM2: <a href=\"https://github.com/raulmur/ORB_SLAM2\"><span style=\" text-decoration: underline; color:#0000ff;\">https://github.com/UZ-SLAMLab/ORB_SLAM3</span></a><br/>ORB SLAM3: <a href=\"https://github.com/UZ-SLAMLab/ORB_SLAM3\"><span style=\" text-decoration: underline; color:#0000ff;\">https://github.com/raulmur/ORB_SLAM2</span></a></p><p>Tracking and local mapping from ORB SLAM is done but loop closure detection is disabled, using RTAB-Map loop closure detection instead. See ORB panel for ORB parameters (pyramid scale and number of pyramids). See FAST panel for FAST parameters (init and min thresholds). Vis/MaxFeatures (see Visual Registration panel) is used as the number of ORB features extracted.<br/></p></body></html>", Q_NULLPTR));
        doubleSpinBox_OdomORBSLAMBf->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_414->setText(QApplication::translate("preferencesDialog", "Close/Far threshold. Baseline times.", Q_NULLPTR));
        doubleSpinBox_OdomORBSLAMThDepth->setSuffix(QString());
        label_413->setText(QApplication::translate("preferencesDialog", "Fake IR projector baseline used only when stereo is not used.", Q_NULLPTR));
        label_415->setText(QApplication::translate("preferencesDialog", "Path to ORB vocabulary (*.txt).", Q_NULLPTR));
        toolButton_OdomORBSLAMVocPath->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        label_444->setText(QApplication::translate("preferencesDialog", "Camera FPS. This parameter is linked to \"Input rate\" of the Source panel.", Q_NULLPTR));
        doubleSpinBox_OdomORBSLAMFps->setSuffix(QString());
        label_445->setText(QApplication::translate("preferencesDialog", "Maximum ORB features extracted per frame.", Q_NULLPTR));
        label_449->setText(QApplication::translate("preferencesDialog", "Maximum size of the feature map (0 means infinite).", Q_NULLPTR));
        groupBox_odomOKVIS->setTitle(QApplication::translate("preferencesDialog", "OKVIS", Q_NULLPTR));
        label_466->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>OKVIS: <a href=\"https://github.com/ethz-asl/okvis\"><span style=\" text-decoration: underline; color:#0000ff;\">https://github.com/ethz-asl/okvis</span></a></p><p>IMU input required. Currently tested only with EuRoC data set.</p></body></html>", Q_NULLPTR));
        label_469->setText(QApplication::translate("preferencesDialog", "Path to OKVIS config file (*.yaml). While OKVIS requires calibrations in the config file, they are ignored as those from received images are used instead.", Q_NULLPTR));
        toolButton_OdomOkvisPath->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        groupBox_odomLOAM->setTitle(QApplication::translate("preferencesDialog", "LOAM", Q_NULLPTR));
        label_472->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>LOAM: <a href=\"https://github.com/laboshinl/loam_velodyne\"><span style=\" text-decoration: underline; color:#0000ff;\">https://github.com/laboshinl/loam_velodyne</span></a></p><p>Velodyne input required. Currently tested only with KITTI data set and with pull request #66.</p></body></html>", Q_NULLPTR));
        label_475->setText(QApplication::translate("preferencesDialog", "Linear output variance.", Q_NULLPTR));
        label_473->setText(QApplication::translate("preferencesDialog", "Velodyne sensor.", Q_NULLPTR));
        label_474->setText(QApplication::translate("preferencesDialog", "Scan period (s).", Q_NULLPTR));
        odom_loam_sensor->clear();
        odom_loam_sensor->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "VLP-16", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "HDL-32", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "HDL-64E", Q_NULLPTR)
        );
        label_476->setText(QApplication::translate("preferencesDialog", "Angular output variance.", Q_NULLPTR));
        odom_loam_localMapping->setText(QString());
        label_477->setText(QApplication::translate("preferencesDialog", "Local mapping. It adds more time to compute odometry, but accuracy is significantly improved.", Q_NULLPTR));
        groupBox_odomMSCKF->setTitle(QApplication::translate("preferencesDialog", "MSCKF_VIO", Q_NULLPTR));
        label_480->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>MSCKF_VIO: <a href=\"https://github.com/KumarRobotics/msckf_vio\"><span style=\" text-decoration: underline; color:#0000ff;\">https://github.com/KumarRobotics/msckf_vio</span></a></p><p>Stereo and IMU inputs required. Currently tested only with EuRoC data set.</p></body></html>", Q_NULLPTR));
        groupBox_7->setTitle(QApplication::translate("preferencesDialog", "Image processing", Q_NULLPTR));
        label_538->setText(QApplication::translate("preferencesDialog", "Grid row.", Q_NULLPTR));
        label_537->setText(QApplication::translate("preferencesDialog", "Grid col.", Q_NULLPTR));
        label_536->setText(QApplication::translate("preferencesDialog", "Grid min feature num.", Q_NULLPTR));
        label_535->setText(QApplication::translate("preferencesDialog", "Grid max feature num.", Q_NULLPTR));
        label_534->setText(QApplication::translate("preferencesDialog", "Pyramid levels.", Q_NULLPTR));
        label_533->setText(QApplication::translate("preferencesDialog", "Patch size.", Q_NULLPTR));
        label_532->setText(QApplication::translate("preferencesDialog", "Fast threshold.", Q_NULLPTR));
        label_531->setText(QApplication::translate("preferencesDialog", "Max iteration.", Q_NULLPTR));
        label_495->setText(QApplication::translate("preferencesDialog", "Tracking precision.", Q_NULLPTR));
        label_496->setText(QApplication::translate("preferencesDialog", "Ransac treshold.", Q_NULLPTR));
        label_497->setText(QApplication::translate("preferencesDialog", "Stereo threshold.", Q_NULLPTR));
        groupBox_28->setTitle(QApplication::translate("preferencesDialog", "Odometry estimation", Q_NULLPTR));
        label_539->setText(QApplication::translate("preferencesDialog", "Max cam state size.", Q_NULLPTR));
        label_486->setText(QApplication::translate("preferencesDialog", "Position std threshold.", Q_NULLPTR));
        label_487->setText(QApplication::translate("preferencesDialog", "Rotation threshold.", Q_NULLPTR));
        label_488->setText(QApplication::translate("preferencesDialog", "Translation threshold.", Q_NULLPTR));
        label_489->setText(QApplication::translate("preferencesDialog", "Tracking rate threshold.", Q_NULLPTR));
        label_490->setText(QApplication::translate("preferencesDialog", "Optimization translation threshold.", Q_NULLPTR));
        label_491->setText(QApplication::translate("preferencesDialog", "Noise gyro.", Q_NULLPTR));
        label_492->setText(QApplication::translate("preferencesDialog", "Noise acceleration.", Q_NULLPTR));
        label_493->setText(QApplication::translate("preferencesDialog", "Noise gyro bias.", Q_NULLPTR));
        label_494->setText(QApplication::translate("preferencesDialog", "Noise acceleration bias.", Q_NULLPTR));
        label_507->setText(QApplication::translate("preferencesDialog", "Noise feature.", Q_NULLPTR));
        label_508->setText(QApplication::translate("preferencesDialog", "Initial covariance velocity.", Q_NULLPTR));
        label_509->setText(QApplication::translate("preferencesDialog", "Initial covariance gyro bias.", Q_NULLPTR));
        label_510->setText(QApplication::translate("preferencesDialog", "Initial covariance acceleration bias.", Q_NULLPTR));
        label_511->setText(QApplication::translate("preferencesDialog", "Initial covariance extrincic rotation.", Q_NULLPTR));
        label_513->setText(QApplication::translate("preferencesDialog", "Initial covariance extrinsic translation.", Q_NULLPTR));
        groupBox_odomVINS->setTitle(QApplication::translate("preferencesDialog", "VINS-Fusion", Q_NULLPTR));
        label_525->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>VINS-Fusion: <a href=\"https://github.com/HKUST-Aerial-Robotics/VINS-Fusion\"><span style=\" text-decoration: underline; color:#0000ff;\">https://github.com/HKUST-Aerial-Robotics/VINS-Fusion</span></a></p><p>IMU input optional depending on the parameters in the config file. Currently tested only with EuRoC data set.</p></body></html>", Q_NULLPTR));
        label_526->setText(QApplication::translate("preferencesDialog", "Path to VINS-Fusion config file (*.yaml). While VINS-Fusion requires calibrations in the config file, they are ignored as those from received images are used instead.", Q_NULLPTR));
        toolButton_OdomVinsPath->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        groupBox_odomMono->setTitle(QApplication::translate("preferencesDialog", "Mono", Q_NULLPTR));
        label_672->setText(QApplication::translate("preferencesDialog", "Mono is for single camera motion estimation (MonoSLAM). On initialization, the camera must be translated on the side until a first transform can be computed.", Q_NULLPTR));
        label_661->setText(QApplication::translate("preferencesDialog", "Parameters from BOW and OpticalFlow are also used here. PnP parameters on Odometry panel are used too.", Q_NULLPTR));
        doubleSpinBox_minFlow->setSuffix(QApplication::translate("preferencesDialog", " pixels", Q_NULLPTR));
        label_659->setText(QApplication::translate("preferencesDialog", "Minimum optical flow required for the initialization step.", Q_NULLPTR));
        doubleSpinBox_minTranslation->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_660->setText(QApplication::translate("preferencesDialog", "Minimum translation to add new points to local map. On initialization, translation x 5 is used as the required minimum transformation.", Q_NULLPTR));
        label_662->setText(QApplication::translate("preferencesDialog", "Minimum translation required for the initialization step.", Q_NULLPTR));
        doubleSpinBox_minInitTranslation->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_663->setText(QApplication::translate("preferencesDialog", "Maximum variance to add new points to local map.", Q_NULLPTR));
        doubleSpinBox_maxVariance->setSuffix(QString());
        groupBox_odometryKalmanFilter2->setTitle(QApplication::translate("preferencesDialog", "Kalman Filter", Q_NULLPTR));
        label_673->setText(QApplication::translate("preferencesDialog", "Parameters for the Kalman filter when used to smooth the odometry trajectory.", Q_NULLPTR));
        doubleSpinBox_kalmanMeasurementNoise->setSuffix(QString());
        doubleSpinBox_kalmanProcessNoise->setSuffix(QString());
        label_674->setText(QApplication::translate("preferencesDialog", "Process noise.", Q_NULLPTR));
        label_675->setText(QApplication::translate("preferencesDialog", "Measurement noise.", Q_NULLPTR));
        groupBox_odometryParticleFilter2->setTitle(QApplication::translate("preferencesDialog", "Particle Filter", Q_NULLPTR));
        label_665->setText(QApplication::translate("preferencesDialog", "Parameters for the particle filter when used to smooth the odometry trajectory.", Q_NULLPTR));
        doubleSpinBox_particleLambdaT->setSuffix(QString());
        doubleSpinBox_particleNoiseR->setSuffix(QApplication::translate("preferencesDialog", " rad", Q_NULLPTR));
        doubleSpinBox_particleNoiseT->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_666->setText(QApplication::translate("preferencesDialog", "Noise of translation components (x,y,z).", Q_NULLPTR));
        label_667->setText(QApplication::translate("preferencesDialog", "Noise of rotation components (roll, pitch, yaw).", Q_NULLPTR));
        label_668->setText(QApplication::translate("preferencesDialog", "Lambda of translation components (x,y,z).", Q_NULLPTR));
        label_669->setText(QApplication::translate("preferencesDialog", "Lambda of rotation components (roll, pitch, yaw).", Q_NULLPTR));
        doubleSpinBox_particleLambdaR->setSuffix(QString());
        label_670->setText(QApplication::translate("preferencesDialog", "Particle size.", Q_NULLPTR));
        label_162->setText(QString());
        groupBox_motionEst1->setTitle(QApplication::translate("preferencesDialog", "Motion Estimation", Q_NULLPTR));
        comboBox_registrationStrategy->clear();
        comboBox_registrationStrategy->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Visual", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Geometry (ICP)", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Visual + Geometry", Q_NULLPTR)
        );
        label_11->setText(QApplication::translate("preferencesDialog", "For the parameters for each registration approach, see the sub panels. Approaches with '+' means that both registrations are done in that order.", Q_NULLPTR));
        loopClosure_bowForce2D->setText(QString());
        label_184->setText(QApplication::translate("preferencesDialog", "Force 2D transform (3DoF: x,y and yaw).", Q_NULLPTR));
        label_370->setText(QApplication::translate("preferencesDialog", "Do a second registration with the output of the first registration as guess. Only done if no guess was provided for the first registration. It can be useful if the registration approach used can use a guess to get better matches.", Q_NULLPTR));
        reg_repeatOnce->setText(QString());
        groupBox_visualTransform2->setTitle(QApplication::translate("preferencesDialog", "Visual Registration", Q_NULLPTR));
        loopClosure_estimationType->clear();
        loopClosure_estimationType->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "3D to 3D", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "3D to 2D (PnP)", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "2D to 2D", Q_NULLPTR)
        );
        label_15->setText(QApplication::translate("preferencesDialog", "Maximum RANSAC iterations.", Q_NULLPTR));
        label_298->setText(QApplication::translate("preferencesDialog", "Forward estimation only (A->B). If false, a transformation is also computed in backward direction (B->A), then the two resulting transforms are merged (middle interpolation between the transforms).", Q_NULLPTR));
        loopClosure_forwardEst->setText(QString());
        label_146->setText(QApplication::translate("preferencesDialog", "Motion estimation approach using 3D and/or 2D visual words correspondences.", Q_NULLPTR));
        label_2->setText(QApplication::translate("preferencesDialog", "Minimum correspondences to accept the estimated transformation.", Q_NULLPTR));
        loopClosure_bundle->clear();
        loopClosure_bundle->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Disabled", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "g2o", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "cvsba", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Ceres", Q_NULLPTR)
        );
        label_346->setText(QApplication::translate("preferencesDialog", "Refine transformation with bundle adjustment. See Optimizer panel.", Q_NULLPTR));
        label_554->setText(QApplication::translate("preferencesDialog", "Maximum distance (m) of the mean distance of inliers from the camera to accept the transformation. 0 means disabled.", Q_NULLPTR));
        label_555->setText(QApplication::translate("preferencesDialog", "Minimum distribution value of the inliers in the image to accept the transformation. The distribution is the second eigen value of the PCA (Principal Component Analysis) on the keypoints of the normalized image [-0.5, 0.5]. 0 means disabled.", Q_NULLPTR));
        visMeanDistance->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        groupBox_4->setTitle(QApplication::translate("preferencesDialog", "Features Matching", Q_NULLPTR));
        reextract_nn->clear();
        reextract_nn->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "FLANN Linear", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "FLANN KdTree", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "FLANN LSH", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Brute Force", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Brute Force GPU", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Brute Force Cross Check", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "PyMatcher", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "GMS", Q_NULLPTR)
        );
        label_179->setText(QApplication::translate("preferencesDialog", "Nearest neighbor strategy. FLANN LSH must be used only with binary feature detector.", Q_NULLPTR));
        label_181->setText(QApplication::translate("preferencesDialog", "NNDR (Nearest Neighbor Distance Ratio)\n"
"(A matching pair is accepted, if its distance is closer than X times the distance of the second nearest neighbor)\n"
"Lower the ratio -> higher the precision.", Q_NULLPTR));
        spinBox_visCorGuessWinSize->setSuffix(QApplication::translate("preferencesDialog", " pixels", Q_NULLPTR));
        label_303->setText(QApplication::translate("preferencesDialog", "Matching window size around projected points when a guess transform is provided to find correspondences. 0 means that global matching will be done.", Q_NULLPTR));
        checkBox__visCorGuessMatchToProjection->setText(QString());
        label_448->setText(QApplication::translate("preferencesDialog", "Match frame's corners to source's projected points (when guess transform is provided) instead of projected points to frame's corners.", Q_NULLPTR));
        groupBox_pymatcher->setTitle(QApplication::translate("preferencesDialog", "PyMatcher", Q_NULLPTR));
        label_586->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>Python wrapper for python matcher like <a href=\"https://github.com/magicleap/SuperGluePretrainedNetwork\"><span style=\" text-decoration: underline; color:#0000ff;\">SuperGlue</span></a> or <a href=\"https://github.com/zjhthu/OANet\"><span style=\" text-decoration: underline; color:#0000ff;\">OANet</span></a>. Download the <a href=\"https://github.com/introlab/rtabmap/tree/master/corelib/src/python\"><span style=\" text-decoration: underline; color:#0000ff;\">scripts</span></a> and copy them at the root folder of SuperGlue git or in demo folder of OANet respectively, then set its path below. <span style=\" font-weight:600;\">Important</span>: Only float descriptors are allowed and SuperGlue works only with SuperPoint descriptors! You may have to train a new model for the feature used.</p></body></html>", Q_NULLPTR));
        toolButton_pymatcher_path->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        label_585->setText(QApplication::translate("preferencesDialog", "Match threshold. Used by SuperGlue.", Q_NULLPTR));
        label_584->setText(QApplication::translate("preferencesDialog", "Iterations. Used by SuperGlue.", Q_NULLPTR));
        label_583->setText(QApplication::translate("preferencesDialog", "[Required] Path to rtabmap python script.", Q_NULLPTR));
        label_587->setText(QApplication::translate("preferencesDialog", "Cuda. Used by SuperGlue.", Q_NULLPTR));
        checkBox_pymatcher_cuda->setText(QString());
        label_590->setText(QApplication::translate("preferencesDialog", "For SuperGlue, set only \"indoor\" or \"outdoor\". For OANet, set path to one of the pth file (e.g., \"~/OANet/model/gl3d/sift-4000/model_best.pth\").", Q_NULLPTR));
        toolButton_pymatcher_model->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        groupBox_gms->setTitle(QApplication::translate("preferencesDialog", "GMS", Q_NULLPTR));
        label_588->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>OpenCV implementation of <a href=\"https://github.com/JiawangBian/GMS-Feature-Matcher\"><span style=\" text-decoration: underline; color:#0000ff;\">GMS</span></a>. Since GMS works well when the number of features is large, it is recommended to use the ORB feature and set FastThreshold to 0 to get as many as possible features quickly. If matching results are not satisfying, please add more features (they used 10000 for images of size 640 X 480). If your images have big rotation and scale changes, please set withRotation or withScale to true.</p></body></html>", Q_NULLPTR));
        label_592->setText(QApplication::translate("preferencesDialog", "With rotation. Take rotation transformation into account.", Q_NULLPTR));
        checkBox_gms_withRotation->setText(QString());
        label_589->setText(QApplication::translate("preferencesDialog", "Theshold factor. The higher, the less matches.", Q_NULLPTR));
        label_593->setText(QApplication::translate("preferencesDialog", "With Scale. Take scale transformation into account.", Q_NULLPTR));
        checkBox_gms_withScale->setText(QString());
        groupBox_3->setTitle(QApplication::translate("preferencesDialog", "Motion estimation: 3D to 3D", Q_NULLPTR));
        loopClosure_bowInlierDistance->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_14->setText(QApplication::translate("preferencesDialog", "Maximum distance accepted between visual word correspondences.", Q_NULLPTR));
        label_38->setText(QApplication::translate("preferencesDialog", "Refine iterations of the resulting transformation computed by RANSAC. 0 means no refining.", Q_NULLPTR));
        groupBox_13->setTitle(QApplication::translate("preferencesDialog", "Motion Estimation: 3D to 2D (PnP)", Q_NULLPTR));
        loopClosure_pnpReprojError->setSuffix(QApplication::translate("preferencesDialog", " pix", Q_NULLPTR));
        label_236->setText(QApplication::translate("preferencesDialog", "Reprojection error.", Q_NULLPTR));
        loopClosure_pnpFlags->clear();
        loopClosure_pnpFlags->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Iterative", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "EPNP", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "P3P", Q_NULLPTR)
        );
        label_235->setText(QApplication::translate("preferencesDialog", "Flags.", Q_NULLPTR));
        label_loopClosure_pnpOpenCV2->setText(QApplication::translate("preferencesDialog", "Refine iterations.", Q_NULLPTR));
        groupBox_14->setTitle(QApplication::translate("preferencesDialog", "2D to 2D (Epipolar Geometry)", Q_NULLPTR));
        label_671->setText(QApplication::translate("preferencesDialog", "Experimental! PnP parameters are also used here.", Q_NULLPTR));
        loopClosure_bowEpipolarGeometryVar->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_664->setText(QApplication::translate("preferencesDialog", "Epipolar geometry maximum variance to accept the loop closure.", Q_NULLPTR));
        groupBox_VisExtract->setTitle(QApplication::translate("preferencesDialog", "Visual Feature", Q_NULLPTR));
        label_182->setText(QApplication::translate("preferencesDialog", "The features are not extracted if the compared nodes have already features extracted.", Q_NULLPTR));
        label_237->setText(QApplication::translate("preferencesDialog", "Maximum feature depth.", Q_NULLPTR));
        label_261->setText(QApplication::translate("preferencesDialog", "ROI ratios [left right top bottom] between 0 and 1.", Q_NULLPTR));
        label_180->setText(QApplication::translate("preferencesDialog", "Max features extracted from the images (0 means inf).", Q_NULLPTR));
        label_275->setText(QApplication::translate("preferencesDialog", "Minimum feature depth.", Q_NULLPTR));
        vis_feature_detector->clear();
        vis_feature_detector->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "SURF", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "SIFT", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "ORB", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "FAST+FREAK", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "FAST+BRIEF", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "GFTT+FREAK", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "GFTT+BRIEF", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "BRISK", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "GFTT+ORB", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "KAZE", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "ORB OCTREE", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "SuperPoint Torch", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "SURF+FREAK", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "GFTT+DAISY", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "SURF+DAISY", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "PyDetector", Q_NULLPTR)
        );
        loopClosure_bowMaxDepth->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_178->setText(QApplication::translate("preferencesDialog", "Feature detector ", Q_NULLPTR));
        loopClosure_bowMinDepth->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_438->setText(QApplication::translate("preferencesDialog", "Number of rows of the grid used to extract uniformly \"max features / grid cells\" features from each cell.", Q_NULLPTR));
        label_439->setText(QApplication::translate("preferencesDialog", "Number of columns of the grid used to extract uniformly \"max features / grid cells\" features from each cell.", Q_NULLPTR));
        label_450->setText(QApplication::translate("preferencesDialog", "Use depth image as mask when extracting features.", Q_NULLPTR));
        checkBox_visDepthAsMask->setText(QString());
        groupBox_11->setTitle(QApplication::translate("preferencesDialog", "Sub Pixel Refining", Q_NULLPTR));
        label_35->setText(QApplication::translate("preferencesDialog", "Refining corners to sub pixel. Sub pixel corners may not be needed for features like SURF/SIFT, which are already sub pixel.", Q_NULLPTR));
        label_200->setText(QApplication::translate("preferencesDialog", "Window size.", Q_NULLPTR));
        label_203->setText(QApplication::translate("preferencesDialog", "Iterations. 0 disables sub pixel refining.", Q_NULLPTR));
        label_204->setText(QApplication::translate("preferencesDialog", "Epsilon.", Q_NULLPTR));
        groupBox_icp2->setTitle(QApplication::translate("preferencesDialog", "ICP Registration", Q_NULLPTR));
        label_7->setText(QApplication::translate("preferencesDialog", "Iterative closest point (ICP) parameters.", Q_NULLPTR));
        label_139->setText(QApplication::translate("preferencesDialog", "Downsampling step of the cloud. Set to 1 to disable. This is done before uniform sampling.", Q_NULLPTR));
        loopClosure_icpRangeMax->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_55->setText(QApplication::translate("preferencesDialog", "Maximum ICP rotation correction accepted (>0). A large rotation difference between the visual transformation and ICP transformation results in wrong transformations in most cases.", Q_NULLPTR));
        label_136->setText(QApplication::translate("preferencesDialog", "Set the transformation epsilon (maximum allowable difference between two consecutive transformations) in order for an optimization to be considered as having converged to the final solution.", Q_NULLPTR));
        loopClosure_icpRangeMin->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        comboBox_icpStrategy->clear();
        comboBox_icpStrategy->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "PCL (Point Cloud Library)", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "libpointmatcher", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "CCCoreLib (CloudCompare)", Q_NULLPTR)
        );
        loopClosure_icpPointToPlane->setText(QString());
        loopClosure_icpForce4DoF->setText(QString());
        loopClosure_icpPointToPlaneLowComplexityStrategy->clear();
        loopClosure_icpPointToPlaneLowComplexityStrategy->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Reject", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Constrained PointToPoint", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "PointToPoint", Q_NULLPTR)
        );
        label_51->setText(QApplication::translate("preferencesDialog", "Maximum ICP translation correction accepted (>0). A large translation difference between the visual transformation and ICP transformation results in wrong transformations in most cases.", Q_NULLPTR));
        label_559->setText(QApplication::translate("preferencesDialog", "Minimum range filtering. Set to 0 to disable.", Q_NULLPTR));
        loopClosure_icpVoxelSize->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_426->setText(QApplication::translate("preferencesDialog", "Search radius to compute normals for point to plane. Normals won't be recomputed if uniform sampling is disabled and that there are already normals in the laser scans.", Q_NULLPTR));
        label_133->setText(QApplication::translate("preferencesDialog", "Ratio of matching correspondences to accept the transform. If the maximum laser scans is set, this is the minimum ratio of correspondences on laser scan maximum size to accept the transform.", Q_NULLPTR));
        label_124->setText(QApplication::translate("preferencesDialog", "Max iterations.", Q_NULLPTR));
        label_212->setText(QApplication::translate("preferencesDialog", "Number of neighbors to compute normals for point to plane. Normals won't be recomputed if uniform sampling is disabled and that there are already normals in the laser scans.", Q_NULLPTR));
        label_622->setText(QApplication::translate("preferencesDialog", "Force 4 DoF: Limit ICP to x, y, z and yaw DoF. Available only with libpointmatcher or CCCoreLib strategies.", Q_NULLPTR));
        label_122->setText(QApplication::translate("preferencesDialog", "Max distance for point correspondences.", Q_NULLPTR));
        label_144->setText(QApplication::translate("preferencesDialog", "Point to plane ICP.", Q_NULLPTR));
        label_608->setText(QApplication::translate("preferencesDialog", "Invert normals on ground if they are pointing down (useful for ring-like 3D LiDARs). 0 means disabled, 1 means only normals perfectly aligned with -z axis. This is only done with 3D scans.", Q_NULLPTR));
        globalDetection_icpMaxRotation->setSuffix(QApplication::translate("preferencesDialog", " rad", Q_NULLPTR));
        globalDetection_icpMaxTranslation->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        loopClosure_icpEpsilon->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_617->setText(QApplication::translate("preferencesDialog", "Strategy.", Q_NULLPTR));
        label_125->setText(QApplication::translate("preferencesDialog", "Uniform sampling voxel size. Set to 0 to disable.", Q_NULLPTR));
        label_607->setText(QApplication::translate("preferencesDialog", "Stuctural complexity strategy. If structural complexity is below the minimum complexity threshold: set to 0 to so that the transform is automatically rejected, set to 1 to limit ICP correction in axes with most constraints (e.g., for a corridor-like environment, the resulting transform will be limited in y and yaw, x will taken from the guess), set to 2 to accept \"as is\" the transform computed by PointToPoint.", Q_NULLPTR));
        label_560->setText(QApplication::translate("preferencesDialog", "Maximum range filtering. Set to 0 to disable.", Q_NULLPTR));
        label_429->setText(QApplication::translate("preferencesDialog", "Minimum structural complexity (0.0=low, 1.0=high) of the scan to do PointToPlane registration, otherwise PointToPoint registration is done instead and strategy from %s is used. This check is done only when PointToPlane is enabled.", Q_NULLPTR));
        label_446->setText(QApplication::translate("preferencesDialog", "Outlier ratio used when ICP strategy is not PCL. For libpointmatcher, this parameter set TrimmedDistOutlierFilter / ratio for convenience when configuration file is not set. For CCCoreLib, this parameter set the \"finalOverlapRatio\". ", Q_NULLPTR));
        doubleSpinBox_icpOutlierRatio->setSuffix(QString());
        groupBox_33->setTitle(QApplication::translate("preferencesDialog", "Point Cloud Library", Q_NULLPTR));
        label_628->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p><a href=\"https://pointclouds.org/documentation/classpcl_1_1_iterative_closest_point.html#details\"><span style=\" text-decoration: underline; color:#0000ff;\">https://pointclouds.org/</span></a></p>", Q_NULLPTR));
        groupBox_libpointmatcher->setTitle(QApplication::translate("preferencesDialog", "libpointmatcher", Q_NULLPTR));
        label_422->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p><a href=\"https://github.com/ethz-asl/libpointmatcher\"><span style=\" text-decoration: underline; color:#0000ff;\">https://github.com/ethz-asl/libpointmatcher</span></a></p><p>libpointmatcher is a modular library implementing the Iterative Closest Point (ICP) algorithm for aligning point clouds. It has applications in robotics and computer vision. When enabled, libpointmatcher is used for ICP registration instead of PCL's implementation.</p><p>Below we can set a <a href=\"https://github.com/ethz-asl/libpointmatcher/blob/master/doc/Configuration.md\"><span style=\" text-decoration: underline; color:#0000ff;\">configuration file</span></a> (*.yaml) used by libpointmatcher. Note that data filters set for libpointmatcher are done after filtering done by rtabmap (i.e., voxel filtering or downsampling above), so make sure to disable those in rtabmap if you want to use only those from libpointmatcher. Maximum iterations, epsilon and max correspondence distance parameters are also ignored if con"
                        "figuration file is set.<br/></p></body></html>", Q_NULLPTR));
        toolButton_IcpConfigPath->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        label_423->setText(QApplication::translate("preferencesDialog", "Configuration file (*.yaml).", Q_NULLPTR));
        label_424->setText(QApplication::translate("preferencesDialog", "KDTreeMatcher/knn: number of nearest neighbors to consider it the reference. For convenience when configuration file is not set.", Q_NULLPTR));
        loopClosure_icpPMMatcherIntensity->setText(QString());
        doubleSpinBox_icpPMMatcherEpsilon->setSuffix(QString());
        label_447->setText(QApplication::translate("preferencesDialog", "KDTreeMatcher/epsilon: approximation to use for the nearest-neighbor search. For convenience when configuration file is not set.", Q_NULLPTR));
        label_609->setText(QApplication::translate("preferencesDialog", "KDTreeMatcher:  among nearest neighbors, keep only the one with the most similar intensity. This only work with knn>1.", Q_NULLPTR));
        groupBox_32->setTitle(QApplication::translate("preferencesDialog", "CCCoreLib", Q_NULLPTR));
        label_623->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p><a href=\"https://github.com/CloudCompare/CCCoreLib\"><span style=\" text-decoration: underline; color:#0000ff;\">https://github.com/CloudCompare/CCCoreLib</span></a></p>", Q_NULLPTR));
        doubleSpinBox_icpCCMaxFinalRMS->setSuffix(QString());
        label_630->setText(QApplication::translate("preferencesDialog", "If true, the algorithm will automatically ignore farthest points from the reference, for better convergence.", Q_NULLPTR));
        label_631->setText(QApplication::translate("preferencesDialog", "Maximum final RMS error.", Q_NULLPTR));
        label_629->setText(QApplication::translate("preferencesDialog", "Maximum number of points per cloud (they are randomly resampled below this limit otherwise).", Q_NULLPTR));
        checkBox_icpCCFilterOutFarthestPoints->setText(QString());
        groupBox_optimizer1->setTitle(QApplication::translate("preferencesDialog", "Optimizer", Q_NULLPTR));
        label_141->setText(QApplication::translate("preferencesDialog", "Ignore constraints' variance. If checked, identity information matrix is used for each constraint. Otherwise, an information matrix is generated from the variance saved in the links (transitional and rotational variances).", Q_NULLPTR));
        label_64->setText(QApplication::translate("preferencesDialog", "Stop optimizing when the error improvement is less than this value.", Q_NULLPTR));
        graphOptimization_covarianceIgnored->setText(QString());
        label_31->setText(QApplication::translate("preferencesDialog", "Iterations.", Q_NULLPTR));
        groupBoxx_g2o->setTitle(QApplication::translate("preferencesDialog", "g2o", Q_NULLPTR));
        comboBox_g2o_solver->clear();
        comboBox_g2o_solver->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "CSparse", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "PCG", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Cholmod", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Eigen", Q_NULLPTR)
        );
        label_107->setText(QApplication::translate("preferencesDialog", "Solver.", Q_NULLPTR));
        comboBox_g2o_optimizer->clear();
        comboBox_g2o_optimizer->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Levenberg", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Gauss Newton", Q_NULLPTR)
        );
        label_138->setText(QApplication::translate("preferencesDialog", "Optimization algorithm.", Q_NULLPTR));
        groupBox_20->setTitle(QApplication::translate("preferencesDialog", "Bundle Adjustement", Q_NULLPTR));
        label_148->setText(QApplication::translate("preferencesDialog", "Pixel variance used for bundle adjustment.", Q_NULLPTR));
        label_350->setText(QApplication::translate("preferencesDialog", "Robust kernel delta used for bundle adjustment (0 means don't use robust kernel). Observations with chi2 over this threshold will be ignored in the second optimization pass.", Q_NULLPTR));
        doubleSpinBox_g2o_baseline->setSuffix(QApplication::translate("preferencesDialog", " m", Q_NULLPTR));
        label_352->setText(QApplication::translate("preferencesDialog", "When doing bundle adjustment with RGB-D data, we can set a fake baseline to do stereo bundle adjustment (if 0, mono bundle adjustment is done). For stereo data, the baseline in the calibration is used directly.", Q_NULLPTR));
        groupBox_19->setTitle(QApplication::translate("preferencesDialog", "GTSAM", Q_NULLPTR));
        comboBox_gtsam_optimizer->clear();
        comboBox_gtsam_optimizer->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "Levenberg", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Gauss Newton", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "Dogleg", Q_NULLPTR)
        );
        label_364->setText(QApplication::translate("preferencesDialog", "Optimization algorithm.", Q_NULLPTR));
        groupBox_stereo1->setTitle(QApplication::translate("preferencesDialog", "Stereo", Q_NULLPTR));
        label_187->setText(QApplication::translate("preferencesDialog", "These parameters are used to find the 3D positions of visual words using disparity between the left and right images. Optical flow can be used to find corresponding corners in the right image of words extracted from the left image.", Q_NULLPTR));
        groupBox_10->setTitle(QString());
        label_284->setText(QApplication::translate("preferencesDialog", "Window height.", Q_NULLPTR));
        label_285->setText(QApplication::translate("preferencesDialog", "Minimum disparity.", Q_NULLPTR));
        label_205->setText(QApplication::translate("preferencesDialog", "Window width.", Q_NULLPTR));
        label_206->setText(QApplication::translate("preferencesDialog", "Maximum iterations.", Q_NULLPTR));
        stereo_ssd->setText(QString());
        label_207->setText(QApplication::translate("preferencesDialog", "[OpticalFlow = true] Epsilon stop criterion.", Q_NULLPTR));
        label_208->setText(QApplication::translate("preferencesDialog", "Maximum pyramid level.", Q_NULLPTR));
        label_287->setText(QApplication::translate("preferencesDialog", "[OpticalFlow = false] Use Sum of Squared Differences (SSD) window, otherwise Sum of Absolute Differences (SAD) window is used.", Q_NULLPTR));
        label_214->setText(QApplication::translate("preferencesDialog", "Use optical flow to find stereo correspondences, otherwise a simple block matching approach is used.", Q_NULLPTR));
        label_286->setText(QApplication::translate("preferencesDialog", "Maximum disparity.", Q_NULLPTR));
        stereo_opticalFlow->setText(QString());
        groupBox_stereoDense2->setTitle(QApplication::translate("preferencesDialog", "Stereo Dense", Q_NULLPTR));
        comboBox_stereoDense_strategy->clear();
        comboBox_stereoDense_strategy->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "StereoBM", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "StereoSGBM", Q_NULLPTR)
        );
        label_515->setText(QApplication::translate("preferencesDialog", "This approach is used to construct a dense disparity image for visualization (RGB-D point clouds) or to create a depth image when acquiring from a stereo source. ", Q_NULLPTR));
        label_197->setText(QApplication::translate("preferencesDialog", "Stereo correspondence using the block matching algorithm (cv::StereoBM), introduced and contributed to OpenCV by K. Konolige. This approach is used to construct a dense disparity image for visualization (RGB-D point clouds) or to create a depth image when acquiring from a stereo source. ", Q_NULLPTR));
        groupBox_16->setTitle(QString());
        label_278->setText(QApplication::translate("preferencesDialog", "Pre-filter Size.", Q_NULLPTR));
        label_276->setText(QApplication::translate("preferencesDialog", "Minimum disparity.", Q_NULLPTR));
        label_281->setText(QApplication::translate("preferencesDialog", "Texture threshold.", Q_NULLPTR));
        label_279->setText(QApplication::translate("preferencesDialog", "Pre-filter Cap.", Q_NULLPTR));
        label_277->setText(QApplication::translate("preferencesDialog", "Number of disparities.", Q_NULLPTR));
        label_216->setText(QApplication::translate("preferencesDialog", "Block size.", Q_NULLPTR));
        label_280->setText(QApplication::translate("preferencesDialog", "Uniqueness ratio.", Q_NULLPTR));
        label_282->setText(QApplication::translate("preferencesDialog", "Speckle window size.", Q_NULLPTR));
        label_283->setText(QApplication::translate("preferencesDialog", "Speckle range.", Q_NULLPTR));
        label_501->setText(QApplication::translate("preferencesDialog", "Disp 12 max diff.", Q_NULLPTR));
        label_514->setText(QApplication::translate("preferencesDialog", "Stereo correspondence approach (cv::StereoSGBM) implementing the modified H. Hirschmuller algorithm.", Q_NULLPTR));
        groupBox_30->setTitle(QString());
        label_517->setText(QApplication::translate("preferencesDialog", "P1.", Q_NULLPTR));
        label_516->setText(QApplication::translate("preferencesDialog", "Disp 12 max diff.", Q_NULLPTR));
        label_499->setText(QApplication::translate("preferencesDialog", "Pre-filter Cap.", Q_NULLPTR));
        label_500->setText(QApplication::translate("preferencesDialog", "Minimum disparity.", Q_NULLPTR));
        label_504->setText(QApplication::translate("preferencesDialog", "Block size.", Q_NULLPTR));
        label_503->setText(QApplication::translate("preferencesDialog", "Number of disparities.", Q_NULLPTR));
        label_505->setText(QApplication::translate("preferencesDialog", "Uniqueness ratio.", Q_NULLPTR));
        label_506->setText(QApplication::translate("preferencesDialog", "Speckle window size.", Q_NULLPTR));
        label_512->setText(QApplication::translate("preferencesDialog", "Speckle range.", Q_NULLPTR));
        label_518->setText(QApplication::translate("preferencesDialog", "P2.", Q_NULLPTR));
        label_519->setText(QApplication::translate("preferencesDialog", "Mode.", Q_NULLPTR));
        stereosgbm_mode->clear();
        stereosgbm_mode->insertItems(0, QStringList()
         << QApplication::translate("preferencesDialog", "SGBM", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "HH", Q_NULLPTR)
         << QApplication::translate("preferencesDialog", "SGBM_3WAY", Q_NULLPTR)
        );
        groupBox_feature1->setTitle(QApplication::translate("preferencesDialog", "Feature", Q_NULLPTR));
        label_10->setText(QApplication::translate("preferencesDialog", "Parameters for each corresponding OpenCV feature type. See the sub panels.", Q_NULLPTR));
        groupBox_detector_surf2->setTitle(QApplication::translate("preferencesDialog", "SURF", Q_NULLPTR));
        surf_checkBox_upright->setText(QString());
        label_41->setText(QApplication::translate("preferencesDialog", "Octave layers.", Q_NULLPTR));
        label_8->setText(QApplication::translate("preferencesDialog", "Descriptor extended (true=128, false=64).", Q_NULLPTR));
        label_surf_checkBox_gpuKeypointsRatio->setText(QApplication::translate("preferencesDialog", "GPU keypoints ratio.", Q_NULLPTR));
        label_surf_checkBox_gpuVersion->setText(QApplication::translate("preferencesDialog", "GPU version.", Q_NULLPTR));
        surf_checkBox_gpuVersion->setText(QString());
        label_49->setText(QApplication::translate("preferencesDialog", "U-SURF used.", Q_NULLPTR));
        checkBox_surfExtended->setText(QString());
        label_26->setText(QApplication::translate("preferencesDialog", "Octaves.", Q_NULLPTR));
        label_4->setText(QApplication::translate("preferencesDialog", "Hessian threshold.", Q_NULLPTR));
        groupBox_detector_sift2->setTitle(QApplication::translate("preferencesDialog", "SIFT", Q_NULLPTR));
        label_129->setText(QApplication::translate("preferencesDialog", "Sigma.", Q_NULLPTR));
        label_5->setText(QApplication::translate("preferencesDialog", "Contrast threshold.", Q_NULLPTR));
        label_30->setText(QApplication::translate("preferencesDialog", "Edge threshold.", Q_NULLPTR));
        label_130->setText(QApplication::translate("preferencesDialog", "nOctaveLayers.", Q_NULLPTR));
        label_131->setText(QApplication::translate("preferencesDialog", "nFeatures.", Q_NULLPTR));
        label_591->setText(QApplication::translate("preferencesDialog", "RootSIFT.", Q_NULLPTR));
        sift_checkBox_rootsift->setText(QString());
        groupBox_detector_fast2->setTitle(QApplication::translate("preferencesDialog", "FAST", Q_NULLPTR));
        label_56->setText(QApplication::translate("preferencesDialog", "Threshold on difference between intensity of the central pixel and pixels of a circle around this pixel.", Q_NULLPTR));
        label_111->setText(QApplication::translate("preferencesDialog", "Minimum threshold. Used when FAST/GridRows and FAST/GridCols are set or if ORBSLAM2 odometry is used.", Q_NULLPTR));
        fastSuppressNonMax->setText(QString());
        label_69->setText(QApplication::translate("preferencesDialog", "If true, non-maximum suppression is applied to detected corners (keypoints).", Q_NULLPTR));
        label_561->setText(QApplication::translate("preferencesDialog", "Enable FastCV implementation if non-zero. Values should be 9 and 10 to choose which FastCV approach is used..", Q_NULLPTR));
        groupBox_fast_opencv2->setTitle(QApplication::translate("preferencesDialog", "OpenCV 2", Q_NULLPTR));
        label_115->setText(QApplication::translate("preferencesDialog", "Maximum threshold. Used only when FAST/GridRows and FAST/GridCols are set.", Q_NULLPTR));
        label_104->setText(QApplication::translate("preferencesDialog", "Grid rows (0 to disable). Adapts a detector to partition the source image into a grid and detect points in each cell.", Q_NULLPTR));
        label_109->setText(QApplication::translate("preferencesDialog", "Grid cols (0 to disable). Adapts a detector to partition the source image into a grid and detect points in each cell.", Q_NULLPTR));
        fastGpu->setText(QString());
        label_fastGPU->setText(QApplication::translate("preferencesDialog", "GPU-FAST: Use GPU version of FAST. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.", Q_NULLPTR));
        label_fastGPUKptRatio->setText(QApplication::translate("preferencesDialog", "Used with FAST GPU.", Q_NULLPTR));
        groupBox_detector_brief2->setTitle(QApplication::translate("preferencesDialog", "BRIEF", Q_NULLPTR));
        label_52->setText(QApplication::translate("preferencesDialog", "Bytes is a length of descriptor in bytes. It can be equal 16, 32 or 64 bytes.", Q_NULLPTR));
        groupBox_detector_orb2->setTitle(QApplication::translate("preferencesDialog", "ORB", Q_NULLPTR));
        label_32->setText(QApplication::translate("preferencesDialog", "Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.", Q_NULLPTR));
        label_43->setText(QApplication::translate("preferencesDialog", "The number of pyramid levels. The smallest level will have linear size equal to input_image_linear_size/pow(scaleFactor, nlevels).", Q_NULLPTR));
        label_95->setText(QApplication::translate("preferencesDialog", "This is size of the border where the features are not detected. It should roughly match the patchSize parameter.", Q_NULLPTR));
        label_127->setText(QApplication::translate("preferencesDialog", "It should be 0 in the current implementation.", Q_NULLPTR));
        label_128->setText(QApplication::translate("preferencesDialog", "WTA_K: The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).", Q_NULLPTR));
        label_140->setText(QApplication::translate("preferencesDialog", "The default HARRIS_SCORE=0 means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE=1 is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.", Q_NULLPTR));
        label_147->setText(QApplication::translate("preferencesDialog", "size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.", Q_NULLPTR));
        checkBox_ORBGpu->setText(QString());
        label_orbGpu->setText(QApplication::translate("preferencesDialog", "GPU-ORB: Use GPU version of ORB. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.", Q_NULLPTR));
        groupBox_detector_freak2->setTitle(QApplication::translate("preferencesDialog", "FREAK", Q_NULLPTR));
        checkBox_FREAKOrientationNormalized->setText(QString());
        label_6->setText(QApplication::translate("preferencesDialog", "Enable orientation normalization.", Q_NULLPTR));
        checkBox_FREAKScaleNormalized->setText(QString());
        label_9->setText(QApplication::translate("preferencesDialog", "Enable scale normalization.", Q_NULLPTR));
        label_13->setText(QApplication::translate("preferencesDialog", "Scaling of the description pattern.", Q_NULLPTR));
        label_23->setText(QApplication::translate("preferencesDialog", "Number of octaves covered by the detected keypoints.", Q_NULLPTR));
        groupBox_detector_gftt2->setTitle(QApplication::translate("preferencesDialog", "GFTT", Q_NULLPTR));
        checkBox_GFTT_useHarrisDetector->setText(QString());
        label_172->setText(QApplication::translate("preferencesDialog", "Use Harris detector.", Q_NULLPTR));
        label_173->setText(QApplication::translate("preferencesDialog", "K. Harris detector free parameter.", Q_NULLPTR));
        label_175->setText(QApplication::translate("preferencesDialog", "Quality level. Parameter characterizing the minimal accepted quality of image corners. The parameter value is multiplied by the best corner quality measure, which is the minimal eigenvalue (see cornerMinEigenVal() ) or the Harris function response. The corners with the quality measure less than the product are rejected. For example, if the best corner has the quality measure = 1500, and the qualityLevel=0.01 , then all the corners with the quality measure less than 15 are rejected.", Q_NULLPTR));
        label_176->setText(QApplication::translate("preferencesDialog", "Minimum possible Euclidean distance between the returned corners.", Q_NULLPTR));
        label_177->setText(QApplication::translate("preferencesDialog", "Block size. Size of an average block for computing a derivative covariation matrix over each pixel neighborhood.", Q_NULLPTR));
        groupBox_detector_brisk2->setTitle(QApplication::translate("preferencesDialog", "BRISK", Q_NULLPTR));
        label_191->setText(QApplication::translate("preferencesDialog", "FAST/AGAST detection threshold score.", Q_NULLPTR));
        label_188->setText(QApplication::translate("preferencesDialog", "Detection octaves. Use 0 to do single scale.", Q_NULLPTR));
        label_189->setText(QApplication::translate("preferencesDialog", "Apply this scale to the pattern used for sampling the neighbourhood of a keypoint.", Q_NULLPTR));
        groupBox_detector_kaze2->setTitle(QApplication::translate("preferencesDialog", "KAZE", Q_NULLPTR));
        checkBox_kaze_extended->setText(QString());
        label_420->setText(QApplication::translate("preferencesDialog", "Set to enable use of upright descriptors (non rotation-invariant).", Q_NULLPTR));
        label_416->setText(QApplication::translate("preferencesDialog", "Maximum octave evolution of the image.", Q_NULLPTR));
        checkBox_kaze_upright->setText(QString());
        label_417->setText(QApplication::translate("preferencesDialog", "Default number of sublevels per scale level.", Q_NULLPTR));
        label_419->setText(QApplication::translate("preferencesDialog", "Set to enable extraction of extended (128-byte) descriptor.", Q_NULLPTR));
        label_418->setText(QApplication::translate("preferencesDialog", "Detector response threshold to accept point.", Q_NULLPTR));
        label_421->setText(QApplication::translate("preferencesDialog", "Diffusivity type: 0=DIFF_PM_G1, 1=DIFF_PM_G2, 2=DIFF_WEICKERT or 3=DIFF_CHARBONNIER.", Q_NULLPTR));
        groupBox_detector_superpoint_torch2->setTitle(QApplication::translate("preferencesDialog", "SupertPoint Torch", Q_NULLPTR));
        label_575->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>SupertPoint c++ implementation from <a href=\"https://github.com/KinglittleQ/SuperPoint_SLAM\"><span style=\" text-decoration: underline; color:#0000ff;\">SuperPoint_SLAM</span></a> project based on pytorch. A <span style=\" font-weight:600;\">superpoint.pt</span> weights file can be downloaded from its Git. It should be the same weights file <a href=\"https://pytorch.org/tutorials/advanced/cpp_export.html\"><span style=\" text-decoration: underline; color:#0000ff;\">converted</span></a> from <span style=\" font-weight:600;\">superpoint.pth</span> of the <a href=\"https://github.com/magicleap/SuperPointPretrainedNetwork\"><span style=\" text-decoration: underline; color:#0000ff;\">SuperPointPretrainedNetwork</span></a> project.</p></body></html>", Q_NULLPTR));
        toolButton_sptorch_path->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        label_579->setText(QApplication::translate("preferencesDialog", "[Required] Path to pre-trained weights Torch file of SuperPoint.", Q_NULLPTR));
        label_578->setText(QApplication::translate("preferencesDialog", "Detector response threshold to accept keypoint.", Q_NULLPTR));
        checkBox_sptorch_nms->setText(QString());
        label_577->setText(QApplication::translate("preferencesDialog", "If true, non-maximum suppression is applied to detected keypoints.", Q_NULLPTR));
        checkBox_sptorch_cuda->setText(QString());
        label_574->setText(QApplication::translate("preferencesDialog", "Use Cuda device for Torch, otherwise CPU device is used by default.", Q_NULLPTR));
        label_580->setText(QApplication::translate("preferencesDialog", "Minimum distance (pixels) between keypoints (when non-maximum suppression is enabled).", Q_NULLPTR));
        groupBox_pydetector2->setTitle(QApplication::translate("preferencesDialog", "PyDetector", Q_NULLPTR));
        label_614->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>Python wrapper for python feature detector like <a href=\"https://github.com/magicleap/SuperPointPretrainedNetwork\"><span style=\" text-decoration: underline; color:#0000ff;\">SuperPoint</span></a>. Download the <a href=\"https://github.com/introlab/rtabmap/tree/master/corelib/src/python\"><span style=\" text-decoration: underline; color:#0000ff;\">script</span></a> and copy it at the root folder of SuperPoint git, then set its path below.</p></body></html>", Q_NULLPTR));
        label_615->setText(QApplication::translate("preferencesDialog", "<html><head/><body><p>[Required] Path to python script file.</p></body></html>", Q_NULLPTR));
        toolButton_pydetector_path->setText(QApplication::translate("preferencesDialog", "...", Q_NULLPTR));
        checkBox_pydetector_cuda->setText(QString());
        label_618->setText(QApplication::translate("preferencesDialog", "Use Cuda device for Torch, otherwise CPU device is used by default.", Q_NULLPTR));
        radioButton_basic->setText(QApplication::translate("preferencesDialog", "Basic", Q_NULLPTR));
        radioButton_advanced->setText(QApplication::translate("preferencesDialog", "Advanced", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class preferencesDialog: public Ui_preferencesDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PREFERENCESDIALOG_H
