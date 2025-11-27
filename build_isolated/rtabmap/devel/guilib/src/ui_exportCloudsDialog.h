/********************************************************************************
** Form generated from reading UI file 'exportCloudsDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_EXPORTCLOUDSDIALOG_H
#define UI_EXPORTCLOUDSDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ExportCloudsDialog
{
public:
    QVBoxLayout *verticalLayout_2;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_13;
    QGridLayout *gridLayout_8;
    QComboBox *comboBox_pipeline;
    QLabel *label_12;
    QCheckBox *checkBox_regenerate;
    QCheckBox *checkBox_filtering;
    QLabel *label_regenerate;
    QLabel *label_voxel;
    QLabel *label_binaryFile_2;
    QLabel *label_frame;
    QSpinBox *spinBox_normalKSearch;
    QCheckBox *checkBox_smoothing;
    QLabel *label_binaryFile_9;
    QLabel *label_binaryFile;
    QLabel *label_smoothing;
    QDoubleSpinBox *doubleSpinBox_voxelSize_assembled;
    QCheckBox *checkBox_assemble;
    QLabel *label_normal;
    QComboBox *comboBox_frame;
    QCheckBox *checkBox_binary;
    QLabel *label_binaryFile_11;
    QCheckBox *checkBox_fromDepth;
    QLabel *label_normal_2;
    QDoubleSpinBox *doubleSpinBox_normalRadiusSearch;
    QLabel *label_intensityColormap;
    QComboBox *comboBox_intensityColormap;
    QLabel *label_gainCompensation;
    QLabel *label_cameraProjection;
    QCheckBox *checkBox_gainCompensation;
    QCheckBox *checkBox_cameraProjection;
    QLabel *label_binaryFile_12;
    QCheckBox *checkBox_meshing;
    QLabel *label_binaryFile_10;
    QCheckBox *checkBox_nodes_filtering;
    QGroupBox *groupBox_nodes_filtering;
    QVBoxLayout *verticalLayout_19;
    QLabel *label_114;
    QGridLayout *gridLayout_21;
    QLabel *label_113;
    QLabel *label_142;
    QDoubleSpinBox *doubleSpinBox_nodes_filtering_ymin;
    QLabel *label_115;
    QLabel *label_116;
    QLabel *label_119;
    QLabel *label_111;
    QSpacerItem *horizontalSpacer;
    QDoubleSpinBox *doubleSpinBox_nodes_filtering_xmin;
    QDoubleSpinBox *doubleSpinBox_nodes_filtering_xmax;
    QDoubleSpinBox *doubleSpinBox_nodes_filtering_zmin;
    QDoubleSpinBox *doubleSpinBox_nodes_filtering_ymax;
    QDoubleSpinBox *doubleSpinBox_nodes_filtering_zmax;
    QGroupBox *groupBox_regenerateScans;
    QGridLayout *gridLayout_18;
    QSpinBox *spinBox_decimation_scan;
    QLabel *label_109;
    QLabel *label_138;
    QDoubleSpinBox *doubleSpinBox_rangeMin;
    QDoubleSpinBox *doubleSpinBox_rangeMax;
    QLabel *label_139;
    QGroupBox *groupBox_regenerate;
    QVBoxLayout *verticalLayout_14;
    QGridLayout *gridLayout;
    QToolButton *toolButton_distortionModel;
    QSpinBox *spinBox_fillDepthHoles;
    QSpinBox *spinBox_decimation;
    QDoubleSpinBox *doubleSpinBox_minDepth;
    QLabel *label_108;
    QLineEdit *lineEdit_distortionModel;
    QSpinBox *spinBox_fillDepthHolesError;
    QLabel *label_132;
    QLabel *label_133;
    QLabel *label_197;
    QDoubleSpinBox *doubleSpinBox_maxDepth;
    QLabel *label_134;
    QLabel *label_198;
    QLabel *label_135;
    QLineEdit *lineEdit_roiRatios;
    QCheckBox *checkBox_bilateral;
    QLabel *label_136;
    QLabel *label_137;
    QCheckBox *checkBox_subtraction;
    QLabel *label_140;
    QLabel *label_141;
    QDoubleSpinBox *doubleSpinBox_ceilingHeight;
    QDoubleSpinBox *doubleSpinBox_floorHeight;
    QGroupBox *groupBox_bilateral;
    QGridLayout *gridLayout_13;
    QDoubleSpinBox *doubleSpinBox_bilateral_sigmaR;
    QLabel *label_194;
    QDoubleSpinBox *doubleSpinBox_bilateral_sigmaS;
    QLabel *label_195;
    QGroupBox *groupBox_subtraction;
    QGridLayout *gridLayout_11;
    QLabel *label_174;
    QLabel *label_193;
    QDoubleSpinBox *doubleSpinBox_subtractPointFilteringRadius;
    QLabel *label_192;
    QSpinBox *spinBox_subtractFilteringMinPts;
    QDoubleSpinBox *doubleSpinBox_subtractPointFilteringAngle;
    QGroupBox *groupBox_filtering;
    QGridLayout *gridLayout_9;
    QDoubleSpinBox *doubleSpinBox_filteringRadius;
    QLabel *label_110;
    QSpinBox *spinBox_filteringMinNeighbors;
    QLabel *label_112;
    QGroupBox *groupBox_mls;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QGridLayout *gridLayout_3;
    QDoubleSpinBox *doubleSpinBox_mlsRadius;
    QComboBox *comboBox_upsamplingMethod;
    QLabel *label_87;
    QLabel *label_88;
    QLabel *label_89;
    QSpinBox *spinBox_polygonialOrder;
    QLabel *label_90;
    QDoubleSpinBox *doubleSpinBox_mls_outputVoxelSize;
    QStackedWidget *stackedWidget_upsampling;
    QWidget *page;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_8;
    QLabel *label_2;
    QSpacerItem *verticalSpacer_2;
    QWidget *page_2;
    QVBoxLayout *verticalLayout_4;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_9;
    QLabel *label_3;
    QSpacerItem *verticalSpacer_3;
    QWidget *page_3;
    QVBoxLayout *verticalLayout_7;
    QGroupBox *groupBox_3;
    QVBoxLayout *verticalLayout_10;
    QLabel *label_4;
    QGridLayout *gridLayout_5;
    QLabel *label_8;
    QDoubleSpinBox *doubleSpinBox_sampleRadius;
    QLabel *label_7;
    QDoubleSpinBox *doubleSpinBox_sampleStep;
    QSpacerItem *verticalSpacer_4;
    QWidget *page_4;
    QVBoxLayout *verticalLayout_6;
    QGroupBox *groupBox_4;
    QVBoxLayout *verticalLayout_11;
    QLabel *label_5;
    QGridLayout *gridLayout_6;
    QSpinBox *spinBox_randomPoints;
    QLabel *label_9;
    QSpacerItem *verticalSpacer_5;
    QWidget *page_5;
    QVBoxLayout *verticalLayout_5;
    QGroupBox *groupBox_5;
    QVBoxLayout *verticalLayout_12;
    QLabel *label_6;
    QGridLayout *gridLayout_7;
    QLabel *label_10;
    QDoubleSpinBox *doubleSpinBox_dilationVoxelSize;
    QLabel *label_11;
    QSpinBox *spinBox_dilationSteps;
    QSpacerItem *verticalSpacer_6;
    QGroupBox *groupBox_gain;
    QGridLayout *gridLayout_12;
    QDoubleSpinBox *doubleSpinBox_gainBeta;
    QLabel *label_binaryFile_7;
    QDoubleSpinBox *doubleSpinBox_gainOverlap;
    QCheckBox *checkBox_gainFull;
    QLabel *label_binaryFile_3;
    QLabel *label_binaryFile_5;
    QDoubleSpinBox *doubleSpinBox_gainRadius;
    QLabel *label_binaryFile_6;
    QLabel *label_binaryFile_8;
    QCheckBox *checkBox_gainRGB;
    QGroupBox *groupBox_cameraProjection;
    QGridLayout *gridLayout_20;
    QDoubleSpinBox *doubleSpinBox_camProjMaxAngle;
    QLabel *label_meshingTextureSize_15;
    QCheckBox *checkBox_camProjDistanceToCamPolicy;
    QLabel *label_meshingTextureSize_14;
    QLabel *label_meshingTextureSize_13;
    QLabel *label_meshingTextureSize_12;
    QDoubleSpinBox *doubleSpinBox_camProjMaxDistance;
    QCheckBox *checkBox_camProjKeepPointsNotSeenByCameras;
    QLabel *label_camProjExportCamera;
    QComboBox *comboBox_camProjExportCamera;
    QLineEdit *lineEdit_camProjRoiRatios;
    QLabel *label_meshingTextureSize_17;
    QLabel *label_meshingTextureSize_18;
    QCheckBox *checkBox_camProjRecolorPoints;
    QGroupBox *groupBox_meshing;
    QVBoxLayout *verticalLayout_15;
    QGridLayout *gridLayout_10;
    QLabel *label_meshClean;
    QLabel *label_denseReconstruction;
    QDoubleSpinBox *doubleSpinBox_meshDecimationFactor;
    QLabel *label_meshDecimation_2;
    QLabel *label_meshDecimation;
    QLabel *label_textureMapping;
    QComboBox *comboBox_meshingApproach;
    QCheckBox *checkBox_textureMapping;
    QDoubleSpinBox *doubleSpinBox_transferColorRadius;
    QSpinBox *spinBox_mesh_minClusterSize;
    QCheckBox *checkBox_cleanMesh;
    QLabel *label_16;
    QLabel *label_meshMaxPolygons;
    QSpinBox *spinBox_meshMaxPolygons;
    QGroupBox *groupBox_textureMapping;
    QVBoxLayout *verticalLayout_16;
    QGridLayout *gridLayout_15;
    QSpinBox *spinBox_textureBrightnessContrastRatioHigh;
    QLabel *label_meshingTextureSize_6;
    QComboBox *comboBox_blendingDecimation;
    QCheckBox *checkBox_blending;
    QLabel *label_exposureFusion_3;
    QLabel *label_meshingTextureSize_7;
    QLabel *label_meshingTextureSize_8;
    QLabel *label_exposureFusion_2;
    QLabel *label_meshingTextureSize_2;
    QDoubleSpinBox *doubleSpinBox_meshingTextureMaxDistance;
    QLabel *label_meshingTextureSize_5;
    QLabel *label_meshingTextureFormat;
    QComboBox *comboBox_meshingTextureFormat;
    QComboBox *comboBox_meshingTextureSize;
    QCheckBox *checkBox_exposureFusion;
    QLabel *label_meshingTextureSize_4;
    QLineEdit *lineEdit_meshingTextureRoiRatios;
    QLabel *label_meshingTextureSize;
    QCheckBox *checkBox_cameraFilter;
    QSpinBox *spinBox_textureBrightnessContrastRatioLow;
    QSpinBox *spinBox_mesh_minTextureClusterSize;
    QLabel *label_meshingTextureSize_3;
    QLabel *label_exposureFusion;
    QSpinBox *spinBox_mesh_maxTextures;
    QLabel *label_meshingTextureSize_9;
    QDoubleSpinBox *doubleSpinBox_meshingTextureMaxDepthError;
    QLabel *label_meshingTextureSize_10;
    QDoubleSpinBox *doubleSpinBox_meshingTextureMaxAngle;
    QLabel *label_multiband;
    QCheckBox *checkBox_multiband;
    QLabel *label_meshingTextureSize_11;
    QCheckBox *checkBox_distanceToCamPolicy;
    QGroupBox *groupBox_cameraFilter;
    QGridLayout *gridLayout_16;
    QDoubleSpinBox *doubleSpinBox_cameraFilterRadius;
    QLabel *label_37;
    QDoubleSpinBox *doubleSpinBox_laplacianVariance;
    QDoubleSpinBox *doubleSpinBox_cameraFilterAngle;
    QLabel *label_38;
    QDoubleSpinBox *doubleSpinBox_cameraFilterVelRad;
    QDoubleSpinBox *doubleSpinBox_cameraFilterVel;
    QLabel *label_39;
    QGroupBox *groupBox_gp3;
    QGridLayout *gridLayout_4;
    QLabel *label_169;
    QDoubleSpinBox *doubleSpinBox_gp3Radius;
    QLabel *label_168;
    QDoubleSpinBox *doubleSpinBox_gp3Mu;
    QGroupBox *groupBox_poisson;
    QGridLayout *gridLayout_14;
    QLabel *label_172;
    QLabel *label_177;
    QSpinBox *spinBox_poisson_iso;
    QLabel *label_176;
    QDoubleSpinBox *doubleSpinBox_poisson_samples;
    QDoubleSpinBox *doubleSpinBox_poisson_scale;
    QLabel *label_181;
    QLabel *label_182;
    QLabel *label_178;
    QCheckBox *checkBox_poisson_manifold;
    QLabel *label_179;
    QLabel *label_180;
    QCheckBox *checkBox_poisson_outputPolygons;
    QSpinBox *spinBox_poisson_solver;
    QLabel *label_outputPolygons;
    QDoubleSpinBox *doubleSpinBox_poisson_pointWeight;
    QSpinBox *spinBox_poisson_minDepth;
    QSpinBox *spinBox_poisson_depth;
    QLabel *label_outputPolygons_2;
    QDoubleSpinBox *doubleSpinBox_poisson_targetPolygonSize;
    QGroupBox *groupBox_cputsdf;
    QVBoxLayout *verticalLayout_17;
    QLabel *label_25;
    QGridLayout *gridLayout_17;
    QDoubleSpinBox *doubleSpinBox_cputsdf_size;
    QLabel *label_19;
    QDoubleSpinBox *doubleSpinBox_cputsdf_resolution;
    QLabel *label_20;
    QDoubleSpinBox *doubleSpinBox_cputsdf_tuncPos;
    QLabel *label_21;
    QDoubleSpinBox *doubleSpinBox_cputsdf_tuncNeg;
    QLabel *label_22;
    QSpinBox *spinBox_cputsdf_randomSplit;
    QLabel *label_23;
    QDoubleSpinBox *doubleSpinBox_cputsdf_minWeight;
    QLabel *label_18;
    QDoubleSpinBox *doubleSpinBox_cputsdf_flattenRadius;
    QLabel *label_24;
    QGroupBox *groupBox_organized;
    QGridLayout *gridLayout_2;
    QLabel *label_15;
    QLabel *label_quad;
    QCheckBox *checkBox_mesh_quad;
    QDoubleSpinBox *doubleSpinBox_mesh_angleTolerance;
    QLabel *label_14;
    QSpinBox *spinBox_mesh_triangleSize;
    QGroupBox *groupBox_openchisel;
    QVBoxLayout *verticalLayout_18;
    QLabel *label_26;
    QGridLayout *gridLayout_19;
    QLabel *label_36;
    QLabel *label_35;
    QLabel *label_40;
    QLabel *label_43;
    QLabel *label_46;
    QLabel *label_34;
    QSpinBox *spinBox_openchisel_chunk_size_y;
    QDoubleSpinBox *doubleSpinBox_openchisel_truncation_constant;
    QSpinBox *spinBox_openchisel_chunk_size_z;
    QSpinBox *spinBox_openchisel_chunk_size_x;
    QLabel *label_31;
    QLabel *label_42;
    QLabel *label_41;
    QLabel *label_44;
    QLabel *label_45;
    QLabel *label_47;
    QDoubleSpinBox *doubleSpinBox_openchisel_truncation_linear;
    QDoubleSpinBox *doubleSpinBox_openchisel_truncation_quadratic;
    QDoubleSpinBox *doubleSpinBox_openchisel_truncation_scale;
    QSpinBox *spinBox_openchisel_integration_weight;
    QCheckBox *checkBox_openchisel_use_voxel_carving;
    QDoubleSpinBox *doubleSpinBox_openchisel_carving_dist_m;
    QDoubleSpinBox *doubleSpinBox_openchisel_near_plane_dist;
    QDoubleSpinBox *doubleSpinBox_openchisel_far_plane_dist;
    QLabel *label_48;
    QCheckBox *checkBox_openchisel_mergeVertices;
    QSpacerItem *verticalSpacer_8;
    QSpacerItem *verticalSpacer_9;
    QSpacerItem *verticalSpacer_7;
    QSpacerItem *verticalSpacer;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *ExportCloudsDialog)
    {
        if (ExportCloudsDialog->objectName().isEmpty())
            ExportCloudsDialog->setObjectName(QStringLiteral("ExportCloudsDialog"));
        ExportCloudsDialog->resize(814, 680);
        verticalLayout_2 = new QVBoxLayout(ExportCloudsDialog);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        scrollArea = new QScrollArea(ExportCloudsDialog);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, -3906, 780, 5635));
        verticalLayout_13 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_13->setObjectName(QStringLiteral("verticalLayout_13"));
        gridLayout_8 = new QGridLayout();
        gridLayout_8->setObjectName(QStringLiteral("gridLayout_8"));
        comboBox_pipeline = new QComboBox(scrollAreaWidgetContents);
        comboBox_pipeline->setObjectName(QStringLiteral("comboBox_pipeline"));

        gridLayout_8->addWidget(comboBox_pipeline, 1, 0, 1, 1);

        label_12 = new QLabel(scrollAreaWidgetContents);
        label_12->setObjectName(QStringLiteral("label_12"));

        gridLayout_8->addWidget(label_12, 1, 1, 1, 1);

        checkBox_regenerate = new QCheckBox(scrollAreaWidgetContents);
        checkBox_regenerate->setObjectName(QStringLiteral("checkBox_regenerate"));

        gridLayout_8->addWidget(checkBox_regenerate, 11, 0, 1, 1);

        checkBox_filtering = new QCheckBox(scrollAreaWidgetContents);
        checkBox_filtering->setObjectName(QStringLiteral("checkBox_filtering"));

        gridLayout_8->addWidget(checkBox_filtering, 12, 0, 1, 1);

        label_regenerate = new QLabel(scrollAreaWidgetContents);
        label_regenerate->setObjectName(QStringLiteral("label_regenerate"));
        label_regenerate->setWordWrap(true);

        gridLayout_8->addWidget(label_regenerate, 11, 1, 1, 1);

        label_voxel = new QLabel(scrollAreaWidgetContents);
        label_voxel->setObjectName(QStringLiteral("label_voxel"));
        label_voxel->setWordWrap(true);

        gridLayout_8->addWidget(label_voxel, 8, 1, 1, 1);

        label_binaryFile_2 = new QLabel(scrollAreaWidgetContents);
        label_binaryFile_2->setObjectName(QStringLiteral("label_binaryFile_2"));
        label_binaryFile_2->setWordWrap(true);

        gridLayout_8->addWidget(label_binaryFile_2, 4, 1, 1, 1);

        label_frame = new QLabel(scrollAreaWidgetContents);
        label_frame->setObjectName(QStringLiteral("label_frame"));
        label_frame->setWordWrap(true);

        gridLayout_8->addWidget(label_frame, 5, 1, 1, 1);

        spinBox_normalKSearch = new QSpinBox(scrollAreaWidgetContents);
        spinBox_normalKSearch->setObjectName(QStringLiteral("spinBox_normalKSearch"));
        spinBox_normalKSearch->setMinimum(0);
        spinBox_normalKSearch->setValue(20);

        gridLayout_8->addWidget(spinBox_normalKSearch, 6, 0, 1, 1);

        checkBox_smoothing = new QCheckBox(scrollAreaWidgetContents);
        checkBox_smoothing->setObjectName(QStringLiteral("checkBox_smoothing"));

        gridLayout_8->addWidget(checkBox_smoothing, 13, 0, 1, 1);

        label_binaryFile_9 = new QLabel(scrollAreaWidgetContents);
        label_binaryFile_9->setObjectName(QStringLiteral("label_binaryFile_9"));
        label_binaryFile_9->setWordWrap(true);

        gridLayout_8->addWidget(label_binaryFile_9, 12, 1, 1, 1);

        label_binaryFile = new QLabel(scrollAreaWidgetContents);
        label_binaryFile->setObjectName(QStringLiteral("label_binaryFile"));
        label_binaryFile->setWordWrap(true);

        gridLayout_8->addWidget(label_binaryFile, 3, 1, 1, 1);

        label_smoothing = new QLabel(scrollAreaWidgetContents);
        label_smoothing->setObjectName(QStringLiteral("label_smoothing"));
        label_smoothing->setWordWrap(true);

        gridLayout_8->addWidget(label_smoothing, 13, 1, 1, 1);

        doubleSpinBox_voxelSize_assembled = new QDoubleSpinBox(scrollAreaWidgetContents);
        doubleSpinBox_voxelSize_assembled->setObjectName(QStringLiteral("doubleSpinBox_voxelSize_assembled"));
        doubleSpinBox_voxelSize_assembled->setDecimals(3);
        doubleSpinBox_voxelSize_assembled->setMaximum(1);
        doubleSpinBox_voxelSize_assembled->setSingleStep(0.01);
        doubleSpinBox_voxelSize_assembled->setValue(0.005);

        gridLayout_8->addWidget(doubleSpinBox_voxelSize_assembled, 8, 0, 1, 1);

        checkBox_assemble = new QCheckBox(scrollAreaWidgetContents);
        checkBox_assemble->setObjectName(QStringLiteral("checkBox_assemble"));

        gridLayout_8->addWidget(checkBox_assemble, 4, 0, 1, 1);

        label_normal = new QLabel(scrollAreaWidgetContents);
        label_normal->setObjectName(QStringLiteral("label_normal"));
        label_normal->setWordWrap(true);

        gridLayout_8->addWidget(label_normal, 6, 1, 1, 1);

        comboBox_frame = new QComboBox(scrollAreaWidgetContents);
        comboBox_frame->setObjectName(QStringLiteral("comboBox_frame"));

        gridLayout_8->addWidget(comboBox_frame, 5, 0, 1, 1);

        checkBox_binary = new QCheckBox(scrollAreaWidgetContents);
        checkBox_binary->setObjectName(QStringLiteral("checkBox_binary"));
        checkBox_binary->setChecked(true);

        gridLayout_8->addWidget(checkBox_binary, 3, 0, 1, 1);

        label_binaryFile_11 = new QLabel(scrollAreaWidgetContents);
        label_binaryFile_11->setObjectName(QStringLiteral("label_binaryFile_11"));
        label_binaryFile_11->setWordWrap(true);

        gridLayout_8->addWidget(label_binaryFile_11, 0, 1, 1, 1);

        checkBox_fromDepth = new QCheckBox(scrollAreaWidgetContents);
        checkBox_fromDepth->setObjectName(QStringLiteral("checkBox_fromDepth"));
        checkBox_fromDepth->setChecked(true);

        gridLayout_8->addWidget(checkBox_fromDepth, 0, 0, 1, 1);

        label_normal_2 = new QLabel(scrollAreaWidgetContents);
        label_normal_2->setObjectName(QStringLiteral("label_normal_2"));
        label_normal_2->setWordWrap(true);

        gridLayout_8->addWidget(label_normal_2, 7, 1, 1, 1);

        doubleSpinBox_normalRadiusSearch = new QDoubleSpinBox(scrollAreaWidgetContents);
        doubleSpinBox_normalRadiusSearch->setObjectName(QStringLiteral("doubleSpinBox_normalRadiusSearch"));
        doubleSpinBox_normalRadiusSearch->setSingleStep(0.01);

        gridLayout_8->addWidget(doubleSpinBox_normalRadiusSearch, 7, 0, 1, 1);

        label_intensityColormap = new QLabel(scrollAreaWidgetContents);
        label_intensityColormap->setObjectName(QStringLiteral("label_intensityColormap"));
        label_intensityColormap->setWordWrap(true);

        gridLayout_8->addWidget(label_intensityColormap, 9, 1, 1, 1);

        comboBox_intensityColormap = new QComboBox(scrollAreaWidgetContents);
        comboBox_intensityColormap->setObjectName(QStringLiteral("comboBox_intensityColormap"));

        gridLayout_8->addWidget(comboBox_intensityColormap, 9, 0, 1, 1);

        label_gainCompensation = new QLabel(scrollAreaWidgetContents);
        label_gainCompensation->setObjectName(QStringLiteral("label_gainCompensation"));
        label_gainCompensation->setWordWrap(true);

        gridLayout_8->addWidget(label_gainCompensation, 14, 1, 1, 1);

        label_cameraProjection = new QLabel(scrollAreaWidgetContents);
        label_cameraProjection->setObjectName(QStringLiteral("label_cameraProjection"));
        label_cameraProjection->setWordWrap(true);

        gridLayout_8->addWidget(label_cameraProjection, 15, 1, 1, 1);

        checkBox_gainCompensation = new QCheckBox(scrollAreaWidgetContents);
        checkBox_gainCompensation->setObjectName(QStringLiteral("checkBox_gainCompensation"));

        gridLayout_8->addWidget(checkBox_gainCompensation, 14, 0, 1, 1);

        checkBox_cameraProjection = new QCheckBox(scrollAreaWidgetContents);
        checkBox_cameraProjection->setObjectName(QStringLiteral("checkBox_cameraProjection"));

        gridLayout_8->addWidget(checkBox_cameraProjection, 15, 0, 1, 1);

        label_binaryFile_12 = new QLabel(scrollAreaWidgetContents);
        label_binaryFile_12->setObjectName(QStringLiteral("label_binaryFile_12"));
        label_binaryFile_12->setWordWrap(true);

        gridLayout_8->addWidget(label_binaryFile_12, 16, 1, 1, 1);

        checkBox_meshing = new QCheckBox(scrollAreaWidgetContents);
        checkBox_meshing->setObjectName(QStringLiteral("checkBox_meshing"));

        gridLayout_8->addWidget(checkBox_meshing, 16, 0, 1, 1);

        label_binaryFile_10 = new QLabel(scrollAreaWidgetContents);
        label_binaryFile_10->setObjectName(QStringLiteral("label_binaryFile_10"));
        label_binaryFile_10->setWordWrap(true);

        gridLayout_8->addWidget(label_binaryFile_10, 10, 1, 1, 1);

        checkBox_nodes_filtering = new QCheckBox(scrollAreaWidgetContents);
        checkBox_nodes_filtering->setObjectName(QStringLiteral("checkBox_nodes_filtering"));

        gridLayout_8->addWidget(checkBox_nodes_filtering, 10, 0, 1, 1);

        gridLayout_8->setColumnStretch(1, 1);

        verticalLayout_13->addLayout(gridLayout_8);

        groupBox_nodes_filtering = new QGroupBox(scrollAreaWidgetContents);
        groupBox_nodes_filtering->setObjectName(QStringLiteral("groupBox_nodes_filtering"));
        verticalLayout_19 = new QVBoxLayout(groupBox_nodes_filtering);
        verticalLayout_19->setObjectName(QStringLiteral("verticalLayout_19"));
        label_114 = new QLabel(groupBox_nodes_filtering);
        label_114->setObjectName(QStringLiteral("label_114"));
        label_114->setWordWrap(true);

        verticalLayout_19->addWidget(label_114);

        gridLayout_21 = new QGridLayout();
        gridLayout_21->setObjectName(QStringLiteral("gridLayout_21"));
        label_113 = new QLabel(groupBox_nodes_filtering);
        label_113->setObjectName(QStringLiteral("label_113"));
        label_113->setWordWrap(true);

        gridLayout_21->addWidget(label_113, 0, 2, 1, 1);

        label_142 = new QLabel(groupBox_nodes_filtering);
        label_142->setObjectName(QStringLiteral("label_142"));
        label_142->setWordWrap(true);

        gridLayout_21->addWidget(label_142, 1, 0, 1, 1);

        doubleSpinBox_nodes_filtering_ymin = new QDoubleSpinBox(groupBox_nodes_filtering);
        doubleSpinBox_nodes_filtering_ymin->setObjectName(QStringLiteral("doubleSpinBox_nodes_filtering_ymin"));
        doubleSpinBox_nodes_filtering_ymin->setDecimals(0);
        doubleSpinBox_nodes_filtering_ymin->setMinimum(-1e+7);
        doubleSpinBox_nodes_filtering_ymin->setMaximum(1e+7);

        gridLayout_21->addWidget(doubleSpinBox_nodes_filtering_ymin, 1, 1, 1, 1);

        label_115 = new QLabel(groupBox_nodes_filtering);
        label_115->setObjectName(QStringLiteral("label_115"));
        label_115->setWordWrap(true);

        gridLayout_21->addWidget(label_115, 1, 2, 1, 1);

        label_116 = new QLabel(groupBox_nodes_filtering);
        label_116->setObjectName(QStringLiteral("label_116"));
        label_116->setWordWrap(true);

        gridLayout_21->addWidget(label_116, 2, 0, 1, 1);

        label_119 = new QLabel(groupBox_nodes_filtering);
        label_119->setObjectName(QStringLiteral("label_119"));
        label_119->setWordWrap(true);

        gridLayout_21->addWidget(label_119, 2, 2, 1, 1);

        label_111 = new QLabel(groupBox_nodes_filtering);
        label_111->setObjectName(QStringLiteral("label_111"));
        label_111->setWordWrap(true);

        gridLayout_21->addWidget(label_111, 0, 0, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_21->addItem(horizontalSpacer, 1, 4, 1, 1);

        doubleSpinBox_nodes_filtering_xmin = new QDoubleSpinBox(groupBox_nodes_filtering);
        doubleSpinBox_nodes_filtering_xmin->setObjectName(QStringLiteral("doubleSpinBox_nodes_filtering_xmin"));
        doubleSpinBox_nodes_filtering_xmin->setDecimals(0);
        doubleSpinBox_nodes_filtering_xmin->setMinimum(-1e+7);
        doubleSpinBox_nodes_filtering_xmin->setMaximum(1e+7);

        gridLayout_21->addWidget(doubleSpinBox_nodes_filtering_xmin, 0, 1, 1, 1);

        doubleSpinBox_nodes_filtering_xmax = new QDoubleSpinBox(groupBox_nodes_filtering);
        doubleSpinBox_nodes_filtering_xmax->setObjectName(QStringLiteral("doubleSpinBox_nodes_filtering_xmax"));
        doubleSpinBox_nodes_filtering_xmax->setDecimals(0);
        doubleSpinBox_nodes_filtering_xmax->setMinimum(-1e+7);
        doubleSpinBox_nodes_filtering_xmax->setMaximum(1e+7);

        gridLayout_21->addWidget(doubleSpinBox_nodes_filtering_xmax, 0, 3, 1, 1);

        doubleSpinBox_nodes_filtering_zmin = new QDoubleSpinBox(groupBox_nodes_filtering);
        doubleSpinBox_nodes_filtering_zmin->setObjectName(QStringLiteral("doubleSpinBox_nodes_filtering_zmin"));
        doubleSpinBox_nodes_filtering_zmin->setDecimals(0);
        doubleSpinBox_nodes_filtering_zmin->setMinimum(-1e+7);
        doubleSpinBox_nodes_filtering_zmin->setMaximum(1e+7);

        gridLayout_21->addWidget(doubleSpinBox_nodes_filtering_zmin, 2, 1, 1, 1);

        doubleSpinBox_nodes_filtering_ymax = new QDoubleSpinBox(groupBox_nodes_filtering);
        doubleSpinBox_nodes_filtering_ymax->setObjectName(QStringLiteral("doubleSpinBox_nodes_filtering_ymax"));
        doubleSpinBox_nodes_filtering_ymax->setDecimals(0);
        doubleSpinBox_nodes_filtering_ymax->setMinimum(-1e+7);
        doubleSpinBox_nodes_filtering_ymax->setMaximum(1e+7);

        gridLayout_21->addWidget(doubleSpinBox_nodes_filtering_ymax, 1, 3, 1, 1);

        doubleSpinBox_nodes_filtering_zmax = new QDoubleSpinBox(groupBox_nodes_filtering);
        doubleSpinBox_nodes_filtering_zmax->setObjectName(QStringLiteral("doubleSpinBox_nodes_filtering_zmax"));
        doubleSpinBox_nodes_filtering_zmax->setDecimals(0);
        doubleSpinBox_nodes_filtering_zmax->setMinimum(-1e+7);
        doubleSpinBox_nodes_filtering_zmax->setMaximum(1e+7);

        gridLayout_21->addWidget(doubleSpinBox_nodes_filtering_zmax, 2, 3, 1, 1);


        verticalLayout_19->addLayout(gridLayout_21);


        verticalLayout_13->addWidget(groupBox_nodes_filtering);

        groupBox_regenerateScans = new QGroupBox(scrollAreaWidgetContents);
        groupBox_regenerateScans->setObjectName(QStringLiteral("groupBox_regenerateScans"));
        gridLayout_18 = new QGridLayout(groupBox_regenerateScans);
        gridLayout_18->setObjectName(QStringLiteral("gridLayout_18"));
        spinBox_decimation_scan = new QSpinBox(groupBox_regenerateScans);
        spinBox_decimation_scan->setObjectName(QStringLiteral("spinBox_decimation_scan"));
        spinBox_decimation_scan->setMinimum(-32);
        spinBox_decimation_scan->setMaximum(32);
        spinBox_decimation_scan->setValue(1);

        gridLayout_18->addWidget(spinBox_decimation_scan, 0, 0, 1, 1);

        label_109 = new QLabel(groupBox_regenerateScans);
        label_109->setObjectName(QStringLiteral("label_109"));
        label_109->setWordWrap(true);

        gridLayout_18->addWidget(label_109, 0, 1, 1, 1);

        label_138 = new QLabel(groupBox_regenerateScans);
        label_138->setObjectName(QStringLiteral("label_138"));
        label_138->setWordWrap(true);

        gridLayout_18->addWidget(label_138, 1, 1, 1, 1);

        doubleSpinBox_rangeMin = new QDoubleSpinBox(groupBox_regenerateScans);
        doubleSpinBox_rangeMin->setObjectName(QStringLiteral("doubleSpinBox_rangeMin"));
        doubleSpinBox_rangeMin->setDecimals(1);
        doubleSpinBox_rangeMin->setMaximum(100);
        doubleSpinBox_rangeMin->setSingleStep(0.1);
        doubleSpinBox_rangeMin->setValue(0);

        gridLayout_18->addWidget(doubleSpinBox_rangeMin, 1, 0, 1, 1);

        doubleSpinBox_rangeMax = new QDoubleSpinBox(groupBox_regenerateScans);
        doubleSpinBox_rangeMax->setObjectName(QStringLiteral("doubleSpinBox_rangeMax"));
        doubleSpinBox_rangeMax->setDecimals(1);
        doubleSpinBox_rangeMax->setMaximum(1000);
        doubleSpinBox_rangeMax->setSingleStep(0.1);
        doubleSpinBox_rangeMax->setValue(0);

        gridLayout_18->addWidget(doubleSpinBox_rangeMax, 2, 0, 1, 1);

        label_139 = new QLabel(groupBox_regenerateScans);
        label_139->setObjectName(QStringLiteral("label_139"));
        label_139->setWordWrap(true);

        gridLayout_18->addWidget(label_139, 2, 1, 1, 1);

        gridLayout_18->setColumnStretch(1, 1);

        verticalLayout_13->addWidget(groupBox_regenerateScans);

        groupBox_regenerate = new QGroupBox(scrollAreaWidgetContents);
        groupBox_regenerate->setObjectName(QStringLiteral("groupBox_regenerate"));
        groupBox_regenerate->setCheckable(false);
        verticalLayout_14 = new QVBoxLayout(groupBox_regenerate);
        verticalLayout_14->setObjectName(QStringLiteral("verticalLayout_14"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        toolButton_distortionModel = new QToolButton(groupBox_regenerate);
        toolButton_distortionModel->setObjectName(QStringLiteral("toolButton_distortionModel"));

        gridLayout->addWidget(toolButton_distortionModel, 8, 0, 1, 1);

        spinBox_fillDepthHoles = new QSpinBox(groupBox_regenerate);
        spinBox_fillDepthHoles->setObjectName(QStringLiteral("spinBox_fillDepthHoles"));

        gridLayout->addWidget(spinBox_fillDepthHoles, 6, 1, 1, 1);

        spinBox_decimation = new QSpinBox(groupBox_regenerate);
        spinBox_decimation->setObjectName(QStringLiteral("spinBox_decimation"));
        spinBox_decimation->setMinimum(-32);
        spinBox_decimation->setMaximum(32);
        spinBox_decimation->setValue(1);

        gridLayout->addWidget(spinBox_decimation, 0, 1, 1, 1);

        doubleSpinBox_minDepth = new QDoubleSpinBox(groupBox_regenerate);
        doubleSpinBox_minDepth->setObjectName(QStringLiteral("doubleSpinBox_minDepth"));
        doubleSpinBox_minDepth->setDecimals(2);
        doubleSpinBox_minDepth->setMaximum(100);
        doubleSpinBox_minDepth->setSingleStep(0.1);
        doubleSpinBox_minDepth->setValue(0);

        gridLayout->addWidget(doubleSpinBox_minDepth, 2, 1, 1, 1);

        label_108 = new QLabel(groupBox_regenerate);
        label_108->setObjectName(QStringLiteral("label_108"));
        label_108->setWordWrap(true);

        gridLayout->addWidget(label_108, 0, 2, 1, 1);

        lineEdit_distortionModel = new QLineEdit(groupBox_regenerate);
        lineEdit_distortionModel->setObjectName(QStringLiteral("lineEdit_distortionModel"));

        gridLayout->addWidget(lineEdit_distortionModel, 8, 1, 1, 1);

        spinBox_fillDepthHolesError = new QSpinBox(groupBox_regenerate);
        spinBox_fillDepthHolesError->setObjectName(QStringLiteral("spinBox_fillDepthHolesError"));

        gridLayout->addWidget(spinBox_fillDepthHolesError, 7, 1, 1, 1);

        label_132 = new QLabel(groupBox_regenerate);
        label_132->setObjectName(QStringLiteral("label_132"));
        label_132->setWordWrap(true);

        gridLayout->addWidget(label_132, 1, 2, 1, 1);

        label_133 = new QLabel(groupBox_regenerate);
        label_133->setObjectName(QStringLiteral("label_133"));
        label_133->setWordWrap(true);

        gridLayout->addWidget(label_133, 2, 2, 1, 1);

        label_197 = new QLabel(groupBox_regenerate);
        label_197->setObjectName(QStringLiteral("label_197"));
        label_197->setWordWrap(true);
        label_197->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout->addWidget(label_197, 6, 2, 1, 1);

        doubleSpinBox_maxDepth = new QDoubleSpinBox(groupBox_regenerate);
        doubleSpinBox_maxDepth->setObjectName(QStringLiteral("doubleSpinBox_maxDepth"));
        doubleSpinBox_maxDepth->setDecimals(2);
        doubleSpinBox_maxDepth->setMaximum(100);
        doubleSpinBox_maxDepth->setSingleStep(0.1);
        doubleSpinBox_maxDepth->setValue(4);

        gridLayout->addWidget(doubleSpinBox_maxDepth, 1, 1, 1, 1);

        label_134 = new QLabel(groupBox_regenerate);
        label_134->setObjectName(QStringLiteral("label_134"));
        label_134->setWordWrap(true);

        gridLayout->addWidget(label_134, 8, 2, 1, 1);

        label_198 = new QLabel(groupBox_regenerate);
        label_198->setObjectName(QStringLiteral("label_198"));
        label_198->setWordWrap(true);
        label_198->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout->addWidget(label_198, 7, 2, 1, 1);

        label_135 = new QLabel(groupBox_regenerate);
        label_135->setObjectName(QStringLiteral("label_135"));
        label_135->setWordWrap(true);

        gridLayout->addWidget(label_135, 5, 2, 1, 1);

        lineEdit_roiRatios = new QLineEdit(groupBox_regenerate);
        lineEdit_roiRatios->setObjectName(QStringLiteral("lineEdit_roiRatios"));

        gridLayout->addWidget(lineEdit_roiRatios, 5, 1, 1, 1);

        checkBox_bilateral = new QCheckBox(groupBox_regenerate);
        checkBox_bilateral->setObjectName(QStringLiteral("checkBox_bilateral"));

        gridLayout->addWidget(checkBox_bilateral, 9, 1, 1, 1);

        label_136 = new QLabel(groupBox_regenerate);
        label_136->setObjectName(QStringLiteral("label_136"));
        label_136->setWordWrap(true);

        gridLayout->addWidget(label_136, 9, 2, 1, 1);

        label_137 = new QLabel(groupBox_regenerate);
        label_137->setObjectName(QStringLiteral("label_137"));
        label_137->setWordWrap(true);

        gridLayout->addWidget(label_137, 10, 2, 1, 1);

        checkBox_subtraction = new QCheckBox(groupBox_regenerate);
        checkBox_subtraction->setObjectName(QStringLiteral("checkBox_subtraction"));

        gridLayout->addWidget(checkBox_subtraction, 10, 1, 1, 1);

        label_140 = new QLabel(groupBox_regenerate);
        label_140->setObjectName(QStringLiteral("label_140"));
        label_140->setWordWrap(true);

        gridLayout->addWidget(label_140, 3, 2, 1, 1);

        label_141 = new QLabel(groupBox_regenerate);
        label_141->setObjectName(QStringLiteral("label_141"));
        label_141->setWordWrap(true);

        gridLayout->addWidget(label_141, 4, 2, 1, 1);

        doubleSpinBox_ceilingHeight = new QDoubleSpinBox(groupBox_regenerate);
        doubleSpinBox_ceilingHeight->setObjectName(QStringLiteral("doubleSpinBox_ceilingHeight"));
        doubleSpinBox_ceilingHeight->setDecimals(2);
        doubleSpinBox_ceilingHeight->setMinimum(-1000);
        doubleSpinBox_ceilingHeight->setMaximum(1000);
        doubleSpinBox_ceilingHeight->setSingleStep(0.1);
        doubleSpinBox_ceilingHeight->setValue(0);

        gridLayout->addWidget(doubleSpinBox_ceilingHeight, 3, 1, 1, 1);

        doubleSpinBox_floorHeight = new QDoubleSpinBox(groupBox_regenerate);
        doubleSpinBox_floorHeight->setObjectName(QStringLiteral("doubleSpinBox_floorHeight"));
        doubleSpinBox_floorHeight->setDecimals(2);
        doubleSpinBox_floorHeight->setMinimum(-1000);
        doubleSpinBox_floorHeight->setMaximum(1000);
        doubleSpinBox_floorHeight->setSingleStep(0.1);
        doubleSpinBox_floorHeight->setValue(0);

        gridLayout->addWidget(doubleSpinBox_floorHeight, 4, 1, 1, 1);

        gridLayout->setColumnStretch(2, 1);

        verticalLayout_14->addLayout(gridLayout);

        groupBox_bilateral = new QGroupBox(groupBox_regenerate);
        groupBox_bilateral->setObjectName(QStringLiteral("groupBox_bilateral"));
        groupBox_bilateral->setCheckable(false);
        gridLayout_13 = new QGridLayout(groupBox_bilateral);
        gridLayout_13->setObjectName(QStringLiteral("gridLayout_13"));
        doubleSpinBox_bilateral_sigmaR = new QDoubleSpinBox(groupBox_bilateral);
        doubleSpinBox_bilateral_sigmaR->setObjectName(QStringLiteral("doubleSpinBox_bilateral_sigmaR"));
        doubleSpinBox_bilateral_sigmaR->setDecimals(4);
        doubleSpinBox_bilateral_sigmaR->setMinimum(0.0001);
        doubleSpinBox_bilateral_sigmaR->setMaximum(1);
        doubleSpinBox_bilateral_sigmaR->setSingleStep(0.001);
        doubleSpinBox_bilateral_sigmaR->setValue(0.005);

        gridLayout_13->addWidget(doubleSpinBox_bilateral_sigmaR, 1, 0, 1, 1);

        label_194 = new QLabel(groupBox_bilateral);
        label_194->setObjectName(QStringLiteral("label_194"));
        label_194->setWordWrap(true);
        label_194->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_13->addWidget(label_194, 1, 1, 1, 1);

        doubleSpinBox_bilateral_sigmaS = new QDoubleSpinBox(groupBox_bilateral);
        doubleSpinBox_bilateral_sigmaS->setObjectName(QStringLiteral("doubleSpinBox_bilateral_sigmaS"));
        doubleSpinBox_bilateral_sigmaS->setDecimals(1);
        doubleSpinBox_bilateral_sigmaS->setMinimum(0.1);
        doubleSpinBox_bilateral_sigmaS->setMaximum(100);
        doubleSpinBox_bilateral_sigmaS->setSingleStep(1);
        doubleSpinBox_bilateral_sigmaS->setValue(5);

        gridLayout_13->addWidget(doubleSpinBox_bilateral_sigmaS, 0, 0, 1, 1);

        label_195 = new QLabel(groupBox_bilateral);
        label_195->setObjectName(QStringLiteral("label_195"));
        label_195->setWordWrap(true);
        label_195->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_13->addWidget(label_195, 0, 1, 1, 1);

        gridLayout_13->setColumnStretch(1, 1);

        verticalLayout_14->addWidget(groupBox_bilateral);

        groupBox_subtraction = new QGroupBox(groupBox_regenerate);
        groupBox_subtraction->setObjectName(QStringLiteral("groupBox_subtraction"));
        groupBox_subtraction->setCheckable(false);
        gridLayout_11 = new QGridLayout(groupBox_subtraction);
        gridLayout_11->setObjectName(QStringLiteral("gridLayout_11"));
        label_174 = new QLabel(groupBox_subtraction);
        label_174->setObjectName(QStringLiteral("label_174"));
        label_174->setWordWrap(true);
        label_174->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_11->addWidget(label_174, 2, 1, 1, 1);

        label_193 = new QLabel(groupBox_subtraction);
        label_193->setObjectName(QStringLiteral("label_193"));
        label_193->setWordWrap(true);
        label_193->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_11->addWidget(label_193, 1, 1, 1, 1);

        doubleSpinBox_subtractPointFilteringRadius = new QDoubleSpinBox(groupBox_subtraction);
        doubleSpinBox_subtractPointFilteringRadius->setObjectName(QStringLiteral("doubleSpinBox_subtractPointFilteringRadius"));
        doubleSpinBox_subtractPointFilteringRadius->setMinimum(0.01);
        doubleSpinBox_subtractPointFilteringRadius->setMaximum(1);
        doubleSpinBox_subtractPointFilteringRadius->setSingleStep(0.01);
        doubleSpinBox_subtractPointFilteringRadius->setValue(0.02);

        gridLayout_11->addWidget(doubleSpinBox_subtractPointFilteringRadius, 0, 0, 1, 1);

        label_192 = new QLabel(groupBox_subtraction);
        label_192->setObjectName(QStringLiteral("label_192"));
        label_192->setWordWrap(true);
        label_192->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_11->addWidget(label_192, 0, 1, 1, 1);

        spinBox_subtractFilteringMinPts = new QSpinBox(groupBox_subtraction);
        spinBox_subtractFilteringMinPts->setObjectName(QStringLiteral("spinBox_subtractFilteringMinPts"));
        spinBox_subtractFilteringMinPts->setMinimum(1);

        gridLayout_11->addWidget(spinBox_subtractFilteringMinPts, 2, 0, 1, 1);

        doubleSpinBox_subtractPointFilteringAngle = new QDoubleSpinBox(groupBox_subtraction);
        doubleSpinBox_subtractPointFilteringAngle->setObjectName(QStringLiteral("doubleSpinBox_subtractPointFilteringAngle"));
        doubleSpinBox_subtractPointFilteringAngle->setDecimals(0);
        doubleSpinBox_subtractPointFilteringAngle->setMaximum(180);
        doubleSpinBox_subtractPointFilteringAngle->setValue(30);

        gridLayout_11->addWidget(doubleSpinBox_subtractPointFilteringAngle, 1, 0, 1, 1);

        gridLayout_11->setColumnStretch(1, 1);

        verticalLayout_14->addWidget(groupBox_subtraction);


        verticalLayout_13->addWidget(groupBox_regenerate);

        groupBox_filtering = new QGroupBox(scrollAreaWidgetContents);
        groupBox_filtering->setObjectName(QStringLiteral("groupBox_filtering"));
        groupBox_filtering->setCheckable(false);
        gridLayout_9 = new QGridLayout(groupBox_filtering);
        gridLayout_9->setObjectName(QStringLiteral("gridLayout_9"));
        doubleSpinBox_filteringRadius = new QDoubleSpinBox(groupBox_filtering);
        doubleSpinBox_filteringRadius->setObjectName(QStringLiteral("doubleSpinBox_filteringRadius"));
        doubleSpinBox_filteringRadius->setDecimals(3);
        doubleSpinBox_filteringRadius->setMinimum(0.001);
        doubleSpinBox_filteringRadius->setMaximum(1);
        doubleSpinBox_filteringRadius->setSingleStep(0.01);
        doubleSpinBox_filteringRadius->setValue(0.02);

        gridLayout_9->addWidget(doubleSpinBox_filteringRadius, 0, 0, 1, 1);

        label_110 = new QLabel(groupBox_filtering);
        label_110->setObjectName(QStringLiteral("label_110"));
        label_110->setWordWrap(true);

        gridLayout_9->addWidget(label_110, 0, 1, 1, 1);

        spinBox_filteringMinNeighbors = new QSpinBox(groupBox_filtering);
        spinBox_filteringMinNeighbors->setObjectName(QStringLiteral("spinBox_filteringMinNeighbors"));
        spinBox_filteringMinNeighbors->setMinimum(1);
        spinBox_filteringMinNeighbors->setMaximum(100);
        spinBox_filteringMinNeighbors->setValue(2);

        gridLayout_9->addWidget(spinBox_filteringMinNeighbors, 1, 0, 1, 1);

        label_112 = new QLabel(groupBox_filtering);
        label_112->setObjectName(QStringLiteral("label_112"));
        label_112->setWordWrap(true);

        gridLayout_9->addWidget(label_112, 1, 1, 1, 1);

        gridLayout_9->setColumnStretch(1, 1);

        verticalLayout_13->addWidget(groupBox_filtering);

        groupBox_mls = new QGroupBox(scrollAreaWidgetContents);
        groupBox_mls->setObjectName(QStringLiteral("groupBox_mls"));
        groupBox_mls->setCheckable(false);
        groupBox_mls->setChecked(false);
        verticalLayout = new QVBoxLayout(groupBox_mls);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        label = new QLabel(groupBox_mls);
        label->setObjectName(QStringLiteral("label"));
        label->setWordWrap(true);

        verticalLayout->addWidget(label);

        gridLayout_3 = new QGridLayout();
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        doubleSpinBox_mlsRadius = new QDoubleSpinBox(groupBox_mls);
        doubleSpinBox_mlsRadius->setObjectName(QStringLiteral("doubleSpinBox_mlsRadius"));
        doubleSpinBox_mlsRadius->setDecimals(3);
        doubleSpinBox_mlsRadius->setMaximum(1);
        doubleSpinBox_mlsRadius->setSingleStep(0.01);
        doubleSpinBox_mlsRadius->setValue(0.04);

        gridLayout_3->addWidget(doubleSpinBox_mlsRadius, 0, 0, 1, 1);

        comboBox_upsamplingMethod = new QComboBox(groupBox_mls);
        comboBox_upsamplingMethod->setObjectName(QStringLiteral("comboBox_upsamplingMethod"));

        gridLayout_3->addWidget(comboBox_upsamplingMethod, 2, 0, 1, 1);

        label_87 = new QLabel(groupBox_mls);
        label_87->setObjectName(QStringLiteral("label_87"));
        label_87->setWordWrap(true);

        gridLayout_3->addWidget(label_87, 0, 1, 1, 1);

        label_88 = new QLabel(groupBox_mls);
        label_88->setObjectName(QStringLiteral("label_88"));
        label_88->setWordWrap(true);

        gridLayout_3->addWidget(label_88, 2, 1, 1, 1);

        label_89 = new QLabel(groupBox_mls);
        label_89->setObjectName(QStringLiteral("label_89"));
        label_89->setWordWrap(true);

        gridLayout_3->addWidget(label_89, 1, 1, 1, 1);

        spinBox_polygonialOrder = new QSpinBox(groupBox_mls);
        spinBox_polygonialOrder->setObjectName(QStringLiteral("spinBox_polygonialOrder"));
        spinBox_polygonialOrder->setMinimum(0);
        spinBox_polygonialOrder->setMaximum(4);
        spinBox_polygonialOrder->setValue(2);

        gridLayout_3->addWidget(spinBox_polygonialOrder, 1, 0, 1, 1);

        label_90 = new QLabel(groupBox_mls);
        label_90->setObjectName(QStringLiteral("label_90"));
        label_90->setWordWrap(true);

        gridLayout_3->addWidget(label_90, 3, 1, 1, 1);

        doubleSpinBox_mls_outputVoxelSize = new QDoubleSpinBox(groupBox_mls);
        doubleSpinBox_mls_outputVoxelSize->setObjectName(QStringLiteral("doubleSpinBox_mls_outputVoxelSize"));
        doubleSpinBox_mls_outputVoxelSize->setDecimals(3);

        gridLayout_3->addWidget(doubleSpinBox_mls_outputVoxelSize, 3, 0, 1, 1);

        gridLayout_3->setColumnStretch(1, 1);

        verticalLayout->addLayout(gridLayout_3);

        stackedWidget_upsampling = new QStackedWidget(groupBox_mls);
        stackedWidget_upsampling->setObjectName(QStringLiteral("stackedWidget_upsampling"));
        page = new QWidget();
        page->setObjectName(QStringLiteral("page"));
        verticalLayout_3 = new QVBoxLayout(page);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        groupBox = new QGroupBox(page);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        verticalLayout_8 = new QVBoxLayout(groupBox);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setWordWrap(true);

        verticalLayout_8->addWidget(label_2);

        verticalSpacer_2 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_8->addItem(verticalSpacer_2);


        verticalLayout_3->addWidget(groupBox);

        stackedWidget_upsampling->addWidget(page);
        page_2 = new QWidget();
        page_2->setObjectName(QStringLiteral("page_2"));
        verticalLayout_4 = new QVBoxLayout(page_2);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        groupBox_2 = new QGroupBox(page_2);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        verticalLayout_9 = new QVBoxLayout(groupBox_2);
        verticalLayout_9->setObjectName(QStringLiteral("verticalLayout_9"));
        label_3 = new QLabel(groupBox_2);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setWordWrap(true);

        verticalLayout_9->addWidget(label_3);

        verticalSpacer_3 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_9->addItem(verticalSpacer_3);


        verticalLayout_4->addWidget(groupBox_2);

        stackedWidget_upsampling->addWidget(page_2);
        page_3 = new QWidget();
        page_3->setObjectName(QStringLiteral("page_3"));
        verticalLayout_7 = new QVBoxLayout(page_3);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        groupBox_3 = new QGroupBox(page_3);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        verticalLayout_10 = new QVBoxLayout(groupBox_3);
        verticalLayout_10->setObjectName(QStringLiteral("verticalLayout_10"));
        label_4 = new QLabel(groupBox_3);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setWordWrap(true);

        verticalLayout_10->addWidget(label_4);

        gridLayout_5 = new QGridLayout();
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        label_8 = new QLabel(groupBox_3);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setWordWrap(true);

        gridLayout_5->addWidget(label_8, 1, 1, 1, 1);

        doubleSpinBox_sampleRadius = new QDoubleSpinBox(groupBox_3);
        doubleSpinBox_sampleRadius->setObjectName(QStringLiteral("doubleSpinBox_sampleRadius"));
        doubleSpinBox_sampleRadius->setDecimals(3);

        gridLayout_5->addWidget(doubleSpinBox_sampleRadius, 0, 0, 1, 1);

        label_7 = new QLabel(groupBox_3);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setWordWrap(true);

        gridLayout_5->addWidget(label_7, 0, 1, 1, 1);

        doubleSpinBox_sampleStep = new QDoubleSpinBox(groupBox_3);
        doubleSpinBox_sampleStep->setObjectName(QStringLiteral("doubleSpinBox_sampleStep"));
        doubleSpinBox_sampleStep->setDecimals(3);

        gridLayout_5->addWidget(doubleSpinBox_sampleStep, 1, 0, 1, 1);

        gridLayout_5->setColumnStretch(1, 1);

        verticalLayout_10->addLayout(gridLayout_5);

        verticalSpacer_4 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_10->addItem(verticalSpacer_4);


        verticalLayout_7->addWidget(groupBox_3);

        stackedWidget_upsampling->addWidget(page_3);
        page_4 = new QWidget();
        page_4->setObjectName(QStringLiteral("page_4"));
        verticalLayout_6 = new QVBoxLayout(page_4);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        groupBox_4 = new QGroupBox(page_4);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        verticalLayout_11 = new QVBoxLayout(groupBox_4);
        verticalLayout_11->setObjectName(QStringLiteral("verticalLayout_11"));
        label_5 = new QLabel(groupBox_4);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setWordWrap(true);

        verticalLayout_11->addWidget(label_5);

        gridLayout_6 = new QGridLayout();
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        spinBox_randomPoints = new QSpinBox(groupBox_4);
        spinBox_randomPoints->setObjectName(QStringLiteral("spinBox_randomPoints"));

        gridLayout_6->addWidget(spinBox_randomPoints, 0, 0, 1, 1);

        label_9 = new QLabel(groupBox_4);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setWordWrap(true);

        gridLayout_6->addWidget(label_9, 0, 1, 1, 1);

        gridLayout_6->setColumnStretch(1, 1);

        verticalLayout_11->addLayout(gridLayout_6);

        verticalSpacer_5 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_11->addItem(verticalSpacer_5);


        verticalLayout_6->addWidget(groupBox_4);

        stackedWidget_upsampling->addWidget(page_4);
        page_5 = new QWidget();
        page_5->setObjectName(QStringLiteral("page_5"));
        verticalLayout_5 = new QVBoxLayout(page_5);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        groupBox_5 = new QGroupBox(page_5);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));
        verticalLayout_12 = new QVBoxLayout(groupBox_5);
        verticalLayout_12->setObjectName(QStringLiteral("verticalLayout_12"));
        label_6 = new QLabel(groupBox_5);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setWordWrap(true);

        verticalLayout_12->addWidget(label_6);

        gridLayout_7 = new QGridLayout();
        gridLayout_7->setObjectName(QStringLiteral("gridLayout_7"));
        label_10 = new QLabel(groupBox_5);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setWordWrap(true);

        gridLayout_7->addWidget(label_10, 0, 1, 1, 1);

        doubleSpinBox_dilationVoxelSize = new QDoubleSpinBox(groupBox_5);
        doubleSpinBox_dilationVoxelSize->setObjectName(QStringLiteral("doubleSpinBox_dilationVoxelSize"));
        doubleSpinBox_dilationVoxelSize->setDecimals(3);

        gridLayout_7->addWidget(doubleSpinBox_dilationVoxelSize, 0, 0, 1, 1);

        label_11 = new QLabel(groupBox_5);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setWordWrap(true);

        gridLayout_7->addWidget(label_11, 1, 1, 1, 1);

        spinBox_dilationSteps = new QSpinBox(groupBox_5);
        spinBox_dilationSteps->setObjectName(QStringLiteral("spinBox_dilationSteps"));

        gridLayout_7->addWidget(spinBox_dilationSteps, 1, 0, 1, 1);

        gridLayout_7->setColumnStretch(1, 1);

        verticalLayout_12->addLayout(gridLayout_7);

        verticalSpacer_6 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_12->addItem(verticalSpacer_6);


        verticalLayout_5->addWidget(groupBox_5);

        stackedWidget_upsampling->addWidget(page_5);

        verticalLayout->addWidget(stackedWidget_upsampling);


        verticalLayout_13->addWidget(groupBox_mls);

        groupBox_gain = new QGroupBox(scrollAreaWidgetContents);
        groupBox_gain->setObjectName(QStringLiteral("groupBox_gain"));
        groupBox_gain->setCheckable(false);
        gridLayout_12 = new QGridLayout(groupBox_gain);
        gridLayout_12->setObjectName(QStringLiteral("gridLayout_12"));
        doubleSpinBox_gainBeta = new QDoubleSpinBox(groupBox_gain);
        doubleSpinBox_gainBeta->setObjectName(QStringLiteral("doubleSpinBox_gainBeta"));
        doubleSpinBox_gainBeta->setDecimals(1);
        doubleSpinBox_gainBeta->setMinimum(0.1);
        doubleSpinBox_gainBeta->setMaximum(1000);
        doubleSpinBox_gainBeta->setSingleStep(1);
        doubleSpinBox_gainBeta->setValue(100);

        gridLayout_12->addWidget(doubleSpinBox_gainBeta, 2, 0, 1, 1);

        label_binaryFile_7 = new QLabel(groupBox_gain);
        label_binaryFile_7->setObjectName(QStringLiteral("label_binaryFile_7"));
        label_binaryFile_7->setWordWrap(true);

        gridLayout_12->addWidget(label_binaryFile_7, 4, 1, 1, 1);

        doubleSpinBox_gainOverlap = new QDoubleSpinBox(groupBox_gain);
        doubleSpinBox_gainOverlap->setObjectName(QStringLiteral("doubleSpinBox_gainOverlap"));
        doubleSpinBox_gainOverlap->setDecimals(2);
        doubleSpinBox_gainOverlap->setMinimum(0);
        doubleSpinBox_gainOverlap->setMaximum(1);
        doubleSpinBox_gainOverlap->setSingleStep(0.01);
        doubleSpinBox_gainOverlap->setValue(0.05);

        gridLayout_12->addWidget(doubleSpinBox_gainOverlap, 1, 0, 1, 1);

        checkBox_gainFull = new QCheckBox(groupBox_gain);
        checkBox_gainFull->setObjectName(QStringLiteral("checkBox_gainFull"));

        gridLayout_12->addWidget(checkBox_gainFull, 4, 0, 1, 1);

        label_binaryFile_3 = new QLabel(groupBox_gain);
        label_binaryFile_3->setObjectName(QStringLiteral("label_binaryFile_3"));
        label_binaryFile_3->setWordWrap(true);

        gridLayout_12->addWidget(label_binaryFile_3, 0, 1, 1, 1);

        label_binaryFile_5 = new QLabel(groupBox_gain);
        label_binaryFile_5->setObjectName(QStringLiteral("label_binaryFile_5"));
        label_binaryFile_5->setWordWrap(true);

        gridLayout_12->addWidget(label_binaryFile_5, 2, 1, 1, 1);

        doubleSpinBox_gainRadius = new QDoubleSpinBox(groupBox_gain);
        doubleSpinBox_gainRadius->setObjectName(QStringLiteral("doubleSpinBox_gainRadius"));
        doubleSpinBox_gainRadius->setDecimals(3);
        doubleSpinBox_gainRadius->setMinimum(0.001);
        doubleSpinBox_gainRadius->setMaximum(10);
        doubleSpinBox_gainRadius->setSingleStep(0.01);
        doubleSpinBox_gainRadius->setValue(0.02);

        gridLayout_12->addWidget(doubleSpinBox_gainRadius, 0, 0, 1, 1);

        label_binaryFile_6 = new QLabel(groupBox_gain);
        label_binaryFile_6->setObjectName(QStringLiteral("label_binaryFile_6"));
        label_binaryFile_6->setWordWrap(true);

        gridLayout_12->addWidget(label_binaryFile_6, 1, 1, 1, 1);

        label_binaryFile_8 = new QLabel(groupBox_gain);
        label_binaryFile_8->setObjectName(QStringLiteral("label_binaryFile_8"));
        label_binaryFile_8->setWordWrap(true);

        gridLayout_12->addWidget(label_binaryFile_8, 3, 1, 1, 1);

        checkBox_gainRGB = new QCheckBox(groupBox_gain);
        checkBox_gainRGB->setObjectName(QStringLiteral("checkBox_gainRGB"));

        gridLayout_12->addWidget(checkBox_gainRGB, 3, 0, 1, 1);

        gridLayout_12->setColumnStretch(1, 1);

        verticalLayout_13->addWidget(groupBox_gain);

        groupBox_cameraProjection = new QGroupBox(scrollAreaWidgetContents);
        groupBox_cameraProjection->setObjectName(QStringLiteral("groupBox_cameraProjection"));
        groupBox_cameraProjection->setCheckable(false);
        gridLayout_20 = new QGridLayout(groupBox_cameraProjection);
        gridLayout_20->setObjectName(QStringLiteral("gridLayout_20"));
        doubleSpinBox_camProjMaxAngle = new QDoubleSpinBox(groupBox_cameraProjection);
        doubleSpinBox_camProjMaxAngle->setObjectName(QStringLiteral("doubleSpinBox_camProjMaxAngle"));
        doubleSpinBox_camProjMaxAngle->setDecimals(0);
        doubleSpinBox_camProjMaxAngle->setMinimum(0);
        doubleSpinBox_camProjMaxAngle->setMaximum(180);
        doubleSpinBox_camProjMaxAngle->setSingleStep(1);
        doubleSpinBox_camProjMaxAngle->setValue(0);

        gridLayout_20->addWidget(doubleSpinBox_camProjMaxAngle, 3, 1, 1, 1);

        label_meshingTextureSize_15 = new QLabel(groupBox_cameraProjection);
        label_meshingTextureSize_15->setObjectName(QStringLiteral("label_meshingTextureSize_15"));
        label_meshingTextureSize_15->setWordWrap(true);

        gridLayout_20->addWidget(label_meshingTextureSize_15, 5, 2, 1, 2);

        checkBox_camProjDistanceToCamPolicy = new QCheckBox(groupBox_cameraProjection);
        checkBox_camProjDistanceToCamPolicy->setObjectName(QStringLiteral("checkBox_camProjDistanceToCamPolicy"));

        gridLayout_20->addWidget(checkBox_camProjDistanceToCamPolicy, 4, 1, 1, 1);

        label_meshingTextureSize_14 = new QLabel(groupBox_cameraProjection);
        label_meshingTextureSize_14->setObjectName(QStringLiteral("label_meshingTextureSize_14"));
        label_meshingTextureSize_14->setWordWrap(true);

        gridLayout_20->addWidget(label_meshingTextureSize_14, 4, 2, 1, 2);

        label_meshingTextureSize_13 = new QLabel(groupBox_cameraProjection);
        label_meshingTextureSize_13->setObjectName(QStringLiteral("label_meshingTextureSize_13"));
        label_meshingTextureSize_13->setWordWrap(true);

        gridLayout_20->addWidget(label_meshingTextureSize_13, 2, 2, 1, 2);

        label_meshingTextureSize_12 = new QLabel(groupBox_cameraProjection);
        label_meshingTextureSize_12->setObjectName(QStringLiteral("label_meshingTextureSize_12"));
        label_meshingTextureSize_12->setWordWrap(true);

        gridLayout_20->addWidget(label_meshingTextureSize_12, 3, 2, 1, 2);

        doubleSpinBox_camProjMaxDistance = new QDoubleSpinBox(groupBox_cameraProjection);
        doubleSpinBox_camProjMaxDistance->setObjectName(QStringLiteral("doubleSpinBox_camProjMaxDistance"));
        doubleSpinBox_camProjMaxDistance->setDecimals(1);
        doubleSpinBox_camProjMaxDistance->setMinimum(0);
        doubleSpinBox_camProjMaxDistance->setMaximum(99);
        doubleSpinBox_camProjMaxDistance->setSingleStep(1);
        doubleSpinBox_camProjMaxDistance->setValue(0);

        gridLayout_20->addWidget(doubleSpinBox_camProjMaxDistance, 2, 1, 1, 1);

        checkBox_camProjKeepPointsNotSeenByCameras = new QCheckBox(groupBox_cameraProjection);
        checkBox_camProjKeepPointsNotSeenByCameras->setObjectName(QStringLiteral("checkBox_camProjKeepPointsNotSeenByCameras"));

        gridLayout_20->addWidget(checkBox_camProjKeepPointsNotSeenByCameras, 5, 1, 1, 1);

        label_camProjExportCamera = new QLabel(groupBox_cameraProjection);
        label_camProjExportCamera->setObjectName(QStringLiteral("label_camProjExportCamera"));
        label_camProjExportCamera->setWordWrap(true);

        gridLayout_20->addWidget(label_camProjExportCamera, 7, 2, 1, 2);

        comboBox_camProjExportCamera = new QComboBox(groupBox_cameraProjection);
        comboBox_camProjExportCamera->setObjectName(QStringLiteral("comboBox_camProjExportCamera"));

        gridLayout_20->addWidget(comboBox_camProjExportCamera, 7, 1, 1, 1);

        lineEdit_camProjRoiRatios = new QLineEdit(groupBox_cameraProjection);
        lineEdit_camProjRoiRatios->setObjectName(QStringLiteral("lineEdit_camProjRoiRatios"));

        gridLayout_20->addWidget(lineEdit_camProjRoiRatios, 1, 1, 1, 1);

        label_meshingTextureSize_17 = new QLabel(groupBox_cameraProjection);
        label_meshingTextureSize_17->setObjectName(QStringLiteral("label_meshingTextureSize_17"));
        label_meshingTextureSize_17->setWordWrap(true);

        gridLayout_20->addWidget(label_meshingTextureSize_17, 1, 2, 1, 2);

        label_meshingTextureSize_18 = new QLabel(groupBox_cameraProjection);
        label_meshingTextureSize_18->setObjectName(QStringLiteral("label_meshingTextureSize_18"));
        label_meshingTextureSize_18->setWordWrap(true);

        gridLayout_20->addWidget(label_meshingTextureSize_18, 6, 2, 1, 1);

        checkBox_camProjRecolorPoints = new QCheckBox(groupBox_cameraProjection);
        checkBox_camProjRecolorPoints->setObjectName(QStringLiteral("checkBox_camProjRecolorPoints"));

        gridLayout_20->addWidget(checkBox_camProjRecolorPoints, 6, 1, 1, 1);

        gridLayout_20->setColumnStretch(3, 1);

        verticalLayout_13->addWidget(groupBox_cameraProjection);

        groupBox_meshing = new QGroupBox(scrollAreaWidgetContents);
        groupBox_meshing->setObjectName(QStringLiteral("groupBox_meshing"));
        groupBox_meshing->setCheckable(false);
        verticalLayout_15 = new QVBoxLayout(groupBox_meshing);
        verticalLayout_15->setObjectName(QStringLiteral("verticalLayout_15"));
        gridLayout_10 = new QGridLayout();
        gridLayout_10->setObjectName(QStringLiteral("gridLayout_10"));
        label_meshClean = new QLabel(groupBox_meshing);
        label_meshClean->setObjectName(QStringLiteral("label_meshClean"));
        label_meshClean->setWordWrap(true);

        gridLayout_10->addWidget(label_meshClean, 5, 1, 1, 1);

        label_denseReconstruction = new QLabel(groupBox_meshing);
        label_denseReconstruction->setObjectName(QStringLiteral("label_denseReconstruction"));
        label_denseReconstruction->setWordWrap(true);

        gridLayout_10->addWidget(label_denseReconstruction, 0, 1, 1, 1);

        doubleSpinBox_meshDecimationFactor = new QDoubleSpinBox(groupBox_meshing);
        doubleSpinBox_meshDecimationFactor->setObjectName(QStringLiteral("doubleSpinBox_meshDecimationFactor"));
        doubleSpinBox_meshDecimationFactor->setDecimals(2);
        doubleSpinBox_meshDecimationFactor->setMinimum(0);
        doubleSpinBox_meshDecimationFactor->setMaximum(0.99);
        doubleSpinBox_meshDecimationFactor->setSingleStep(0.1);
        doubleSpinBox_meshDecimationFactor->setValue(0);

        gridLayout_10->addWidget(doubleSpinBox_meshDecimationFactor, 1, 0, 1, 1);

        label_meshDecimation_2 = new QLabel(groupBox_meshing);
        label_meshDecimation_2->setObjectName(QStringLiteral("label_meshDecimation_2"));
        label_meshDecimation_2->setWordWrap(true);

        gridLayout_10->addWidget(label_meshDecimation_2, 3, 1, 1, 1);

        label_meshDecimation = new QLabel(groupBox_meshing);
        label_meshDecimation->setObjectName(QStringLiteral("label_meshDecimation"));
        label_meshDecimation->setWordWrap(true);

        gridLayout_10->addWidget(label_meshDecimation, 1, 1, 1, 1);

        label_textureMapping = new QLabel(groupBox_meshing);
        label_textureMapping->setObjectName(QStringLiteral("label_textureMapping"));
        label_textureMapping->setWordWrap(true);

        gridLayout_10->addWidget(label_textureMapping, 4, 1, 1, 1);

        comboBox_meshingApproach = new QComboBox(groupBox_meshing);
        comboBox_meshingApproach->setObjectName(QStringLiteral("comboBox_meshingApproach"));

        gridLayout_10->addWidget(comboBox_meshingApproach, 0, 0, 1, 1);

        checkBox_textureMapping = new QCheckBox(groupBox_meshing);
        checkBox_textureMapping->setObjectName(QStringLiteral("checkBox_textureMapping"));

        gridLayout_10->addWidget(checkBox_textureMapping, 4, 0, 1, 1);

        doubleSpinBox_transferColorRadius = new QDoubleSpinBox(groupBox_meshing);
        doubleSpinBox_transferColorRadius->setObjectName(QStringLiteral("doubleSpinBox_transferColorRadius"));
        doubleSpinBox_transferColorRadius->setDecimals(2);
        doubleSpinBox_transferColorRadius->setMinimum(-1);
        doubleSpinBox_transferColorRadius->setMaximum(10);
        doubleSpinBox_transferColorRadius->setSingleStep(0.01);
        doubleSpinBox_transferColorRadius->setValue(0.05);

        gridLayout_10->addWidget(doubleSpinBox_transferColorRadius, 3, 0, 1, 1);

        spinBox_mesh_minClusterSize = new QSpinBox(groupBox_meshing);
        spinBox_mesh_minClusterSize->setObjectName(QStringLiteral("spinBox_mesh_minClusterSize"));
        spinBox_mesh_minClusterSize->setMinimum(-1);
        spinBox_mesh_minClusterSize->setMaximum(99999);

        gridLayout_10->addWidget(spinBox_mesh_minClusterSize, 6, 0, 1, 1);

        checkBox_cleanMesh = new QCheckBox(groupBox_meshing);
        checkBox_cleanMesh->setObjectName(QStringLiteral("checkBox_cleanMesh"));

        gridLayout_10->addWidget(checkBox_cleanMesh, 5, 0, 1, 1);

        label_16 = new QLabel(groupBox_meshing);
        label_16->setObjectName(QStringLiteral("label_16"));

        gridLayout_10->addWidget(label_16, 6, 1, 1, 1);

        label_meshMaxPolygons = new QLabel(groupBox_meshing);
        label_meshMaxPolygons->setObjectName(QStringLiteral("label_meshMaxPolygons"));
        label_meshMaxPolygons->setWordWrap(true);

        gridLayout_10->addWidget(label_meshMaxPolygons, 2, 1, 1, 1);

        spinBox_meshMaxPolygons = new QSpinBox(groupBox_meshing);
        spinBox_meshMaxPolygons->setObjectName(QStringLiteral("spinBox_meshMaxPolygons"));
        spinBox_meshMaxPolygons->setMinimum(0);
        spinBox_meshMaxPolygons->setMaximum(10000000);
        spinBox_meshMaxPolygons->setSingleStep(50000);
        spinBox_meshMaxPolygons->setValue(9);

        gridLayout_10->addWidget(spinBox_meshMaxPolygons, 2, 0, 1, 1);

        gridLayout_10->setColumnStretch(1, 1);

        verticalLayout_15->addLayout(gridLayout_10);

        groupBox_textureMapping = new QGroupBox(groupBox_meshing);
        groupBox_textureMapping->setObjectName(QStringLiteral("groupBox_textureMapping"));
        groupBox_textureMapping->setCheckable(false);
        groupBox_textureMapping->setChecked(false);
        verticalLayout_16 = new QVBoxLayout(groupBox_textureMapping);
        verticalLayout_16->setObjectName(QStringLiteral("verticalLayout_16"));
        gridLayout_15 = new QGridLayout();
        gridLayout_15->setObjectName(QStringLiteral("gridLayout_15"));
        spinBox_textureBrightnessContrastRatioHigh = new QSpinBox(groupBox_textureMapping);
        spinBox_textureBrightnessContrastRatioHigh->setObjectName(QStringLiteral("spinBox_textureBrightnessContrastRatioHigh"));
        spinBox_textureBrightnessContrastRatioHigh->setMinimum(0);
        spinBox_textureBrightnessContrastRatioHigh->setMaximum(49);

        gridLayout_15->addWidget(spinBox_textureBrightnessContrastRatioHigh, 13, 0, 1, 1);

        label_meshingTextureSize_6 = new QLabel(groupBox_textureMapping);
        label_meshingTextureSize_6->setObjectName(QStringLiteral("label_meshingTextureSize_6"));
        label_meshingTextureSize_6->setWordWrap(true);

        gridLayout_15->addWidget(label_meshingTextureSize_6, 13, 1, 1, 1);

        comboBox_blendingDecimation = new QComboBox(groupBox_textureMapping);
        comboBox_blendingDecimation->setObjectName(QStringLiteral("comboBox_blendingDecimation"));

        gridLayout_15->addWidget(comboBox_blendingDecimation, 11, 0, 1, 1);

        checkBox_blending = new QCheckBox(groupBox_textureMapping);
        checkBox_blending->setObjectName(QStringLiteral("checkBox_blending"));

        gridLayout_15->addWidget(checkBox_blending, 10, 0, 1, 1);

        label_exposureFusion_3 = new QLabel(groupBox_textureMapping);
        label_exposureFusion_3->setObjectName(QStringLiteral("label_exposureFusion_3"));
        label_exposureFusion_3->setWordWrap(true);

        gridLayout_15->addWidget(label_exposureFusion_3, 11, 1, 1, 1);

        label_meshingTextureSize_7 = new QLabel(groupBox_textureMapping);
        label_meshingTextureSize_7->setObjectName(QStringLiteral("label_meshingTextureSize_7"));
        label_meshingTextureSize_7->setWordWrap(true);

        gridLayout_15->addWidget(label_meshingTextureSize_7, 3, 1, 1, 1);

        label_meshingTextureSize_8 = new QLabel(groupBox_textureMapping);
        label_meshingTextureSize_8->setObjectName(QStringLiteral("label_meshingTextureSize_8"));
        label_meshingTextureSize_8->setWordWrap(true);

        gridLayout_15->addWidget(label_meshingTextureSize_8, 2, 1, 1, 1);

        label_exposureFusion_2 = new QLabel(groupBox_textureMapping);
        label_exposureFusion_2->setObjectName(QStringLiteral("label_exposureFusion_2"));
        label_exposureFusion_2->setWordWrap(true);

        gridLayout_15->addWidget(label_exposureFusion_2, 10, 1, 1, 1);

        label_meshingTextureSize_2 = new QLabel(groupBox_textureMapping);
        label_meshingTextureSize_2->setObjectName(QStringLiteral("label_meshingTextureSize_2"));
        label_meshingTextureSize_2->setWordWrap(true);

        gridLayout_15->addWidget(label_meshingTextureSize_2, 4, 1, 1, 1);

        doubleSpinBox_meshingTextureMaxDistance = new QDoubleSpinBox(groupBox_textureMapping);
        doubleSpinBox_meshingTextureMaxDistance->setObjectName(QStringLiteral("doubleSpinBox_meshingTextureMaxDistance"));
        doubleSpinBox_meshingTextureMaxDistance->setDecimals(1);
        doubleSpinBox_meshingTextureMaxDistance->setMinimum(0);
        doubleSpinBox_meshingTextureMaxDistance->setMaximum(99);
        doubleSpinBox_meshingTextureMaxDistance->setSingleStep(1);
        doubleSpinBox_meshingTextureMaxDistance->setValue(3);

        gridLayout_15->addWidget(doubleSpinBox_meshingTextureMaxDistance, 4, 0, 1, 1);

        label_meshingTextureSize_5 = new QLabel(groupBox_textureMapping);
        label_meshingTextureSize_5->setObjectName(QStringLiteral("label_meshingTextureSize_5"));
        label_meshingTextureSize_5->setWordWrap(true);

        gridLayout_15->addWidget(label_meshingTextureSize_5, 12, 1, 1, 1);

        label_meshingTextureFormat = new QLabel(groupBox_textureMapping);
        label_meshingTextureFormat->setObjectName(QStringLiteral("label_meshingTextureFormat"));
        label_meshingTextureFormat->setWordWrap(true);

        gridLayout_15->addWidget(label_meshingTextureFormat, 0, 1, 1, 1);

        comboBox_meshingTextureFormat = new QComboBox(groupBox_textureMapping);
        comboBox_meshingTextureFormat->setObjectName(QStringLiteral("comboBox_meshingTextureFormat"));

        gridLayout_15->addWidget(comboBox_meshingTextureFormat, 0, 0, 1, 1);

        comboBox_meshingTextureSize = new QComboBox(groupBox_textureMapping);
        comboBox_meshingTextureSize->setObjectName(QStringLiteral("comboBox_meshingTextureSize"));

        gridLayout_15->addWidget(comboBox_meshingTextureSize, 1, 0, 1, 1);

        checkBox_exposureFusion = new QCheckBox(groupBox_textureMapping);
        checkBox_exposureFusion->setObjectName(QStringLiteral("checkBox_exposureFusion"));

        gridLayout_15->addWidget(checkBox_exposureFusion, 15, 0, 1, 1);

        label_meshingTextureSize_4 = new QLabel(groupBox_textureMapping);
        label_meshingTextureSize_4->setObjectName(QStringLiteral("label_meshingTextureSize_4"));
        label_meshingTextureSize_4->setWordWrap(true);

        gridLayout_15->addWidget(label_meshingTextureSize_4, 7, 1, 1, 1);

        lineEdit_meshingTextureRoiRatios = new QLineEdit(groupBox_textureMapping);
        lineEdit_meshingTextureRoiRatios->setObjectName(QStringLiteral("lineEdit_meshingTextureRoiRatios"));

        gridLayout_15->addWidget(lineEdit_meshingTextureRoiRatios, 3, 0, 1, 1);

        label_meshingTextureSize = new QLabel(groupBox_textureMapping);
        label_meshingTextureSize->setObjectName(QStringLiteral("label_meshingTextureSize"));
        label_meshingTextureSize->setWordWrap(true);

        gridLayout_15->addWidget(label_meshingTextureSize, 1, 1, 1, 1);

        checkBox_cameraFilter = new QCheckBox(groupBox_textureMapping);
        checkBox_cameraFilter->setObjectName(QStringLiteral("checkBox_cameraFilter"));

        gridLayout_15->addWidget(checkBox_cameraFilter, 9, 0, 1, 1);

        spinBox_textureBrightnessContrastRatioLow = new QSpinBox(groupBox_textureMapping);
        spinBox_textureBrightnessContrastRatioLow->setObjectName(QStringLiteral("spinBox_textureBrightnessContrastRatioLow"));
        spinBox_textureBrightnessContrastRatioLow->setMinimum(0);
        spinBox_textureBrightnessContrastRatioLow->setMaximum(49);

        gridLayout_15->addWidget(spinBox_textureBrightnessContrastRatioLow, 12, 0, 1, 1);

        spinBox_mesh_minTextureClusterSize = new QSpinBox(groupBox_textureMapping);
        spinBox_mesh_minTextureClusterSize->setObjectName(QStringLiteral("spinBox_mesh_minTextureClusterSize"));
        spinBox_mesh_minTextureClusterSize->setMinimum(0);
        spinBox_mesh_minTextureClusterSize->setMaximum(99999);

        gridLayout_15->addWidget(spinBox_mesh_minTextureClusterSize, 7, 0, 1, 1);

        label_meshingTextureSize_3 = new QLabel(groupBox_textureMapping);
        label_meshingTextureSize_3->setObjectName(QStringLiteral("label_meshingTextureSize_3"));
        label_meshingTextureSize_3->setWordWrap(true);

        gridLayout_15->addWidget(label_meshingTextureSize_3, 9, 1, 1, 1);

        label_exposureFusion = new QLabel(groupBox_textureMapping);
        label_exposureFusion->setObjectName(QStringLiteral("label_exposureFusion"));
        label_exposureFusion->setWordWrap(true);

        gridLayout_15->addWidget(label_exposureFusion, 15, 1, 1, 1);

        spinBox_mesh_maxTextures = new QSpinBox(groupBox_textureMapping);
        spinBox_mesh_maxTextures->setObjectName(QStringLiteral("spinBox_mesh_maxTextures"));
        spinBox_mesh_maxTextures->setMinimum(1);
        spinBox_mesh_maxTextures->setMaximum(20);

        gridLayout_15->addWidget(spinBox_mesh_maxTextures, 2, 0, 1, 1);

        label_meshingTextureSize_9 = new QLabel(groupBox_textureMapping);
        label_meshingTextureSize_9->setObjectName(QStringLiteral("label_meshingTextureSize_9"));
        label_meshingTextureSize_9->setWordWrap(true);

        gridLayout_15->addWidget(label_meshingTextureSize_9, 5, 1, 1, 1);

        doubleSpinBox_meshingTextureMaxDepthError = new QDoubleSpinBox(groupBox_textureMapping);
        doubleSpinBox_meshingTextureMaxDepthError->setObjectName(QStringLiteral("doubleSpinBox_meshingTextureMaxDepthError"));
        doubleSpinBox_meshingTextureMaxDepthError->setDecimals(2);
        doubleSpinBox_meshingTextureMaxDepthError->setMinimum(-1);
        doubleSpinBox_meshingTextureMaxDepthError->setMaximum(99);
        doubleSpinBox_meshingTextureMaxDepthError->setSingleStep(0.05);
        doubleSpinBox_meshingTextureMaxDepthError->setValue(0);

        gridLayout_15->addWidget(doubleSpinBox_meshingTextureMaxDepthError, 5, 0, 1, 1);

        label_meshingTextureSize_10 = new QLabel(groupBox_textureMapping);
        label_meshingTextureSize_10->setObjectName(QStringLiteral("label_meshingTextureSize_10"));
        label_meshingTextureSize_10->setWordWrap(true);

        gridLayout_15->addWidget(label_meshingTextureSize_10, 6, 1, 1, 1);

        doubleSpinBox_meshingTextureMaxAngle = new QDoubleSpinBox(groupBox_textureMapping);
        doubleSpinBox_meshingTextureMaxAngle->setObjectName(QStringLiteral("doubleSpinBox_meshingTextureMaxAngle"));
        doubleSpinBox_meshingTextureMaxAngle->setDecimals(0);
        doubleSpinBox_meshingTextureMaxAngle->setMinimum(0);
        doubleSpinBox_meshingTextureMaxAngle->setMaximum(180);
        doubleSpinBox_meshingTextureMaxAngle->setSingleStep(1);
        doubleSpinBox_meshingTextureMaxAngle->setValue(0);

        gridLayout_15->addWidget(doubleSpinBox_meshingTextureMaxAngle, 6, 0, 1, 1);

        label_multiband = new QLabel(groupBox_textureMapping);
        label_multiband->setObjectName(QStringLiteral("label_multiband"));
        label_multiband->setWordWrap(true);

        gridLayout_15->addWidget(label_multiband, 16, 1, 1, 1);

        checkBox_multiband = new QCheckBox(groupBox_textureMapping);
        checkBox_multiband->setObjectName(QStringLiteral("checkBox_multiband"));

        gridLayout_15->addWidget(checkBox_multiband, 16, 0, 1, 1);

        label_meshingTextureSize_11 = new QLabel(groupBox_textureMapping);
        label_meshingTextureSize_11->setObjectName(QStringLiteral("label_meshingTextureSize_11"));
        label_meshingTextureSize_11->setWordWrap(true);

        gridLayout_15->addWidget(label_meshingTextureSize_11, 8, 1, 1, 1);

        checkBox_distanceToCamPolicy = new QCheckBox(groupBox_textureMapping);
        checkBox_distanceToCamPolicy->setObjectName(QStringLiteral("checkBox_distanceToCamPolicy"));

        gridLayout_15->addWidget(checkBox_distanceToCamPolicy, 8, 0, 1, 1);

        gridLayout_15->setColumnStretch(1, 1);

        verticalLayout_16->addLayout(gridLayout_15);

        groupBox_cameraFilter = new QGroupBox(groupBox_textureMapping);
        groupBox_cameraFilter->setObjectName(QStringLiteral("groupBox_cameraFilter"));
        gridLayout_16 = new QGridLayout(groupBox_cameraFilter);
        gridLayout_16->setObjectName(QStringLiteral("gridLayout_16"));
        doubleSpinBox_cameraFilterRadius = new QDoubleSpinBox(groupBox_cameraFilter);
        doubleSpinBox_cameraFilterRadius->setObjectName(QStringLiteral("doubleSpinBox_cameraFilterRadius"));
        doubleSpinBox_cameraFilterRadius->setDecimals(3);
        doubleSpinBox_cameraFilterRadius->setMinimum(0);
        doubleSpinBox_cameraFilterRadius->setValue(0.1);

        gridLayout_16->addWidget(doubleSpinBox_cameraFilterRadius, 0, 0, 1, 1);

        label_37 = new QLabel(groupBox_cameraFilter);
        label_37->setObjectName(QStringLiteral("label_37"));
        label_37->setWordWrap(true);
        label_37->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_16->addWidget(label_37, 0, 2, 1, 1);

        doubleSpinBox_laplacianVariance = new QDoubleSpinBox(groupBox_cameraFilter);
        doubleSpinBox_laplacianVariance->setObjectName(QStringLiteral("doubleSpinBox_laplacianVariance"));
        doubleSpinBox_laplacianVariance->setDecimals(0);
        doubleSpinBox_laplacianVariance->setMinimum(0);
        doubleSpinBox_laplacianVariance->setMaximum(9999);
        doubleSpinBox_laplacianVariance->setValue(0);

        gridLayout_16->addWidget(doubleSpinBox_laplacianVariance, 1, 1, 1, 1);

        doubleSpinBox_cameraFilterAngle = new QDoubleSpinBox(groupBox_cameraFilter);
        doubleSpinBox_cameraFilterAngle->setObjectName(QStringLiteral("doubleSpinBox_cameraFilterAngle"));
        doubleSpinBox_cameraFilterAngle->setDecimals(1);
        doubleSpinBox_cameraFilterAngle->setMaximum(180);
        doubleSpinBox_cameraFilterAngle->setValue(30);

        gridLayout_16->addWidget(doubleSpinBox_cameraFilterAngle, 0, 1, 1, 1);

        label_38 = new QLabel(groupBox_cameraFilter);
        label_38->setObjectName(QStringLiteral("label_38"));
        label_38->setWordWrap(true);
        label_38->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_16->addWidget(label_38, 1, 2, 1, 1);

        doubleSpinBox_cameraFilterVelRad = new QDoubleSpinBox(groupBox_cameraFilter);
        doubleSpinBox_cameraFilterVelRad->setObjectName(QStringLiteral("doubleSpinBox_cameraFilterVelRad"));
        doubleSpinBox_cameraFilterVelRad->setDecimals(3);
        doubleSpinBox_cameraFilterVelRad->setMinimum(0);
        doubleSpinBox_cameraFilterVelRad->setValue(0.1);

        gridLayout_16->addWidget(doubleSpinBox_cameraFilterVelRad, 2, 1, 1, 1);

        doubleSpinBox_cameraFilterVel = new QDoubleSpinBox(groupBox_cameraFilter);
        doubleSpinBox_cameraFilterVel->setObjectName(QStringLiteral("doubleSpinBox_cameraFilterVel"));
        doubleSpinBox_cameraFilterVel->setDecimals(3);
        doubleSpinBox_cameraFilterVel->setMinimum(0);
        doubleSpinBox_cameraFilterVel->setValue(0.1);

        gridLayout_16->addWidget(doubleSpinBox_cameraFilterVel, 2, 0, 1, 1);

        label_39 = new QLabel(groupBox_cameraFilter);
        label_39->setObjectName(QStringLiteral("label_39"));
        label_39->setWordWrap(true);
        label_39->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_16->addWidget(label_39, 2, 2, 1, 1);

        gridLayout_16->setColumnStretch(2, 1);

        verticalLayout_16->addWidget(groupBox_cameraFilter);


        verticalLayout_15->addWidget(groupBox_textureMapping);

        groupBox_gp3 = new QGroupBox(groupBox_meshing);
        groupBox_gp3->setObjectName(QStringLiteral("groupBox_gp3"));
        groupBox_gp3->setCheckable(false);
        groupBox_gp3->setChecked(false);
        gridLayout_4 = new QGridLayout(groupBox_gp3);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        label_169 = new QLabel(groupBox_gp3);
        label_169->setObjectName(QStringLiteral("label_169"));
        label_169->setWordWrap(true);

        gridLayout_4->addWidget(label_169, 2, 1, 1, 1);

        doubleSpinBox_gp3Radius = new QDoubleSpinBox(groupBox_gp3);
        doubleSpinBox_gp3Radius->setObjectName(QStringLiteral("doubleSpinBox_gp3Radius"));
        doubleSpinBox_gp3Radius->setDecimals(3);
        doubleSpinBox_gp3Radius->setMaximum(1);
        doubleSpinBox_gp3Radius->setSingleStep(0.01);
        doubleSpinBox_gp3Radius->setValue(0.04);

        gridLayout_4->addWidget(doubleSpinBox_gp3Radius, 1, 0, 1, 1);

        label_168 = new QLabel(groupBox_gp3);
        label_168->setObjectName(QStringLiteral("label_168"));
        label_168->setWordWrap(true);

        gridLayout_4->addWidget(label_168, 1, 1, 1, 1);

        doubleSpinBox_gp3Mu = new QDoubleSpinBox(groupBox_gp3);
        doubleSpinBox_gp3Mu->setObjectName(QStringLiteral("doubleSpinBox_gp3Mu"));
        doubleSpinBox_gp3Mu->setDecimals(1);
        doubleSpinBox_gp3Mu->setMinimum(0.1);
        doubleSpinBox_gp3Mu->setMaximum(100);
        doubleSpinBox_gp3Mu->setSingleStep(1);
        doubleSpinBox_gp3Mu->setValue(2.5);

        gridLayout_4->addWidget(doubleSpinBox_gp3Mu, 2, 0, 1, 1);

        gridLayout_4->setColumnStretch(1, 1);

        verticalLayout_15->addWidget(groupBox_gp3);

        groupBox_poisson = new QGroupBox(groupBox_meshing);
        groupBox_poisson->setObjectName(QStringLiteral("groupBox_poisson"));
        groupBox_poisson->setCheckable(false);
        groupBox_poisson->setChecked(false);
        gridLayout_14 = new QGridLayout(groupBox_poisson);
        gridLayout_14->setObjectName(QStringLiteral("gridLayout_14"));
        label_172 = new QLabel(groupBox_poisson);
        label_172->setObjectName(QStringLiteral("label_172"));
        label_172->setWordWrap(true);

        gridLayout_14->addWidget(label_172, 0, 1, 1, 1);

        label_177 = new QLabel(groupBox_poisson);
        label_177->setObjectName(QStringLiteral("label_177"));
        label_177->setWordWrap(true);

        gridLayout_14->addWidget(label_177, 8, 1, 1, 1);

        spinBox_poisson_iso = new QSpinBox(groupBox_poisson);
        spinBox_poisson_iso->setObjectName(QStringLiteral("spinBox_poisson_iso"));
        spinBox_poisson_iso->setMinimum(1);

        gridLayout_14->addWidget(spinBox_poisson_iso, 9, 0, 1, 1);

        label_176 = new QLabel(groupBox_poisson);
        label_176->setObjectName(QStringLiteral("label_176"));
        label_176->setWordWrap(true);

        gridLayout_14->addWidget(label_176, 3, 1, 1, 1);

        doubleSpinBox_poisson_samples = new QDoubleSpinBox(groupBox_poisson);
        doubleSpinBox_poisson_samples->setObjectName(QStringLiteral("doubleSpinBox_poisson_samples"));
        doubleSpinBox_poisson_samples->setDecimals(1);
        doubleSpinBox_poisson_samples->setMinimum(0.1);
        doubleSpinBox_poisson_samples->setMaximum(30);

        gridLayout_14->addWidget(doubleSpinBox_poisson_samples, 8, 0, 1, 1);

        doubleSpinBox_poisson_scale = new QDoubleSpinBox(groupBox_poisson);
        doubleSpinBox_poisson_scale->setObjectName(QStringLiteral("doubleSpinBox_poisson_scale"));
        doubleSpinBox_poisson_scale->setDecimals(1);
        doubleSpinBox_poisson_scale->setMinimum(1);
        doubleSpinBox_poisson_scale->setMaximum(2);

        gridLayout_14->addWidget(doubleSpinBox_poisson_scale, 6, 0, 1, 1);

        label_181 = new QLabel(groupBox_poisson);
        label_181->setObjectName(QStringLiteral("label_181"));
        label_181->setWordWrap(true);

        gridLayout_14->addWidget(label_181, 4, 1, 1, 1);

        label_182 = new QLabel(groupBox_poisson);
        label_182->setObjectName(QStringLiteral("label_182"));
        label_182->setWordWrap(true);

        gridLayout_14->addWidget(label_182, 5, 1, 1, 1);

        label_178 = new QLabel(groupBox_poisson);
        label_178->setObjectName(QStringLiteral("label_178"));
        label_178->setWordWrap(true);

        gridLayout_14->addWidget(label_178, 9, 1, 1, 1);

        checkBox_poisson_manifold = new QCheckBox(groupBox_poisson);
        checkBox_poisson_manifold->setObjectName(QStringLiteral("checkBox_poisson_manifold"));

        gridLayout_14->addWidget(checkBox_poisson_manifold, 3, 0, 1, 1);

        label_179 = new QLabel(groupBox_poisson);
        label_179->setObjectName(QStringLiteral("label_179"));
        label_179->setWordWrap(true);

        gridLayout_14->addWidget(label_179, 10, 1, 1, 1);

        label_180 = new QLabel(groupBox_poisson);
        label_180->setObjectName(QStringLiteral("label_180"));
        label_180->setWordWrap(true);

        gridLayout_14->addWidget(label_180, 6, 1, 1, 1);

        checkBox_poisson_outputPolygons = new QCheckBox(groupBox_poisson);
        checkBox_poisson_outputPolygons->setObjectName(QStringLiteral("checkBox_poisson_outputPolygons"));

        gridLayout_14->addWidget(checkBox_poisson_outputPolygons, 2, 0, 1, 1);

        spinBox_poisson_solver = new QSpinBox(groupBox_poisson);
        spinBox_poisson_solver->setObjectName(QStringLiteral("spinBox_poisson_solver"));
        spinBox_poisson_solver->setMinimum(1);

        gridLayout_14->addWidget(spinBox_poisson_solver, 10, 0, 1, 1);

        label_outputPolygons = new QLabel(groupBox_poisson);
        label_outputPolygons->setObjectName(QStringLiteral("label_outputPolygons"));
        label_outputPolygons->setWordWrap(true);

        gridLayout_14->addWidget(label_outputPolygons, 2, 1, 1, 1);

        doubleSpinBox_poisson_pointWeight = new QDoubleSpinBox(groupBox_poisson);
        doubleSpinBox_poisson_pointWeight->setObjectName(QStringLiteral("doubleSpinBox_poisson_pointWeight"));
        doubleSpinBox_poisson_pointWeight->setDecimals(1);
        doubleSpinBox_poisson_pointWeight->setMinimum(0.1);

        gridLayout_14->addWidget(doubleSpinBox_poisson_pointWeight, 4, 0, 1, 1);

        spinBox_poisson_minDepth = new QSpinBox(groupBox_poisson);
        spinBox_poisson_minDepth->setObjectName(QStringLiteral("spinBox_poisson_minDepth"));
        spinBox_poisson_minDepth->setMinimum(1);

        gridLayout_14->addWidget(spinBox_poisson_minDepth, 5, 0, 1, 1);

        spinBox_poisson_depth = new QSpinBox(groupBox_poisson);
        spinBox_poisson_depth->setObjectName(QStringLiteral("spinBox_poisson_depth"));
        spinBox_poisson_depth->setMinimum(0);

        gridLayout_14->addWidget(spinBox_poisson_depth, 0, 0, 1, 1);

        label_outputPolygons_2 = new QLabel(groupBox_poisson);
        label_outputPolygons_2->setObjectName(QStringLiteral("label_outputPolygons_2"));
        label_outputPolygons_2->setWordWrap(true);

        gridLayout_14->addWidget(label_outputPolygons_2, 1, 1, 1, 1);

        doubleSpinBox_poisson_targetPolygonSize = new QDoubleSpinBox(groupBox_poisson);
        doubleSpinBox_poisson_targetPolygonSize->setObjectName(QStringLiteral("doubleSpinBox_poisson_targetPolygonSize"));
        doubleSpinBox_poisson_targetPolygonSize->setDecimals(3);
        doubleSpinBox_poisson_targetPolygonSize->setMinimum(0.01);
        doubleSpinBox_poisson_targetPolygonSize->setSingleStep(0.1);
        doubleSpinBox_poisson_targetPolygonSize->setValue(0.03);

        gridLayout_14->addWidget(doubleSpinBox_poisson_targetPolygonSize, 1, 0, 1, 1);

        gridLayout_14->setColumnStretch(1, 1);

        verticalLayout_15->addWidget(groupBox_poisson);

        groupBox_cputsdf = new QGroupBox(groupBox_meshing);
        groupBox_cputsdf->setObjectName(QStringLiteral("groupBox_cputsdf"));
        verticalLayout_17 = new QVBoxLayout(groupBox_cputsdf);
        verticalLayout_17->setObjectName(QStringLiteral("verticalLayout_17"));
        label_25 = new QLabel(groupBox_cputsdf);
        label_25->setObjectName(QStringLiteral("label_25"));
        label_25->setWordWrap(true);
        label_25->setOpenExternalLinks(true);

        verticalLayout_17->addWidget(label_25);

        gridLayout_17 = new QGridLayout();
        gridLayout_17->setObjectName(QStringLiteral("gridLayout_17"));
        doubleSpinBox_cputsdf_size = new QDoubleSpinBox(groupBox_cputsdf);
        doubleSpinBox_cputsdf_size->setObjectName(QStringLiteral("doubleSpinBox_cputsdf_size"));
        doubleSpinBox_cputsdf_size->setDecimals(1);
        doubleSpinBox_cputsdf_size->setMinimum(0.1);
        doubleSpinBox_cputsdf_size->setMaximum(9999);
        doubleSpinBox_cputsdf_size->setSingleStep(1);
        doubleSpinBox_cputsdf_size->setValue(12);

        gridLayout_17->addWidget(doubleSpinBox_cputsdf_size, 0, 0, 1, 1);

        label_19 = new QLabel(groupBox_cputsdf);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setWordWrap(true);

        gridLayout_17->addWidget(label_19, 0, 1, 1, 1);

        doubleSpinBox_cputsdf_resolution = new QDoubleSpinBox(groupBox_cputsdf);
        doubleSpinBox_cputsdf_resolution->setObjectName(QStringLiteral("doubleSpinBox_cputsdf_resolution"));
        doubleSpinBox_cputsdf_resolution->setDecimals(3);
        doubleSpinBox_cputsdf_resolution->setMinimum(0.001);
        doubleSpinBox_cputsdf_resolution->setMaximum(99);
        doubleSpinBox_cputsdf_resolution->setSingleStep(0.01);
        doubleSpinBox_cputsdf_resolution->setValue(0.01);

        gridLayout_17->addWidget(doubleSpinBox_cputsdf_resolution, 1, 0, 1, 1);

        label_20 = new QLabel(groupBox_cputsdf);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setWordWrap(true);

        gridLayout_17->addWidget(label_20, 1, 1, 1, 1);

        doubleSpinBox_cputsdf_tuncPos = new QDoubleSpinBox(groupBox_cputsdf);
        doubleSpinBox_cputsdf_tuncPos->setObjectName(QStringLiteral("doubleSpinBox_cputsdf_tuncPos"));
        doubleSpinBox_cputsdf_tuncPos->setDecimals(2);
        doubleSpinBox_cputsdf_tuncPos->setMinimum(0);
        doubleSpinBox_cputsdf_tuncPos->setMaximum(1);
        doubleSpinBox_cputsdf_tuncPos->setSingleStep(0.01);
        doubleSpinBox_cputsdf_tuncPos->setValue(0.03);

        gridLayout_17->addWidget(doubleSpinBox_cputsdf_tuncPos, 2, 0, 1, 1);

        label_21 = new QLabel(groupBox_cputsdf);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setWordWrap(true);

        gridLayout_17->addWidget(label_21, 2, 1, 1, 1);

        doubleSpinBox_cputsdf_tuncNeg = new QDoubleSpinBox(groupBox_cputsdf);
        doubleSpinBox_cputsdf_tuncNeg->setObjectName(QStringLiteral("doubleSpinBox_cputsdf_tuncNeg"));
        doubleSpinBox_cputsdf_tuncNeg->setDecimals(2);
        doubleSpinBox_cputsdf_tuncNeg->setMinimum(0);
        doubleSpinBox_cputsdf_tuncNeg->setMaximum(1);
        doubleSpinBox_cputsdf_tuncNeg->setSingleStep(0.01);
        doubleSpinBox_cputsdf_tuncNeg->setValue(0.03);

        gridLayout_17->addWidget(doubleSpinBox_cputsdf_tuncNeg, 3, 0, 1, 1);

        label_22 = new QLabel(groupBox_cputsdf);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setWordWrap(true);

        gridLayout_17->addWidget(label_22, 3, 1, 1, 1);

        spinBox_cputsdf_randomSplit = new QSpinBox(groupBox_cputsdf);
        spinBox_cputsdf_randomSplit->setObjectName(QStringLiteral("spinBox_cputsdf_randomSplit"));
        spinBox_cputsdf_randomSplit->setMinimum(1);
        spinBox_cputsdf_randomSplit->setMaximum(99);

        gridLayout_17->addWidget(spinBox_cputsdf_randomSplit, 4, 0, 1, 1);

        label_23 = new QLabel(groupBox_cputsdf);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setWordWrap(true);

        gridLayout_17->addWidget(label_23, 4, 1, 1, 1);

        doubleSpinBox_cputsdf_minWeight = new QDoubleSpinBox(groupBox_cputsdf);
        doubleSpinBox_cputsdf_minWeight->setObjectName(QStringLiteral("doubleSpinBox_cputsdf_minWeight"));
        doubleSpinBox_cputsdf_minWeight->setDecimals(1);
        doubleSpinBox_cputsdf_minWeight->setMinimum(0);
        doubleSpinBox_cputsdf_minWeight->setMaximum(99);
        doubleSpinBox_cputsdf_minWeight->setSingleStep(0.5);
        doubleSpinBox_cputsdf_minWeight->setValue(0);

        gridLayout_17->addWidget(doubleSpinBox_cputsdf_minWeight, 5, 0, 1, 1);

        label_18 = new QLabel(groupBox_cputsdf);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setWordWrap(true);

        gridLayout_17->addWidget(label_18, 5, 1, 1, 1);

        doubleSpinBox_cputsdf_flattenRadius = new QDoubleSpinBox(groupBox_cputsdf);
        doubleSpinBox_cputsdf_flattenRadius->setObjectName(QStringLiteral("doubleSpinBox_cputsdf_flattenRadius"));
        doubleSpinBox_cputsdf_flattenRadius->setDecimals(4);
        doubleSpinBox_cputsdf_flattenRadius->setMinimum(0);
        doubleSpinBox_cputsdf_flattenRadius->setMaximum(9);
        doubleSpinBox_cputsdf_flattenRadius->setSingleStep(0.005);
        doubleSpinBox_cputsdf_flattenRadius->setValue(0.005);

        gridLayout_17->addWidget(doubleSpinBox_cputsdf_flattenRadius, 6, 0, 1, 1);

        label_24 = new QLabel(groupBox_cputsdf);
        label_24->setObjectName(QStringLiteral("label_24"));
        label_24->setWordWrap(true);

        gridLayout_17->addWidget(label_24, 6, 1, 1, 1);

        gridLayout_17->setColumnStretch(1, 1);

        verticalLayout_17->addLayout(gridLayout_17);


        verticalLayout_15->addWidget(groupBox_cputsdf);

        groupBox_organized = new QGroupBox(groupBox_meshing);
        groupBox_organized->setObjectName(QStringLiteral("groupBox_organized"));
        gridLayout_2 = new QGridLayout(groupBox_organized);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        label_15 = new QLabel(groupBox_organized);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setWordWrap(true);

        gridLayout_2->addWidget(label_15, 2, 1, 1, 1);

        label_quad = new QLabel(groupBox_organized);
        label_quad->setObjectName(QStringLiteral("label_quad"));
        label_quad->setWordWrap(true);

        gridLayout_2->addWidget(label_quad, 1, 1, 1, 1);

        checkBox_mesh_quad = new QCheckBox(groupBox_organized);
        checkBox_mesh_quad->setObjectName(QStringLiteral("checkBox_mesh_quad"));
        checkBox_mesh_quad->setChecked(true);

        gridLayout_2->addWidget(checkBox_mesh_quad, 1, 0, 1, 1);

        doubleSpinBox_mesh_angleTolerance = new QDoubleSpinBox(groupBox_organized);
        doubleSpinBox_mesh_angleTolerance->setObjectName(QStringLiteral("doubleSpinBox_mesh_angleTolerance"));
        doubleSpinBox_mesh_angleTolerance->setDecimals(0);
        doubleSpinBox_mesh_angleTolerance->setMinimum(1);
        doubleSpinBox_mesh_angleTolerance->setMaximum(99);
        doubleSpinBox_mesh_angleTolerance->setValue(15);

        gridLayout_2->addWidget(doubleSpinBox_mesh_angleTolerance, 0, 0, 1, 1);

        label_14 = new QLabel(groupBox_organized);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setWordWrap(true);

        gridLayout_2->addWidget(label_14, 0, 1, 1, 1);

        spinBox_mesh_triangleSize = new QSpinBox(groupBox_organized);
        spinBox_mesh_triangleSize->setObjectName(QStringLiteral("spinBox_mesh_triangleSize"));
        spinBox_mesh_triangleSize->setMinimum(1);
        spinBox_mesh_triangleSize->setMaximum(99);

        gridLayout_2->addWidget(spinBox_mesh_triangleSize, 2, 0, 1, 1);

        gridLayout_2->setColumnStretch(1, 1);

        verticalLayout_15->addWidget(groupBox_organized);

        groupBox_openchisel = new QGroupBox(groupBox_meshing);
        groupBox_openchisel->setObjectName(QStringLiteral("groupBox_openchisel"));
        verticalLayout_18 = new QVBoxLayout(groupBox_openchisel);
        verticalLayout_18->setObjectName(QStringLiteral("verticalLayout_18"));
        label_26 = new QLabel(groupBox_openchisel);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setWordWrap(true);
        label_26->setOpenExternalLinks(true);

        verticalLayout_18->addWidget(label_26);

        gridLayout_19 = new QGridLayout();
        gridLayout_19->setObjectName(QStringLiteral("gridLayout_19"));
        label_36 = new QLabel(groupBox_openchisel);
        label_36->setObjectName(QStringLiteral("label_36"));
        label_36->setWordWrap(true);

        gridLayout_19->addWidget(label_36, 4, 1, 1, 1);

        label_35 = new QLabel(groupBox_openchisel);
        label_35->setObjectName(QStringLiteral("label_35"));
        label_35->setWordWrap(true);

        gridLayout_19->addWidget(label_35, 3, 1, 1, 1);

        label_40 = new QLabel(groupBox_openchisel);
        label_40->setObjectName(QStringLiteral("label_40"));
        label_40->setWordWrap(true);

        gridLayout_19->addWidget(label_40, 5, 1, 1, 1);

        label_43 = new QLabel(groupBox_openchisel);
        label_43->setObjectName(QStringLiteral("label_43"));
        label_43->setWordWrap(true);

        gridLayout_19->addWidget(label_43, 8, 1, 1, 1);

        label_46 = new QLabel(groupBox_openchisel);
        label_46->setObjectName(QStringLiteral("label_46"));
        label_46->setWordWrap(true);

        gridLayout_19->addWidget(label_46, 11, 1, 1, 1);

        label_34 = new QLabel(groupBox_openchisel);
        label_34->setObjectName(QStringLiteral("label_34"));
        label_34->setWordWrap(true);

        gridLayout_19->addWidget(label_34, 2, 1, 1, 1);

        spinBox_openchisel_chunk_size_y = new QSpinBox(groupBox_openchisel);
        spinBox_openchisel_chunk_size_y->setObjectName(QStringLiteral("spinBox_openchisel_chunk_size_y"));
        spinBox_openchisel_chunk_size_y->setMinimum(1);
        spinBox_openchisel_chunk_size_y->setMaximum(99);
        spinBox_openchisel_chunk_size_y->setValue(16);

        gridLayout_19->addWidget(spinBox_openchisel_chunk_size_y, 2, 0, 1, 1);

        doubleSpinBox_openchisel_truncation_constant = new QDoubleSpinBox(groupBox_openchisel);
        doubleSpinBox_openchisel_truncation_constant->setObjectName(QStringLiteral("doubleSpinBox_openchisel_truncation_constant"));
        doubleSpinBox_openchisel_truncation_constant->setDecimals(6);
        doubleSpinBox_openchisel_truncation_constant->setMinimum(1e-6);
        doubleSpinBox_openchisel_truncation_constant->setMaximum(1);
        doubleSpinBox_openchisel_truncation_constant->setSingleStep(0.001);
        doubleSpinBox_openchisel_truncation_constant->setValue(0.001504);

        gridLayout_19->addWidget(doubleSpinBox_openchisel_truncation_constant, 4, 0, 1, 1);

        spinBox_openchisel_chunk_size_z = new QSpinBox(groupBox_openchisel);
        spinBox_openchisel_chunk_size_z->setObjectName(QStringLiteral("spinBox_openchisel_chunk_size_z"));
        spinBox_openchisel_chunk_size_z->setMinimum(1);
        spinBox_openchisel_chunk_size_z->setMaximum(99);
        spinBox_openchisel_chunk_size_z->setValue(16);

        gridLayout_19->addWidget(spinBox_openchisel_chunk_size_z, 3, 0, 1, 1);

        spinBox_openchisel_chunk_size_x = new QSpinBox(groupBox_openchisel);
        spinBox_openchisel_chunk_size_x->setObjectName(QStringLiteral("spinBox_openchisel_chunk_size_x"));
        spinBox_openchisel_chunk_size_x->setMinimum(1);
        spinBox_openchisel_chunk_size_x->setMaximum(99);
        spinBox_openchisel_chunk_size_x->setValue(16);

        gridLayout_19->addWidget(spinBox_openchisel_chunk_size_x, 1, 0, 1, 1);

        label_31 = new QLabel(groupBox_openchisel);
        label_31->setObjectName(QStringLiteral("label_31"));
        label_31->setWordWrap(true);

        gridLayout_19->addWidget(label_31, 1, 1, 1, 1);

        label_42 = new QLabel(groupBox_openchisel);
        label_42->setObjectName(QStringLiteral("label_42"));
        label_42->setWordWrap(true);

        gridLayout_19->addWidget(label_42, 7, 1, 1, 1);

        label_41 = new QLabel(groupBox_openchisel);
        label_41->setObjectName(QStringLiteral("label_41"));
        label_41->setWordWrap(true);

        gridLayout_19->addWidget(label_41, 6, 1, 1, 1);

        label_44 = new QLabel(groupBox_openchisel);
        label_44->setObjectName(QStringLiteral("label_44"));
        label_44->setWordWrap(true);

        gridLayout_19->addWidget(label_44, 9, 1, 1, 1);

        label_45 = new QLabel(groupBox_openchisel);
        label_45->setObjectName(QStringLiteral("label_45"));
        label_45->setWordWrap(true);

        gridLayout_19->addWidget(label_45, 10, 1, 1, 1);

        label_47 = new QLabel(groupBox_openchisel);
        label_47->setObjectName(QStringLiteral("label_47"));
        label_47->setWordWrap(true);

        gridLayout_19->addWidget(label_47, 12, 1, 1, 1);

        doubleSpinBox_openchisel_truncation_linear = new QDoubleSpinBox(groupBox_openchisel);
        doubleSpinBox_openchisel_truncation_linear->setObjectName(QStringLiteral("doubleSpinBox_openchisel_truncation_linear"));
        doubleSpinBox_openchisel_truncation_linear->setDecimals(6);
        doubleSpinBox_openchisel_truncation_linear->setMinimum(1e-6);
        doubleSpinBox_openchisel_truncation_linear->setMaximum(1);
        doubleSpinBox_openchisel_truncation_linear->setSingleStep(0.001);
        doubleSpinBox_openchisel_truncation_linear->setValue(0.001504);

        gridLayout_19->addWidget(doubleSpinBox_openchisel_truncation_linear, 5, 0, 1, 1);

        doubleSpinBox_openchisel_truncation_quadratic = new QDoubleSpinBox(groupBox_openchisel);
        doubleSpinBox_openchisel_truncation_quadratic->setObjectName(QStringLiteral("doubleSpinBox_openchisel_truncation_quadratic"));
        doubleSpinBox_openchisel_truncation_quadratic->setDecimals(6);
        doubleSpinBox_openchisel_truncation_quadratic->setMinimum(1e-6);
        doubleSpinBox_openchisel_truncation_quadratic->setMaximum(1);
        doubleSpinBox_openchisel_truncation_quadratic->setSingleStep(0.001);
        doubleSpinBox_openchisel_truncation_quadratic->setValue(0.001504);

        gridLayout_19->addWidget(doubleSpinBox_openchisel_truncation_quadratic, 6, 0, 1, 1);

        doubleSpinBox_openchisel_truncation_scale = new QDoubleSpinBox(groupBox_openchisel);
        doubleSpinBox_openchisel_truncation_scale->setObjectName(QStringLiteral("doubleSpinBox_openchisel_truncation_scale"));
        doubleSpinBox_openchisel_truncation_scale->setDecimals(1);
        doubleSpinBox_openchisel_truncation_scale->setMinimum(0.1);
        doubleSpinBox_openchisel_truncation_scale->setMaximum(100);
        doubleSpinBox_openchisel_truncation_scale->setSingleStep(0.001);
        doubleSpinBox_openchisel_truncation_scale->setValue(0.1);

        gridLayout_19->addWidget(doubleSpinBox_openchisel_truncation_scale, 7, 0, 1, 1);

        spinBox_openchisel_integration_weight = new QSpinBox(groupBox_openchisel);
        spinBox_openchisel_integration_weight->setObjectName(QStringLiteral("spinBox_openchisel_integration_weight"));
        spinBox_openchisel_integration_weight->setMinimum(1);
        spinBox_openchisel_integration_weight->setMaximum(99);
        spinBox_openchisel_integration_weight->setValue(16);

        gridLayout_19->addWidget(spinBox_openchisel_integration_weight, 8, 0, 1, 1);

        checkBox_openchisel_use_voxel_carving = new QCheckBox(groupBox_openchisel);
        checkBox_openchisel_use_voxel_carving->setObjectName(QStringLiteral("checkBox_openchisel_use_voxel_carving"));
        checkBox_openchisel_use_voxel_carving->setChecked(true);

        gridLayout_19->addWidget(checkBox_openchisel_use_voxel_carving, 9, 0, 1, 1);

        doubleSpinBox_openchisel_carving_dist_m = new QDoubleSpinBox(groupBox_openchisel);
        doubleSpinBox_openchisel_carving_dist_m->setObjectName(QStringLiteral("doubleSpinBox_openchisel_carving_dist_m"));
        doubleSpinBox_openchisel_carving_dist_m->setDecimals(2);
        doubleSpinBox_openchisel_carving_dist_m->setMinimum(0.01);
        doubleSpinBox_openchisel_carving_dist_m->setMaximum(1);
        doubleSpinBox_openchisel_carving_dist_m->setSingleStep(0.001);
        doubleSpinBox_openchisel_carving_dist_m->setValue(0.01);

        gridLayout_19->addWidget(doubleSpinBox_openchisel_carving_dist_m, 10, 0, 1, 1);

        doubleSpinBox_openchisel_near_plane_dist = new QDoubleSpinBox(groupBox_openchisel);
        doubleSpinBox_openchisel_near_plane_dist->setObjectName(QStringLiteral("doubleSpinBox_openchisel_near_plane_dist"));
        doubleSpinBox_openchisel_near_plane_dist->setDecimals(2);
        doubleSpinBox_openchisel_near_plane_dist->setMinimum(0);
        doubleSpinBox_openchisel_near_plane_dist->setMaximum(100);
        doubleSpinBox_openchisel_near_plane_dist->setSingleStep(0.001);
        doubleSpinBox_openchisel_near_plane_dist->setValue(0.01);

        gridLayout_19->addWidget(doubleSpinBox_openchisel_near_plane_dist, 11, 0, 1, 1);

        doubleSpinBox_openchisel_far_plane_dist = new QDoubleSpinBox(groupBox_openchisel);
        doubleSpinBox_openchisel_far_plane_dist->setObjectName(QStringLiteral("doubleSpinBox_openchisel_far_plane_dist"));
        doubleSpinBox_openchisel_far_plane_dist->setDecimals(2);
        doubleSpinBox_openchisel_far_plane_dist->setMinimum(0);
        doubleSpinBox_openchisel_far_plane_dist->setMaximum(100);
        doubleSpinBox_openchisel_far_plane_dist->setSingleStep(0.001);
        doubleSpinBox_openchisel_far_plane_dist->setValue(0);

        gridLayout_19->addWidget(doubleSpinBox_openchisel_far_plane_dist, 12, 0, 1, 1);

        label_48 = new QLabel(groupBox_openchisel);
        label_48->setObjectName(QStringLiteral("label_48"));
        label_48->setWordWrap(true);

        gridLayout_19->addWidget(label_48, 0, 1, 1, 1);

        checkBox_openchisel_mergeVertices = new QCheckBox(groupBox_openchisel);
        checkBox_openchisel_mergeVertices->setObjectName(QStringLiteral("checkBox_openchisel_mergeVertices"));
        checkBox_openchisel_mergeVertices->setChecked(true);

        gridLayout_19->addWidget(checkBox_openchisel_mergeVertices, 0, 0, 1, 1);

        gridLayout_19->setColumnStretch(1, 1);

        verticalLayout_18->addLayout(gridLayout_19);


        verticalLayout_15->addWidget(groupBox_openchisel);

        verticalSpacer_8 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_15->addItem(verticalSpacer_8);


        verticalLayout_13->addWidget(groupBox_meshing);

        verticalSpacer_9 = new QSpacerItem(0, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_13->addItem(verticalSpacer_9);

        verticalSpacer_7 = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_13->addItem(verticalSpacer_7);

        scrollArea->setWidget(scrollAreaWidgetContents);

        verticalLayout_2->addWidget(scrollArea);

        verticalSpacer = new QSpacerItem(20, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);

        buttonBox = new QDialogButtonBox(ExportCloudsDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok|QDialogButtonBox::RestoreDefaults|QDialogButtonBox::Save);

        verticalLayout_2->addWidget(buttonBox);

        verticalLayout_2->setStretch(0, 1);

        retranslateUi(ExportCloudsDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), ExportCloudsDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), ExportCloudsDialog, SLOT(reject()));

        stackedWidget_upsampling->setCurrentIndex(4);


        QMetaObject::connectSlotsByName(ExportCloudsDialog);
    } // setupUi

    void retranslateUi(QDialog *ExportCloudsDialog)
    {
        ExportCloudsDialog->setWindowTitle(QApplication::translate("ExportCloudsDialog", "Export 3D Clouds", Q_NULLPTR));
        comboBox_pipeline->clear();
        comboBox_pipeline->insertItems(0, QStringList()
         << QApplication::translate("ExportCloudsDialog", "Organized Point Cloud", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "Dense Point Cloud", Q_NULLPTR)
        );
        label_12->setText(QApplication::translate("ExportCloudsDialog", "Reconstruction flavor.", Q_NULLPTR));
        checkBox_regenerate->setText(QString());
        checkBox_filtering->setText(QString());
        label_regenerate->setText(QApplication::translate("ExportCloudsDialog", "Regenerate clouds. This can be used to regenerate the point clouds at higher density than those used for online visualization.", Q_NULLPTR));
        label_voxel->setText(QApplication::translate("ExportCloudsDialog", "Voxel size. Set 0 to disable. When organized meshes are assembled, this is the radius in which the vertices of the polygons are merged.", Q_NULLPTR));
        label_binaryFile_2->setText(QApplication::translate("ExportCloudsDialog", "Assemble clouds/meshes to a single output cloud/mesh.", Q_NULLPTR));
        label_frame->setText(QApplication::translate("ExportCloudsDialog", "Output frame.", Q_NULLPTR));
        checkBox_smoothing->setText(QString());
        label_binaryFile_9->setText(QApplication::translate("ExportCloudsDialog", "Cloud filtering. Remove sparse points that are far from surfaces.", Q_NULLPTR));
        label_binaryFile->setText(QApplication::translate("ExportCloudsDialog", "Binary file (for ply and pcd outputs).", Q_NULLPTR));
        label_smoothing->setText(QApplication::translate("ExportCloudsDialog", "Cloud smoothing using Moving Least Squares algorithm (MLS).", Q_NULLPTR));
        doubleSpinBox_voxelSize_assembled->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        checkBox_assemble->setText(QString());
        label_normal->setText(QApplication::translate("ExportCloudsDialog", "Set the number of k nearest neighbors to use for the normal estimation. Set 0 to disable.", Q_NULLPTR));
        comboBox_frame->clear();
        comboBox_frame->insertItems(0, QStringList()
         << QApplication::translate("ExportCloudsDialog", "Map", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "Robot", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "Camera", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "Scan", Q_NULLPTR)
        );
        checkBox_binary->setText(QString());
        label_binaryFile_11->setText(QApplication::translate("ExportCloudsDialog", "From RGB-D images. If not checked, clouds will be generated from laser scans.", Q_NULLPTR));
        checkBox_fromDepth->setText(QString());
        label_normal_2->setText(QApplication::translate("ExportCloudsDialog", "Set the search radius for the normal estimation.", Q_NULLPTR));
        label_intensityColormap->setText(QApplication::translate("ExportCloudsDialog", "Intensity colormap.", Q_NULLPTR));
        comboBox_intensityColormap->clear();
        comboBox_intensityColormap->insertItems(0, QStringList()
         << QApplication::translate("ExportCloudsDialog", "GrayScale", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "RedYellow", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "Rainbow", Q_NULLPTR)
        );
        label_gainCompensation->setText(QApplication::translate("ExportCloudsDialog", "Gain compensation. Normalize brightness of images.", Q_NULLPTR));
        label_cameraProjection->setText(QApplication::translate("ExportCloudsDialog", "Camera projection. This can be used to colorize point cloud created from scans and/or export camera IDs for each point of the cloud.", Q_NULLPTR));
        checkBox_gainCompensation->setText(QString());
        checkBox_cameraProjection->setText(QString());
        label_binaryFile_12->setText(QApplication::translate("ExportCloudsDialog", "Meshing.", Q_NULLPTR));
        checkBox_meshing->setText(QString());
        label_binaryFile_10->setText(QApplication::translate("ExportCloudsDialog", "Nodes filtering. Filter nodes to be exported in a specified region .", Q_NULLPTR));
        checkBox_nodes_filtering->setText(QString());
        groupBox_nodes_filtering->setTitle(QApplication::translate("ExportCloudsDialog", "Nodes Filtering", Q_NULLPTR));
        label_114->setText(QApplication::translate("ExportCloudsDialog", "Set min and max range values on each axis. Nodes inside those ranges will be exported. If the min and max are equals, there is no filtering on that axis. For example for 2D map, set Z min and Z max to 0. To filter just in altitude, set only Z ranges.", Q_NULLPTR));
        label_113->setText(QApplication::translate("ExportCloudsDialog", "X max", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_142->setToolTip(QApplication::translate("ExportCloudsDialog", "Length along X-Axis", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_142->setText(QApplication::translate("ExportCloudsDialog", "Y min", Q_NULLPTR));
        doubleSpinBox_nodes_filtering_ymin->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_115->setToolTip(QApplication::translate("ExportCloudsDialog", "Width along Y-Axis", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_115->setText(QApplication::translate("ExportCloudsDialog", "Y max", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_116->setToolTip(QApplication::translate("ExportCloudsDialog", "Height along Z-Axis", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_116->setText(QApplication::translate("ExportCloudsDialog", "Z min", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_119->setToolTip(QApplication::translate("ExportCloudsDialog", "Width along Y-Axis", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_119->setText(QApplication::translate("ExportCloudsDialog", "Z max", Q_NULLPTR));
        label_111->setText(QApplication::translate("ExportCloudsDialog", "X min", Q_NULLPTR));
        doubleSpinBox_nodes_filtering_xmin->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        doubleSpinBox_nodes_filtering_xmax->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        doubleSpinBox_nodes_filtering_zmin->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        doubleSpinBox_nodes_filtering_ymax->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        doubleSpinBox_nodes_filtering_zmax->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        groupBox_regenerateScans->setTitle(QApplication::translate("ExportCloudsDialog", "Regenerate Scans", Q_NULLPTR));
        label_109->setText(QApplication::translate("ExportCloudsDialog", "Downsampling step.", Q_NULLPTR));
        label_138->setText(QApplication::translate("ExportCloudsDialog", "Minimum range.", Q_NULLPTR));
        doubleSpinBox_rangeMin->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        doubleSpinBox_rangeMax->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_139->setText(QApplication::translate("ExportCloudsDialog", "Maximum range (0 means no limit).", Q_NULLPTR));
        groupBox_regenerate->setTitle(QApplication::translate("ExportCloudsDialog", "Regenerate Clouds", Q_NULLPTR));
        toolButton_distortionModel->setText(QApplication::translate("ExportCloudsDialog", "...", Q_NULLPTR));
        spinBox_fillDepthHoles->setSuffix(QApplication::translate("ExportCloudsDialog", " pixels", Q_NULLPTR));
        doubleSpinBox_minDepth->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_108->setText(QApplication::translate("ExportCloudsDialog", "3D cloud decimation (1-2-4-8-...). Negative decimation is done from RGB size instead of depth size (if depth is smaller than RGB, it may be interpolated depending of the decimation value).", Q_NULLPTR));
        spinBox_fillDepthHolesError->setSuffix(QApplication::translate("ExportCloudsDialog", " %", Q_NULLPTR));
        label_132->setText(QApplication::translate("ExportCloudsDialog", "3D cloud maximum depth (0 means no limit).", Q_NULLPTR));
        label_133->setText(QApplication::translate("ExportCloudsDialog", "3D cloud minimum depth.", Q_NULLPTR));
        label_197->setText(QApplication::translate("ExportCloudsDialog", "Fill depth hole size (0 means no fill).", Q_NULLPTR));
        doubleSpinBox_maxDepth->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_134->setText(QApplication::translate("ExportCloudsDialog", "Path to a depth distortion model to apply (output from depth calibration).", Q_NULLPTR));
        label_198->setText(QApplication::translate("ExportCloudsDialog", "Fill depth hole error.", Q_NULLPTR));
        label_135->setText(QApplication::translate("ExportCloudsDialog", "ROI ratios [left, right, top, bottom] between 0 and 1. Only generate 3D points for pixels inside the region of interest (RGB image).", Q_NULLPTR));
        checkBox_bilateral->setText(QString());
        label_136->setText(QApplication::translate("ExportCloudsDialog", "Bilateral filtering of the depth image. Reduce noise in depth images.", Q_NULLPTR));
        label_137->setText(QApplication::translate("ExportCloudsDialog", "Cloud subtraction. Superposed points from different nodes are filtered.", Q_NULLPTR));
        checkBox_subtraction->setText(QString());
        label_140->setText(QApplication::translate("ExportCloudsDialog", "Ceiling filtering height.", Q_NULLPTR));
        label_141->setText(QApplication::translate("ExportCloudsDialog", "Floor filtering height.", Q_NULLPTR));
        doubleSpinBox_ceilingHeight->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        doubleSpinBox_floorHeight->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        groupBox_bilateral->setTitle(QApplication::translate("ExportCloudsDialog", "Bilateral Filtering of the Depth Image", Q_NULLPTR));
        doubleSpinBox_bilateral_sigmaR->setSuffix(QString());
        label_194->setText(QApplication::translate("ExportCloudsDialog", "Standard deviation of the Gaussian for the intensity difference. Set the standard deviation of the Gaussian used to control how much an adjacent pixel is downweighted because of the intensity difference (depth in our case).", Q_NULLPTR));
        doubleSpinBox_bilateral_sigmaS->setSuffix(QString());
        label_195->setText(QApplication::translate("ExportCloudsDialog", "Size of the Gaussian bilateral filter window to use. Set the standard deviation of the Gaussian used by the bilateral filter for the spatial neighborhood/window.", Q_NULLPTR));
        groupBox_subtraction->setTitle(QApplication::translate("ExportCloudsDialog", "Cloud Subtraction", Q_NULLPTR));
        label_174->setText(QApplication::translate("ExportCloudsDialog", "Minimum points of the previous cloud in the fixed point radius in order to substract the point in the new cloud. Increasing this value reduces the black contours between clouds.", Q_NULLPTR));
        label_193->setText(QApplication::translate("ExportCloudsDialog", "Point angle used to keep only points with similar normal between the current cloud and the previous one.", Q_NULLPTR));
        doubleSpinBox_subtractPointFilteringRadius->setSuffix(QString());
        label_192->setText(QApplication::translate("ExportCloudsDialog", "Point radius used to find close points between the current cloud and the previous one.", Q_NULLPTR));
        doubleSpinBox_subtractPointFilteringAngle->setSuffix(QApplication::translate("ExportCloudsDialog", " degrees", Q_NULLPTR));
        groupBox_filtering->setTitle(QApplication::translate("ExportCloudsDialog", "Cloud Filtering", Q_NULLPTR));
        doubleSpinBox_filteringRadius->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_110->setText(QApplication::translate("ExportCloudsDialog", "Radius search.", Q_NULLPTR));
        label_112->setText(QApplication::translate("ExportCloudsDialog", "Minimum neighbors in the search radius.", Q_NULLPTR));
        groupBox_mls->setTitle(QApplication::translate("ExportCloudsDialog", "Cloud Smoothing", Q_NULLPTR));
        label->setText(QApplication::translate("ExportCloudsDialog", "WARNING: This adds significative time to process, though the clouds will be more smooth.", Q_NULLPTR));
        doubleSpinBox_mlsRadius->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        comboBox_upsamplingMethod->clear();
        comboBox_upsamplingMethod->insertItems(0, QStringList()
         << QApplication::translate("ExportCloudsDialog", "NONE", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "DISTINCT_CLOUD", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "SAMPLE_LOCAL_PLANE", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "RANDOM_UNIFORM_DENSITY", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "VOXEL_GRID_DILATION", Q_NULLPTR)
        );
        label_87->setText(QApplication::translate("ExportCloudsDialog", "MLS search radius: Set the sphere radius that is to be used for determining the k-nearest neighbors used for fitting. \n"
"Guidelines: 4 times the voxel size, 0.025 for voxel=0.", Q_NULLPTR));
        label_88->setText(QApplication::translate("ExportCloudsDialog", "Upsampling method.", Q_NULLPTR));
        label_89->setText(QApplication::translate("ExportCloudsDialog", "Polygonial order. Sets whether the surface and normal are approximated using a polynomial, or only via tangent estimation (value=0).", Q_NULLPTR));
        label_90->setText(QApplication::translate("ExportCloudsDialog", "Output voxel size. Used to filter superposed points after upsampling.", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("ExportCloudsDialog", "NONE", Q_NULLPTR));
        label_2->setText(QApplication::translate("ExportCloudsDialog", "No upsampling will be done, only the input points will be projected to their own MLS surfaces.", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("ExportCloudsDialog", "DISTINCT_CLOUD", Q_NULLPTR));
        label_3->setText(QApplication::translate("ExportCloudsDialog", "Will project the points of the distinct cloud to the closest point on the MLS surface.", Q_NULLPTR));
        groupBox_3->setTitle(QApplication::translate("ExportCloudsDialog", "SAMPLE_LOCAL_PLANE", Q_NULLPTR));
        label_4->setText(QApplication::translate("ExportCloudsDialog", "The local plane of each input point will be sampled in a circular fashion.", Q_NULLPTR));
        label_8->setText(QApplication::translate("ExportCloudsDialog", "Set the step size for the local plane sampling.", Q_NULLPTR));
        label_7->setText(QApplication::translate("ExportCloudsDialog", "Set the radius of the circle in the local point plane that will be sampled.", Q_NULLPTR));
        groupBox_4->setTitle(QApplication::translate("ExportCloudsDialog", "RANDOM_UNIFORM_DENSITY", Q_NULLPTR));
        label_5->setText(QApplication::translate("ExportCloudsDialog", "The local plane of each input point will be sampled using an uniform random distribution such that the density of points is constant throughout the cloud.", Q_NULLPTR));
        label_9->setText(QApplication::translate("ExportCloudsDialog", "Set the parameter that specifies the desired number of points within the search radius.", Q_NULLPTR));
        groupBox_5->setTitle(QApplication::translate("ExportCloudsDialog", "VOXEL_GRID_DILATION", Q_NULLPTR));
        label_6->setText(QApplication::translate("ExportCloudsDialog", "The input cloud will be inserted into a voxel grid. This voxel grid will be dilated and the resulting points will be projected to the MLS surface of the closest point in the input cloud. The result is a point cloud with filled holes and a constant point density.", Q_NULLPTR));
        label_10->setText(QApplication::translate("ExportCloudsDialog", "Set the voxel size for the voxel grid.", Q_NULLPTR));
        label_11->setText(QApplication::translate("ExportCloudsDialog", "Set the number of dilation steps of the voxel grid.", Q_NULLPTR));
        groupBox_gain->setTitle(QApplication::translate("ExportCloudsDialog", "Gain Compensation", Q_NULLPTR));
        doubleSpinBox_gainBeta->setSuffix(QString());
        label_binaryFile_7->setText(QApplication::translate("ExportCloudsDialog", "Do full compensation between all locations (longer to do but quality is better). Otherwise, compensation is done only between linked locations.", Q_NULLPTR));
        doubleSpinBox_gainOverlap->setSuffix(QString());
        checkBox_gainFull->setText(QString());
        label_binaryFile_3->setText(QApplication::translate("ExportCloudsDialog", "Correspondence radius", Q_NULLPTR));
        label_binaryFile_5->setText(QApplication::translate("ExportCloudsDialog", "Beta. The lower, the higher the compensation is but images are darker. When texturing, brightness and contrast balance below can be activated to re-increase brightness (e.g., set high ratio to 20%).", Q_NULLPTR));
        doubleSpinBox_gainRadius->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_binaryFile_6->setText(QApplication::translate("ExportCloudsDialog", "Min overlap (%)", Q_NULLPTR));
        label_binaryFile_8->setText(QApplication::translate("ExportCloudsDialog", "Gain on each RGB channel separatly.", Q_NULLPTR));
        checkBox_gainRGB->setText(QString());
        groupBox_cameraProjection->setTitle(QApplication::translate("ExportCloudsDialog", "Camera Projection", Q_NULLPTR));
        doubleSpinBox_camProjMaxAngle->setSuffix(QApplication::translate("ExportCloudsDialog", " deg", Q_NULLPTR));
        label_meshingTextureSize_15->setText(QApplication::translate("ExportCloudsDialog", "Keep points not seen by the cameras. These points will be set with a pure red color (255,0,0) if the cloud was created from laser scans.", Q_NULLPTR));
        checkBox_camProjDistanceToCamPolicy->setText(QString());
        label_meshingTextureSize_14->setText(QApplication::translate("ExportCloudsDialog", "Distance to camera policy. The closest camera from a point is used to colorize the point. If disabled, the camera for which the point projection is the closest of the image center is used to colorize the point.", Q_NULLPTR));
        label_meshingTextureSize_13->setText(QApplication::translate("ExportCloudsDialog", "Maximum distance from the camera for points to be colorized by this camera (0 means infinite).", Q_NULLPTR));
        label_meshingTextureSize_12->setText(QApplication::translate("ExportCloudsDialog", "Maximum angle between camera and point's normal to apply color (0 means disabled).", Q_NULLPTR));
        doubleSpinBox_camProjMaxDistance->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        checkBox_camProjKeepPointsNotSeenByCameras->setText(QString());
        label_camProjExportCamera->setText(QApplication::translate("ExportCloudsDialog", "ID format of the camera selected for each point.", Q_NULLPTR));
        comboBox_camProjExportCamera->clear();
        comboBox_camProjExportCamera->insertItems(0, QStringList()
         << QApplication::translate("ExportCloudsDialog", "Disabled", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "By Node ID", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "By Camera Index", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "By Node ID and Camera Index", Q_NULLPTR)
        );
#ifndef QT_NO_TOOLTIP
        comboBox_camProjExportCamera->setToolTip(QApplication::translate("ExportCloudsDialog", "By Node ID: cameras of same node have same ID\n"
"By Camera Index: of a multi-cameras setup\n"
"By Node ID and Camera Index: NodeID*10+CameraIndex", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_meshingTextureSize_17->setText(QApplication::translate("ExportCloudsDialog", "ROI ratios [left right top bottom] between 0 and 1. This can be used to ignore black borders of RGB images caused by calibration. ", Q_NULLPTR));
        label_meshingTextureSize_18->setText(QApplication::translate("ExportCloudsDialog", "Recolor points from camera projection. This would be used to color laser scans with the cameras.", Q_NULLPTR));
        checkBox_camProjRecolorPoints->setText(QString());
        groupBox_meshing->setTitle(QApplication::translate("ExportCloudsDialog", "Meshing", Q_NULLPTR));
        label_meshClean->setText(QApplication::translate("ExportCloudsDialog", "Clean mesh from polygons without color or texture.", Q_NULLPTR));
        label_denseReconstruction->setText(QApplication::translate("ExportCloudsDialog", "Surface reconstruction approach. Poisson is available when clouds are assembled and dense reconstruction flavor is selected. CPU-TSDF is available when clouds are assembled, organized reconstruction flavor is selected and RTAB-Map is built with CPU-TSDF support. OpenChisel is available when clouds are assembled and RTAB-Map is built with OpenChisel support.", Q_NULLPTR));
        label_meshDecimation_2->setText(QApplication::translate("ExportCloudsDialog", "Transferring color radius. Radius used to transfer color from original cloud to resampled reconstructed surface (e.g., Poisson or mesh decimation). Negative means disabled, 0 means take the nearest point.", Q_NULLPTR));
        label_meshDecimation->setText(QApplication::translate("ExportCloudsDialog", "Mesh quadric decimation factor (0=no decimation). Used to reduce the number of polygons. Higher the factor, lower the output resolution (less polygons). Can be used when organized mesh approach is not selected. Can reduce a lot texturing time.", Q_NULLPTR));
        label_textureMapping->setText(QApplication::translate("ExportCloudsDialog", "Texture mapping. Images of the cameras will be projected on the mesh(es). Output is a *.obj format. Available on Export or when clouds are not assembled.", Q_NULLPTR));
        comboBox_meshingApproach->clear();
        comboBox_meshingApproach->insertItems(0, QStringList()
         << QApplication::translate("ExportCloudsDialog", "Fast GP3", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "Poisson", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "CPU-TSDF", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "Organized", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "Open Chisel (TSDF)", Q_NULLPTR)
        );
        checkBox_textureMapping->setText(QString());
        doubleSpinBox_transferColorRadius->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        checkBox_cleanMesh->setText(QString());
        label_16->setText(QApplication::translate("ExportCloudsDialog", "Minimum polygon cluster size (-1 means that only the biggest cluster is kept).", Q_NULLPTR));
        label_meshMaxPolygons->setText(QApplication::translate("ExportCloudsDialog", "Maximum polygons (0=no max). Another way to do mesh quadric decimation, the factor will be computed depending on the polygons generated.", Q_NULLPTR));
        spinBox_meshMaxPolygons->setSuffix(QString());
        groupBox_textureMapping->setTitle(QApplication::translate("ExportCloudsDialog", "Texturing", Q_NULLPTR));
        spinBox_textureBrightnessContrastRatioHigh->setSuffix(QApplication::translate("ExportCloudsDialog", " %", Q_NULLPTR));
        label_meshingTextureSize_6->setText(QApplication::translate("ExportCloudsDialog", "Brightness and contrast balance high ratio. Only used when textures are merged.", Q_NULLPTR));
        comboBox_blendingDecimation->clear();
        comboBox_blendingDecimation->insertItems(0, QStringList()
         << QApplication::translate("ExportCloudsDialog", "Auto", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "1", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "2", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "4", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "8", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "16", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "32", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "64", Q_NULLPTR)
        );
        checkBox_blending->setText(QString());
        label_exposureFusion_3->setText(QApplication::translate("ExportCloudsDialog", "Blending decimation. ", Q_NULLPTR));
        label_meshingTextureSize_7->setText(QApplication::translate("ExportCloudsDialog", "ROI ratios [left right top bottom] between 0 and 1. This can be used to ignore black borders of RGB images caused by calibration. ", Q_NULLPTR));
        label_meshingTextureSize_8->setText(QApplication::translate("ExportCloudsDialog", "Maximum output textures.", Q_NULLPTR));
        label_exposureFusion_2->setText(QApplication::translate("ExportCloudsDialog", "Blending. Only used with dense reconstruction flavor and if clouds are assembled.", Q_NULLPTR));
        label_meshingTextureSize_2->setText(QApplication::translate("ExportCloudsDialog", "Maximum distance from the camera for polygons to be textured by this camera (0 means infinite). Only used with dense reconstruction flavor.", Q_NULLPTR));
        doubleSpinBox_meshingTextureMaxDistance->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_meshingTextureSize_5->setText(QApplication::translate("ExportCloudsDialog", "Brightness and contrast balance low ratio. Only used when textures are merged.", Q_NULLPTR));
        label_meshingTextureFormat->setText(QApplication::translate("ExportCloudsDialog", "Texture format.", Q_NULLPTR));
        comboBox_meshingTextureFormat->clear();
        comboBox_meshingTextureFormat->insertItems(0, QStringList()
         << QApplication::translate("ExportCloudsDialog", ".jpg", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", ".png", Q_NULLPTR)
        );
        comboBox_meshingTextureSize->clear();
        comboBox_meshingTextureSize->insertItems(0, QStringList()
         << QApplication::translate("ExportCloudsDialog", "Disabled", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "256x256", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "512x512", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "1024x1024", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "2048x2048", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "4096x4096", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "8192x8192", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "16384x16384", Q_NULLPTR)
         << QApplication::translate("ExportCloudsDialog", "32768x32768", Q_NULLPTR)
        );
        checkBox_exposureFusion->setText(QString());
        label_meshingTextureSize_4->setText(QApplication::translate("ExportCloudsDialog", "Minimum polygon cluster size for texturing. This removes textures applied on sparse polygons. Only used with dense reconstruction flavor.", Q_NULLPTR));
        label_meshingTextureSize->setText(QApplication::translate("ExportCloudsDialog", "Output texture size. If not set or when clouds are not assembled, all textures are saved separately. Warning: values higher than 2048 may not be compatible with all GPUs.", Q_NULLPTR));
        checkBox_cameraFilter->setText(QString());
        spinBox_textureBrightnessContrastRatioLow->setSuffix(QApplication::translate("ExportCloudsDialog", " %", Q_NULLPTR));
        label_meshingTextureSize_3->setText(QApplication::translate("ExportCloudsDialog", "Camera filtering. Some criteria to select which cameras are used for texturing.", Q_NULLPTR));
        label_exposureFusion->setText(QApplication::translate("ExportCloudsDialog", "Exposure fusion. Only used when textures are merged and brightness/contrast balance ratios are used (merging original texture + 1 for each ratios).", Q_NULLPTR));
        label_meshingTextureSize_9->setText(QApplication::translate("ExportCloudsDialog", "Maximum depth error between reconstructed mesh projection in camera and corresponding value in depth image to apply texturing (negative means disabled, 0 means that longest edge length of the face is used as maximum error).", Q_NULLPTR));
        doubleSpinBox_meshingTextureMaxDepthError->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_meshingTextureSize_10->setText(QApplication::translate("ExportCloudsDialog", "Maximum angle between camera and face to apply texture (0 means disabled).", Q_NULLPTR));
        doubleSpinBox_meshingTextureMaxAngle->setSuffix(QApplication::translate("ExportCloudsDialog", " deg", Q_NULLPTR));
        label_multiband->setText(QApplication::translate("ExportCloudsDialog", "MultiBand texturing. Only available on export and RTAB-Map should be built with AliceVision support.", Q_NULLPTR));
        checkBox_multiband->setText(QString());
        label_meshingTextureSize_11->setText(QApplication::translate("ExportCloudsDialog", "Distance to camera policy. The closest camera from a polygon is used to texture the polygon. If disabled, the camera for which the polygon projection is the closest of the image center is used to texture the polygon.", Q_NULLPTR));
        checkBox_distanceToCamPolicy->setText(QString());
        groupBox_cameraFilter->setTitle(QApplication::translate("ExportCloudsDialog", "Camera Filtering", Q_NULLPTR));
        doubleSpinBox_cameraFilterRadius->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_37->setText(QApplication::translate("ExportCloudsDialog", "Only one camera in a fixed radius and angle is used for texturing. Radius of 0 m means disabled.", Q_NULLPTR));
        doubleSpinBox_laplacianVariance->setSuffix(QString());
        doubleSpinBox_cameraFilterAngle->setSuffix(QApplication::translate("ExportCloudsDialog", " deg", Q_NULLPTR));
        label_38->setText(QApplication::translate("ExportCloudsDialog", "Laplacian variance threshold. Below this threshold, the image is considered blurred. 0 means disabled. 50 can be good default.", Q_NULLPTR));
        doubleSpinBox_cameraFilterVelRad->setSuffix(QApplication::translate("ExportCloudsDialog", " rad/s", Q_NULLPTR));
        doubleSpinBox_cameraFilterVel->setSuffix(QApplication::translate("ExportCloudsDialog", " m/s", Q_NULLPTR));
        label_39->setText(QApplication::translate("ExportCloudsDialog", "Maximum speed to keep a camera view for texturing. Images taken on fast motions would be more blurry. 0 means disabled. 0.1 m/s and 0.4 rad/s can be good defaults.", Q_NULLPTR));
        groupBox_gp3->setTitle(QApplication::translate("ExportCloudsDialog", "Greedy Projection Triangulation (GP3) Meshing", Q_NULLPTR));
        label_169->setText(QApplication::translate("ExportCloudsDialog", "Set the multiplier of the nearest neighbor distance to obtain the final search radius for each point  (this will make the algorithm adapt to different point densities in the cloud).", Q_NULLPTR));
        doubleSpinBox_gp3Radius->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_168->setText(QApplication::translate("ExportCloudsDialog", "Sphere radius that is to be used for determining the k-nearest neighbors used for triangulating (GP3). \n"
"Guidelines: 4 times the voxel size, 0.025 for voxel=0.", Q_NULLPTR));
        groupBox_poisson->setTitle(QApplication::translate("ExportCloudsDialog", "Poisson Surface Reconstruction", Q_NULLPTR));
        label_172->setText(QApplication::translate("ExportCloudsDialog", "Depth. Lowering this parameter decreases reconstruction time, but geometry precision is lower. Minimum polygon size: map length / 2^depth), you can start tuning from depth of 8 or 9. 0 means that depth is chosen so that polygon size is just under target polygon size below.", Q_NULLPTR));
        label_177->setText(QApplication::translate("ExportCloudsDialog", "Samples per node. Set the minimum number of sample points that should fall within an octree node as the octree construction is adapted to sampling density. Note for noise-free samples, small values in the range [1.0 - 5.0] can be used. For more noisy samples, larger values in the range [15.0 - 20.0] may be needed to provide a smoother, noise-reduced, reconstruction.", Q_NULLPTR));
        label_176->setText(QApplication::translate("ExportCloudsDialog", "Manifold flag. Enabling this flag tells the reconstructor to add the polygon barycenter when triangulating polygons with more than three vertices.", Q_NULLPTR));
        label_181->setText(QApplication::translate("ExportCloudsDialog", "Point weight.", Q_NULLPTR));
        label_182->setText(QApplication::translate("ExportCloudsDialog", "Min depth.", Q_NULLPTR));
        label_178->setText(QApplication::translate("ExportCloudsDialog", "Iso divide. Set the depth at which a block iso-surface extractor should be used to extract the iso-surface note Using this parameter helps reduce the memory overhead at the cost of a small increase in extraction time. (In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)", Q_NULLPTR));
        checkBox_poisson_manifold->setText(QString());
        label_179->setText(QApplication::translate("ExportCloudsDialog", "Solver divide. Set the the depth at which a block Gauss-Seidel solver is used to solve the Laplacian equation. Note that using this parameter helps reduce the memory overhead at the cost of a small increase in reconstruction time. (In practice, we have found that for reconstructions of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)", Q_NULLPTR));
        label_180->setText(QApplication::translate("ExportCloudsDialog", "Scale. Set the ratio between the diameter of the cube used for reconstruction and the diameter of the samples' bounding cube.", Q_NULLPTR));
        checkBox_poisson_outputPolygons->setText(QString());
        label_outputPolygons->setText(QApplication::translate("ExportCloudsDialog", "Output polygons flag. Enabling this flag tells the reconstructor to output a polygon mesh (rather than triangulating of the results of Marching Cubes). Disabled when exporting to file, if mesh decimation is set or when texturing.", Q_NULLPTR));
        label_outputPolygons_2->setText(QApplication::translate("ExportCloudsDialog", "Target polygon size used to compute Poisson's depth based on the map size.", Q_NULLPTR));
        doubleSpinBox_poisson_targetPolygonSize->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        groupBox_cputsdf->setTitle(QApplication::translate("ExportCloudsDialog", "CPU-TSDF Reconstruction", Q_NULLPTR));
        label_25->setText(QApplication::translate("ExportCloudsDialog", "<html><head/><body><p><a href=\"https://github.com/sdmiller/cpu_tsdf\"><span style=\" text-decoration: underline; color:#0000ff;\">https://github.com/sdmiller/cpu_tsdf</span></a></p></body></html>", Q_NULLPTR));
        doubleSpinBox_cputsdf_size->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_19->setText(QApplication::translate("ExportCloudsDialog", "Volume size.", Q_NULLPTR));
        doubleSpinBox_cputsdf_resolution->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_20->setText(QApplication::translate("ExportCloudsDialog", "Resolution.", Q_NULLPTR));
        doubleSpinBox_cputsdf_tuncPos->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_21->setText(QApplication::translate("ExportCloudsDialog", "Depth truncation positive limit. Maximum distance a voxel will be set to positive.", Q_NULLPTR));
        doubleSpinBox_cputsdf_tuncNeg->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_22->setText(QApplication::translate("ExportCloudsDialog", "Depth truncation negative limit. Maximum distance a voxel will be set to negative.", Q_NULLPTR));
        spinBox_cputsdf_randomSplit->setSuffix(QString());
        label_23->setText(QApplication::translate("ExportCloudsDialog", "Set the number of random samples per surface point the octree should split on. If you don't know what this does, it's almost certainly best to leave it at 1.", Q_NULLPTR));
        doubleSpinBox_cputsdf_minWeight->setSuffix(QString());
        label_18->setText(QApplication::translate("ExportCloudsDialog", "Minimum weight of a voxel to be added in the mesh.", Q_NULLPTR));
        doubleSpinBox_cputsdf_flattenRadius->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_24->setText(QApplication::translate("ExportCloudsDialog", "Flatten radius (0=disabled).", Q_NULLPTR));
        groupBox_organized->setTitle(QApplication::translate("ExportCloudsDialog", "Organized Meshing", Q_NULLPTR));
        label_15->setText(QApplication::translate("ExportCloudsDialog", "Triangle size", Q_NULLPTR));
        label_quad->setText(QApplication::translate("ExportCloudsDialog", "Quad", Q_NULLPTR));
        checkBox_mesh_quad->setText(QString());
        doubleSpinBox_mesh_angleTolerance->setSuffix(QApplication::translate("ExportCloudsDialog", " deg", Q_NULLPTR));
        label_14->setText(QApplication::translate("ExportCloudsDialog", "Angle tolerance", Q_NULLPTR));
        spinBox_mesh_triangleSize->setSuffix(QApplication::translate("ExportCloudsDialog", " pix", Q_NULLPTR));
        groupBox_openchisel->setTitle(QApplication::translate("ExportCloudsDialog", "OpenChisel Reconstruction", Q_NULLPTR));
        label_26->setText(QApplication::translate("ExportCloudsDialog", "<html><head/><body><p><a href=\"https://github.com/personalrobotics/OpenChisel\"><span style=\" text-decoration: underline; color:#0000ff;\">https://github.com/personalrobotics/OpenChisel</span></a></p><p>Parameters below match those in chisel_ros package. Voxel resolution is taken from the general voxel parameter above, it should not be null. <br/><span style=\" font-weight:600;\">Issue: </span>If voxel size or chunk size are changed after exporting/viewing one time, the app should be restarted to avoid black meshes.</p></body></html>", Q_NULLPTR));
        label_36->setText(QApplication::translate("ExportCloudsDialog", "truncation_constant", Q_NULLPTR));
        label_35->setText(QApplication::translate("ExportCloudsDialog", "chunk_size_z", Q_NULLPTR));
        label_40->setText(QApplication::translate("ExportCloudsDialog", "truncation_linear", Q_NULLPTR));
        label_43->setText(QApplication::translate("ExportCloudsDialog", "integration_weight", Q_NULLPTR));
        label_46->setText(QApplication::translate("ExportCloudsDialog", "near_plane_dist", Q_NULLPTR));
        label_34->setText(QApplication::translate("ExportCloudsDialog", "chunk_size_y", Q_NULLPTR));
        spinBox_openchisel_chunk_size_y->setSuffix(QString());
        doubleSpinBox_openchisel_truncation_constant->setSuffix(QString());
        spinBox_openchisel_chunk_size_z->setSuffix(QString());
        spinBox_openchisel_chunk_size_x->setSuffix(QString());
        label_31->setText(QApplication::translate("ExportCloudsDialog", "chunk_size_x", Q_NULLPTR));
        label_42->setText(QApplication::translate("ExportCloudsDialog", "truncation_scale", Q_NULLPTR));
        label_41->setText(QApplication::translate("ExportCloudsDialog", "truncation_quadratic", Q_NULLPTR));
        label_44->setText(QApplication::translate("ExportCloudsDialog", "use_voxel_carving", Q_NULLPTR));
        label_45->setText(QApplication::translate("ExportCloudsDialog", "carving_dist_m", Q_NULLPTR));
        label_47->setText(QApplication::translate("ExportCloudsDialog", "far_plane_dist", Q_NULLPTR));
        doubleSpinBox_openchisel_truncation_linear->setSuffix(QString());
        doubleSpinBox_openchisel_truncation_quadratic->setSuffix(QString());
        doubleSpinBox_openchisel_truncation_scale->setSuffix(QString());
        spinBox_openchisel_integration_weight->setSuffix(QString());
        checkBox_openchisel_use_voxel_carving->setText(QString());
        doubleSpinBox_openchisel_carving_dist_m->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        doubleSpinBox_openchisel_near_plane_dist->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        doubleSpinBox_openchisel_far_plane_dist->setSuffix(QApplication::translate("ExportCloudsDialog", " m", Q_NULLPTR));
        label_48->setText(QApplication::translate("ExportCloudsDialog", "Simplify generated mesh by merging close vertices together. Required for some post mesh filtering approaches to work properly.", Q_NULLPTR));
        checkBox_openchisel_mergeVertices->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class ExportCloudsDialog: public Ui_ExportCloudsDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_EXPORTCLOUDSDIALOG_H
