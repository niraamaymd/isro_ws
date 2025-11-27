/********************************************************************************
** Form generated from reading UI file 'aboutDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ABOUTDIALOG_H
#define UI_ABOUTDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_aboutDialog
{
public:
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout;
    QLabel *label_10;
    QFrame *frame;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QLabel *label_9;
    QGridLayout *gridLayout;
    QLabel *label_2;
    QLabel *label_8;
    QLabel *label_4;
    QLabel *label_HomePage_2;
    QLabel *label_HomePage_3;
    QLabel *label_6;
    QLabel *label_version;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout_2;
    QGridLayout *gridLayout_2;
    QLabel *label_g2o;
    QLabel *label_aliceVision;
    QLabel *label_34;
    QLabel *label_31;
    QLabel *label_ceres;
    QLabel *label_msckf;
    QLabel *label_realsense2_license;
    QLabel *label_42;
    QLabel *label_pymatcher_license;
    QLabel *label_pcl_version;
    QLabel *label_41;
    QLabel *label_orbslam;
    QLabel *label_29;
    QLabel *label_15;
    QLabel *label_viso2_license;
    QLabel *label_11;
    QLabel *label_k4a;
    QLabel *label_40;
    QLabel *label_20;
    QLabel *label_orbslam_title;
    QLabel *label_fastcv_license;
    QLabel *label_depthai_license;
    QLabel *label_gtsam;
    QLabel *label_freenect;
    QLabel *label_dc1394;
    QLabel *label_26;
    QLabel *label_77;
    QLabel *label_qt_version;
    QLabel *label_zed;
    QLabel *label_12;
    QLabel *label_19;
    QLabel *label_cvsba_license;
    QLabel *label_32;
    QLabel *label_38;
    QLabel *label_orboctree_license;
    QLabel *label_okvis_license;
    QLabel *label_toro_license;
    QLabel *label_pcl_license;
    QLabel *label_27;
    QLabel *label_g2o_license;
    QLabel *label_freenect2;
    QLabel *label_orbslam_license;
    QLabel *label_vtk_version;
    QLabel *label_libpointmatcher_license;
    QLabel *label_freenect2_license;
    QLabel *label_22;
    QLabel *label_vtk_license;
    QLabel *label_okvis;
    QLabel *label_mynteye;
    QLabel *label_msckf_license;
    QLabel *label_dvo_license;
    QLabel *label_sptorch;
    QLabel *label_44;
    QLabel *label_43;
    QLabel *label_69;
    QLabel *label_23;
    QLabel *label_octomap;
    QLabel *label_flycapture2;
    QLabel *label_aliceVision_license;
    QLabel *label_25;
    QLabel *label_sptorch_license;
    QLabel *label_fastcv;
    QLabel *label_21;
    QLabel *label_dvo;
    QLabel *label_opencv_license;
    QLabel *label_7;
    QLabel *label_ceres_license;
    QLabel *label_79;
    QLabel *label_octomap_license;
    QLabel *label_dc1394_license;
    QLabel *label_39;
    QLabel *label_opencv_version;
    QLabel *label_g2o_license_3;
    QLabel *label_cvsba;
    QLabel *label_35;
    QLabel *label_k4w2;
    QLabel *label_17;
    QLabel *label_16;
    QLabel *label_freenect_license;
    QLabel *label_zedOC_license;
    QLabel *label_70;
    QLabel *label_realsense2;
    QLabel *label_14;
    QLabel *label_openni2_license;
    QLabel *label_toro;
    QLabel *label_loam_license;
    QLabel *label_realsense_license;
    QLabel *label_viso2;
    QLabel *label_68;
    QLabel *label_orboctree;
    QLabel *label_24;
    QLabel *label_18;
    QLabel *label_loam;
    QLabel *label_cputsdf;
    QLabel *label_libpointmatcher;
    QLabel *label_30;
    QLabel *label_gtsam_license;
    QLabel *label_qt_license;
    QLabel *label_13;
    QLabel *label_depthai;
    QLabel *label_realsense;
    QLabel *label_37;
    QLabel *label_cputsdf_license;
    QLabel *label_openchisel;
    QLabel *label_zedOC;
    QLabel *label_36;
    QLabel *label_fovis;
    QLabel *label_fovis_license;
    QLabel *label_pymatcher;
    QLabel *label_33;
    QLabel *label_openni2;
    QLabel *label_cccorelib_license;
    QLabel *label_cccorelib;
    QLabel *label_45;
    QSpacerItem *verticalSpacer_3;
    QLabel *label_3;
    QLabel *label_5;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *aboutDialog)
    {
        if (aboutDialog->objectName().isEmpty())
            aboutDialog->setObjectName(QStringLiteral("aboutDialog"));
        aboutDialog->resize(760, 738);
        verticalLayout_3 = new QVBoxLayout(aboutDialog);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label_10 = new QLabel(aboutDialog);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setMaximumSize(QSize(100, 100));
        label_10->setPixmap(QPixmap(QString::fromUtf8(":/images/RTAB-Map.png")));
        label_10->setScaledContents(true);
        label_10->setAlignment(Qt::AlignCenter);

        horizontalLayout->addWidget(label_10);

        frame = new QFrame(aboutDialog);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setFrameShape(QFrame::Box);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout = new QVBoxLayout(frame);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        label = new QLabel(frame);
        label->setObjectName(QStringLiteral("label"));
        label->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        verticalLayout->addWidget(label);

        label_9 = new QLabel(frame);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_9->setWordWrap(true);

        verticalLayout->addWidget(label_9);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        label_2 = new QLabel(frame);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 0, 0, 1, 1);

        label_8 = new QLabel(frame);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        gridLayout->addWidget(label_8, 0, 1, 1, 1);

        label_4 = new QLabel(frame);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 1, 0, 1, 1);

        label_HomePage_2 = new QLabel(frame);
        label_HomePage_2->setObjectName(QStringLiteral("label_HomePage_2"));
        label_HomePage_2->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_HomePage_2->setOpenExternalLinks(true);

        gridLayout->addWidget(label_HomePage_2, 1, 1, 1, 1);

        label_HomePage_3 = new QLabel(frame);
        label_HomePage_3->setObjectName(QStringLiteral("label_HomePage_3"));
        label_HomePage_3->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_HomePage_3->setOpenExternalLinks(true);

        gridLayout->addWidget(label_HomePage_3, 2, 1, 1, 1);

        label_6 = new QLabel(frame);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout->addWidget(label_6, 3, 0, 1, 1);

        label_version = new QLabel(frame);
        label_version->setObjectName(QStringLiteral("label_version"));
        label_version->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        gridLayout->addWidget(label_version, 3, 1, 1, 1);

        gridLayout->setColumnStretch(1, 1);

        verticalLayout->addLayout(gridLayout);

        scrollArea = new QScrollArea(frame);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, -493, 596, 935));
        verticalLayout_2 = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout_2->setSpacing(0);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 1, 0, 1);
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        label_g2o = new QLabel(scrollAreaWidgetContents);
        label_g2o->setObjectName(QStringLiteral("label_g2o"));
        label_g2o->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_g2o->setWordWrap(true);

        gridLayout_2->addWidget(label_g2o, 23, 1, 1, 1);

        label_aliceVision = new QLabel(scrollAreaWidgetContents);
        label_aliceVision->setObjectName(QStringLiteral("label_aliceVision"));
        label_aliceVision->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_aliceVision->setWordWrap(true);

        gridLayout_2->addWidget(label_aliceVision, 32, 1, 1, 1);

        label_34 = new QLabel(scrollAreaWidgetContents);
        label_34->setObjectName(QStringLiteral("label_34"));
        label_34->setWordWrap(true);

        gridLayout_2->addWidget(label_34, 39, 0, 1, 1);

        label_31 = new QLabel(scrollAreaWidgetContents);
        label_31->setObjectName(QStringLiteral("label_31"));
        label_31->setWordWrap(true);

        gridLayout_2->addWidget(label_31, 0, 2, 1, 1);

        label_ceres = new QLabel(scrollAreaWidgetContents);
        label_ceres->setObjectName(QStringLiteral("label_ceres"));
        label_ceres->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_ceres->setWordWrap(true);

        gridLayout_2->addWidget(label_ceres, 26, 1, 1, 1);

        label_msckf = new QLabel(scrollAreaWidgetContents);
        label_msckf->setObjectName(QStringLiteral("label_msckf"));
        label_msckf->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_msckf->setWordWrap(true);

        gridLayout_2->addWidget(label_msckf, 39, 1, 1, 1);

        label_realsense2_license = new QLabel(scrollAreaWidgetContents);
        label_realsense2_license->setObjectName(QStringLiteral("label_realsense2_license"));
        label_realsense2_license->setWordWrap(true);

        gridLayout_2->addWidget(label_realsense2_license, 13, 2, 1, 1);

        label_42 = new QLabel(scrollAreaWidgetContents);
        label_42->setObjectName(QStringLiteral("label_42"));
        label_42->setWordWrap(true);

        gridLayout_2->addWidget(label_42, 6, 0, 1, 1);

        label_pymatcher_license = new QLabel(scrollAreaWidgetContents);
        label_pymatcher_license->setObjectName(QStringLiteral("label_pymatcher_license"));
        label_pymatcher_license->setWordWrap(true);

        gridLayout_2->addWidget(label_pymatcher_license, 7, 2, 1, 1);

        label_pcl_version = new QLabel(scrollAreaWidgetContents);
        label_pcl_version->setObjectName(QStringLiteral("label_pcl_version"));
        label_pcl_version->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_pcl_version->setWordWrap(true);

        gridLayout_2->addWidget(label_pcl_version, 2, 1, 1, 1);

        label_41 = new QLabel(scrollAreaWidgetContents);
        label_41->setObjectName(QStringLiteral("label_41"));
        label_41->setWordWrap(true);

        gridLayout_2->addWidget(label_41, 32, 0, 1, 1);

        label_orbslam = new QLabel(scrollAreaWidgetContents);
        label_orbslam->setObjectName(QStringLiteral("label_orbslam"));
        label_orbslam->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_orbslam->setWordWrap(true);

        gridLayout_2->addWidget(label_orbslam, 36, 1, 1, 1);

        label_29 = new QLabel(scrollAreaWidgetContents);
        label_29->setObjectName(QStringLiteral("label_29"));
        label_29->setWordWrap(true);

        gridLayout_2->addWidget(label_29, 27, 0, 1, 1);

        label_15 = new QLabel(scrollAreaWidgetContents);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setWordWrap(true);

        gridLayout_2->addWidget(label_15, 11, 0, 1, 1);

        label_viso2_license = new QLabel(scrollAreaWidgetContents);
        label_viso2_license->setObjectName(QStringLiteral("label_viso2_license"));
        label_viso2_license->setWordWrap(true);

        gridLayout_2->addWidget(label_viso2_license, 34, 2, 1, 1);

        label_11 = new QLabel(scrollAreaWidgetContents);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setWordWrap(true);

        gridLayout_2->addWidget(label_11, 2, 0, 1, 1);

        label_k4a = new QLabel(scrollAreaWidgetContents);
        label_k4a->setObjectName(QStringLiteral("label_k4a"));
        label_k4a->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_k4a->setWordWrap(true);

        gridLayout_2->addWidget(label_k4a, 19, 1, 1, 1);

        label_40 = new QLabel(scrollAreaWidgetContents);
        label_40->setObjectName(QStringLiteral("label_40"));
        label_40->setWordWrap(true);

        gridLayout_2->addWidget(label_40, 26, 0, 1, 1);

        label_20 = new QLabel(scrollAreaWidgetContents);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setWordWrap(true);

        gridLayout_2->addWidget(label_20, 29, 0, 1, 1);

        label_orbslam_title = new QLabel(scrollAreaWidgetContents);
        label_orbslam_title->setObjectName(QStringLiteral("label_orbslam_title"));
        label_orbslam_title->setWordWrap(true);

        gridLayout_2->addWidget(label_orbslam_title, 36, 0, 1, 1);

        label_fastcv_license = new QLabel(scrollAreaWidgetContents);
        label_fastcv_license->setObjectName(QStringLiteral("label_fastcv_license"));
        label_fastcv_license->setWordWrap(true);

        gridLayout_2->addWidget(label_fastcv_license, 8, 2, 1, 1);

        label_depthai_license = new QLabel(scrollAreaWidgetContents);
        label_depthai_license->setObjectName(QStringLiteral("label_depthai_license"));
        label_depthai_license->setWordWrap(true);

        gridLayout_2->addWidget(label_depthai_license, 21, 2, 1, 1);

        label_gtsam = new QLabel(scrollAreaWidgetContents);
        label_gtsam->setObjectName(QStringLiteral("label_gtsam"));
        label_gtsam->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_gtsam->setWordWrap(true);

        gridLayout_2->addWidget(label_gtsam, 24, 1, 1, 1);

        label_freenect = new QLabel(scrollAreaWidgetContents);
        label_freenect->setObjectName(QStringLiteral("label_freenect"));
        label_freenect->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_freenect->setWordWrap(true);

        gridLayout_2->addWidget(label_freenect, 9, 1, 1, 1);

        label_dc1394 = new QLabel(scrollAreaWidgetContents);
        label_dc1394->setObjectName(QStringLiteral("label_dc1394"));
        label_dc1394->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_dc1394->setWordWrap(true);

        gridLayout_2->addWidget(label_dc1394, 14, 1, 1, 1);

        label_26 = new QLabel(scrollAreaWidgetContents);
        label_26->setObjectName(QStringLiteral("label_26"));
        label_26->setWordWrap(true);

        gridLayout_2->addWidget(label_26, 34, 0, 1, 1);

        label_77 = new QLabel(scrollAreaWidgetContents);
        label_77->setObjectName(QStringLiteral("label_77"));
        label_77->setWordWrap(true);

        gridLayout_2->addWidget(label_77, 22, 0, 1, 1);

        label_qt_version = new QLabel(scrollAreaWidgetContents);
        label_qt_version->setObjectName(QStringLiteral("label_qt_version"));
        label_qt_version->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_qt_version->setWordWrap(true);

        gridLayout_2->addWidget(label_qt_version, 4, 1, 1, 1);

        label_zed = new QLabel(scrollAreaWidgetContents);
        label_zed->setObjectName(QStringLiteral("label_zed"));
        label_zed->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_zed->setWordWrap(true);

        gridLayout_2->addWidget(label_zed, 16, 1, 1, 1);

        label_12 = new QLabel(scrollAreaWidgetContents);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setWordWrap(true);

        gridLayout_2->addWidget(label_12, 9, 0, 1, 1);

        label_19 = new QLabel(scrollAreaWidgetContents);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setWordWrap(true);

        gridLayout_2->addWidget(label_19, 24, 0, 1, 1);

        label_cvsba_license = new QLabel(scrollAreaWidgetContents);
        label_cvsba_license->setObjectName(QStringLiteral("label_cvsba_license"));
        label_cvsba_license->setWordWrap(true);

        gridLayout_2->addWidget(label_cvsba_license, 25, 2, 1, 1);

        label_32 = new QLabel(scrollAreaWidgetContents);
        label_32->setObjectName(QStringLiteral("label_32"));
        label_32->setTextFormat(Qt::RichText);
        label_32->setWordWrap(true);

        gridLayout_2->addWidget(label_32, 0, 0, 1, 1);

        label_38 = new QLabel(scrollAreaWidgetContents);
        label_38->setObjectName(QStringLiteral("label_38"));
        label_38->setWordWrap(true);

        gridLayout_2->addWidget(label_38, 18, 0, 1, 1);

        label_orboctree_license = new QLabel(scrollAreaWidgetContents);
        label_orboctree_license->setObjectName(QStringLiteral("label_orboctree_license"));
        label_orboctree_license->setWordWrap(true);

        gridLayout_2->addWidget(label_orboctree_license, 5, 2, 1, 1);

        label_okvis_license = new QLabel(scrollAreaWidgetContents);
        label_okvis_license->setObjectName(QStringLiteral("label_okvis_license"));
        label_okvis_license->setWordWrap(true);

        gridLayout_2->addWidget(label_okvis_license, 37, 2, 1, 1);

        label_toro_license = new QLabel(scrollAreaWidgetContents);
        label_toro_license->setObjectName(QStringLiteral("label_toro_license"));
        label_toro_license->setWordWrap(true);

        gridLayout_2->addWidget(label_toro_license, 22, 2, 1, 1);

        label_pcl_license = new QLabel(scrollAreaWidgetContents);
        label_pcl_license->setObjectName(QStringLiteral("label_pcl_license"));
        label_pcl_license->setWordWrap(true);

        gridLayout_2->addWidget(label_pcl_license, 2, 2, 1, 1);

        label_27 = new QLabel(scrollAreaWidgetContents);
        label_27->setObjectName(QStringLiteral("label_27"));
        label_27->setWordWrap(true);

        gridLayout_2->addWidget(label_27, 35, 0, 1, 1);

        label_g2o_license = new QLabel(scrollAreaWidgetContents);
        label_g2o_license->setObjectName(QStringLiteral("label_g2o_license"));
        label_g2o_license->setWordWrap(true);

        gridLayout_2->addWidget(label_g2o_license, 23, 2, 1, 1);

        label_freenect2 = new QLabel(scrollAreaWidgetContents);
        label_freenect2->setObjectName(QStringLiteral("label_freenect2"));
        label_freenect2->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_freenect2->setWordWrap(true);

        gridLayout_2->addWidget(label_freenect2, 11, 1, 1, 1);

        label_orbslam_license = new QLabel(scrollAreaWidgetContents);
        label_orbslam_license->setObjectName(QStringLiteral("label_orbslam_license"));
        label_orbslam_license->setWordWrap(true);

        gridLayout_2->addWidget(label_orbslam_license, 36, 2, 1, 1);

        label_vtk_version = new QLabel(scrollAreaWidgetContents);
        label_vtk_version->setObjectName(QStringLiteral("label_vtk_version"));
        label_vtk_version->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_vtk_version->setWordWrap(true);

        gridLayout_2->addWidget(label_vtk_version, 3, 1, 1, 1);

        label_libpointmatcher_license = new QLabel(scrollAreaWidgetContents);
        label_libpointmatcher_license->setObjectName(QStringLiteral("label_libpointmatcher_license"));
        label_libpointmatcher_license->setWordWrap(true);

        gridLayout_2->addWidget(label_libpointmatcher_license, 27, 2, 1, 1);

        label_freenect2_license = new QLabel(scrollAreaWidgetContents);
        label_freenect2_license->setObjectName(QStringLiteral("label_freenect2_license"));
        label_freenect2_license->setWordWrap(true);

        gridLayout_2->addWidget(label_freenect2_license, 11, 2, 1, 1);

        label_22 = new QLabel(scrollAreaWidgetContents);
        label_22->setObjectName(QStringLiteral("label_22"));
        label_22->setWordWrap(true);

        gridLayout_2->addWidget(label_22, 12, 0, 1, 1);

        label_vtk_license = new QLabel(scrollAreaWidgetContents);
        label_vtk_license->setObjectName(QStringLiteral("label_vtk_license"));
        label_vtk_license->setWordWrap(true);

        gridLayout_2->addWidget(label_vtk_license, 3, 2, 1, 1);

        label_okvis = new QLabel(scrollAreaWidgetContents);
        label_okvis->setObjectName(QStringLiteral("label_okvis"));
        label_okvis->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_okvis->setWordWrap(true);

        gridLayout_2->addWidget(label_okvis, 37, 1, 1, 1);

        label_mynteye = new QLabel(scrollAreaWidgetContents);
        label_mynteye->setObjectName(QStringLiteral("label_mynteye"));
        label_mynteye->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_mynteye->setWordWrap(true);

        gridLayout_2->addWidget(label_mynteye, 20, 1, 1, 1);

        label_msckf_license = new QLabel(scrollAreaWidgetContents);
        label_msckf_license->setObjectName(QStringLiteral("label_msckf_license"));
        label_msckf_license->setWordWrap(true);

        gridLayout_2->addWidget(label_msckf_license, 39, 2, 1, 1);

        label_dvo_license = new QLabel(scrollAreaWidgetContents);
        label_dvo_license->setObjectName(QStringLiteral("label_dvo_license"));
        label_dvo_license->setWordWrap(true);

        gridLayout_2->addWidget(label_dvo_license, 35, 2, 1, 1);

        label_sptorch = new QLabel(scrollAreaWidgetContents);
        label_sptorch->setObjectName(QStringLiteral("label_sptorch"));
        label_sptorch->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_sptorch->setWordWrap(true);

        gridLayout_2->addWidget(label_sptorch, 6, 1, 1, 1);

        label_44 = new QLabel(scrollAreaWidgetContents);
        label_44->setObjectName(QStringLiteral("label_44"));
        label_44->setWordWrap(true);

        gridLayout_2->addWidget(label_44, 17, 0, 1, 1);

        label_43 = new QLabel(scrollAreaWidgetContents);
        label_43->setObjectName(QStringLiteral("label_43"));
        label_43->setWordWrap(true);

        gridLayout_2->addWidget(label_43, 7, 0, 1, 1);

        label_69 = new QLabel(scrollAreaWidgetContents);
        label_69->setObjectName(QStringLiteral("label_69"));
        label_69->setWordWrap(true);

        gridLayout_2->addWidget(label_69, 20, 0, 1, 1);

        label_23 = new QLabel(scrollAreaWidgetContents);
        label_23->setObjectName(QStringLiteral("label_23"));
        label_23->setWordWrap(true);

        gridLayout_2->addWidget(label_23, 3, 0, 1, 1);

        label_octomap = new QLabel(scrollAreaWidgetContents);
        label_octomap->setObjectName(QStringLiteral("label_octomap"));
        label_octomap->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_octomap->setWordWrap(true);

        gridLayout_2->addWidget(label_octomap, 29, 1, 1, 1);

        label_flycapture2 = new QLabel(scrollAreaWidgetContents);
        label_flycapture2->setObjectName(QStringLiteral("label_flycapture2"));
        label_flycapture2->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_flycapture2->setWordWrap(true);

        gridLayout_2->addWidget(label_flycapture2, 15, 1, 1, 1);

        label_aliceVision_license = new QLabel(scrollAreaWidgetContents);
        label_aliceVision_license->setObjectName(QStringLiteral("label_aliceVision_license"));
        label_aliceVision_license->setWordWrap(true);

        gridLayout_2->addWidget(label_aliceVision_license, 32, 2, 1, 1);

        label_25 = new QLabel(scrollAreaWidgetContents);
        label_25->setObjectName(QStringLiteral("label_25"));
        label_25->setWordWrap(true);

        gridLayout_2->addWidget(label_25, 33, 0, 1, 1);

        label_sptorch_license = new QLabel(scrollAreaWidgetContents);
        label_sptorch_license->setObjectName(QStringLiteral("label_sptorch_license"));
        label_sptorch_license->setWordWrap(true);

        gridLayout_2->addWidget(label_sptorch_license, 6, 2, 1, 1);

        label_fastcv = new QLabel(scrollAreaWidgetContents);
        label_fastcv->setObjectName(QStringLiteral("label_fastcv"));
        label_fastcv->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_fastcv->setWordWrap(true);

        gridLayout_2->addWidget(label_fastcv, 8, 1, 1, 1);

        label_21 = new QLabel(scrollAreaWidgetContents);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setWordWrap(true);

        gridLayout_2->addWidget(label_21, 16, 0, 1, 1);

        label_dvo = new QLabel(scrollAreaWidgetContents);
        label_dvo->setObjectName(QStringLiteral("label_dvo"));
        label_dvo->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_dvo->setWordWrap(true);

        gridLayout_2->addWidget(label_dvo, 35, 1, 1, 1);

        label_opencv_license = new QLabel(scrollAreaWidgetContents);
        label_opencv_license->setObjectName(QStringLiteral("label_opencv_license"));
        label_opencv_license->setWordWrap(true);

        gridLayout_2->addWidget(label_opencv_license, 1, 2, 1, 1);

        label_7 = new QLabel(scrollAreaWidgetContents);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setWordWrap(true);

        gridLayout_2->addWidget(label_7, 1, 0, 1, 1);

        label_ceres_license = new QLabel(scrollAreaWidgetContents);
        label_ceres_license->setObjectName(QStringLiteral("label_ceres_license"));
        label_ceres_license->setWordWrap(true);

        gridLayout_2->addWidget(label_ceres_license, 26, 2, 1, 1);

        label_79 = new QLabel(scrollAreaWidgetContents);
        label_79->setObjectName(QStringLiteral("label_79"));
        label_79->setWordWrap(true);

        gridLayout_2->addWidget(label_79, 4, 0, 1, 1);

        label_octomap_license = new QLabel(scrollAreaWidgetContents);
        label_octomap_license->setObjectName(QStringLiteral("label_octomap_license"));
        label_octomap_license->setWordWrap(true);

        gridLayout_2->addWidget(label_octomap_license, 29, 2, 1, 1);

        label_dc1394_license = new QLabel(scrollAreaWidgetContents);
        label_dc1394_license->setObjectName(QStringLiteral("label_dc1394_license"));
        label_dc1394_license->setWordWrap(true);

        gridLayout_2->addWidget(label_dc1394_license, 14, 2, 1, 1);

        label_39 = new QLabel(scrollAreaWidgetContents);
        label_39->setObjectName(QStringLiteral("label_39"));
        label_39->setWordWrap(true);

        gridLayout_2->addWidget(label_39, 8, 0, 1, 1);

        label_opencv_version = new QLabel(scrollAreaWidgetContents);
        label_opencv_version->setObjectName(QStringLiteral("label_opencv_version"));
        label_opencv_version->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_opencv_version->setWordWrap(true);

        gridLayout_2->addWidget(label_opencv_version, 1, 1, 1, 1);

        label_g2o_license_3 = new QLabel(scrollAreaWidgetContents);
        label_g2o_license_3->setObjectName(QStringLiteral("label_g2o_license_3"));
        label_g2o_license_3->setWordWrap(true);

        gridLayout_2->addWidget(label_g2o_license_3, 19, 2, 1, 1);

        label_cvsba = new QLabel(scrollAreaWidgetContents);
        label_cvsba->setObjectName(QStringLiteral("label_cvsba"));
        label_cvsba->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_cvsba->setWordWrap(true);

        gridLayout_2->addWidget(label_cvsba, 25, 1, 1, 1);

        label_35 = new QLabel(scrollAreaWidgetContents);
        label_35->setObjectName(QStringLiteral("label_35"));
        label_35->setWordWrap(true);

        gridLayout_2->addWidget(label_35, 37, 0, 1, 1);

        label_k4w2 = new QLabel(scrollAreaWidgetContents);
        label_k4w2->setObjectName(QStringLiteral("label_k4w2"));
        label_k4w2->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_k4w2->setWordWrap(true);

        gridLayout_2->addWidget(label_k4w2, 18, 1, 1, 1);

        label_17 = new QLabel(scrollAreaWidgetContents);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setWordWrap(true);

        gridLayout_2->addWidget(label_17, 15, 0, 1, 1);

        label_16 = new QLabel(scrollAreaWidgetContents);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setWordWrap(true);

        gridLayout_2->addWidget(label_16, 14, 0, 1, 1);

        label_freenect_license = new QLabel(scrollAreaWidgetContents);
        label_freenect_license->setObjectName(QStringLiteral("label_freenect_license"));
        label_freenect_license->setWordWrap(true);

        gridLayout_2->addWidget(label_freenect_license, 9, 2, 1, 1);

        label_zedOC_license = new QLabel(scrollAreaWidgetContents);
        label_zedOC_license->setObjectName(QStringLiteral("label_zedOC_license"));
        label_zedOC_license->setWordWrap(true);

        gridLayout_2->addWidget(label_zedOC_license, 17, 2, 1, 1);

        label_70 = new QLabel(scrollAreaWidgetContents);
        label_70->setObjectName(QStringLiteral("label_70"));
        label_70->setWordWrap(true);

        gridLayout_2->addWidget(label_70, 21, 0, 1, 1);

        label_realsense2 = new QLabel(scrollAreaWidgetContents);
        label_realsense2->setObjectName(QStringLiteral("label_realsense2"));
        label_realsense2->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_realsense2->setWordWrap(true);

        gridLayout_2->addWidget(label_realsense2, 13, 1, 1, 1);

        label_14 = new QLabel(scrollAreaWidgetContents);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setWordWrap(true);

        gridLayout_2->addWidget(label_14, 23, 0, 1, 1);

        label_openni2_license = new QLabel(scrollAreaWidgetContents);
        label_openni2_license->setObjectName(QStringLiteral("label_openni2_license"));
        label_openni2_license->setWordWrap(true);

        gridLayout_2->addWidget(label_openni2_license, 10, 2, 1, 1);

        label_toro = new QLabel(scrollAreaWidgetContents);
        label_toro->setObjectName(QStringLiteral("label_toro"));
        label_toro->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_toro->setWordWrap(true);

        gridLayout_2->addWidget(label_toro, 22, 1, 1, 1);

        label_loam_license = new QLabel(scrollAreaWidgetContents);
        label_loam_license->setObjectName(QStringLiteral("label_loam_license"));
        label_loam_license->setWordWrap(true);

        gridLayout_2->addWidget(label_loam_license, 38, 2, 1, 1);

        label_realsense_license = new QLabel(scrollAreaWidgetContents);
        label_realsense_license->setObjectName(QStringLiteral("label_realsense_license"));
        label_realsense_license->setWordWrap(true);

        gridLayout_2->addWidget(label_realsense_license, 12, 2, 1, 1);

        label_viso2 = new QLabel(scrollAreaWidgetContents);
        label_viso2->setObjectName(QStringLiteral("label_viso2"));
        label_viso2->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_viso2->setWordWrap(true);

        gridLayout_2->addWidget(label_viso2, 34, 1, 1, 1);

        label_68 = new QLabel(scrollAreaWidgetContents);
        label_68->setObjectName(QStringLiteral("label_68"));
        label_68->setWordWrap(true);

        gridLayout_2->addWidget(label_68, 19, 0, 1, 1);

        label_orboctree = new QLabel(scrollAreaWidgetContents);
        label_orboctree->setObjectName(QStringLiteral("label_orboctree"));
        label_orboctree->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_orboctree->setWordWrap(true);

        gridLayout_2->addWidget(label_orboctree, 5, 1, 1, 1);

        label_24 = new QLabel(scrollAreaWidgetContents);
        label_24->setObjectName(QStringLiteral("label_24"));
        label_24->setWordWrap(true);

        gridLayout_2->addWidget(label_24, 30, 0, 1, 1);

        label_18 = new QLabel(scrollAreaWidgetContents);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setWordWrap(true);

        gridLayout_2->addWidget(label_18, 25, 0, 1, 1);

        label_loam = new QLabel(scrollAreaWidgetContents);
        label_loam->setObjectName(QStringLiteral("label_loam"));
        label_loam->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_loam->setWordWrap(true);

        gridLayout_2->addWidget(label_loam, 38, 1, 1, 1);

        label_cputsdf = new QLabel(scrollAreaWidgetContents);
        label_cputsdf->setObjectName(QStringLiteral("label_cputsdf"));
        label_cputsdf->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_cputsdf->setWordWrap(true);

        gridLayout_2->addWidget(label_cputsdf, 30, 1, 1, 1);

        label_libpointmatcher = new QLabel(scrollAreaWidgetContents);
        label_libpointmatcher->setObjectName(QStringLiteral("label_libpointmatcher"));
        label_libpointmatcher->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_libpointmatcher->setWordWrap(true);

        gridLayout_2->addWidget(label_libpointmatcher, 27, 1, 1, 1);

        label_30 = new QLabel(scrollAreaWidgetContents);
        label_30->setObjectName(QStringLiteral("label_30"));
        label_30->setWordWrap(true);

        gridLayout_2->addWidget(label_30, 31, 0, 1, 1);

        label_gtsam_license = new QLabel(scrollAreaWidgetContents);
        label_gtsam_license->setObjectName(QStringLiteral("label_gtsam_license"));
        label_gtsam_license->setWordWrap(true);

        gridLayout_2->addWidget(label_gtsam_license, 24, 2, 1, 1);

        label_qt_license = new QLabel(scrollAreaWidgetContents);
        label_qt_license->setObjectName(QStringLiteral("label_qt_license"));
        label_qt_license->setWordWrap(true);

        gridLayout_2->addWidget(label_qt_license, 4, 2, 1, 1);

        label_13 = new QLabel(scrollAreaWidgetContents);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setWordWrap(true);

        gridLayout_2->addWidget(label_13, 10, 0, 1, 1);

        label_depthai = new QLabel(scrollAreaWidgetContents);
        label_depthai->setObjectName(QStringLiteral("label_depthai"));
        label_depthai->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_depthai->setWordWrap(true);

        gridLayout_2->addWidget(label_depthai, 21, 1, 1, 1);

        label_realsense = new QLabel(scrollAreaWidgetContents);
        label_realsense->setObjectName(QStringLiteral("label_realsense"));
        label_realsense->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_realsense->setWordWrap(true);

        gridLayout_2->addWidget(label_realsense, 12, 1, 1, 1);

        label_37 = new QLabel(scrollAreaWidgetContents);
        label_37->setObjectName(QStringLiteral("label_37"));
        label_37->setWordWrap(true);

        gridLayout_2->addWidget(label_37, 5, 0, 1, 1);

        label_cputsdf_license = new QLabel(scrollAreaWidgetContents);
        label_cputsdf_license->setObjectName(QStringLiteral("label_cputsdf_license"));
        label_cputsdf_license->setWordWrap(true);

        gridLayout_2->addWidget(label_cputsdf_license, 30, 2, 1, 1);

        label_openchisel = new QLabel(scrollAreaWidgetContents);
        label_openchisel->setObjectName(QStringLiteral("label_openchisel"));
        label_openchisel->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_openchisel->setWordWrap(true);

        gridLayout_2->addWidget(label_openchisel, 31, 1, 1, 1);

        label_zedOC = new QLabel(scrollAreaWidgetContents);
        label_zedOC->setObjectName(QStringLiteral("label_zedOC"));
        label_zedOC->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_zedOC->setWordWrap(true);

        gridLayout_2->addWidget(label_zedOC, 17, 1, 1, 1);

        label_36 = new QLabel(scrollAreaWidgetContents);
        label_36->setObjectName(QStringLiteral("label_36"));
        label_36->setWordWrap(true);

        gridLayout_2->addWidget(label_36, 38, 0, 1, 1);

        label_fovis = new QLabel(scrollAreaWidgetContents);
        label_fovis->setObjectName(QStringLiteral("label_fovis"));
        label_fovis->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_fovis->setWordWrap(true);

        gridLayout_2->addWidget(label_fovis, 33, 1, 1, 1);

        label_fovis_license = new QLabel(scrollAreaWidgetContents);
        label_fovis_license->setObjectName(QStringLiteral("label_fovis_license"));
        label_fovis_license->setWordWrap(true);

        gridLayout_2->addWidget(label_fovis_license, 33, 2, 1, 1);

        label_pymatcher = new QLabel(scrollAreaWidgetContents);
        label_pymatcher->setObjectName(QStringLiteral("label_pymatcher"));
        label_pymatcher->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_pymatcher->setWordWrap(true);

        gridLayout_2->addWidget(label_pymatcher, 7, 1, 1, 1);

        label_33 = new QLabel(scrollAreaWidgetContents);
        label_33->setObjectName(QStringLiteral("label_33"));
        label_33->setWordWrap(true);

        gridLayout_2->addWidget(label_33, 13, 0, 1, 1);

        label_openni2 = new QLabel(scrollAreaWidgetContents);
        label_openni2->setObjectName(QStringLiteral("label_openni2"));
        label_openni2->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_openni2->setWordWrap(true);

        gridLayout_2->addWidget(label_openni2, 10, 1, 1, 1);

        label_cccorelib_license = new QLabel(scrollAreaWidgetContents);
        label_cccorelib_license->setObjectName(QStringLiteral("label_cccorelib_license"));
        label_cccorelib_license->setWordWrap(true);

        gridLayout_2->addWidget(label_cccorelib_license, 28, 2, 1, 1);

        label_cccorelib = new QLabel(scrollAreaWidgetContents);
        label_cccorelib->setObjectName(QStringLiteral("label_cccorelib"));
        label_cccorelib->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        label_cccorelib->setWordWrap(true);

        gridLayout_2->addWidget(label_cccorelib, 28, 1, 1, 1);

        label_45 = new QLabel(scrollAreaWidgetContents);
        label_45->setObjectName(QStringLiteral("label_45"));
        label_45->setWordWrap(true);

        gridLayout_2->addWidget(label_45, 28, 0, 1, 1);

        gridLayout_2->setColumnStretch(2, 1);

        verticalLayout_2->addLayout(gridLayout_2);

        scrollArea->setWidget(scrollAreaWidgetContents);

        verticalLayout->addWidget(scrollArea);

        verticalSpacer_3 = new QSpacerItem(20, 1, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_3);

        label_3 = new QLabel(frame);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setPixmap(QPixmap(QString::fromUtf8(":/images/IntRoLabSmall.png")));
        label_3->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(label_3);

        label_5 = new QLabel(frame);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(label_5);


        horizontalLayout->addWidget(frame);


        verticalLayout_3->addLayout(horizontalLayout);

        buttonBox = new QDialogButtonBox(aboutDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Close);

        verticalLayout_3->addWidget(buttonBox);


        retranslateUi(aboutDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), aboutDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), aboutDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(aboutDialog);
    } // setupUi

    void retranslateUi(QDialog *aboutDialog)
    {
        aboutDialog->setWindowTitle(QApplication::translate("aboutDialog", "About RTAB-Map", Q_NULLPTR));
        label_10->setText(QString());
        label->setText(QApplication::translate("aboutDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Lucida Grande'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'MS Shell Dlg 2'; font-size:14pt; font-weight:600;\">RTAB-Map : Real-Time Appearance-Based Mapping</span></p></body></html>", Q_NULLPTR));
        label_9->setText(QApplication::translate("aboutDialog", "RTAB-Map (Real-Time Appearance-Based Mapping) is a RGB-D Graph-Based SLAM approach based on an incremental appearance-based loop closure detector. RTAB-Map can be used alone with a hand-held Kinect or stereo camera for 6DoF RGB-D mapping, or on a ROS compatible robot equipped with a laser rangefinder for 3DoF and 6DoF mapping.", Q_NULLPTR));
        label_2->setText(QApplication::translate("aboutDialog", "Author :", Q_NULLPTR));
        label_8->setText(QApplication::translate("aboutDialog", "Mathieu Labb\303\251, matlabbe@gmail.com", Q_NULLPTR));
        label_4->setText(QApplication::translate("aboutDialog", "Links :", Q_NULLPTR));
        label_HomePage_2->setText(QApplication::translate("aboutDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Lucida Grande'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><a href=\"http://github.com/introlab/rtabmap\"><span style=\" text-decoration: underline; color:#0000ff;\">Source page</span></a></p></body></html>", Q_NULLPTR));
        label_HomePage_3->setText(QApplication::translate("aboutDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Lucida Grande'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><a href=\"http://introlab.github.io/rtabmap\"><span style=\" text-decoration: underline; color:#0000ff;\">Home page</span></a></p></body></html>", Q_NULLPTR));
        label_6->setText(QApplication::translate("aboutDialog", "Version :", Q_NULLPTR));
        label_version->setText(QString());
        label_g2o->setText(QString());
        label_aliceVision->setText(QString());
        label_34->setText(QApplication::translate("aboutDialog", "With MSCKF :", Q_NULLPTR));
        label_31->setText(QApplication::translate("aboutDialog", "<html><head/><body><p><span style=\" font-weight:600;\">License</span></p></body></html>", Q_NULLPTR));
        label_ceres->setText(QString());
        label_msckf->setText(QString());
        label_realsense2_license->setText(QApplication::translate("aboutDialog", "Apache-2", Q_NULLPTR));
        label_42->setText(QApplication::translate("aboutDialog", "With SuperPoint Torch :", Q_NULLPTR));
        label_pymatcher_license->setText(QApplication::translate("aboutDialog", "PSF", Q_NULLPTR));
        label_pcl_version->setText(QString());
        label_41->setText(QApplication::translate("aboutDialog", "With AliceVision :", Q_NULLPTR));
        label_orbslam->setText(QString());
        label_29->setText(QApplication::translate("aboutDialog", "With libpointmatcher :", Q_NULLPTR));
        label_15->setText(QApplication::translate("aboutDialog", "With Freenect2 :", Q_NULLPTR));
        label_viso2_license->setText(QApplication::translate("aboutDialog", "GPLv3", Q_NULLPTR));
        label_11->setText(QApplication::translate("aboutDialog", "PCL version :", Q_NULLPTR));
        label_k4a->setText(QString());
        label_40->setText(QApplication::translate("aboutDialog", "With Ceres :", Q_NULLPTR));
        label_20->setText(QApplication::translate("aboutDialog", "With Octomap :", Q_NULLPTR));
        label_orbslam_title->setText(QApplication::translate("aboutDialog", "With ORB SLAM 2 :", Q_NULLPTR));
        label_fastcv_license->setText(QApplication::translate("aboutDialog", "Apache v2", Q_NULLPTR));
        label_depthai_license->setText(QApplication::translate("aboutDialog", "MIT", Q_NULLPTR));
        label_gtsam->setText(QString());
        label_freenect->setText(QString());
        label_dc1394->setText(QString());
        label_26->setText(QApplication::translate("aboutDialog", "With Viso2 :", Q_NULLPTR));
        label_77->setText(QApplication::translate("aboutDialog", "With TORO :", Q_NULLPTR));
        label_qt_version->setText(QString());
        label_zed->setText(QString());
        label_12->setText(QApplication::translate("aboutDialog", "With Freenect :", Q_NULLPTR));
        label_19->setText(QApplication::translate("aboutDialog", "With GTSAM :", Q_NULLPTR));
        label_cvsba_license->setText(QApplication::translate("aboutDialog", "GPLv2", Q_NULLPTR));
        label_32->setText(QApplication::translate("aboutDialog", "<html><head/><body><p><span style=\" font-weight:600;\">Third Party Libraries</span></p></body></html>", Q_NULLPTR));
        label_38->setText(QApplication::translate("aboutDialog", "With K4W2 :", Q_NULLPTR));
        label_orboctree_license->setText(QApplication::translate("aboutDialog", "GPLv3", Q_NULLPTR));
        label_okvis_license->setText(QApplication::translate("aboutDialog", "BSD", Q_NULLPTR));
        label_toro_license->setText(QApplication::translate("aboutDialog", "Creative Commons [Attribution-NonCommercial-ShareAlike]", Q_NULLPTR));
        label_pcl_license->setText(QApplication::translate("aboutDialog", "BSD", Q_NULLPTR));
        label_27->setText(QApplication::translate("aboutDialog", "With DVO :", Q_NULLPTR));
        label_g2o_license->setText(QApplication::translate("aboutDialog", "BSD", Q_NULLPTR));
        label_freenect2->setText(QString());
        label_orbslam_license->setText(QApplication::translate("aboutDialog", "GPLv3", Q_NULLPTR));
        label_vtk_version->setText(QString());
        label_libpointmatcher_license->setText(QApplication::translate("aboutDialog", "BSD", Q_NULLPTR));
        label_freenect2_license->setText(QApplication::translate("aboutDialog", "Apache v2 and/or GPLv2", Q_NULLPTR));
        label_22->setText(QApplication::translate("aboutDialog", "With RealSense :", Q_NULLPTR));
        label_vtk_license->setText(QApplication::translate("aboutDialog", "BSD", Q_NULLPTR));
        label_okvis->setText(QString());
        label_mynteye->setText(QString());
        label_msckf_license->setText(QApplication::translate("aboutDialog", "Penn Software License", Q_NULLPTR));
        label_dvo_license->setText(QApplication::translate("aboutDialog", "GPLv3", Q_NULLPTR));
        label_sptorch->setText(QString());
        label_44->setText(QApplication::translate("aboutDialog", "With Zed Open Capture :", Q_NULLPTR));
        label_43->setText(QApplication::translate("aboutDialog", "With Python3 :", Q_NULLPTR));
        label_69->setText(QApplication::translate("aboutDialog", "With MYNTEYE S :", Q_NULLPTR));
        label_23->setText(QApplication::translate("aboutDialog", "VTK version :", Q_NULLPTR));
        label_octomap->setText(QString());
        label_flycapture2->setText(QString());
        label_aliceVision_license->setText(QApplication::translate("aboutDialog", "MPL2", Q_NULLPTR));
        label_25->setText(QApplication::translate("aboutDialog", "With FOVIS :", Q_NULLPTR));
        label_sptorch_license->setText(QApplication::translate("aboutDialog", "GPLv3", Q_NULLPTR));
        label_fastcv->setText(QString());
        label_21->setText(QApplication::translate("aboutDialog", "With Zed SDK :", Q_NULLPTR));
        label_dvo->setText(QString());
        label_opencv_license->setText(QApplication::translate("aboutDialog", "BSD", Q_NULLPTR));
        label_7->setText(QApplication::translate("aboutDialog", "OpenCV version :", Q_NULLPTR));
        label_ceres_license->setText(QApplication::translate("aboutDialog", "BSD", Q_NULLPTR));
        label_79->setText(QApplication::translate("aboutDialog", "Qt version :", Q_NULLPTR));
        label_octomap_license->setText(QApplication::translate("aboutDialog", "BSD", Q_NULLPTR));
        label_dc1394_license->setText(QApplication::translate("aboutDialog", "LGPL", Q_NULLPTR));
        label_39->setText(QApplication::translate("aboutDialog", "With FastCV :", Q_NULLPTR));
        label_opencv_version->setText(QString());
        label_g2o_license_3->setText(QApplication::translate("aboutDialog", "MIT", Q_NULLPTR));
        label_cvsba->setText(QString());
        label_35->setText(QApplication::translate("aboutDialog", "With OKVIS :", Q_NULLPTR));
        label_k4w2->setText(QString());
        label_17->setText(QApplication::translate("aboutDialog", "With stereo FlyCapture2 :", Q_NULLPTR));
        label_16->setText(QApplication::translate("aboutDialog", "With stereo dc1394 :", Q_NULLPTR));
        label_freenect_license->setText(QApplication::translate("aboutDialog", "Apache v2 and/or GPLv2", Q_NULLPTR));
        label_zedOC_license->setText(QApplication::translate("aboutDialog", "MIT", Q_NULLPTR));
        label_70->setText(QApplication::translate("aboutDialog", "With DepthAI :", Q_NULLPTR));
        label_realsense2->setText(QString());
        label_14->setText(QApplication::translate("aboutDialog", "With g2o :", Q_NULLPTR));
        label_openni2_license->setText(QApplication::translate("aboutDialog", "Apache v2", Q_NULLPTR));
        label_toro->setText(QString());
        label_loam_license->setText(QApplication::translate("aboutDialog", "BSD", Q_NULLPTR));
        label_realsense_license->setText(QApplication::translate("aboutDialog", "Apache-2", Q_NULLPTR));
        label_viso2->setText(QString());
        label_68->setText(QApplication::translate("aboutDialog", "With K4A :", Q_NULLPTR));
        label_orboctree->setText(QString());
        label_24->setText(QApplication::translate("aboutDialog", "With CPU-TSDF :", Q_NULLPTR));
        label_18->setText(QApplication::translate("aboutDialog", "With cvsba :", Q_NULLPTR));
        label_loam->setText(QString());
        label_cputsdf->setText(QString());
        label_libpointmatcher->setText(QString());
        label_30->setText(QApplication::translate("aboutDialog", "With OpenChisel :", Q_NULLPTR));
        label_gtsam_license->setText(QApplication::translate("aboutDialog", "BSD", Q_NULLPTR));
        label_qt_license->setText(QApplication::translate("aboutDialog", "Open Source or Commercial", Q_NULLPTR));
        label_13->setText(QApplication::translate("aboutDialog", "With OpenNI2 :", Q_NULLPTR));
        label_depthai->setText(QString());
        label_realsense->setText(QString());
        label_37->setText(QApplication::translate("aboutDialog", "With ORB Octree :", Q_NULLPTR));
        label_cputsdf_license->setText(QApplication::translate("aboutDialog", "BSD", Q_NULLPTR));
        label_openchisel->setText(QString());
        label_zedOC->setText(QString());
        label_36->setText(QApplication::translate("aboutDialog", "With loam_velodyne :", Q_NULLPTR));
        label_fovis->setText(QString());
        label_fovis_license->setText(QApplication::translate("aboutDialog", "GPLv2", Q_NULLPTR));
        label_pymatcher->setText(QString());
        label_33->setText(QApplication::translate("aboutDialog", "With RealSense2 :", Q_NULLPTR));
        label_openni2->setText(QString());
        label_cccorelib_license->setText(QApplication::translate("aboutDialog", "GPLv2", Q_NULLPTR));
        label_cccorelib->setText(QString());
        label_45->setText(QApplication::translate("aboutDialog", "With CCCoreLib :", Q_NULLPTR));
        label_3->setText(QString());
        label_5->setText(QApplication::translate("aboutDialog", "Copyright (C) 2010-2020 IntRoLab - Universit\303\251 de Sherbrooke", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class aboutDialog: public Ui_aboutDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ABOUTDIALOG_H
