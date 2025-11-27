/********************************************************************************
** Form generated from reading UI file 'depthCalibrationDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DEPTHCALIBRATIONDIALOG_H
#define UI_DEPTHCALIBRATIONDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_DepthCalibrationDialog
{
public:
    QVBoxLayout *verticalLayout;
    QLabel *label_111;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_3;
    QCheckBox *checkBox_laserScan;
    QLabel *label_110;
    QSpinBox *spinBox_decimation;
    QLabel *label_108;
    QDoubleSpinBox *doubleSpinBox_maxDepth;
    QLabel *label_132;
    QDoubleSpinBox *doubleSpinBox_minDepth;
    QLabel *label_133;
    QDoubleSpinBox *doubleSpinBox_voxelSize;
    QLabel *label_voxel;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QSpinBox *spinBox_bin_width;
    QSpinBox *spinBox_smoothing;
    QLabel *label_113;
    QLabel *label_117;
    QLabel *label_120;
    QLabel *label_116;
    QSpinBox *spinBox_bin_height;
    QLabel *label_118;
    QLabel *label_115;
    QLabel *label_119;
    QLabel *label_width;
    QLabel *label_height;
    QDoubleSpinBox *doubleSpinBox_bin_depth;
    QDoubleSpinBox *doubleSpinBox_maxDepthModel;
    QGridLayout *gridLayout;
    QDoubleSpinBox *doubleSpinBox_coneRadius;
    QDoubleSpinBox *doubleSpinBox_coneStdDevThresh;
    QLabel *label_voxel_2;
    QLabel *label_voxel_3;
    QLabel *label_109;
    QCheckBox *checkBox_resetModel;
    QHBoxLayout *horizontalLayout;
    QLabel *label_112;
    QLabel *label_trainingSamples;
    QLabel *label_114;
    QSpacerItem *horizontalSpacer;
    QSpacerItem *verticalSpacer;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *DepthCalibrationDialog)
    {
        if (DepthCalibrationDialog->objectName().isEmpty())
            DepthCalibrationDialog->setObjectName(QStringLiteral("DepthCalibrationDialog"));
        DepthCalibrationDialog->resize(523, 781);
        verticalLayout = new QVBoxLayout(DepthCalibrationDialog);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        label_111 = new QLabel(DepthCalibrationDialog);
        label_111->setObjectName(QStringLiteral("label_111"));
        label_111->setTextFormat(Qt::RichText);
        label_111->setWordWrap(true);

        verticalLayout->addWidget(label_111);

        groupBox_2 = new QGroupBox(DepthCalibrationDialog);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        gridLayout_3 = new QGridLayout(groupBox_2);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        checkBox_laserScan = new QCheckBox(groupBox_2);
        checkBox_laserScan->setObjectName(QStringLiteral("checkBox_laserScan"));

        gridLayout_3->addWidget(checkBox_laserScan, 0, 0, 1, 1);

        label_110 = new QLabel(groupBox_2);
        label_110->setObjectName(QStringLiteral("label_110"));
        label_110->setWordWrap(true);
        label_110->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_110, 0, 1, 1, 1);

        spinBox_decimation = new QSpinBox(groupBox_2);
        spinBox_decimation->setObjectName(QStringLiteral("spinBox_decimation"));
        spinBox_decimation->setMinimum(1);
        spinBox_decimation->setMaximum(32);
        spinBox_decimation->setValue(1);

        gridLayout_3->addWidget(spinBox_decimation, 1, 0, 1, 1);

        label_108 = new QLabel(groupBox_2);
        label_108->setObjectName(QStringLiteral("label_108"));
        label_108->setWordWrap(true);
        label_108->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_108, 1, 1, 1, 1);

        doubleSpinBox_maxDepth = new QDoubleSpinBox(groupBox_2);
        doubleSpinBox_maxDepth->setObjectName(QStringLiteral("doubleSpinBox_maxDepth"));
        doubleSpinBox_maxDepth->setDecimals(1);
        doubleSpinBox_maxDepth->setMaximum(100);
        doubleSpinBox_maxDepth->setSingleStep(0.1);
        doubleSpinBox_maxDepth->setValue(4);

        gridLayout_3->addWidget(doubleSpinBox_maxDepth, 2, 0, 1, 1);

        label_132 = new QLabel(groupBox_2);
        label_132->setObjectName(QStringLiteral("label_132"));
        label_132->setWordWrap(true);
        label_132->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_132, 2, 1, 1, 1);

        doubleSpinBox_minDepth = new QDoubleSpinBox(groupBox_2);
        doubleSpinBox_minDepth->setObjectName(QStringLiteral("doubleSpinBox_minDepth"));
        doubleSpinBox_minDepth->setDecimals(1);
        doubleSpinBox_minDepth->setMaximum(100);
        doubleSpinBox_minDepth->setSingleStep(0.1);
        doubleSpinBox_minDepth->setValue(0);

        gridLayout_3->addWidget(doubleSpinBox_minDepth, 3, 0, 1, 1);

        label_133 = new QLabel(groupBox_2);
        label_133->setObjectName(QStringLiteral("label_133"));
        label_133->setWordWrap(true);
        label_133->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_133, 3, 1, 1, 1);

        doubleSpinBox_voxelSize = new QDoubleSpinBox(groupBox_2);
        doubleSpinBox_voxelSize->setObjectName(QStringLiteral("doubleSpinBox_voxelSize"));
        doubleSpinBox_voxelSize->setDecimals(3);
        doubleSpinBox_voxelSize->setMaximum(1);
        doubleSpinBox_voxelSize->setSingleStep(0.01);
        doubleSpinBox_voxelSize->setValue(0.01);

        gridLayout_3->addWidget(doubleSpinBox_voxelSize, 4, 0, 1, 1);

        label_voxel = new QLabel(groupBox_2);
        label_voxel->setObjectName(QStringLiteral("label_voxel"));
        label_voxel->setWordWrap(true);
        label_voxel->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_3->addWidget(label_voxel, 4, 1, 1, 1);

        gridLayout_3->setColumnStretch(1, 1);

        verticalLayout->addWidget(groupBox_2);

        groupBox = new QGroupBox(DepthCalibrationDialog);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        spinBox_bin_width = new QSpinBox(groupBox);
        spinBox_bin_width->setObjectName(QStringLiteral("spinBox_bin_width"));
        spinBox_bin_width->setMinimum(1);
        spinBox_bin_width->setMaximum(999);
        spinBox_bin_width->setValue(8);

        gridLayout_2->addWidget(spinBox_bin_width, 2, 0, 1, 1);

        spinBox_smoothing = new QSpinBox(groupBox);
        spinBox_smoothing->setObjectName(QStringLiteral("spinBox_smoothing"));
        spinBox_smoothing->setMinimum(1);
        spinBox_smoothing->setMaximum(32);
        spinBox_smoothing->setValue(1);

        gridLayout_2->addWidget(spinBox_smoothing, 6, 0, 1, 1);

        label_113 = new QLabel(groupBox);
        label_113->setObjectName(QStringLiteral("label_113"));
        label_113->setWordWrap(true);
        label_113->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_113, 2, 1, 1, 1);

        label_117 = new QLabel(groupBox);
        label_117->setObjectName(QStringLiteral("label_117"));
        label_117->setWordWrap(true);
        label_117->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_117, 4, 1, 1, 1);

        label_120 = new QLabel(groupBox);
        label_120->setObjectName(QStringLiteral("label_120"));
        label_120->setWordWrap(true);
        label_120->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_120, 1, 1, 1, 1);

        label_116 = new QLabel(groupBox);
        label_116->setObjectName(QStringLiteral("label_116"));
        label_116->setWordWrap(true);
        label_116->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_116, 3, 1, 1, 1);

        spinBox_bin_height = new QSpinBox(groupBox);
        spinBox_bin_height->setObjectName(QStringLiteral("spinBox_bin_height"));
        spinBox_bin_height->setMinimum(1);
        spinBox_bin_height->setMaximum(999);
        spinBox_bin_height->setValue(6);

        gridLayout_2->addWidget(spinBox_bin_height, 3, 0, 1, 1);

        label_118 = new QLabel(groupBox);
        label_118->setObjectName(QStringLiteral("label_118"));
        label_118->setWordWrap(true);
        label_118->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_118, 5, 1, 1, 1);

        label_115 = new QLabel(groupBox);
        label_115->setObjectName(QStringLiteral("label_115"));
        label_115->setWordWrap(true);
        label_115->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_115, 6, 1, 1, 1);

        label_119 = new QLabel(groupBox);
        label_119->setObjectName(QStringLiteral("label_119"));
        label_119->setWordWrap(true);
        label_119->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_119, 0, 1, 1, 1);

        label_width = new QLabel(groupBox);
        label_width->setObjectName(QStringLiteral("label_width"));
        label_width->setWordWrap(true);
        label_width->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_width, 0, 0, 1, 1);

        label_height = new QLabel(groupBox);
        label_height->setObjectName(QStringLiteral("label_height"));
        label_height->setWordWrap(true);
        label_height->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_height, 1, 0, 1, 1);

        doubleSpinBox_bin_depth = new QDoubleSpinBox(groupBox);
        doubleSpinBox_bin_depth->setObjectName(QStringLiteral("doubleSpinBox_bin_depth"));
        doubleSpinBox_bin_depth->setDecimals(1);
        doubleSpinBox_bin_depth->setMinimum(0.1);
        doubleSpinBox_bin_depth->setMaximum(10);
        doubleSpinBox_bin_depth->setSingleStep(1);
        doubleSpinBox_bin_depth->setValue(2);

        gridLayout_2->addWidget(doubleSpinBox_bin_depth, 4, 0, 1, 1);

        doubleSpinBox_maxDepthModel = new QDoubleSpinBox(groupBox);
        doubleSpinBox_maxDepthModel->setObjectName(QStringLiteral("doubleSpinBox_maxDepthModel"));
        doubleSpinBox_maxDepthModel->setDecimals(0);
        doubleSpinBox_maxDepthModel->setMinimum(1);
        doubleSpinBox_maxDepthModel->setMaximum(100);
        doubleSpinBox_maxDepthModel->setSingleStep(1);
        doubleSpinBox_maxDepthModel->setValue(10);

        gridLayout_2->addWidget(doubleSpinBox_maxDepthModel, 5, 0, 1, 1);

        gridLayout_2->setColumnStretch(1, 1);

        verticalLayout->addWidget(groupBox);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        doubleSpinBox_coneRadius = new QDoubleSpinBox(DepthCalibrationDialog);
        doubleSpinBox_coneRadius->setObjectName(QStringLiteral("doubleSpinBox_coneRadius"));
        doubleSpinBox_coneRadius->setDecimals(2);
        doubleSpinBox_coneRadius->setMinimum(0.01);
        doubleSpinBox_coneRadius->setMaximum(1);
        doubleSpinBox_coneRadius->setSingleStep(0.01);
        doubleSpinBox_coneRadius->setValue(0.02);

        gridLayout->addWidget(doubleSpinBox_coneRadius, 2, 0, 1, 1);

        doubleSpinBox_coneStdDevThresh = new QDoubleSpinBox(DepthCalibrationDialog);
        doubleSpinBox_coneStdDevThresh->setObjectName(QStringLiteral("doubleSpinBox_coneStdDevThresh"));
        doubleSpinBox_coneStdDevThresh->setDecimals(2);
        doubleSpinBox_coneStdDevThresh->setMinimum(0.01);
        doubleSpinBox_coneStdDevThresh->setMaximum(1);
        doubleSpinBox_coneStdDevThresh->setSingleStep(0.01);
        doubleSpinBox_coneStdDevThresh->setValue(0.03);

        gridLayout->addWidget(doubleSpinBox_coneStdDevThresh, 3, 0, 1, 1);

        label_voxel_2 = new QLabel(DepthCalibrationDialog);
        label_voxel_2->setObjectName(QStringLiteral("label_voxel_2"));
        label_voxel_2->setWordWrap(true);
        label_voxel_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout->addWidget(label_voxel_2, 2, 1, 1, 1);

        label_voxel_3 = new QLabel(DepthCalibrationDialog);
        label_voxel_3->setObjectName(QStringLiteral("label_voxel_3"));
        label_voxel_3->setWordWrap(true);
        label_voxel_3->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout->addWidget(label_voxel_3, 3, 1, 1, 1);

        label_109 = new QLabel(DepthCalibrationDialog);
        label_109->setObjectName(QStringLiteral("label_109"));
        label_109->setWordWrap(true);
        label_109->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout->addWidget(label_109, 4, 1, 1, 1);

        checkBox_resetModel = new QCheckBox(DepthCalibrationDialog);
        checkBox_resetModel->setObjectName(QStringLiteral("checkBox_resetModel"));

        gridLayout->addWidget(checkBox_resetModel, 4, 0, 1, 1);

        gridLayout->setColumnStretch(1, 1);

        verticalLayout->addLayout(gridLayout);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label_112 = new QLabel(DepthCalibrationDialog);
        label_112->setObjectName(QStringLiteral("label_112"));
        label_112->setWordWrap(true);
        label_112->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        horizontalLayout->addWidget(label_112);

        label_trainingSamples = new QLabel(DepthCalibrationDialog);
        label_trainingSamples->setObjectName(QStringLiteral("label_trainingSamples"));
        label_trainingSamples->setWordWrap(true);
        label_trainingSamples->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        horizontalLayout->addWidget(label_trainingSamples);

        label_114 = new QLabel(DepthCalibrationDialog);
        label_114->setObjectName(QStringLiteral("label_114"));
        label_114->setWordWrap(true);
        label_114->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        horizontalLayout->addWidget(label_114);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);


        verticalLayout->addLayout(horizontalLayout);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        buttonBox = new QDialogButtonBox(DepthCalibrationDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok|QDialogButtonBox::RestoreDefaults|QDialogButtonBox::Save);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(DepthCalibrationDialog);
        QObject::connect(buttonBox, SIGNAL(rejected()), DepthCalibrationDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(DepthCalibrationDialog);
    } // setupUi

    void retranslateUi(QDialog *DepthCalibrationDialog)
    {
        DepthCalibrationDialog->setWindowTitle(QApplication::translate("DepthCalibrationDialog", "Depth Calibration", Q_NULLPTR));
        label_111->setText(QApplication::translate("DepthCalibrationDialog", "<html><head/><body><p>CLAMS approach is used for depth calibration. Please visit <a href=\"http://www.alexteichman.com/octo/clams/\"><span style=\" text-decoration: underline; color:#0000ff;\">CLAMS website</span></a> for tips about how to get a good map for depth calibration. If you want to process multiple mapping sessions, uncheck &quot;Reset previous model&quot;. If logger's level is debug, 3D map and generated depth images will be shown during the calibration.</p></body></html>", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("DepthCalibrationDialog", "Map generation", Q_NULLPTR));
        checkBox_laserScan->setText(QString());
        label_110->setText(QApplication::translate("DepthCalibrationDialog", "Use 3D laser scans for 3D map.", Q_NULLPTR));
        label_108->setText(QApplication::translate("DepthCalibrationDialog", "Decimation (1-2-4-8-...).", Q_NULLPTR));
        doubleSpinBox_maxDepth->setSuffix(QApplication::translate("DepthCalibrationDialog", " m", Q_NULLPTR));
        label_132->setText(QApplication::translate("DepthCalibrationDialog", "Maximum depth (0 means no limit).", Q_NULLPTR));
        doubleSpinBox_minDepth->setSuffix(QApplication::translate("DepthCalibrationDialog", " m", Q_NULLPTR));
        label_133->setText(QApplication::translate("DepthCalibrationDialog", "Minimum depth.", Q_NULLPTR));
        doubleSpinBox_voxelSize->setSuffix(QApplication::translate("DepthCalibrationDialog", " m", Q_NULLPTR));
        label_voxel->setText(QApplication::translate("DepthCalibrationDialog", "Voxel size. Set 0 to disable.", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("DepthCalibrationDialog", "Distortion Model", Q_NULLPTR));
        spinBox_bin_width->setSuffix(QApplication::translate("DepthCalibrationDialog", " pix", Q_NULLPTR));
        label_113->setText(QApplication::translate("DepthCalibrationDialog", "Bin width. Should be a multiple of image width.", Q_NULLPTR));
        label_117->setText(QApplication::translate("DepthCalibrationDialog", "Bin depth.", Q_NULLPTR));
        label_120->setText(QApplication::translate("DepthCalibrationDialog", "Image height.", Q_NULLPTR));
        label_116->setText(QApplication::translate("DepthCalibrationDialog", "Bin height. Should be a multiple of image height.", Q_NULLPTR));
        spinBox_bin_height->setSuffix(QApplication::translate("DepthCalibrationDialog", " pix", Q_NULLPTR));
        label_118->setText(QApplication::translate("DepthCalibrationDialog", "Maximum depth.", Q_NULLPTR));
        label_115->setText(QApplication::translate("DepthCalibrationDialog", "Smoothing.", Q_NULLPTR));
        label_119->setText(QApplication::translate("DepthCalibrationDialog", "Image width.", Q_NULLPTR));
        label_width->setText(QApplication::translate("DepthCalibrationDialog", "NA", Q_NULLPTR));
        label_height->setText(QApplication::translate("DepthCalibrationDialog", "NA", Q_NULLPTR));
        doubleSpinBox_bin_depth->setSuffix(QApplication::translate("DepthCalibrationDialog", " m", Q_NULLPTR));
        doubleSpinBox_maxDepthModel->setSuffix(QApplication::translate("DepthCalibrationDialog", " m", Q_NULLPTR));
        doubleSpinBox_coneRadius->setSuffix(QApplication::translate("DepthCalibrationDialog", " m", Q_NULLPTR));
        doubleSpinBox_coneStdDevThresh->setSuffix(QApplication::translate("DepthCalibrationDialog", " m", Q_NULLPTR));
        label_voxel_2->setText(QApplication::translate("DepthCalibrationDialog", "Cone radius.", Q_NULLPTR));
        label_voxel_3->setText(QApplication::translate("DepthCalibrationDialog", "Cone standard deviation threshold.", Q_NULLPTR));
        label_109->setText(QApplication::translate("DepthCalibrationDialog", "Reset previous model.", Q_NULLPTR));
        checkBox_resetModel->setText(QString());
        label_112->setText(QApplication::translate("DepthCalibrationDialog", "Current model:", Q_NULLPTR));
        label_trainingSamples->setText(QApplication::translate("DepthCalibrationDialog", "0", Q_NULLPTR));
        label_114->setText(QApplication::translate("DepthCalibrationDialog", "training samples", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class DepthCalibrationDialog: public Ui_DepthCalibrationDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DEPTHCALIBRATIONDIALOG_H
