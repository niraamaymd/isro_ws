/********************************************************************************
** Form generated from reading UI file 'calibrationDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CALIBRATIONDIALOG_H
#define UI_CALIBRATIONDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
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
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "rtabmap/utilite/UImageView.h"

QT_BEGIN_NAMESPACE

class Ui_calibrationDialog
{
public:
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_left;
    UImageView *image_view;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_right;
    UImageView *image_view_2;
    QHBoxLayout *horizontalLayout_2;
    QRadioButton *radioButton_raw;
    QRadioButton *radioButton_rectified;
    QRadioButton *radioButton_stereoRectified;
    QFrame *line;
    QCheckBox *checkBox_mirror;
    QCheckBox *checkBox_showHorizontalLines;
    QSpacerItem *horizontalSpacer;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QVBoxLayout *verticalLayout;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_3;
    QSpinBox *spinBox_boardHeight;
    QLabel *label_15;
    QLabel *label_12;
    QLabel *label_16;
    QSpinBox *spinBox_boardWidth;
    QDoubleSpinBox *doubleSpinBox_squareSize;
    QLabel *label_21;
    QLabel *label_20;
    QSpinBox *spinBox_maxScale;
    QLabel *label_22;
    QLabel *label_stereoBaseline;
    QDoubleSpinBox *doubleSpinBox_stereoBaseline;
    QGroupBox *groupBox_progress;
    QVBoxLayout *verticalLayout_6;
    QGridLayout *gridLayout;
    QLabel *label_4;
    QLabel *label_19;
    QLabel *label_serial;
    QCheckBox *checkBox_switchImages;
    QLabel *label_5;
    QProgressBar *progressBar_count;
    QProgressBar *progressBar_count_2;
    QLabel *label;
    QProgressBar *progressBar_x;
    QProgressBar *progressBar_x_2;
    QLabel *label_2;
    QProgressBar *progressBar_y;
    QProgressBar *progressBar_y_2;
    QLabel *label_3;
    QProgressBar *progressBar_size;
    QProgressBar *progressBar_size_2;
    QProgressBar *progressBar_skew;
    QProgressBar *progressBar_skew_2;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *pushButton_calibrate;
    QCheckBox *checkBox_fisheye;
    QCheckBox *checkBox_unlock;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout_2;
    QLineEdit *lineEdit_P_2;
    QLabel *label_fx;
    QLabel *label_fy;
    QLineEdit *lineEdit_K;
    QLabel *label_cx;
    QLabel *label_11;
    QLabel *label_cy;
    QLabel *label_6;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_8;
    QLabel *label_17;
    QLineEdit *lineEdit_D_2;
    QLabel *label_7;
    QLineEdit *lineEdit_D;
    QLabel *label_14;
    QLabel *label_error;
    QLabel *label_fx_2;
    QLabel *label_cx_2;
    QLabel *label_fy_2;
    QLabel *label_cy_2;
    QLineEdit *lineEdit_K_2;
    QLabel *label_18;
    QLineEdit *lineEdit_R;
    QLineEdit *lineEdit_P;
    QLineEdit *lineEdit_R_2;
    QLabel *label_baseline_name;
    QLabel *label_baseline;
    QLabel *label_error_2;
    QPushButton *pushButton_save;
    QPushButton *pushButton_restart;
    QSpacerItem *verticalSpacer;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *calibrationDialog)
    {
        if (calibrationDialog->objectName().isEmpty())
            calibrationDialog->setObjectName(QStringLiteral("calibrationDialog"));
        calibrationDialog->resize(1314, 847);
        verticalLayout_5 = new QVBoxLayout(calibrationDialog);
        verticalLayout_5->setSpacing(0);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(0);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(0);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(0);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        label_left = new QLabel(calibrationDialog);
        label_left->setObjectName(QStringLiteral("label_left"));
        label_left->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(label_left);

        image_view = new UImageView(calibrationDialog);
        image_view->setObjectName(QStringLiteral("image_view"));

        verticalLayout_2->addWidget(image_view);

        verticalLayout_2->setStretch(1, 1);

        horizontalLayout_3->addLayout(verticalLayout_2);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(0);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        label_right = new QLabel(calibrationDialog);
        label_right->setObjectName(QStringLiteral("label_right"));
        label_right->setAlignment(Qt::AlignCenter);

        verticalLayout_3->addWidget(label_right);

        image_view_2 = new UImageView(calibrationDialog);
        image_view_2->setObjectName(QStringLiteral("image_view_2"));

        verticalLayout_3->addWidget(image_view_2);

        verticalLayout_3->setStretch(1, 1);

        horizontalLayout_3->addLayout(verticalLayout_3);


        verticalLayout_4->addLayout(horizontalLayout_3);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        radioButton_raw = new QRadioButton(calibrationDialog);
        radioButton_raw->setObjectName(QStringLiteral("radioButton_raw"));

        horizontalLayout_2->addWidget(radioButton_raw);

        radioButton_rectified = new QRadioButton(calibrationDialog);
        radioButton_rectified->setObjectName(QStringLiteral("radioButton_rectified"));

        horizontalLayout_2->addWidget(radioButton_rectified);

        radioButton_stereoRectified = new QRadioButton(calibrationDialog);
        radioButton_stereoRectified->setObjectName(QStringLiteral("radioButton_stereoRectified"));

        horizontalLayout_2->addWidget(radioButton_stereoRectified);

        line = new QFrame(calibrationDialog);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);

        horizontalLayout_2->addWidget(line);

        checkBox_mirror = new QCheckBox(calibrationDialog);
        checkBox_mirror->setObjectName(QStringLiteral("checkBox_mirror"));

        horizontalLayout_2->addWidget(checkBox_mirror);

        checkBox_showHorizontalLines = new QCheckBox(calibrationDialog);
        checkBox_showHorizontalLines->setObjectName(QStringLiteral("checkBox_showHorizontalLines"));

        horizontalLayout_2->addWidget(checkBox_showHorizontalLines);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);


        verticalLayout_4->addLayout(horizontalLayout_2);


        horizontalLayout_4->addLayout(verticalLayout_4);

        scrollArea = new QScrollArea(calibrationDialog);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 387, 826));
        verticalLayout = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        groupBox = new QGroupBox(scrollAreaWidgetContents);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        gridLayout_3 = new QGridLayout(groupBox);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        spinBox_boardHeight = new QSpinBox(groupBox);
        spinBox_boardHeight->setObjectName(QStringLiteral("spinBox_boardHeight"));
        spinBox_boardHeight->setMinimum(2);
        spinBox_boardHeight->setValue(6);

        gridLayout_3->addWidget(spinBox_boardHeight, 1, 3, 1, 1);

        label_15 = new QLabel(groupBox);
        label_15->setObjectName(QStringLiteral("label_15"));

        gridLayout_3->addWidget(label_15, 3, 0, 1, 1);

        label_12 = new QLabel(groupBox);
        label_12->setObjectName(QStringLiteral("label_12"));

        gridLayout_3->addWidget(label_12, 1, 0, 1, 1);

        label_16 = new QLabel(groupBox);
        label_16->setObjectName(QStringLiteral("label_16"));

        gridLayout_3->addWidget(label_16, 3, 3, 1, 1);

        spinBox_boardWidth = new QSpinBox(groupBox);
        spinBox_boardWidth->setObjectName(QStringLiteral("spinBox_boardWidth"));
        spinBox_boardWidth->setMinimum(2);
        spinBox_boardWidth->setValue(8);

        gridLayout_3->addWidget(spinBox_boardWidth, 1, 1, 1, 1);

        doubleSpinBox_squareSize = new QDoubleSpinBox(groupBox);
        doubleSpinBox_squareSize->setObjectName(QStringLiteral("doubleSpinBox_squareSize"));
        doubleSpinBox_squareSize->setDecimals(4);
        doubleSpinBox_squareSize->setMaximum(1);
        doubleSpinBox_squareSize->setValue(0.033);

        gridLayout_3->addWidget(doubleSpinBox_squareSize, 3, 1, 1, 1);

        label_21 = new QLabel(groupBox);
        label_21->setObjectName(QStringLiteral("label_21"));

        gridLayout_3->addWidget(label_21, 0, 0, 1, 1);

        label_20 = new QLabel(groupBox);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setOpenExternalLinks(true);

        gridLayout_3->addWidget(label_20, 0, 1, 1, 1);

        spinBox_maxScale = new QSpinBox(groupBox);
        spinBox_maxScale->setObjectName(QStringLiteral("spinBox_maxScale"));
        spinBox_maxScale->setMinimum(1);
        spinBox_maxScale->setMaximum(3);
        spinBox_maxScale->setValue(1);

        gridLayout_3->addWidget(spinBox_maxScale, 4, 1, 1, 1);

        label_22 = new QLabel(groupBox);
        label_22->setObjectName(QStringLiteral("label_22"));

        gridLayout_3->addWidget(label_22, 4, 0, 1, 1);

        label_stereoBaseline = new QLabel(groupBox);
        label_stereoBaseline->setObjectName(QStringLiteral("label_stereoBaseline"));

        gridLayout_3->addWidget(label_stereoBaseline, 5, 0, 1, 1);

        doubleSpinBox_stereoBaseline = new QDoubleSpinBox(groupBox);
        doubleSpinBox_stereoBaseline->setObjectName(QStringLiteral("doubleSpinBox_stereoBaseline"));
        doubleSpinBox_stereoBaseline->setDecimals(4);
        doubleSpinBox_stereoBaseline->setMaximum(10);
        doubleSpinBox_stereoBaseline->setValue(0);

        gridLayout_3->addWidget(doubleSpinBox_stereoBaseline, 5, 1, 1, 1);


        verticalLayout->addWidget(groupBox);

        groupBox_progress = new QGroupBox(scrollAreaWidgetContents);
        groupBox_progress->setObjectName(QStringLiteral("groupBox_progress"));
        verticalLayout_6 = new QVBoxLayout(groupBox_progress);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        label_4 = new QLabel(groupBox_progress);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 5, 0, 1, 1);

        label_19 = new QLabel(groupBox_progress);
        label_19->setObjectName(QStringLiteral("label_19"));

        gridLayout->addWidget(label_19, 0, 0, 1, 1);

        label_serial = new QLabel(groupBox_progress);
        label_serial->setObjectName(QStringLiteral("label_serial"));
        label_serial->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout->addWidget(label_serial, 0, 1, 1, 1);

        checkBox_switchImages = new QCheckBox(groupBox_progress);
        checkBox_switchImages->setObjectName(QStringLiteral("checkBox_switchImages"));

        gridLayout->addWidget(checkBox_switchImages, 0, 2, 1, 1);

        label_5 = new QLabel(groupBox_progress);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout->addWidget(label_5, 1, 0, 1, 1);

        progressBar_count = new QProgressBar(groupBox_progress);
        progressBar_count->setObjectName(QStringLiteral("progressBar_count"));
        progressBar_count->setValue(24);

        gridLayout->addWidget(progressBar_count, 1, 1, 1, 1);

        progressBar_count_2 = new QProgressBar(groupBox_progress);
        progressBar_count_2->setObjectName(QStringLiteral("progressBar_count_2"));
        progressBar_count_2->setValue(24);
        progressBar_count_2->setInvertedAppearance(true);

        gridLayout->addWidget(progressBar_count_2, 1, 2, 1, 1);

        label = new QLabel(groupBox_progress);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 2, 0, 1, 1);

        progressBar_x = new QProgressBar(groupBox_progress);
        progressBar_x->setObjectName(QStringLiteral("progressBar_x"));
        progressBar_x->setValue(24);

        gridLayout->addWidget(progressBar_x, 2, 1, 1, 1);

        progressBar_x_2 = new QProgressBar(groupBox_progress);
        progressBar_x_2->setObjectName(QStringLiteral("progressBar_x_2"));
        progressBar_x_2->setValue(24);
        progressBar_x_2->setInvertedAppearance(true);
        progressBar_x_2->setTextDirection(QProgressBar::TopToBottom);

        gridLayout->addWidget(progressBar_x_2, 2, 2, 1, 1);

        label_2 = new QLabel(groupBox_progress);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 3, 0, 1, 1);

        progressBar_y = new QProgressBar(groupBox_progress);
        progressBar_y->setObjectName(QStringLiteral("progressBar_y"));
        progressBar_y->setValue(24);

        gridLayout->addWidget(progressBar_y, 3, 1, 1, 1);

        progressBar_y_2 = new QProgressBar(groupBox_progress);
        progressBar_y_2->setObjectName(QStringLiteral("progressBar_y_2"));
        progressBar_y_2->setValue(24);
        progressBar_y_2->setInvertedAppearance(true);

        gridLayout->addWidget(progressBar_y_2, 3, 2, 1, 1);

        label_3 = new QLabel(groupBox_progress);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 4, 0, 1, 1);

        progressBar_size = new QProgressBar(groupBox_progress);
        progressBar_size->setObjectName(QStringLiteral("progressBar_size"));
        progressBar_size->setValue(24);

        gridLayout->addWidget(progressBar_size, 4, 1, 1, 1);

        progressBar_size_2 = new QProgressBar(groupBox_progress);
        progressBar_size_2->setObjectName(QStringLiteral("progressBar_size_2"));
        progressBar_size_2->setValue(24);
        progressBar_size_2->setInvertedAppearance(true);

        gridLayout->addWidget(progressBar_size_2, 4, 2, 1, 1);

        progressBar_skew = new QProgressBar(groupBox_progress);
        progressBar_skew->setObjectName(QStringLiteral("progressBar_skew"));
        progressBar_skew->setValue(24);

        gridLayout->addWidget(progressBar_skew, 5, 1, 1, 1);

        progressBar_skew_2 = new QProgressBar(groupBox_progress);
        progressBar_skew_2->setObjectName(QStringLiteral("progressBar_skew_2"));
        progressBar_skew_2->setValue(24);
        progressBar_skew_2->setInvertedAppearance(true);

        gridLayout->addWidget(progressBar_skew_2, 5, 2, 1, 1);


        verticalLayout_6->addLayout(gridLayout);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        pushButton_calibrate = new QPushButton(groupBox_progress);
        pushButton_calibrate->setObjectName(QStringLiteral("pushButton_calibrate"));

        horizontalLayout_5->addWidget(pushButton_calibrate);

        checkBox_fisheye = new QCheckBox(groupBox_progress);
        checkBox_fisheye->setObjectName(QStringLiteral("checkBox_fisheye"));

        horizontalLayout_5->addWidget(checkBox_fisheye);

        checkBox_unlock = new QCheckBox(groupBox_progress);
        checkBox_unlock->setObjectName(QStringLiteral("checkBox_unlock"));

        horizontalLayout_5->addWidget(checkBox_unlock);

        horizontalLayout_5->setStretch(0, 1);

        verticalLayout_6->addLayout(horizontalLayout_5);

        groupBox_3 = new QGroupBox(groupBox_progress);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        gridLayout_2 = new QGridLayout(groupBox_3);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        lineEdit_P_2 = new QLineEdit(groupBox_3);
        lineEdit_P_2->setObjectName(QStringLiteral("lineEdit_P_2"));
        lineEdit_P_2->setReadOnly(true);

        gridLayout_2->addWidget(lineEdit_P_2, 8, 2, 1, 1);

        label_fx = new QLabel(groupBox_3);
        label_fx->setObjectName(QStringLiteral("label_fx"));
        label_fx->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_fx, 0, 1, 1, 1);

        label_fy = new QLabel(groupBox_3);
        label_fy->setObjectName(QStringLiteral("label_fy"));
        label_fy->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_fy, 1, 1, 1, 1);

        lineEdit_K = new QLineEdit(groupBox_3);
        lineEdit_K->setObjectName(QStringLiteral("lineEdit_K"));
        lineEdit_K->setReadOnly(true);

        gridLayout_2->addWidget(lineEdit_K, 5, 1, 1, 1);

        label_cx = new QLabel(groupBox_3);
        label_cx->setObjectName(QStringLiteral("label_cx"));
        label_cx->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_cx, 2, 1, 1, 1);

        label_11 = new QLabel(groupBox_3);
        label_11->setObjectName(QStringLiteral("label_11"));

        gridLayout_2->addWidget(label_11, 3, 0, 1, 1);

        label_cy = new QLabel(groupBox_3);
        label_cy->setObjectName(QStringLiteral("label_cy"));
        label_cy->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_cy, 3, 1, 1, 1);

        label_6 = new QLabel(groupBox_3);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout_2->addWidget(label_6, 5, 0, 1, 1);

        label_9 = new QLabel(groupBox_3);
        label_9->setObjectName(QStringLiteral("label_9"));

        gridLayout_2->addWidget(label_9, 1, 0, 1, 1);

        label_10 = new QLabel(groupBox_3);
        label_10->setObjectName(QStringLiteral("label_10"));

        gridLayout_2->addWidget(label_10, 2, 0, 1, 1);

        label_8 = new QLabel(groupBox_3);
        label_8->setObjectName(QStringLiteral("label_8"));

        gridLayout_2->addWidget(label_8, 0, 0, 1, 1);

        label_17 = new QLabel(groupBox_3);
        label_17->setObjectName(QStringLiteral("label_17"));

        gridLayout_2->addWidget(label_17, 7, 0, 1, 1);

        lineEdit_D_2 = new QLineEdit(groupBox_3);
        lineEdit_D_2->setObjectName(QStringLiteral("lineEdit_D_2"));
        lineEdit_D_2->setReadOnly(true);

        gridLayout_2->addWidget(lineEdit_D_2, 6, 2, 1, 1);

        label_7 = new QLabel(groupBox_3);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout_2->addWidget(label_7, 6, 0, 1, 1);

        lineEdit_D = new QLineEdit(groupBox_3);
        lineEdit_D->setObjectName(QStringLiteral("lineEdit_D"));
        lineEdit_D->setReadOnly(true);

        gridLayout_2->addWidget(lineEdit_D, 6, 1, 1, 1);

        label_14 = new QLabel(groupBox_3);
        label_14->setObjectName(QStringLiteral("label_14"));

        gridLayout_2->addWidget(label_14, 9, 0, 1, 1);

        label_error = new QLabel(groupBox_3);
        label_error->setObjectName(QStringLiteral("label_error"));

        gridLayout_2->addWidget(label_error, 9, 1, 1, 1);

        label_fx_2 = new QLabel(groupBox_3);
        label_fx_2->setObjectName(QStringLiteral("label_fx_2"));
        label_fx_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_fx_2, 0, 2, 1, 1);

        label_cx_2 = new QLabel(groupBox_3);
        label_cx_2->setObjectName(QStringLiteral("label_cx_2"));
        label_cx_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_cx_2, 2, 2, 1, 1);

        label_fy_2 = new QLabel(groupBox_3);
        label_fy_2->setObjectName(QStringLiteral("label_fy_2"));
        label_fy_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_fy_2, 1, 2, 1, 1);

        label_cy_2 = new QLabel(groupBox_3);
        label_cy_2->setObjectName(QStringLiteral("label_cy_2"));
        label_cy_2->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_cy_2, 3, 2, 1, 1);

        lineEdit_K_2 = new QLineEdit(groupBox_3);
        lineEdit_K_2->setObjectName(QStringLiteral("lineEdit_K_2"));
        lineEdit_K_2->setReadOnly(true);

        gridLayout_2->addWidget(lineEdit_K_2, 5, 2, 1, 1);

        label_18 = new QLabel(groupBox_3);
        label_18->setObjectName(QStringLiteral("label_18"));

        gridLayout_2->addWidget(label_18, 8, 0, 1, 1);

        lineEdit_R = new QLineEdit(groupBox_3);
        lineEdit_R->setObjectName(QStringLiteral("lineEdit_R"));
        lineEdit_R->setReadOnly(true);

        gridLayout_2->addWidget(lineEdit_R, 7, 1, 1, 1);

        lineEdit_P = new QLineEdit(groupBox_3);
        lineEdit_P->setObjectName(QStringLiteral("lineEdit_P"));
        lineEdit_P->setReadOnly(true);

        gridLayout_2->addWidget(lineEdit_P, 8, 1, 1, 1);

        lineEdit_R_2 = new QLineEdit(groupBox_3);
        lineEdit_R_2->setObjectName(QStringLiteral("lineEdit_R_2"));
        lineEdit_R_2->setReadOnly(true);

        gridLayout_2->addWidget(lineEdit_R_2, 7, 2, 1, 1);

        label_baseline_name = new QLabel(groupBox_3);
        label_baseline_name->setObjectName(QStringLiteral("label_baseline_name"));

        gridLayout_2->addWidget(label_baseline_name, 4, 0, 1, 1);

        label_baseline = new QLabel(groupBox_3);
        label_baseline->setObjectName(QStringLiteral("label_baseline"));
        label_baseline->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        gridLayout_2->addWidget(label_baseline, 4, 2, 1, 1);

        label_error_2 = new QLabel(groupBox_3);
        label_error_2->setObjectName(QStringLiteral("label_error_2"));

        gridLayout_2->addWidget(label_error_2, 9, 2, 1, 1);


        verticalLayout_6->addWidget(groupBox_3);

        pushButton_save = new QPushButton(groupBox_progress);
        pushButton_save->setObjectName(QStringLiteral("pushButton_save"));

        verticalLayout_6->addWidget(pushButton_save);

        pushButton_restart = new QPushButton(groupBox_progress);
        pushButton_restart->setObjectName(QStringLiteral("pushButton_restart"));

        verticalLayout_6->addWidget(pushButton_restart);


        verticalLayout->addWidget(groupBox_progress);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        scrollArea->setWidget(scrollAreaWidgetContents);

        horizontalLayout_4->addWidget(scrollArea);

        horizontalLayout_4->setStretch(0, 1);

        verticalLayout_5->addLayout(horizontalLayout_4);

        buttonBox = new QDialogButtonBox(calibrationDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setStandardButtons(QDialogButtonBox::Close);

        verticalLayout_5->addWidget(buttonBox);


        retranslateUi(calibrationDialog);

        QMetaObject::connectSlotsByName(calibrationDialog);
    } // setupUi

    void retranslateUi(QDialog *calibrationDialog)
    {
        calibrationDialog->setWindowTitle(QApplication::translate("calibrationDialog", "Dialog", Q_NULLPTR));
        label_left->setText(QString());
        label_right->setText(QString());
        radioButton_raw->setText(QApplication::translate("calibrationDialog", "Raw", Q_NULLPTR));
        radioButton_rectified->setText(QApplication::translate("calibrationDialog", "Rectified", Q_NULLPTR));
        radioButton_stereoRectified->setText(QApplication::translate("calibrationDialog", "Stereo Rectified", Q_NULLPTR));
        checkBox_mirror->setText(QApplication::translate("calibrationDialog", "Mirror", Q_NULLPTR));
        checkBox_showHorizontalLines->setText(QApplication::translate("calibrationDialog", "Show horizontal lines", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("calibrationDialog", "Chessboard", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_15->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        label_15->setText(QApplication::translate("calibrationDialog", "Square size", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_12->setToolTip(QApplication::translate("calibrationDialog", "Number of inner squares on the board", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_12->setText(QApplication::translate("calibrationDialog", "Board size", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_16->setToolTip(QApplication::translate("calibrationDialog", "Number of inner squares on the board", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_16->setText(QString());
        doubleSpinBox_squareSize->setSuffix(QApplication::translate("calibrationDialog", " m", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_21->setToolTip(QString());
#endif // QT_NO_TOOLTIP
        label_21->setText(QApplication::translate("calibrationDialog", "Download", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_20->setToolTip(QApplication::translate("calibrationDialog", "Number of inner squares on the board", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_20->setText(QApplication::translate("calibrationDialog", "<html><head/><body><p><a href=\"https://github.com/introlab/rtabmap/raw/master/bin/data/check-108.pdf\"><span style=\" text-decoration: underline; color:#0000ff;\">Sample8x6</span></a></p></body></html>", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_22->setToolTip(QApplication::translate("calibrationDialog", "If > 1, image is scaled down to help board detection on high definition images", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_22->setText(QApplication::translate("calibrationDialog", "Max scale", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_stereoBaseline->setToolTip(QApplication::translate("calibrationDialog", "If > 1, image is scaled down to help board detection on high definition images", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_stereoBaseline->setText(QApplication::translate("calibrationDialog", "Stereo Baseline", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        doubleSpinBox_stereoBaseline->setToolTip(QApplication::translate("calibrationDialog", "If you know the exact baseline of the stereo cameras, you can set it here in case the square size is not super accurate.", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        doubleSpinBox_stereoBaseline->setSuffix(QApplication::translate("calibrationDialog", " m", Q_NULLPTR));
        groupBox_progress->setTitle(QApplication::translate("calibrationDialog", "Progress", Q_NULLPTR));
        label_4->setText(QApplication::translate("calibrationDialog", "Skew", Q_NULLPTR));
        label_19->setText(QApplication::translate("calibrationDialog", "Serial", Q_NULLPTR));
        label_serial->setText(QApplication::translate("calibrationDialog", "0", Q_NULLPTR));
        checkBox_switchImages->setText(QApplication::translate("calibrationDialog", "Switch images", Q_NULLPTR));
        label_5->setText(QApplication::translate("calibrationDialog", "Count", Q_NULLPTR));
        progressBar_count->setFormat(QApplication::translate("calibrationDialog", "%p", Q_NULLPTR));
        progressBar_count_2->setFormat(QApplication::translate("calibrationDialog", "%p", Q_NULLPTR));
        label->setText(QApplication::translate("calibrationDialog", "X", Q_NULLPTR));
        progressBar_x->setFormat(QApplication::translate("calibrationDialog", "%p%", Q_NULLPTR));
        progressBar_x_2->setFormat(QApplication::translate("calibrationDialog", "%p%", Q_NULLPTR));
        label_2->setText(QApplication::translate("calibrationDialog", "Y", Q_NULLPTR));
        label_3->setText(QApplication::translate("calibrationDialog", "Size", Q_NULLPTR));
        pushButton_calibrate->setText(QApplication::translate("calibrationDialog", "Calibrate", Q_NULLPTR));
        checkBox_fisheye->setText(QApplication::translate("calibrationDialog", "Fish eye", Q_NULLPTR));
        checkBox_unlock->setText(QApplication::translate("calibrationDialog", "Unlock", Q_NULLPTR));
        groupBox_3->setTitle(QApplication::translate("calibrationDialog", "Camera intrinsic parameters", Q_NULLPTR));
        label_fx->setText(QApplication::translate("calibrationDialog", "0", Q_NULLPTR));
        label_fy->setText(QApplication::translate("calibrationDialog", "0", Q_NULLPTR));
        label_cx->setText(QApplication::translate("calibrationDialog", "0", Q_NULLPTR));
        label_11->setText(QApplication::translate("calibrationDialog", "cy", Q_NULLPTR));
        label_cy->setText(QApplication::translate("calibrationDialog", "0", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_6->setToolTip(QApplication::translate("calibrationDialog", "Camera matrix", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_6->setText(QApplication::translate("calibrationDialog", "K", Q_NULLPTR));
        label_9->setText(QApplication::translate("calibrationDialog", "fy", Q_NULLPTR));
        label_10->setText(QApplication::translate("calibrationDialog", "cx", Q_NULLPTR));
        label_8->setText(QApplication::translate("calibrationDialog", "fx", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_17->setToolTip(QApplication::translate("calibrationDialog", "Distorsion coefficients", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_17->setText(QApplication::translate("calibrationDialog", "R", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_7->setToolTip(QApplication::translate("calibrationDialog", "Distorsion coefficients", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_7->setText(QApplication::translate("calibrationDialog", "D", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_14->setToolTip(QApplication::translate("calibrationDialog", "Avg. reproduction error", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_14->setText(QApplication::translate("calibrationDialog", "Error", Q_NULLPTR));
        label_error->setText(QApplication::translate("calibrationDialog", "0", Q_NULLPTR));
        label_fx_2->setText(QApplication::translate("calibrationDialog", "0", Q_NULLPTR));
        label_cx_2->setText(QApplication::translate("calibrationDialog", "0", Q_NULLPTR));
        label_fy_2->setText(QApplication::translate("calibrationDialog", "0", Q_NULLPTR));
        label_cy_2->setText(QApplication::translate("calibrationDialog", "0", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        label_18->setToolTip(QApplication::translate("calibrationDialog", "Distorsion coefficients", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        label_18->setText(QApplication::translate("calibrationDialog", "P", Q_NULLPTR));
        label_baseline_name->setText(QApplication::translate("calibrationDialog", "baseline", Q_NULLPTR));
        label_baseline->setText(QApplication::translate("calibrationDialog", "0", Q_NULLPTR));
        label_error_2->setText(QApplication::translate("calibrationDialog", "0", Q_NULLPTR));
        pushButton_save->setText(QApplication::translate("calibrationDialog", "Export calibration (*.yaml)", Q_NULLPTR));
        pushButton_restart->setText(QApplication::translate("calibrationDialog", "Restart", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class calibrationDialog: public Ui_calibrationDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CALIBRATIONDIALOG_H
