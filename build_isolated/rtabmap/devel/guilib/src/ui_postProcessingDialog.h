/********************************************************************************
** Form generated from reading UI file 'postProcessingDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_POSTPROCESSINGDIALOG_H
#define UI_POSTPROCESSINGDIALOG_H

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
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_PostProcessingDialog
{
public:
    QVBoxLayout *verticalLayout_3;
    QGroupBox *detectMoreLoopClosures;
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout;
    QDoubleSpinBox *clusterAngle;
    QDoubleSpinBox *clusterRadius;
    QLabel *label_3;
    QLabel *clusterAngle_label;
    QLabel *label_6;
    QSpinBox *iterations;
    QLabel *label_7;
    QLabel *label_9;
    QCheckBox *intraSession;
    QCheckBox *interSession;
    QLabel *label;
    QGroupBox *groupBox_3;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_2;
    QCheckBox *refineNeighborLinks;
    QCheckBox *refineLoopClosureLinks;
    QGroupBox *sba;
    QVBoxLayout *verticalLayout_4;
    QGridLayout *gridLayout_2;
    QSpinBox *sba_iterations;
    QLabel *label_8;
    QLabel *label_variance;
    QDoubleSpinBox *sba_variance;
    QLabel *label_10;
    QComboBox *comboBox_sbaType;
    QLabel *label_variance_2;
    QCheckBox *sba_rematchFeatures;
    QSpacerItem *verticalSpacer;
    QLabel *label_4;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *PostProcessingDialog)
    {
        if (PostProcessingDialog->objectName().isEmpty())
            PostProcessingDialog->setObjectName(QStringLiteral("PostProcessingDialog"));
        PostProcessingDialog->resize(552, 633);
        QIcon icon;
        icon.addFile(QStringLiteral(":/images/RTAB-Map.ico"), QSize(), QIcon::Normal, QIcon::Off);
        PostProcessingDialog->setWindowIcon(icon);
        verticalLayout_3 = new QVBoxLayout(PostProcessingDialog);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        detectMoreLoopClosures = new QGroupBox(PostProcessingDialog);
        detectMoreLoopClosures->setObjectName(QStringLiteral("detectMoreLoopClosures"));
        detectMoreLoopClosures->setCheckable(true);
        verticalLayout = new QVBoxLayout(detectMoreLoopClosures);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        clusterAngle = new QDoubleSpinBox(detectMoreLoopClosures);
        clusterAngle->setObjectName(QStringLiteral("clusterAngle"));
        clusterAngle->setDecimals(0);
        clusterAngle->setMinimum(0);
        clusterAngle->setMaximum(180);
        clusterAngle->setValue(30);

        gridLayout->addWidget(clusterAngle, 1, 0, 1, 1);

        clusterRadius = new QDoubleSpinBox(detectMoreLoopClosures);
        clusterRadius->setObjectName(QStringLiteral("clusterRadius"));
        clusterRadius->setMinimum(0.01);
        clusterRadius->setMaximum(99);
        clusterRadius->setSingleStep(0.01);
        clusterRadius->setValue(0.3);

        gridLayout->addWidget(clusterRadius, 0, 0, 1, 1);

        label_3 = new QLabel(detectMoreLoopClosures);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setWordWrap(true);

        gridLayout->addWidget(label_3, 0, 1, 1, 1);

        clusterAngle_label = new QLabel(detectMoreLoopClosures);
        clusterAngle_label->setObjectName(QStringLiteral("clusterAngle_label"));
        clusterAngle_label->setWordWrap(true);

        gridLayout->addWidget(clusterAngle_label, 1, 1, 1, 1);

        label_6 = new QLabel(detectMoreLoopClosures);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setWordWrap(true);

        gridLayout->addWidget(label_6, 2, 1, 1, 1);

        iterations = new QSpinBox(detectMoreLoopClosures);
        iterations->setObjectName(QStringLiteral("iterations"));
        iterations->setMinimum(1);
        iterations->setMaximum(100);
        iterations->setValue(1);

        gridLayout->addWidget(iterations, 2, 0, 1, 1);

        label_7 = new QLabel(detectMoreLoopClosures);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setWordWrap(true);

        gridLayout->addWidget(label_7, 3, 1, 1, 1);

        label_9 = new QLabel(detectMoreLoopClosures);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setWordWrap(true);

        gridLayout->addWidget(label_9, 4, 1, 1, 1);

        intraSession = new QCheckBox(detectMoreLoopClosures);
        intraSession->setObjectName(QStringLiteral("intraSession"));

        gridLayout->addWidget(intraSession, 3, 0, 1, 1);

        interSession = new QCheckBox(detectMoreLoopClosures);
        interSession->setObjectName(QStringLiteral("interSession"));

        gridLayout->addWidget(interSession, 4, 0, 1, 1);

        gridLayout->setColumnStretch(1, 1);

        verticalLayout->addLayout(gridLayout);

        label = new QLabel(detectMoreLoopClosures);
        label->setObjectName(QStringLiteral("label"));
        label->setWordWrap(true);

        verticalLayout->addWidget(label);


        verticalLayout_3->addWidget(detectMoreLoopClosures);

        groupBox_3 = new QGroupBox(PostProcessingDialog);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        groupBox_3->setCheckable(false);
        verticalLayout_2 = new QVBoxLayout(groupBox_3);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        label_2 = new QLabel(groupBox_3);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setWordWrap(true);

        verticalLayout_2->addWidget(label_2);

        refineNeighborLinks = new QCheckBox(groupBox_3);
        refineNeighborLinks->setObjectName(QStringLiteral("refineNeighborLinks"));
        refineNeighborLinks->setChecked(false);

        verticalLayout_2->addWidget(refineNeighborLinks);

        refineLoopClosureLinks = new QCheckBox(groupBox_3);
        refineLoopClosureLinks->setObjectName(QStringLiteral("refineLoopClosureLinks"));
        refineLoopClosureLinks->setChecked(false);

        verticalLayout_2->addWidget(refineLoopClosureLinks);


        verticalLayout_3->addWidget(groupBox_3);

        sba = new QGroupBox(PostProcessingDialog);
        sba->setObjectName(QStringLiteral("sba"));
        sba->setCheckable(true);
        sba->setChecked(false);
        verticalLayout_4 = new QVBoxLayout(sba);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        sba_iterations = new QSpinBox(sba);
        sba_iterations->setObjectName(QStringLiteral("sba_iterations"));
        sba_iterations->setMinimum(1);
        sba_iterations->setMaximum(999999);
        sba_iterations->setValue(100);

        gridLayout_2->addWidget(sba_iterations, 1, 0, 1, 1);

        label_8 = new QLabel(sba);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setWordWrap(true);

        gridLayout_2->addWidget(label_8, 1, 1, 1, 1);

        label_variance = new QLabel(sba);
        label_variance->setObjectName(QStringLiteral("label_variance"));
        label_variance->setWordWrap(true);

        gridLayout_2->addWidget(label_variance, 2, 1, 1, 1);

        sba_variance = new QDoubleSpinBox(sba);
        sba_variance->setObjectName(QStringLiteral("sba_variance"));
        sba_variance->setDecimals(2);
        sba_variance->setMinimum(0.01);
        sba_variance->setMaximum(999);
        sba_variance->setSingleStep(0.01);
        sba_variance->setValue(1);

        gridLayout_2->addWidget(sba_variance, 2, 0, 1, 1);

        label_10 = new QLabel(sba);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setWordWrap(true);

        gridLayout_2->addWidget(label_10, 0, 1, 1, 1);

        comboBox_sbaType = new QComboBox(sba);
        comboBox_sbaType->setObjectName(QStringLiteral("comboBox_sbaType"));

        gridLayout_2->addWidget(comboBox_sbaType, 0, 0, 1, 1);

        label_variance_2 = new QLabel(sba);
        label_variance_2->setObjectName(QStringLiteral("label_variance_2"));
        label_variance_2->setWordWrap(true);

        gridLayout_2->addWidget(label_variance_2, 3, 1, 1, 1);

        sba_rematchFeatures = new QCheckBox(sba);
        sba_rematchFeatures->setObjectName(QStringLiteral("sba_rematchFeatures"));

        gridLayout_2->addWidget(sba_rematchFeatures, 3, 0, 1, 1);

        gridLayout_2->setColumnStretch(1, 1);

        verticalLayout_4->addLayout(gridLayout_2);


        verticalLayout_3->addWidget(sba);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer);

        label_4 = new QLabel(PostProcessingDialog);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setWordWrap(true);

        verticalLayout_3->addWidget(label_4);

        buttonBox = new QDialogButtonBox(PostProcessingDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok|QDialogButtonBox::RestoreDefaults);

        verticalLayout_3->addWidget(buttonBox);


        retranslateUi(PostProcessingDialog);

        QMetaObject::connectSlotsByName(PostProcessingDialog);
    } // setupUi

    void retranslateUi(QDialog *PostProcessingDialog)
    {
        PostProcessingDialog->setWindowTitle(QApplication::translate("PostProcessingDialog", "Post-Processing", Q_NULLPTR));
        detectMoreLoopClosures->setTitle(QApplication::translate("PostProcessingDialog", "Detect more loop closures", Q_NULLPTR));
        clusterAngle->setSuffix(QApplication::translate("PostProcessingDialog", " degrees", Q_NULLPTR));
        clusterRadius->setSuffix(QApplication::translate("PostProcessingDialog", " m", Q_NULLPTR));
        label_3->setText(QApplication::translate("PostProcessingDialog", "Cluster radius", Q_NULLPTR));
        clusterAngle_label->setText(QApplication::translate("PostProcessingDialog", "Cluster angle", Q_NULLPTR));
        label_6->setText(QApplication::translate("PostProcessingDialog", "Iterations", Q_NULLPTR));
        label_7->setText(QApplication::translate("PostProcessingDialog", "Intra-session", Q_NULLPTR));
        label_9->setText(QApplication::translate("PostProcessingDialog", "Inter-session", Q_NULLPTR));
        intraSession->setText(QString());
        interSession->setText(QString());
        label->setText(QApplication::translate("PostProcessingDialog", "See \"RTAB-Map settings/Motion Estimation\" to tune the parameters used.", Q_NULLPTR));
        groupBox_3->setTitle(QApplication::translate("PostProcessingDialog", "Refine links with ICP", Q_NULLPTR));
        label_2->setText(QApplication::translate("PostProcessingDialog", "Laser scans are required. See \"RTAB-Map settings/Motion Estimation/ICP\" to tune the parameters used.", Q_NULLPTR));
        refineNeighborLinks->setText(QApplication::translate("PostProcessingDialog", "Refine neighbor links", Q_NULLPTR));
        refineLoopClosureLinks->setText(QApplication::translate("PostProcessingDialog", "Refine loop closure links", Q_NULLPTR));
        sba->setTitle(QApplication::translate("PostProcessingDialog", "Sparse Bundle Adjustment (SBA)", Q_NULLPTR));
        label_8->setText(QApplication::translate("PostProcessingDialog", "Iterations", Q_NULLPTR));
        label_variance->setText(QApplication::translate("PostProcessingDialog", "Pixel variance used by g2o.", Q_NULLPTR));
        sba_variance->setSuffix(QString());
        label_10->setText(QApplication::translate("PostProcessingDialog", "Type", Q_NULLPTR));
        comboBox_sbaType->clear();
        comboBox_sbaType->insertItems(0, QStringList()
         << QApplication::translate("PostProcessingDialog", "g2o", Q_NULLPTR)
         << QApplication::translate("PostProcessingDialog", "cvsba", Q_NULLPTR)
         << QApplication::translate("PostProcessingDialog", "Ceres", Q_NULLPTR)
        );
        label_variance_2->setText(QApplication::translate("PostProcessingDialog", "Rematch features.", Q_NULLPTR));
        sba_rematchFeatures->setText(QString());
        label_4->setText(QApplication::translate("PostProcessingDialog", "Note: Post-processing only affects visualization in the GUI and the export actions. To actually push those optimizations to the database, use Database Viewer tool (Tools->Edit database...). You can do Edit->Download graph to revert the changes afterwards.", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class PostProcessingDialog: public Ui_PostProcessingDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_POSTPROCESSINGDIALOG_H
