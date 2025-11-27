/********************************************************************************
** Form generated from reading UI file 'editConstraintDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_EDITCONSTRAINTDIALOG_H
#define UI_EDITCONSTRAINTDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_EditConstraintDialog
{
public:
    QVBoxLayout *verticalLayout;
    QGridLayout *gridLayout;
    QLabel *label;
    QDoubleSpinBox *x;
    QLabel *label_4;
    QDoubleSpinBox *roll;
    QLabel *label_2;
    QDoubleSpinBox *y;
    QLabel *label_5;
    QDoubleSpinBox *pitch;
    QLabel *label_3;
    QDoubleSpinBox *z;
    QLabel *label_6;
    QDoubleSpinBox *yaw;
    QDoubleSpinBox *linear_sigma;
    QDoubleSpinBox *angular_sigma;
    QLabel *label_7;
    QLabel *label_8;
    QCheckBox *checkBox_radians;
    QSpacerItem *verticalSpacer;
    QLabel *label_9;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *EditConstraintDialog)
    {
        if (EditConstraintDialog->objectName().isEmpty())
            EditConstraintDialog->setObjectName(QStringLiteral("EditConstraintDialog"));
        EditConstraintDialog->resize(430, 231);
        verticalLayout = new QVBoxLayout(EditConstraintDialog);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        label = new QLabel(EditConstraintDialog);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        x = new QDoubleSpinBox(EditConstraintDialog);
        x->setObjectName(QStringLiteral("x"));
        x->setDecimals(6);
        x->setMinimum(-9999);
        x->setMaximum(9999);
        x->setSingleStep(0.001);

        gridLayout->addWidget(x, 0, 1, 1, 1);

        label_4 = new QLabel(EditConstraintDialog);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 0, 2, 1, 1);

        roll = new QDoubleSpinBox(EditConstraintDialog);
        roll->setObjectName(QStringLiteral("roll"));
        roll->setDecimals(6);
        roll->setMinimum(-3.15);
        roll->setMaximum(3.15);
        roll->setSingleStep(0.001);

        gridLayout->addWidget(roll, 0, 3, 1, 1);

        label_2 = new QLabel(EditConstraintDialog);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        y = new QDoubleSpinBox(EditConstraintDialog);
        y->setObjectName(QStringLiteral("y"));
        y->setDecimals(6);
        y->setMinimum(-9999);
        y->setMaximum(9999);
        y->setSingleStep(0.001);

        gridLayout->addWidget(y, 1, 1, 1, 1);

        label_5 = new QLabel(EditConstraintDialog);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout->addWidget(label_5, 1, 2, 1, 1);

        pitch = new QDoubleSpinBox(EditConstraintDialog);
        pitch->setObjectName(QStringLiteral("pitch"));
        pitch->setDecimals(6);
        pitch->setMinimum(-3.15);
        pitch->setMaximum(3.15);
        pitch->setSingleStep(0.001);

        gridLayout->addWidget(pitch, 1, 3, 1, 1);

        label_3 = new QLabel(EditConstraintDialog);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 2, 0, 1, 1);

        z = new QDoubleSpinBox(EditConstraintDialog);
        z->setObjectName(QStringLiteral("z"));
        z->setDecimals(6);
        z->setMinimum(-9999);
        z->setMaximum(9999);
        z->setSingleStep(0.001);

        gridLayout->addWidget(z, 2, 1, 1, 1);

        label_6 = new QLabel(EditConstraintDialog);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout->addWidget(label_6, 2, 2, 1, 1);

        yaw = new QDoubleSpinBox(EditConstraintDialog);
        yaw->setObjectName(QStringLiteral("yaw"));
        yaw->setDecimals(6);
        yaw->setMinimum(-3.15);
        yaw->setMaximum(3.15);
        yaw->setSingleStep(0.001);

        gridLayout->addWidget(yaw, 2, 3, 1, 1);

        linear_sigma = new QDoubleSpinBox(EditConstraintDialog);
        linear_sigma->setObjectName(QStringLiteral("linear_sigma"));
        linear_sigma->setDecimals(6);
        linear_sigma->setMinimum(0);
        linear_sigma->setMaximum(9999);
        linear_sigma->setSingleStep(0.001);
        linear_sigma->setValue(0);

        gridLayout->addWidget(linear_sigma, 3, 1, 1, 1);

        angular_sigma = new QDoubleSpinBox(EditConstraintDialog);
        angular_sigma->setObjectName(QStringLiteral("angular_sigma"));
        angular_sigma->setDecimals(6);
        angular_sigma->setMinimum(0);
        angular_sigma->setMaximum(3.15);
        angular_sigma->setSingleStep(0.001);
        angular_sigma->setValue(0);

        gridLayout->addWidget(angular_sigma, 3, 3, 1, 1);

        label_7 = new QLabel(EditConstraintDialog);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout->addWidget(label_7, 3, 0, 1, 1);

        label_8 = new QLabel(EditConstraintDialog);
        label_8->setObjectName(QStringLiteral("label_8"));

        gridLayout->addWidget(label_8, 3, 2, 1, 1);

        checkBox_radians = new QCheckBox(EditConstraintDialog);
        checkBox_radians->setObjectName(QStringLiteral("checkBox_radians"));
        checkBox_radians->setChecked(true);

        gridLayout->addWidget(checkBox_radians, 4, 3, 1, 1);


        verticalLayout->addLayout(gridLayout);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        label_9 = new QLabel(EditConstraintDialog);
        label_9->setObjectName(QStringLiteral("label_9"));

        verticalLayout->addWidget(label_9);

        buttonBox = new QDialogButtonBox(EditConstraintDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        verticalLayout->addWidget(buttonBox);


        retranslateUi(EditConstraintDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), EditConstraintDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), EditConstraintDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(EditConstraintDialog);
    } // setupUi

    void retranslateUi(QDialog *EditConstraintDialog)
    {
        EditConstraintDialog->setWindowTitle(QApplication::translate("EditConstraintDialog", "Edit Constraint", Q_NULLPTR));
        label->setText(QApplication::translate("EditConstraintDialog", "x", Q_NULLPTR));
        x->setSuffix(QApplication::translate("EditConstraintDialog", " m", Q_NULLPTR));
        label_4->setText(QApplication::translate("EditConstraintDialog", "roll", Q_NULLPTR));
        roll->setSuffix(QApplication::translate("EditConstraintDialog", " rad", Q_NULLPTR));
        label_2->setText(QApplication::translate("EditConstraintDialog", "y", Q_NULLPTR));
        y->setSuffix(QApplication::translate("EditConstraintDialog", " m", Q_NULLPTR));
        label_5->setText(QApplication::translate("EditConstraintDialog", "pitch", Q_NULLPTR));
        pitch->setSuffix(QApplication::translate("EditConstraintDialog", " rad", Q_NULLPTR));
        label_3->setText(QApplication::translate("EditConstraintDialog", "z", Q_NULLPTR));
        z->setSuffix(QApplication::translate("EditConstraintDialog", " m", Q_NULLPTR));
        label_6->setText(QApplication::translate("EditConstraintDialog", "yaw", Q_NULLPTR));
        yaw->setSuffix(QApplication::translate("EditConstraintDialog", " rad", Q_NULLPTR));
        linear_sigma->setSuffix(QApplication::translate("EditConstraintDialog", " m", Q_NULLPTR));
        angular_sigma->setSuffix(QApplication::translate("EditConstraintDialog", " rad", Q_NULLPTR));
        label_7->setText(QApplication::translate("EditConstraintDialog", "<html><head/><body><p>Linear &sigma; </p></body></html>", Q_NULLPTR));
        label_8->setText(QApplication::translate("EditConstraintDialog", "<html><head/><body><p>Angular \317\203</p></body></html>", Q_NULLPTR));
        checkBox_radians->setText(QApplication::translate("EditConstraintDialog", "Radians", Q_NULLPTR));
        label_9->setText(QApplication::translate("EditConstraintDialog", "<html><head/><body><p>Setting &sigma; to 0 will set identity covariance.</p></body></html>", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class EditConstraintDialog: public Ui_EditConstraintDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_EDITCONSTRAINTDIALOG_H
