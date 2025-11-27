/********************************************************************************
** Form generated from reading UI file 'loopClosureViewer.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOOPCLOSUREVIEWER_H
#define UI_LOOPCLOSUREVIEWER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "rtabmap/gui/CloudViewer.h"

QT_BEGIN_NAMESPACE

class Ui_loopClosureViewer
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_idA;
    QLabel *label_idB;
    QLabel *label_transform;
    QSpacerItem *horizontalSpacer_2;
    QCheckBox *checkBox_rawCloud;
    rtabmap::CloudViewer *cloudViewerTransform;

    void setupUi(QWidget *loopClosureViewer)
    {
        if (loopClosureViewer->objectName().isEmpty())
            loopClosureViewer->setObjectName(QStringLiteral("loopClosureViewer"));
        loopClosureViewer->resize(515, 390);
        verticalLayout = new QVBoxLayout(loopClosureViewer);
        verticalLayout->setSpacing(0);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_idA = new QLabel(loopClosureViewer);
        label_idA->setObjectName(QStringLiteral("label_idA"));
        label_idA->setAlignment(Qt::AlignCenter);

        horizontalLayout_2->addWidget(label_idA);

        label_idB = new QLabel(loopClosureViewer);
        label_idB->setObjectName(QStringLiteral("label_idB"));
        label_idB->setAlignment(Qt::AlignCenter);

        horizontalLayout_2->addWidget(label_idB);

        label_transform = new QLabel(loopClosureViewer);
        label_transform->setObjectName(QStringLiteral("label_transform"));
        label_transform->setAlignment(Qt::AlignCenter);
        label_transform->setWordWrap(true);
        label_transform->setTextInteractionFlags(Qt::LinksAccessibleByMouse|Qt::TextSelectableByMouse);

        horizontalLayout_2->addWidget(label_transform);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer_2);

        checkBox_rawCloud = new QCheckBox(loopClosureViewer);
        checkBox_rawCloud->setObjectName(QStringLiteral("checkBox_rawCloud"));

        horizontalLayout_2->addWidget(checkBox_rawCloud);


        verticalLayout->addLayout(horizontalLayout_2);

        cloudViewerTransform = new rtabmap::CloudViewer(loopClosureViewer);
        cloudViewerTransform->setObjectName(QStringLiteral("cloudViewerTransform"));

        verticalLayout->addWidget(cloudViewerTransform);

        verticalLayout->setStretch(1, 1);

        retranslateUi(loopClosureViewer);

        QMetaObject::connectSlotsByName(loopClosureViewer);
    } // setupUi

    void retranslateUi(QWidget *loopClosureViewer)
    {
        loopClosureViewer->setWindowTitle(QApplication::translate("loopClosureViewer", "Form", Q_NULLPTR));
        label_idA->setText(QString());
        label_idB->setText(QString());
        label_transform->setText(QString());
        checkBox_rawCloud->setText(QApplication::translate("loopClosureViewer", "Raw  cloud", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class loopClosureViewer: public Ui_loopClosureViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOOPCLOSUREVIEWER_H
