/********************************************************************************
** Form generated from reading UI file 'consoleWidget.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONSOLEWIDGET_H
#define UI_CONSOLEWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_consoleWidget
{
public:
    QVBoxLayout *verticalLayout;
    QTextEdit *textEdit;
    QHBoxLayout *horizontalLayout;
    QSpinBox *spinBox_lines;
    QSpinBox *spinBox_time;
    QPushButton *pushButton_clear;

    void setupUi(QWidget *consoleWidget)
    {
        if (consoleWidget->objectName().isEmpty())
            consoleWidget->setObjectName(QStringLiteral("consoleWidget"));
        consoleWidget->resize(400, 300);
        verticalLayout = new QVBoxLayout(consoleWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        textEdit = new QTextEdit(consoleWidget);
        textEdit->setObjectName(QStringLiteral("textEdit"));
        textEdit->setLineWrapMode(QTextEdit::NoWrap);
        textEdit->setReadOnly(true);

        verticalLayout->addWidget(textEdit);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        spinBox_lines = new QSpinBox(consoleWidget);
        spinBox_lines->setObjectName(QStringLiteral("spinBox_lines"));
        spinBox_lines->setMaximum(99999);
        spinBox_lines->setValue(100);

        horizontalLayout->addWidget(spinBox_lines);

        spinBox_time = new QSpinBox(consoleWidget);
        spinBox_time->setObjectName(QStringLiteral("spinBox_time"));
        spinBox_time->setMaximum(1000);
        spinBox_time->setSingleStep(100);
        spinBox_time->setValue(1000);

        horizontalLayout->addWidget(spinBox_time);

        pushButton_clear = new QPushButton(consoleWidget);
        pushButton_clear->setObjectName(QStringLiteral("pushButton_clear"));

        horizontalLayout->addWidget(pushButton_clear);

        horizontalLayout->setStretch(2, 1);

        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(consoleWidget);

        QMetaObject::connectSlotsByName(consoleWidget);
    } // setupUi

    void retranslateUi(QWidget *consoleWidget)
    {
        consoleWidget->setWindowTitle(QApplication::translate("consoleWidget", "consoleWidget", Q_NULLPTR));
        spinBox_lines->setSuffix(QApplication::translate("consoleWidget", " lines", Q_NULLPTR));
        spinBox_time->setSuffix(QApplication::translate("consoleWidget", " ms", Q_NULLPTR));
        pushButton_clear->setText(QApplication::translate("consoleWidget", "Clear", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class consoleWidget: public Ui_consoleWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONSOLEWIDGET_H
