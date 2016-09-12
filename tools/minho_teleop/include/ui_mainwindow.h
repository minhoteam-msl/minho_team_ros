/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.6.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLabel *lb_robot_name;
    QLabel *lb_pose;
    QLabel *label;
    QLabel *label_2;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_2;
    QPushButton *pushButton_2;
    QPushButton *pushButton;
    QWidget *gridLayoutWidget_3;
    QGridLayout *gridLayout_4;
    QSlider *hs_lin;
    QLabel *label_3;
    QLabel *label_4;
    QSlider *hs_ang;
    QLabel *lb_lin;
    QLabel *lb_ang;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(230, 170);
        MainWindow->setMinimumSize(QSize(230, 170));
        MainWindow->setMaximumSize(QSize(230, 170));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayoutWidget = new QWidget(centralWidget);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(10, 0, 211, 71));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        lb_robot_name = new QLabel(gridLayoutWidget);
        lb_robot_name->setObjectName(QStringLiteral("lb_robot_name"));
        QFont font;
        font.setPointSize(9);
        lb_robot_name->setFont(font);
        lb_robot_name->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(lb_robot_name, 0, 1, 1, 1);

        lb_pose = new QLabel(gridLayoutWidget);
        lb_pose->setObjectName(QStringLiteral("lb_pose"));
        lb_pose->setFont(font);
        lb_pose->setAlignment(Qt::AlignCenter);

        gridLayout->addWidget(lb_pose, 1, 1, 1, 1);

        label = new QLabel(gridLayoutWidget);
        label->setObjectName(QStringLiteral("label"));
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        label->setFont(font);

        gridLayout->addWidget(label, 0, 0, 1, 1);

        label_2 = new QLabel(gridLayoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);
        label_2->setFont(font);

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        gridLayoutWidget_2 = new QWidget(centralWidget);
        gridLayoutWidget_2->setObjectName(QStringLiteral("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(10, 130, 211, 31));
        gridLayout_2 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        pushButton_2 = new QPushButton(gridLayoutWidget_2);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        QFont font1;
        font1.setPointSize(7);
        pushButton_2->setFont(font1);

        gridLayout_2->addWidget(pushButton_2, 0, 2, 1, 1);

        pushButton = new QPushButton(gridLayoutWidget_2);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setFont(font1);

        gridLayout_2->addWidget(pushButton, 0, 1, 1, 1);

        gridLayoutWidget_3 = new QWidget(centralWidget);
        gridLayoutWidget_3->setObjectName(QStringLiteral("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(5, 70, 221, 61));
        gridLayout_4 = new QGridLayout(gridLayoutWidget_3);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        hs_lin = new QSlider(gridLayoutWidget_3);
        hs_lin->setObjectName(QStringLiteral("hs_lin"));
        hs_lin->setMaximum(100);
        hs_lin->setValue(80);
        hs_lin->setOrientation(Qt::Horizontal);

        gridLayout_4->addWidget(hs_lin, 1, 0, 1, 1);

        label_3 = new QLabel(gridLayoutWidget_3);
        label_3->setObjectName(QStringLiteral("label_3"));
        QFont font2;
        font2.setPointSize(8);
        label_3->setFont(font2);

        gridLayout_4->addWidget(label_3, 0, 0, 1, 1);

        label_4 = new QLabel(gridLayoutWidget_3);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setFont(font2);

        gridLayout_4->addWidget(label_4, 0, 1, 1, 1);

        hs_ang = new QSlider(gridLayoutWidget_3);
        hs_ang->setObjectName(QStringLiteral("hs_ang"));
        hs_ang->setMaximum(100);
        hs_ang->setValue(80);
        hs_ang->setOrientation(Qt::Horizontal);

        gridLayout_4->addWidget(hs_ang, 1, 1, 1, 1);

        lb_lin = new QLabel(gridLayoutWidget_3);
        lb_lin->setObjectName(QStringLiteral("lb_lin"));
        QFont font3;
        font3.setPointSize(6);
        lb_lin->setFont(font3);
        lb_lin->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(lb_lin, 2, 0, 1, 1);

        lb_ang = new QLabel(gridLayoutWidget_3);
        lb_ang->setObjectName(QStringLiteral("lb_ang"));
        lb_ang->setFont(font3);
        lb_ang->setAlignment(Qt::AlignCenter);

        gridLayout_4->addWidget(lb_ang, 2, 1, 1, 1);

        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MinhoTeam Teleop", 0));
        lb_robot_name->setText(QApplication::translate("MainWindow", "<html><head/><body><p><br/></p></body></html>", 0));
        lb_pose->setText(QApplication::translate("MainWindow", "<html><head/><body><p><br/></p></body></html>", 0));
        label->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">Robot:</span></p></body></html>", 0));
        label_2->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">Pose:</span></p></body></html>", 0));
        pushButton_2->setText(QApplication::translate("MainWindow", "Disable Teleop", 0));
        pushButton->setText(QApplication::translate("MainWindow", "Enable Teleop", 0));
        label_3->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">Max Lin Vel:</span></p></body></html>", 0));
        label_4->setText(QApplication::translate("MainWindow", "<html><head/><body><p><span style=\" font-weight:600;\">Max Ang Vel:</span></p></body></html>", 0));
        lb_lin->setText(QApplication::translate("MainWindow", "<html><head/><body><p><br/></p></body></html>", 0));
        lb_ang->setText(QApplication::translate("MainWindow", "<html><head/><body><p><br/></p></body></html>", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
