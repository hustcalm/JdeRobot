/********************************************************************************
** Form generated from reading UI file 'pclviewer.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLVIEWER_H
#define UI_PCLVIEWER_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCLViewer
{
public:
    QWidget *centralwidget;
    QVTKWidget *qvtkWidget;
    QPushButton *pushButton_load_cloud;
    QPushButton *pushButton_load_keyframes;
    QPushButton *pushButton_load_image;
    QLabel *label_target_image;
    QPushButton *pushButton_estimate_pose;
    QPushButton *pushButton_save_pose;

    void setupUi(QMainWindow *cameraPoseEstimator3D)
    {
        if (cameraPoseEstimator3D->objectName().isEmpty())
            cameraPoseEstimator3D->setObjectName(QString::fromUtf8("cameraPoseEstimator3D"));
        cameraPoseEstimator3D->resize(1024, 499);
        cameraPoseEstimator3D->setMinimumSize(QSize(0, 0));
        cameraPoseEstimator3D->setMaximumSize(QSize(5000, 5000));
        centralwidget = new QWidget(cameraPoseEstimator3D);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(330, 10, 640, 480));
        pushButton_load_cloud = new QPushButton(centralwidget);
        pushButton_load_cloud->setObjectName(QString::fromUtf8("pushButton_load_cloud"));
        pushButton_load_cloud->setGeometry(QRect(40, 10, 121, 41));
        pushButton_load_keyframes = new QPushButton(centralwidget);
        pushButton_load_keyframes->setObjectName(QString::fromUtf8("pushButton_load_keyframes"));
        pushButton_load_keyframes->setGeometry(QRect(40, 60, 121, 41));
        pushButton_load_image = new QPushButton(centralwidget);
        pushButton_load_image->setObjectName(QString::fromUtf8("pushButton_load_image"));
        pushButton_load_image->setGeometry(QRect(40, 110, 121, 41));
        label_target_image = new QLabel(centralwidget);
        label_target_image->setObjectName(QString::fromUtf8("label_target_image"));
        label_target_image->setGeometry(QRect(5, 155, 320, 240));
        pushButton_estimate_pose = new QPushButton(centralwidget);
        pushButton_estimate_pose->setObjectName(QString::fromUtf8("pushButton_estimate_pose"));
        pushButton_estimate_pose->setGeometry(QRect(40, 405, 121, 41));
        pushButton_save_pose = new QPushButton(centralwidget);
        pushButton_save_pose->setObjectName(QString::fromUtf8("pushButton_save_pose"));
        pushButton_save_pose->setGeometry(QRect(40, 455, 121, 41));
        cameraPoseEstimator3D->setCentralWidget(centralwidget);

        retranslateUi(cameraPoseEstimator3D);

        QMetaObject::connectSlotsByName(cameraPoseEstimator3D);
    } // setupUi

    void retranslateUi(QMainWindow *cameraPoseEstimator3D)
    {
        cameraPoseEstimator3D->setWindowTitle(QApplication::translate("PCLViewer", "cameraPoseEstimator3D", 0, QApplication::UnicodeUTF8));
        pushButton_load_cloud->setText(QApplication::translate("PCLViewer", "Load 3D Map", 0, QApplication::UnicodeUTF8));
        pushButton_load_keyframes->setText(QApplication::translate("PCLViewer", "Load Keyframes", 0, QApplication::UnicodeUTF8));
        pushButton_load_image->setText(QApplication::translate("PCLViewer", "Load image", 0, QApplication::UnicodeUTF8));
        pushButton_estimate_pose->setText(QApplication::translate("PCLViewer", "Estimate Pose", 0, QApplication::UnicodeUTF8));
        pushButton_save_pose->setText(QApplication::translate("PCLViewer", "Save Pose", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H
