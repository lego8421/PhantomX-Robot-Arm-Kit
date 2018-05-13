#ifndef CAMERAVIEWER_H
#define CAMERAVIEWER_H

#include <QGuiApplication>
#include <QGridLayout>
#include <QCamera>
#include <QCameraInfo>
#include <QCameraViewfinder>
#include <QCameraImageCapture>
#include <QCameraImageProcessing>
#include <QScreen>
#include <QImage>

#include <opencv2/core/core.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

class CameraViewer {
public:
    CameraViewer(QWidget *parent, QGridLayout *layout);
    ~CameraViewer();

    cv::Mat getCameraImage();

private:
    QCamera             *mCamera;
    QCameraViewfinder   *mCameraViewFinder;
    QCameraInfo         mCameraInfo;
};

#endif // CAMERAVIEWER_H
