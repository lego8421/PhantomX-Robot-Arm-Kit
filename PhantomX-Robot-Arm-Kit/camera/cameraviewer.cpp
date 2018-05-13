#include "cameraviewer.h"

CameraViewer::CameraViewer(QWidget *parent, QGridLayout *layout) {

    mCamera = NULL;
    mCameraViewFinder = new QCameraViewfinder(parent);
    layout->addWidget(mCameraViewFinder, 0, 1);

    if(QCameraInfo::availableCameras().size()) {
        QCameraViewfinderSettings* setting = new QCameraViewfinderSettings();
        setting->setResolution(640, 480);

        mCameraInfo = QCameraInfo::availableCameras().at(0);
        mCamera = new QCamera(mCameraInfo);
        mCamera->setViewfinder(mCameraViewFinder);
        mCamera->setViewfinderSettings(*setting);

        if (mCamera->imageProcessing()->isAvailable()) {
            if(mCamera->imageProcessing()->isWhiteBalanceModeSupported(QCameraImageProcessing::WhiteBalanceManual)) {
                mCamera->imageProcessing()->setWhiteBalanceMode(QCameraImageProcessing::WhiteBalanceManual);
            }
        }
        mCamera->start();
    }
}

CameraViewer::~CameraViewer() {
    if(mCamera != NULL) {
        mCamera->stop();
        delete mCamera;
    }
}

cv::Mat CameraViewer::getCameraImage() {
    QScreen *screen = QGuiApplication::primaryScreen();
    QImage image = screen->grabWindow(mCameraViewFinder->winId(), 0, 0, -1, -1).toImage();
    return cv::Mat(image.height(), image.width(), CV_8UC3, image.bits(), image.bytesPerLine());
}
