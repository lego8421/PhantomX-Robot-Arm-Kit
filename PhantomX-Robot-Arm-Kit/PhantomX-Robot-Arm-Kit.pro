#-------------------------------------------------
#
# Project created by QtCreator 2018-03-13T11:20:44
#
#-------------------------------------------------

QT       += core gui opengl serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PhantomX-Robot-Arm-Kit
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

QMAKE_CXXFLAGS += -std=c++0x


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    glwidget.cpp \
    kinematics/kinematics.cpp \
    kinematics/posOriInverse.cpp \
    kinematics/quaternion.cpp \
    matrix/transformation.cpp \
    opengl/oglObjects.cpp \
    serialport/serialport.cpp \
    dynamixel/dynamixel.cpp \
    interpolation.cpp

HEADERS += \
        mainwindow.h \
    glwidget.h \
    kinematics/kinematics.h \
    kinematics/posOriInverse.h \
    kinematics/quaternion.h \
    matrix/mat/matrix.h \
    matrix/mathDef.h \
    matrix/matrixAlgebra.h \
    matrix/transformation.h \
    opengl/oglDef.h \
    opengl/oglObjects.h \
    serialport/serialport.h \
    dynamixel/dynamixel.h \
    interpolation.h

FORMS += \
        mainwindow.ui

#windows
win32 {
    LIBS += -lOpengl32 \
            -lglu32
}

#ubuntu
unix:!macx {
LIBS += -lglut \
        -lGLU \
        -lGL
}



RESOURCES += \
    resource.qrc

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
