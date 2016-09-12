#-------------------------------------------------
#
# Project created by QtCreator 2016-04-07T00:29:37
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = mtVisionConfig
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    imageprocessor.cpp \
    Angle.cpp \
    RLE.cpp \
    ScanLines.cpp \
    Vec.cpp

HEADERS  += mainwindow.h \
    types.h \
    imageprocessor.h \
    Angle.h \
    RLE.h \
    ScanLines.h \
    Vec.h

FORMS    += mainwindow.ui

INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /usr/local/include/opencv2

LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc
LIBS += -L/usr/local/lib

unix:!macx: LIBS += -L$$PWD/../../../usr/lib/ -lflycapture

INCLUDEPATH += $$PWD/../../../usr/include
DEPENDPATH += $$PWD/../../../usr/include
