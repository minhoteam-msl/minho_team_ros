#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <imageprocessor.h>
#include "ScanLines.h"
#include "RLE.h"
#include <QMutex>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char *argv[],QWidget *parent = 0);
    void keyPressEvent(QKeyEvent *event);
    void closeEvent(QCloseEvent *event);
    void toggle(bool *t);
    void updateLabelValues();
    int getCurrentValue(int type, int what, int limit);
    ~MainWindow();
private slots:
    void display();
    void displayCurrentFormat();
    void requestDataFromHost(QString data);
    void readPendingDatagrams();
    void connectToHost(QHostAddress ip, int port);
    void showHelp();
    void on_pushButton_clicked();

    void on_comboBox_currentIndexChanged(int index);

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_spinBox_valueChanged(int arg1);

    void on_spinBox_2_valueChanged(int arg1);

    void on_pushButton_5_clicked();

private:
    // Host Connection Data
    Ui::MainWindow *ui;
    QTcpSocket *visServer;
    QHostAddress hostIP;
    int hostPort;
    // Image Buffers
    Mat *original, *segmented, *binary, *processed, *feed;
    // Booleans and others
    int imageFormat, grabbingFormat;
    bool toggleCross, toggleCircle, toggleRunning, toggleVideoFeed, toggleDistMapping;
    QTimer *imgRequestTimer;
    int circleRadius;
    QMutex imgMutex;
    int takenSnapshots;
    //Def data
    Point center;
    // Configuration Algorithms Data
    imageProcessor *proc;
    int yuvDef[4][3][2];
    vector<Point2d>worldMappingVec;
    Point mousePosition;
};

#endif // MAINWINDOW_H

