#include "mainwindow.h"
#include "ui_mainwindow.h"

void onMouse(int event, int x, int y, int, void *ptr)
{
    /*if(event == EVENT_MOUSEMOVE){
        Point *processor = (Point *)ptr;
        processor->x = x;
        processor->y = y;
    }*/

    if(event == EVENT_LBUTTONDBLCLK){
        imageProcessor *proc = (imageProcessor *)ptr;
        qDebug() << sqrt((y-proc->center.y)*(y-proc->center.y)+(x-proc->center.x)*(x-proc->center.x));
    }
}

MainWindow::MainWindow(int argc, char *argv[],QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    visServer = new QTcpSocket();

    if(argc>1){ //Setup image grabber server
        QString command = QString(argv[1]);
        if(command == "-help") {
            showHelp();
            exit(0);
        }
        QHostAddress hostaddr = QHostAddress(argv[1]);
        if(hostaddr.isNull()){
            qDebug() << "Invalid host address ... leaving.";
            QCoreApplication::exit();
        }

        if(argc!=3) {
            qDebug() << "Provide host IP and Port.";
            QCoreApplication::exit();
        }
        hostIP = QHostAddress(argv[1]);
        hostPort = QString(argv[2]).toInt();
    } else {
        QString ipAddress;
        QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();
        // use the first non-localhost IPv4 address
        for (int i = 0; i < ipAddressesList.size(); ++i) {
            if (ipAddressesList.at(i) != QHostAddress::LocalHost &&
                ipAddressesList.at(i).toIPv4Address()) {
                ipAddress = ipAddressesList.at(i).toString();
            }
        }

        qDebug() << "No arguments provided ... using defaults.";
        hostIP = QHostAddress(ipAddress);
        hostPort = defaultPort;
    }

    connectToHost(hostIP,hostPort);
    original = new Mat(480,480,CV_8UC3,Scalar(0,0,0));
    processed = new Mat(480,480,CV_8UC3,Scalar(0,0,0));
    segmented = new Mat(480,480,CV_8UC3,Scalar(0,0,0));
    binary = new Mat(480,480,CV_8UC3,Scalar(0,0,0));
    feed = new Mat(480,480,CV_8UC3,Scalar(0,0,0));

    imageFormat = origFormat;
    grabbingFormat = origGrabbing;
    toggleCross = toggleCircle = toggleRunning = toggleVideoFeed = false;
    circleRadius = 50;

    for(int i=0;i<4;i++){
        yuvDef[i][H][0]=0;
        yuvDef[i][H][1]=180;
        yuvDef[i][S][0]=0;
        yuvDef[i][S][1]=255;
        yuvDef[i][V][0]=0;
        yuvDef[i][V][1]=255;
    }

    proc = new imageProcessor();
    center.x = 240; center.y = 240;
    takenSnapshots = 0;
    ui->spinBox->setValue(center.x);
    ui->spinBox->setValue(center.y);
    imgRequestTimer = new QTimer();
    connect(imgRequestTimer,SIGNAL(timeout()),this,SLOT(display()));
    connect(visServer,SIGNAL(readyRead()),this,SLOT(readPendingDatagrams()));
    imgRequestTimer->start(40);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    switch(event->key()){
        case Qt::Key_1:{ //Set Image Mode to Original
            imageFormat = origFormat;
            break;
        }
        case Qt::Key_2:{ //Set Image Mode to Binary
            imageFormat = binFormat;
            break;
        }
        case Qt::Key_3:{  //Set Image Mode to Segmented
            imageFormat = segmFormat;
            break;
        }
        case Qt::Key_4:{ //Set Image Grabbing Mode to Original
            grabbingFormat = origGrabbing;
            break;
        }
        case Qt::Key_5:{ //Set Image Grabbing  Mode to Segmented
            grabbingFormat = segmGrabbing;
            break;
        }
        case Qt::Key_6:{  //Set Image Grabbing Mode to World State
            grabbingFormat = worldGrabbing;
            break;
        }
        case Qt::Key_7:{  //Set Image Grabbing Mode to  Histogram+Point mapping view
            grabbingFormat = histGrabbing;
            break;
        }
        case Qt::Key_X:{  //Toggle Cross
            toggle(&toggleCross);
            break;
        }
        case Qt::Key_C:{  //Toggle Circle
            toggle(&toggleCircle);
            break;
        }
        case Qt::Key_S:{  //Toggle running image grabbing
            toggle(&toggleRunning);
            break;
        }
        case Qt::Key_R:{  //Toggle running image grabbing
            toggle(&toggleVideoFeed);
            break;
        }
        case Qt::Key_D:{  //Toggle distance line mapping
            toggle(&toggleDistMapping);
            break;
        }
        case Qt::Key_F:{  //Take Snapshot
            QString filename = "./Snapshots/" + QString::number(takenSnapshots) + ".png";
            takenSnapshots++;
            if(imwrite(filename.toStdString(),*original)) qDebug() << "Snapshot taken.";
            break;
        }
        case Qt::Key_Space:{  //Grab one image from host
            if(grabbingFormat==origGrabbing){
                requestDataFromHost("n\n");
            } else if(grabbingFormat==segmGrabbing){
                requestDataFromHost("s\n");
            } else if(grabbingFormat==worldGrabbing){
                requestDataFromHost("w\n");
            } else if(grabbingFormat==histGrabbing){
                requestDataFromHost("h\n");
            }
            break;
        }
        case Qt::Key_Q:{  //Quit
            qDebug() << "Leaving Vision Configurator ...";
            QThread::msleep(500);
            system("clear");
            QCoreApplication::exit();
            break;
        }
        case Qt::Key_G:{ //Generate LUT
            on_pushButton_clicked();
            break;
        }
        case Qt::Key_W:{ //Write LUT
            on_pushButton_3_clicked();
            break;
        }
        case Qt::Key_L:{ //Load LUT
            on_pushButton_5_clicked();
            break;
        }
        case Qt::Key_Backslash:{ //Retry connection to host
            connectToHost(hostIP,hostPort);
            break;
        }
    }

    event->accept();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    qDebug() << "Leaving Vision Configurator ...";
    QThread::msleep(500);
    system("clear");
    event->accept();
}

void MainWindow::toggle(bool *t)
{
    if(*t == true) *t = false;
    else *t = true;
}

void MainWindow::updateLabelValues()
{
    ui->label_8->setText(QString::number(ui->hmin->value()));
    ui->label_9->setText(QString::number(ui->hmax->value()));
    ui->label_10->setText(QString::number(ui->smin->value()));
    ui->label_11->setText(QString::number(ui->smax->value()));
    ui->label_12->setText(QString::number(ui->vmin->value()));
    ui->label_13->setText(QString::number(ui->vmax->value()));
    circleRadius = ui->horizontalSlider->value();
    int type = ui->comboBox->currentIndex();
    yuvDef[type][H][LMIN] = ui->hmin->value();
    yuvDef[type][H][LMAX] = ui->hmax->value();
    yuvDef[type][S][LMIN] = ui->smin->value();
    yuvDef[type][S][LMAX] = ui->smax->value();
    yuvDef[type][V][LMIN] = ui->vmin->value();
    yuvDef[type][V][LMAX] = ui->vmax->value();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::display()
{
    updateLabelValues();
    if(toggleRunning){
        if(grabbingFormat==origGrabbing){
            if(toggleVideoFeed){
                //Grab new image from host
                requestDataFromHost("n\n");
            }

            //Process and show Image
            if(imageFormat == origFormat){
                if(imgMutex.tryLock(1)){
                    original->copyTo(*processed);
                    imgMutex.unlock();
                }
                if(toggleCross){
                    line(*processed,Point(0,center.y),Point(480,center.y),Scalar(255,255,255),1);
                    line(*processed,Point(center.x,0),Point(center.x,480),Scalar(255,255,255),1);
                    line(*processed,Point(0,0),Point(480,480),Scalar(255,255,255),1);
                }

                if(toggleCircle){
                    circle(*processed,center,circleRadius,Scalar(255,255,255),1);
                }
            } else if(imageFormat == binFormat){
                if(imgMutex.tryLock(1)){
                    original->copyTo(*binary);
                    imgMutex.unlock();
                    int type = ui->comboBox->currentIndex();
                    proc->getBinary(binary,
                     getCurrentValue(type,H,LMIN),
                     getCurrentValue(type,H,LMAX),
                     getCurrentValue(type,S,LMIN),
                     getCurrentValue(type,S,LMAX),
                     getCurrentValue(type,V,LMIN),
                     getCurrentValue(type,V,LMAX));
                }
            } else if(imageFormat == segmFormat){
                if(imgMutex.tryLock(1)){
                    original->copyTo(*segmented);
                    imgMutex.unlock();
                    proc->getSegmentedImage(segmented);
                }

                if(toggleDistMapping){ // Look for mouse position on window
                    // Draw scan line from center to mouse position
                    // RLE black white black
                    Mat idxImage = Mat(480,480,CV_8UC1,Scalar(0));
                    ScanLines traceScan(idxImage,center, Point(mousePosition.x,mousePosition.y));
                    //preprocess image
                    proc->preProcessIdx(traceScan,original,&idxImage);
                    //RLE
                    RLE traceRLE(traceScan,UAV_WHITE_BIT,UAV_GREEN_BIT,UAV_WHITE_BIT, 2, 2, 2, 10);
                    traceScan.draw(*segmented,Scalar(255,0,255));
                    traceRLE.drawInterestPoints(Scalar(255,255,0),*segmented,UAV_BLACK_BIT);
                }
            }
        } else if(grabbingFormat==segmGrabbing){
            if(toggleVideoFeed)requestDataFromHost("s\n");
            if(imgMutex.tryLock(1)){
                original->copyTo(*feed);
                imgMutex.unlock();
            }
        } else if(grabbingFormat==worldGrabbing){
            if(toggleVideoFeed)requestDataFromHost("w\n");
            if(imgMutex.tryLock(1)){
                original->copyTo(*feed);
                imgMutex.unlock();
            }
        } else if(grabbingFormat==histGrabbing){
            if(toggleVideoFeed)requestDataFromHost("h\n");
            if(imgMutex.tryLock(1)){
                original->copyTo(*feed);
                imgMutex.unlock();
            }
        }

        displayCurrentFormat();
    } else {
        destroyAllWindows();
    }

    ui->centralWidget->setFocus();
}

void MainWindow::displayCurrentFormat()
{
    if(grabbingFormat==origGrabbing){
        switch(imageFormat){
        case origFormat:{
            putText(*processed,"Grabbing:Original Format:Original",Point(10,15),CV_FONT_HERSHEY_PLAIN,0.8,Scalar(255,255,0));
            if(toggleVideoFeed){
                putText(*processed,"Video Feed:ON",Point(375,15),CV_FONT_HERSHEY_PLAIN,0.8,Scalar(255,0,255));
            } else {
                putText(*processed,"Video Feed:OFF",Point(375,15),CV_FONT_HERSHEY_PLAIN,0.8,Scalar(255,0,255));
            }
            imshow("Image",*processed);
            setMouseCallback("Image",onMouse,proc);
            break;
        }
        case binFormat:{
            putText(*binary,"Grabbing:Original Format:Binary",Point(10,15),CV_FONT_HERSHEY_PLAIN,0.8,Scalar(255,255,0));
            if(toggleVideoFeed){
                putText(*binary,"Video Feed:ON",Point(375,15),CV_FONT_HERSHEY_PLAIN,0.8,Scalar(255,0,255));
            } else {
                putText(*binary,"Video Feed:OFF",Point(375,15),CV_FONT_HERSHEY_PLAIN,0.8,Scalar(255,0,255));
            }
            imshow("Image",*binary);
            break;
        }
        case segmFormat:{
            putText(*segmented,"Grabbing:Original Format:Segmented",Point(10,15),CV_FONT_HERSHEY_PLAIN,0.8,Scalar(255,255,0));
            if(toggleVideoFeed){
                putText(*segmented,"Video Feed:ON",Point(375,15),CV_FONT_HERSHEY_PLAIN,0.8,Scalar(255,0,255));
            } else {
                putText(*segmented,"Video Feed:OFF",Point(375,15),CV_FONT_HERSHEY_PLAIN,0.8,Scalar(255,0,255));
            }
            imshow("Image",*segmented);
            break;
        }
    }
   } else {
        if(grabbingFormat==segmGrabbing){
            putText(*feed,"Grabbing:Segmented",Point(10,15),CV_FONT_HERSHEY_PLAIN,0.8,Scalar(255,255,0));
        }else if(grabbingFormat==worldGrabbing){
            putText(*feed,"Grabbing:World Information",Point(10,15),CV_FONT_HERSHEY_PLAIN,0.8,Scalar(255,255,0));
        }
        if(toggleVideoFeed){
            putText(*feed,"Video Feed:ON",Point(375,15),CV_FONT_HERSHEY_PLAIN,0.8,Scalar(255,0,255));
        } else {
            putText(*feed,"Video Feed:OFF",Point(375,15),CV_FONT_HERSHEY_PLAIN,0.8,Scalar(255,0,255));
        }
        imshow("Image", *feed);
   }
}

void MainWindow::requestDataFromHost(QString data)
{
    if(visServer->isWritable()){ //Write request to host
        visServer->write(data.toLocal8Bit());
    }
}

void MainWindow::readPendingDatagrams()
{
    if(visServer->bytesAvailable()==691200){
        if(imgMutex.tryLock(33)){
            QByteArray datagram = visServer->readAll(); //Read image full data packet
            uchar * imgData = (uchar *)datagram.constData(); //Convert to uchar
            int counter = 0;
            for(int j=0;j<480;j++){ //Fill original Mat with received image from host
                uchar *pixel = original->ptr<uchar>(j);
                for(int i=0;i<480*3;i++){
                    pixel[i] = imgData[counter++];
                }
            }
            imgMutex.unlock();
        }

    } else if(visServer->bytesAvailable()>691200) visServer->readAll();
}

void MainWindow::connectToHost(QHostAddress ip, int port)
{
    visServer->connectToHost(ip,port);
    if(!visServer->waitForConnected(2000)){ //Connect to host robot to grab images
        qDebug() << "Invalid host connection ... Try again later.";
    } else {
        qDebug() << "Connection to host correctly configured.";
    }
}

void MainWindow::showHelp()
{
    qDebug() << "Help command for mtVisionConfig utility from MinhoTeam.";
    qDebug() << "Invoke mtVisionConfig [robotIP] [robotPort] to connect to robot imaging pipeline.";
    qDebug() << "Once connected by pressing:" << endl
             << "Q: Quit the program." << endl
             << "\\: Retry Connection to host." << endl
             << "1: Switch to original image format." << endl
             << "2: Switch to binary image format." << endl
             << "3: Switch to segmented image format." << endl
             << "4: Switch to original image grabbing." << endl
             << "5: Switch to segmented image grabbing." << endl
             << "6: Switch to world info image grabbing." << endl
             << "S: Toggle configurator state [On/Off]." << endl
             << "X: Toggle configuration Cross [On/Off]." << endl
             << "R: Enable/Disable continuous frame grabbing from host." << endl
             << "SPACE: Grab one frame from host." << endl;
}

int MainWindow::getCurrentValue(int type, int what, int limit)
{
    return yuvDef[type][what][limit];
}

void MainWindow::on_pushButton_clicked()
{
    qDebug() << "Generating look up table ...";
    proc->generateLookUpTable(yuvDef);
    qDebug() << "Look up table generated.";
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    ui->hmin->setValue(yuvDef[index][H][LMIN]);
    ui->hmax->setValue(yuvDef[index][H][LMAX]);
    ui->smin->setValue(yuvDef[index][S][LMIN]);
    ui->smax->setValue(yuvDef[index][S][LMAX]);
    ui->vmin->setValue(yuvDef[index][V][LMIN]);
    ui->vmax->setValue(yuvDef[index][V][LMAX]);
}

void MainWindow::on_pushButton_3_clicked()
{
    qDebug() << "Writing look up table to vision.cfg ...";
    if(proc->writeLookUpTable()) qDebug() << "Look up table written.";
}

void MainWindow::on_pushButton_4_clicked()
{
    qDebug() << "Writing mapping to visionparams.cfg ...";
    qDebug() << "visionparams.cfg written";
}

void MainWindow::on_spinBox_valueChanged(int arg1)
{
    center.x = arg1;
    proc->setCenter(center);
}

void MainWindow::on_spinBox_2_valueChanged(int arg1)
{
    center.y = arg1;
    proc->setCenter(center);
}

void MainWindow::on_pushButton_5_clicked()
{
    qDebug() << "Loading look up table from vision.cfg ...";
    if(proc->readLookUpTable()) qDebug() << "Look up table loaded.";
}
