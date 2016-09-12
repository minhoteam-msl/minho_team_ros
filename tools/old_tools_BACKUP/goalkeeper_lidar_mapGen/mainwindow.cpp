#include "mainwindow.h"
#include "ui_mainwindow.h"

static void onMouse(int event, int x, int y, int flags, void* userdata)
{
    Q_UNUSED(flags);
    if(event==EVENT_MOUSEMOVE){
        Point *mouse = (Point *)userdata;
        mouse->x = x;
        mouse->y = y;
    }
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    scale = 10; // 1px -> scalemm
    outputResolution = 50;//in mm
    displayTimer = new QTimer();
    fieldloaded = false;
    connect(displayTimer,SIGNAL(timeout()),this,SLOT(display()));
    mouse = Point(-1,-1);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked() //Generate and Save Map File for LIDAR
{
    QString filename = QFileDialog::getSaveFileName(this,
    tr("Save map File"), "../goalkeeper_lidar_mapGen/config", tr("Map Files (*.map)"));

    if(filename!=""){
        if(!filename.contains(".map")){
            if(filename.contains(".")){
                //Correct
                int idx = filename.indexOf(".");
                filename = filename.left(idx-1);
                filename+=".map";
            } else {
                filename += ".map";
            }
        }
        //Generate and save map file
        saveMapConfiguration(filename);
    }
}

void MainWindow::loadFieldConfiguration(QString file_)
{
    // Create view of current Field
    QFile file(file_);
    if(!file.open(QIODevice::ReadOnly)) {
        qDebug() <<"✖ Error reading " << file_;
        return;
    } QTextStream in(&file);

    QString value; int counter = 0;
    while(!in.atEnd()){
       value = in.readLine();
       fieldAnatomy.dimensions[counter] = value.right(value.size()-value.indexOf('=')-1).toInt();
       counter++;
    }

    fieldloaded = true;
    file.close();
    drawField();
}

void MainWindow::saveMapConfiguration(QString file_)
{
    displayTimer->stop();
    QFile file(file_);
    if(!file.open(QIODevice::WriteOnly) || !fieldloaded) {
        qDebug() <<"✖ Error trying to write to " << file_;
        return;
    } QTextStream out(&file);

    int counter = 0;
    int step = outputResolution/scale; //step in pixels
    ui->progressBar->setValue(counter);
    ui->progressBar->setMaximum(((field.rows-1)/step)*((field.cols-1)/step));
    QString str = QString::number(outputResolution)+"\n";
    out << str;
    double xReal = 0.0, yReal = 0.0;
    for(int y=0;y<field.rows;y+=step){
        yReal = (y*scale)/1000.0; //to meters
        for(int x=0;x<field.cols;x+=step){
            Point pt = Point(x,y);
            Point3i ret = getNearestDistance(pt);
            xReal = (x*scale)/1000.0; //to meters
            QString output = "("+QString::number(xReal,'f',2)+","+QString::number(yReal,'f',2)+"):"
                    +QString::number(ret.z/1000.0,'f',3)+"\n";
            out << output;
            waitKey(1);
            counter++;
            ui->progressBar->setValue(counter);
        }
    }
    waitKey(1000);
    ui->progressBar->setValue(0);
    displayTimer->start(10);
    file.close();
}

void MainWindow::drawOnPoint(Point pt)
{
    field.copyTo(model);
    Point3i ret = getNearestDistance(pt);
    line(model,pt,Point(ret.x,ret.y),Scalar(0,0,255),1);
    QString dist = "Distance -> "+QString::number(ret.z)+"mm";
    putText(model,dist.toStdString(),Point((field.cols/2)*0.5,0.05*field.rows),CV_FONT_HERSHEY_COMPLEX,0.3,Scalar(0,0,0));
    imshow("Model",model);
}

void MainWindow::drawField()
{
    int edge1 = fieldAnatomy.fieldDims.AREA_WIDTH2/scale;
    int edge2 = fieldAnatomy.fieldDims.AREA_LENGTH2/scale;
    int line = fieldAnatomy.fieldDims.LINE_WIDTH/scale;
    int midY = edge1/2;
    int margin = (fieldAnatomy.fieldDims.GOALIE_WIDTH+150)/scale;
    field = Mat(edge1,edge2+margin,CV_8UC3,Scalar(0,120,0));
    //Bigger Area
    rectangle(field,Rect(Point(0+line/2,0+line/2),Point(edge2-line/2,edge1-line/2)),Scalar(255,250,255),line);
    //Smaller Area
    rectangle(field,Rect(edge2-fieldAnatomy.fieldDims.AREA_LENGTH1/scale,
                    midY-(fieldAnatomy.fieldDims.AREA_WIDTH1/(2*scale))+line/2,
                    fieldAnatomy.fieldDims.AREA_LENGTH1/scale-line/2,
                    fieldAnatomy.fieldDims.AREA_WIDTH1/scale-line/2),Scalar(255,250,255),line);
    //Posts
    int centeryp1 = midY-fieldAnatomy.fieldDims.GOALIE_LENGTH/(2*scale)-fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/(2*scale);
    int centerxp = edge2-line+fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/(2*scale)-1;
    int centeryp2 = midY+fieldAnatomy.fieldDims.GOALIE_LENGTH/(2*scale)-fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/(2*scale);
    int hpost = fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/(2*scale);
    rectangle(field,Rect(Point(centerxp-hpost,centeryp1-hpost),Point(centerxp+hpost,centeryp1+hpost)),Scalar(255,0,0),-1);
    rectangle(field,Rect(Point(centerxp-hpost,centeryp2-hpost),Point(centerxp+hpost,centeryp2+hpost)),Scalar(255,0,0),-1);
    //sideplates
    int ht = fieldAnatomy.fieldDims.GOALIE_BOARD_WIDTH/(2*scale);
    int t = fieldAnatomy.fieldDims.GOALIE_BOARD_WIDTH/(scale);
    rectangle(field,Rect(centerxp+hpost,centeryp1-ht,fieldAnatomy.fieldDims.GOALIE_WIDTH/scale,t),Scalar(255,0,0),-1);
    rectangle(field,Rect(centerxp+hpost,centeryp2-ht,fieldAnatomy.fieldDims.GOALIE_WIDTH/scale,t),Scalar(255,0,0),-1);
    //backplate
    rectangle(field,Rect(centerxp+hpost+fieldAnatomy.fieldDims.GOALIE_WIDTH/scale,centeryp1-ht,t,
            fieldAnatomy.fieldDims.GOALIE_LENGTH/scale+t),Scalar(255,0,0),-1);
    field.copyTo(model);
    displayTimer->start(10);
}

void MainWindow::display()
{
    // Draw nearest line to goalie
    drawOnPoint(mouse);
    setMouseCallback("Model",onMouse,&mouse);
}

void MainWindow::on_pushButton_2_clicked() //Load field
{
    QString filename = QFileDialog::getOpenFileName(this,
    tr("Load field File"), "../goalkeeper_lidar_mapGen/config", tr("Field Files (*.view)"));

    if(filename!=""){
        loadFieldConfiguration(filename);
    }
}

Point3i MainWindow::distanceToTarget(double angle,Point pt)
{
    //InitialPoint : mouse
    // return : targetx,targety,distance(mm)
    int x = pt.x; int y = pt.y; Point3i ret = Point3i(pt.x,pt.y,-1);

    // Forward search
    double dist = 0;
    while(x>=0&&x<field.cols&&y>=0&&y<field.rows){
        x = pt.x+cos(angle)*dist; y = pt.y+sin(angle)*dist; dist+=0.5;
        if(isTarget(Point(x,y))){
            ret.x = x; ret.y = y; ret.z = scale*sqrt((x-pt.x)*(x-pt.x)+(y-pt.y)*(y-pt.y));
            break;
        }
    }
    return ret;
}

Point3i MainWindow::getNearestDistance(Point pt)
{
    if(isTarget(mouse)){
        return Point3i(pt.x,pt.y,0);
    }

    int nearestDistance = 1000000;
    Point3i nearestPoint;
    for(double angle=0;angle<=2*M_PI;angle+=0.00872665){
        Point3i distance = distanceToTarget(angle,pt);
        if(distance.z>=0 && distance.z<nearestDistance){
            nearestDistance = distance.z;
            nearestPoint = distance;
        }
    }

    return nearestPoint;
}

bool MainWindow::isTarget(Point pt)
{
    if(pt.x<0 || pt.x>=field.cols || pt.y<0 || pt.y>=field.rows) return false;
    Vec3b *color = field.ptr<Vec3b>(pt.y);
    if(color[pt.x][1]==0) return true;
    else return false;
}

void MainWindow::on_spinBox_valueChanged(int arg1)
{
    outputResolution = arg1;
}
