#include "funrobo_gpsview/mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>

const double LT_MIN = 42.293132;
const double LT_MAX = 42.293790;

const double LG_MIN = -71.264387;
const double LG_MAX = -71.263483;

const int WIDTH=496;
const int HEIGHT=487;

const double RADIUS = 5.0;

MainWindow::MainWindow(QWidget *parent, QString image_path) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_graphicsScene(new QGraphicsScene(this))
{
    ui->setupUi(this);
    ui->graphicsView->setScene(&m_graphicsScene);

    QPixmap bk(image_path);
    if(!bk.isNull()){
        m_graphicsScene.addPixmap(bk);
    }

    gpsPen = QPen(gpsBrush,1.0,Qt::SolidLine,Qt::SquareCap,Qt::BevelJoin);
    gpsBrush = QBrush(QColor(255,0,0),Qt::SolidPattern);


    gpsPointer = new QGraphicsEllipseItem(QRectF(0.0,0.0,RADIUS*2,RADIUS*2));
    gpsPointer->setPen(gpsPen);
    gpsPointer->setBrush(gpsBrush);
    m_graphicsScene.addItem(gpsPointer);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete gpsPointer;
}

void MainWindow::updateGPS(double lt, double lg){
    //img = (height x width) 487 x 496
	//
	float x = (lg - LG_MIN)/(LG_MAX - LG_MIN); // normalize
    float y = (lt - LT_MAX)/(LT_MIN - LT_MAX); // normalize

    x *= WIDTH;
    y *= HEIGHT;

    if(0<=x && x<WIDTH && 0<=y && y<HEIGHT){
       gpsPointer->setRect(x - RADIUS,y - RADIUS,RADIUS*2,RADIUS*2);
        ui->lat_val->setText(QString::number(lt));
        ui->long_val->setText(QString::number(lg));
    }else{
        ui->lat_val->setText("OUT OF RANGE");
        ui->long_val->setText("OUT OF RANGE");
    }
}
