#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>

#include <QPen>
#include <QBrush>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
private:
    QGraphicsScene m_graphicsScene;
    QPen gpsPen;
    QBrush gpsBrush;
    QGraphicsEllipseItem* gpsPointer;
public:
    explicit MainWindow(QWidget *parent = 0, QString image_path="");
    ~MainWindow();

private:
    Ui::MainWindow *ui;

public slots:
    void updateGPS(double lt, double lg);
};

#endif // MAINWINDOW_H
