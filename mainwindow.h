#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>

#include <QtSerialPort/QSerialPortInfo>
#include <D:\Qt\Examples\Qt-5.12.2\serialport\creaderasync\serialportreader.h>
//#include <D:\Qt\Examples\Qt-5.12.0\serialport\cwriterasync\serialportwriter.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void PrintValues(const double area, const int Y, const int X, const double angle);
    void Video();
    void CalibCamera();             //калибровка камеры
    void CalibCamera2();             //калибровка камеры
    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;

    cv::Mat map1;     //коэф для калибровки каммеры
    cv::Mat map2;   //коэф для калибровки каммеры
};

#endif // MAINWINDOW_H
