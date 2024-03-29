#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QDebug>
#include <QMainWindow>
#include <iostream>
#include <cmath>
#include <map>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/video.hpp>
#include <opencv2/videoio.hpp>

#include <QtSerialPort/QSerialPortInfo>
#include <D:\Qt\Examples\Qt-5.12.2\serialport\creaderasync\serialportreader.h>
#include <D:\Qt\Examples\Qt-5.12.2\serialport\cwriterasync\serialportwriter.h>

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
    void PrintValues(const double area, const int Y, const int X, const double angle, const double test);
    void PrintValues(std::pair<int, int> Coordinates);
    void Video();
    void CalibBright(cv::Mat &frame);
    void CalibCamera();             //калибровка камеры
    std::pair<int, int> Coordinates(cv::Rect &boundRect, cv::Mat &frame, cv::Mat &tmp);
    void on_pushButton_clicked();
    void Move(std::pair<int, int> Coordinates);

    void on_Start_Work_clicked();

    void on_Reset_pos_clicked();

private:
    Ui::MainWindow *ui;

    cv::Mat map1;       //коэф для калибровки каммеры
    cv::Mat map2;       //коэф для калибровки каммеры

    double koefR;
    double koefG;
    double koefB;
};

#endif // MAINWINDOW_H
