#include <iostream>
#include <D:/openCV/opencv/build/install/include/opencv2/opencv.hpp>
#include <D:/openCV/opencv/build/install/include/opencv2/highgui.hpp>
#include "D:/openCV/opencv/build/install/include/opencv2/imgproc.hpp"
#include <D:/openCV/opencv/build/install/include/opencv2/video.hpp>
#include <D:/openCV/opencv/build/install/include/opencv2/videoio.hpp>

#include <QtSerialPort/QSerialPortInfo>
#include <D:\Qt\Examples\Qt-5.13.0\serialport\creaderasync\serialportreader.h>
#include <D:\Qt\Examples\Qt-5.13.0\serialport\cwriterasync\serialportwriter.h>



using namespace cv;
using namespace std;

//QSerialPort serial;

void Move(int x, int left, int right, Mat in_frame);

QSerialPort serial;
void SetPort()
{

    serial.setPortName("com3");
    serial.setBaudRate(9600);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);
    serial.open(QIODevice::ReadWrite);
}

void Video()
{
    vector<vector<Point>> contours;        //переменные для определение квадрата

    Mat in_frame;
    Mat in_frame2;
    int right = 0;
    int left = 0;

    VideoCapture inVid(0);
    if (!inVid.isOpened())
    {
        cout << "Камера не готова";
        return;
    }



    namedWindow("win2");
    namedWindow("image");
    while (inVid.read(in_frame))
    {
        inRange(in_frame, Scalar(30, 80, 0), Scalar(170, 255, 50), in_frame2);              //B,G,R достаём нужный цвет
        findContours( in_frame2, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));    //находит контур

        Mat tmp = Mat::zeros(in_frame2.size(), CV_8UC3 );                                   //копирование изображения
        std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
        std::vector<cv::Rect> boundRect( contours.size() );

        if (right <= 0 || left <= 0)
        {
            right = in_frame.cols/2 + 50;
            left = in_frame.cols/2 - 50;
        }

        if (contours.size() == 0)
        {
            serial.write("S");
        }
        for( int i = 0; i < contours.size(); i++ )
        {
            cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 7, true );
            boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );

            double area1 = contourArea(contours[i]);

            if (area1 > 1500 && area1 <20000 && boundRect[i].height > boundRect[i].width)
            {
                int x = boundRect[i].width/2 + boundRect[i].x;  //получаем координ центра по x
                int y = boundRect[i].height/2 + boundRect[i].y; //получаем координ центра по y
                Point center(x, y);                             //присваевоем координаты точке center
                circle(in_frame, center, 5, Scalar(0, 0, 255), 3, 8, 0);      //выводим центер
                rectangle( tmp, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 0, 255), 2, 8, 0 );
                Move (x, left, right, in_frame);
            }
        }
        in_frame = in_frame + tmp;
        imshow("image", in_frame);
        imshow("win2", in_frame2);
        if (waitKey (1000/30) >= 0)
        {
            break;
        }
    }
}

void Move(int x, int left, int right, Mat in_frame)
{
    if (x <= left && x >= left -20)
    {
        line(in_frame, Point(left, 0), Point(left, in_frame.rows), cv::Scalar(255, 0, 0), 1);
        line(in_frame, Point(right, 0), Point(right, in_frame.rows), cv::Scalar(0,0,255), 1);
        cout <<"slow left"<<endl;
        serial.write("l");
    }
    else if (x >= right && x <= right + 20)
    {
        line(in_frame, Point(left, 0), Point(left, in_frame.rows), cv::Scalar(0, 0, 255), 1);
        line(in_frame, Point(right, 0), Point(right, in_frame.rows), cv::Scalar(255,0,0), 1);
        cout <<"slow right"<<endl;
        serial.write("r");
    }
    else if (x <= left - 20)
    {
     line(in_frame, Point(left, 0), Point(left, in_frame.rows), cv::Scalar(255, 0, 0), 1);
     line(in_frame, Point(right, 0), Point(right, in_frame.rows), cv::Scalar(0,0,255), 1);
     cout <<"left"<<endl;
     serial.write("L");
    }
    else if (x >= right + 20)
    {
     line(in_frame, Point(left, 0), Point(left, in_frame.rows), cv::Scalar(0, 0, 255), 1);
     line(in_frame, Point(right, 0), Point(right, in_frame.rows), cv::Scalar(255,0,0), 1);
     cout <<"right"<<endl;
     serial.write("R");
    }
    else
    {
     line(in_frame, Point(right, 0), Point(right, in_frame.rows), cv::Scalar(255,0,0), 1);
     line(in_frame, Point(left, 0), Point(left, in_frame.rows), cv::Scalar(255,0,0), 1);
     cout <<"stop"<<endl;
     serial.write("S");
    }
}
