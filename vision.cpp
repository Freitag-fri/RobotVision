#include <iostream>
#include <D:/openCV/opencv/build/install/include/opencv2/opencv.hpp>
#include <D:/openCV/opencv/build/install/include/opencv2/highgui.hpp>
#include "D:/openCV/opencv/build/install/include/opencv2/imgproc.hpp"
#include <D:/openCV/opencv/build/install/include/opencv2/video.hpp>
#include <D:/openCV/opencv/build/install/include/opencv2/videoio.hpp>

#include <QtSerialPort/QSerialPortInfo>
#include <D:\Qt\Examples\Qt-5.12.2\serialport\creaderasync\serialportreader.h>
//#include <D:\Qt\Examples\Qt-5.12.0\serialport\cwriterasync\serialportwriter.h>

#include <cmath>

using namespace cv;
using namespace std;
#define PI 3.14159265     // число ПИ

//QSerialPort serial;

void Move(int x, int left, int right, Mat in_frame, int area);
void Move2(int param, int angle);
void Move2(int angle);

QSerialPort serial;
void SetPort()
{

    serial.setPortName("com4");
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
        cv::rotate(in_frame,in_frame, cv::ROTATE_180);
        inRange(in_frame, Scalar(30, 80, 0), Scalar(130, 255, 70), in_frame2);              //B,G,R достаём нужный цвет
        findContours( in_frame2, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));    //находит контур

        Mat tmp = Mat::zeros(in_frame2.size(), CV_8UC3 );                                   //копирование изображения
        std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
        std::vector<cv::Rect> boundRect( contours.size() );

        if (right <= 0 || left <= 0)
        {
            right = in_frame.cols/2 + 65;
            left = in_frame.cols/2 - 65;
        }

        if (contours.size() == 0)
        {
            serial.write("S");
        }
        bool presenceContours = false;



        for(unsigned int i = 0; i < contours.size(); i++ )
        {

            cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 7, true );
            boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );

            int area = contourArea(contours[i]);

            if (area > 1000 && area <25000 /*&& boundRect[i].height > boundRect[i].width*/)
            {
                presenceContours = true;
                string s = to_string(area);
                putText(in_frame, "Area " + s, Point(boundRect[i].x -25, boundRect[i].y -15),
                    FONT_HERSHEY_COMPLEX_SMALL, 1.2, Scalar(185,32,233), 1);                //вывод площади

                int centre = boundRect[i].width/2 + boundRect[i].x;     //получаем координ центра по x
                int y = boundRect[i].height/2 + boundRect[i].y;         //получаем координ центра по y

                //int deltaY = in_frame.rows - y - 120;
                int deltaX = in_frame.cols/2 - centre -70 ;
                int deltaY = in_frame.rows - y -120 + 200;

                double angle = (double)deltaX/deltaY;
                angle = atan(angle) * 180.0 / PI;              //находим угол поворота

                deltaY -= 200;

                int hypotenuse = (deltaX*deltaX)+ (deltaY*deltaY);
                hypotenuse = sqrt(hypotenuse);

                string deltX = to_string(deltaX);
                putText(in_frame, "X " + deltX, Point(10, 40),
                    FONT_HERSHEY_COMPLEX_SMALL, 1.2, Scalar(0,0,255), 1);

                string deltY = to_string(deltaY);
                putText(in_frame, "Y " + deltY, Point(10, 80),
                    FONT_HERSHEY_COMPLEX_SMALL, 1.2, Scalar(0,0,255), 1);

                string hypot = to_string(hypotenuse);
                putText(in_frame, "hypotenuse = " + hypot, Point(10, 120),
                    FONT_HERSHEY_COMPLEX_SMALL, 1.2, Scalar(0,0,255), 1);

                string angleStr = to_string(angle);
                putText(in_frame, "angle " + angleStr, Point(10, 200),
                    FONT_HERSHEY_COMPLEX_SMALL, 1.2, Scalar(0,0,255), 1);


                Point center(centre, y);                                //присваевоем координаты точке center
                circle(in_frame, center, 5, Scalar(0, 0, 255), 3, 8, 0);      //выводим центер
                rectangle( tmp, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 0, 255), 2, 8, 0 );
                //Move (centre, left, right, in_frame, area);

                if (/*hypotenuse > 15 && hypotenuse < 180 &&*/ deltaY > 15 && deltaY < 180)
                {
                   // hypotenuse -=370;
                    //hypotenuse *= 0.6;
                        //Move2(angle);
                   // angle *= 0.32;
                        Move2(deltaY,angle);
                }

               // else { Move2 (0);}
               // Move2(param);
            }
        }
        in_frame = in_frame + tmp;  //добавление квадратов к изображению
        imshow("image", in_frame);
        imshow("win2", in_frame2);
        if (waitKey (1000/30) >= 0)
        {
            break;
        }
    }
}


void Move2(int angle)
{

    QByteArray hypBuffer = QByteArray::number(angle);
    hypBuffer += "\n";
    serial.write(hypBuffer);
    cout << angle <<endl;
}

void Move2(int hypotenuse, int angle)
{

    QByteArray hypBuffer = QByteArray::number(hypotenuse);
    QByteArray angleBuffer = QByteArray::number(angle);
    hypBuffer += "q";
    hypBuffer += angleBuffer;
    hypBuffer += "\n";
    serial.write(hypBuffer);
    cout << hypotenuse <<endl;
}

void Move(int centre, int left, int right, Mat in_frame, int area)
{
    if (centre <= left && centre >= left -20)
    {
        line(in_frame, Point(left, 0), Point(left, in_frame.rows), cv::Scalar(255, 0, 0), 1);
        line(in_frame, Point(right, 0), Point(right, in_frame.rows), cv::Scalar(0,0,255), 1);
        cout <<"slow left"<<endl;
        serial.write("l");
    }

    else if (centre >= right && centre <= right + 20)
    {
        line(in_frame, Point(left, 0), Point(left, in_frame.rows), cv::Scalar(0, 0, 255), 1);
        line(in_frame, Point(right, 0), Point(right, in_frame.rows), cv::Scalar(255,0,0), 1);
        cout <<"slow right"<<endl;
        serial.write("r");
    }

    else if (centre <= left - 20)
    {
        line(in_frame, Point(left, 0), Point(left, in_frame.rows), cv::Scalar(255, 0, 0), 1);
        line(in_frame, Point(right, 0), Point(right, in_frame.rows), cv::Scalar(0,0,255), 1);
        cout <<"left"<<endl;
        serial.write("L");
    }

    else if (centre >= right + 20)
    {
        line(in_frame, Point(left, 0), Point(left, in_frame.rows), cv::Scalar(0, 0, 255), 1);
        line(in_frame, Point(right, 0), Point(right, in_frame.rows), cv::Scalar(255,0,0), 1);
        cout <<"right"<<endl;
        serial.write("R");
    }

    else if(centre > left && centre <right && area < 12000)
    {
        line(in_frame, Point(right, 0), Point(right, in_frame.rows), cv::Scalar(255,0,0), 1);
        line(in_frame, Point(left, 0), Point(left, in_frame.rows), cv::Scalar(255,0,0), 1);
        cout <<"go"<<endl;
        serial.write("G");
    }
    else
    {
        line(in_frame, Point(right, 0), Point(right, in_frame.rows), cv::Scalar(255,0,0), 1);
        line(in_frame, Point(left, 0), Point(left, in_frame.rows), cv::Scalar(255,0,0), 1);
        cout <<"stop"<<endl;
        serial.write("S");
    }
}
