#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace cv;
using namespace std;
#define PI 3.14159265     // число ПИ

void Move2(int param, int angle);
void Move2(int angle);

static QSerialPort serial;
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

void MainWindow::Video()
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
        //        int cc = 25;
        //        for(int i = cc; i < 640; i+= cc)
        //        {
        //          line(in_frame, Point(i,0), Point(i,480), cv::Scalar(255, 0, 0), 1);
        //        }

        //        for(int i = cc; i < 480; i+= cc)
        //        {
        //          line(in_frame, Point(0,i), Point(640,i), cv::Scalar(255, 0, 0), 1);
        //        }

        /*
        cv::rotate(in_frame,in_frame, cv::ROTATE_180);
        inRange(in_frame, Scalar(25, 30, 0), Scalar(80, 255, 55), in_frame2);              //B,G,R достаём нужный цвет
        findContours( in_frame2, contours, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));    //находит контур

        Mat tmp = Mat::zeros(in_frame2.size(), CV_8UC3 );                                   //копирование изображения
        std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
        std::vector<cv::Rect> boundRect(contours.size() );

        if (right <= 0 || left <= 0)
        {
            right = in_frame.cols/2 + 65;
            left = in_frame.cols/2 - 65;
        }

        vector<vector<Point>> contoursNew;
        for(unsigned int i = 0; i < contours.size(); i++ )
        {
            int area = contourArea(contours[i]);
            if (area > 1200 && area <2500)
            {
                contoursNew.push_back(contours[i]);               //заполняем вектор подходящими контурами
            }
            ui->size->setText(QString::number(contoursNew.size()));
        }

        if(!contoursNew.size())
        {Move2(0,0);}

        for(unsigned int i = 0; i < contoursNew.size(); i++ )
        {
            cv::approxPolyDP( cv::Mat(contoursNew[i]), contours_poly[i], 7, true );
            boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );

            int centre = boundRect[i].width/2 + boundRect[i].x;     //получаем координ центра по x
            int y = boundRect[i].height/2 + boundRect[i].y;         //получаем координ центра по y

            //int deltaY = in_frame.rows - y - 120;
            int deltaX = in_frame.cols/2 - centre -113 ;
            int deltaY = in_frame.rows - y -145;

            double angle = (double)deltaX/(deltaY + 200);   // ... + коррекция длины
            angle = atan(angle) * 180.0 / PI;              //находим угол поворота

            int hypotenuse = (deltaX*deltaX)+ (deltaY*deltaY);          //зачем???
            hypotenuse = sqrt(hypotenuse);

            PrintValues(contourArea(contoursNew[i]), deltaX, deltaY, angle);       //вывод значений на форму


            Point center(centre, y);                                //присваевоем координаты точке center
            circle(in_frame, center, 5, Scalar(0, 0, 255), 3, 8, 0);      //выводим центер
            rectangle( tmp, boundRect[i].tl(), boundRect[i].br(), cv::Scalar(0, 0, 255), 2, 8, 0 );

            if (deltaY > 15 && deltaY < 180)
            {
                if(ui->startWork->isChecked())
                {
                    Move2(deltaY,angle);
                }
            }
        }

        in_frame = in_frame + tmp;  //добавление квадратов к изображению
        */


        Mat image2;
        remap(in_frame, image2, map1, map2, cv::INTER_CUBIC);

        imshow("image", in_frame);
        imshow("New", image2);
        // imshow("win2", in_frame2);
        if (waitKey (1000/30) >= 0)
        {
            break;
        }

    }
}

void MainWindow::CalibCamera2()
{
    Mat image;
    namedWindow("image", WINDOW_AUTOSIZE);

    for(int i = 0; i < 9; i++)
    {
        string path = "D:\\1111\\marker_";
        path += to_string(i);
        path += ".jpg";
        image = imread(path);


        imshow("image", image);
        waitKey();
    }
}

void MainWindow::CalibCamera()
{
    namedWindow("Test", WINDOW_AUTOSIZE); //create a window called "MyVideo"

    int numCornersHor = 9; // Chessboard dimensions
    int numCornersVer = 6;
    Size boardSize = Size(numCornersHor, numCornersVer);

    Mat image;
    Mat gray_image;

    vector<Point3f> obj;
    vector<Point2f> corners;  // output vectors of image points

    vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > image_points;

    for (int i=0; i<boardSize.height; i++)
    {
        for (int j=0; j<boardSize.width; j++)
        {
            obj.push_back(Point3f(i, j, 0.0f));
        }
    }

    for (int i = 0; i < 30; i++)
    {
        string path = "D:\\qtProgram\\build-Calib_Camera_New-Desktop_Qt_5_12_2_MinGW_32_bit-Debug\\Camera_calib_";
        path += to_string(i);
        path += ".jpg";
        image = imread(path);
        imshow("Test", image);
        // make grayscale frame version for conerSubPix
        cvtColor(image, gray_image, COLOR_BGR2GRAY);

        // Get the chessboard corners
        bool found = findChessboardCorners(image, boardSize, corners);

        if (found)
        {
            // Increase accuracy by subpixels
            cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 0.1));
            drawChessboardCorners(gray_image, boardSize, corners, found);
            imshow("Grayscale", gray_image);

            image_points.push_back(corners);
            object_points.push_back(obj);

        }
    }

    //////////// BEGIN CALIBRATION ////////////////////////
    cout << "Callibration started..." << endl;

    Mat cameraMatrix = Mat(3, 3, CV_64F);
    cameraMatrix.at<double>(0,0) = 1.0;
    Mat distCoeffs;
    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<Mat> rvecs;
    vector<Mat> tvecs;

    Size imageSize = image.size();

    calibrateCamera(object_points, image_points, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

    FileStorage fs;
    fs.open("D:\\1.txt", FileStorage::WRITE | FileStorage::READ);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs << "rvecs" << rvecs;
    fs << "tvecs" << tvecs;

    Mat undistorted;
    undistort(image, undistorted, cameraMatrix, distCoeffs);

    Mat newCamMatrix;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), newCamMatrix, cv::Size(image.cols, image.rows), CV_32FC1, map1, map2);

    cout << "Callibration ended." << endl;
}

void MainWindow::PrintValues(const double area, const int Y, const int X, const double angle)
{
    ui->PrintArea->setText(QString::number(area));
    ui->PrintX->setText(QString::number(Y));
    ui->PrintY->setText(QString::number(X));
    ui->PrintAngle->setText(QString::number(angle));
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
    char messenger[13];

    QByteArray hypBuffer = QByteArray::number(hypotenuse);
    sprintf(messenger, "a%03db%03dc%03d", angle, hypotenuse, hypotenuse +angle);   //кодировка координат a030b102c132

    serial.write(messenger);
    cout << messenger <<endl;

}
