#include "mainwindow.h"
#include "ui_mainwindow.h"

using namespace std;
#define PI 3.14159265     // число ПИ

void Move(pair<int, int> coordinates);

static QSerialPort serial;
void SetPort()
{    
    serial.setPortName("com7");
    serial.setBaudRate(9600);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);
    serial.open(QIODevice::ReadWrite);
}

void MainWindow::Video()
{
    vector<vector<cv::Point>> contours;        //переменные для определение квадрата
    cv::Mat in_frame;
    cv::Mat in_frame2;
    std::multimap<int, int> arrayCoordinates;               //координаты хранятся здесь

    cv::VideoCapture inVid(0);
    if (!inVid.isOpened())
    {
        cout << "Камера не готова";
        return;
    }

    cv::namedWindow("win2");
    cv::namedWindow("image");
    while (inVid.read(in_frame))
    {
        //                int cc = 25;            //33
        //                for(int i = cc; i < 640; i+= cc)
        //                {
        //                    line(in_frame, cv::Point(i,0), cv::Point(i,480), cv::Scalar(255, 0, 0), 1);
        //                }

        //                for(int i = cc; i < 480; i+= cc)
        //                {
        //                    line(in_frame, cv::Point(0,i), cv::Point(640,i), cv::Scalar(255, 0, 0), 1);
        //                }


        arrayCoordinates.clear();
        cv::rotate(in_frame,in_frame, cv::ROTATE_180);          //переворот камеры на 180 градусов


        CalibBright(in_frame);


        inRange(in_frame, cv::Scalar(32*koefB, 40*koefG, 143*koefR), cv::Scalar(68*koefB, 78*koefG, 189*koefR), in_frame2);              //B,G,R достаём нужный цвет
        findContours( in_frame2, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));    //находит контур

        cv::Mat tmp = cv::Mat::zeros(in_frame2.size(), CV_8UC3 );                                   //копирование изображения
        std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
        std::vector<cv::Rect> boundRect(contours.size());

        vector<vector<cv::Point>> contoursNew;
        for(unsigned int i = 0; i < contours.size(); i++ )
        {
            int area = contourArea(contours[i]);
            if (area > 400 && area <2500)
            {
                contoursNew.push_back(contours[i]);               //заполняем вектор подходящими контурами
            }
            ui->size->setText(QString::number(contoursNew.size()));
        }

        for(unsigned int i = 0; i < contoursNew.size(); i++ )
        {
            cv::approxPolyDP( cv::Mat(contoursNew[i]), contours_poly[i], 7, true );
            boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );

            std::pair<int,int> buf = Coordinates(boundRect[i], in_frame, tmp);
            if(buf.first > 10)
            {
                //PrintValues(contourArea(contoursNew[i]), deltaX, deltaY, angle, line);       //вывод значений на форму
                arrayCoordinates.insert(buf);
            }
        }

        if(arrayCoordinates.size() != 0)
        {
            std::pair<int,int> buff1(arrayCoordinates.begin()->first, arrayCoordinates.begin()->second);
            Move(buff1);        //отправка координат роботу
        }

        ui->textEdit->clear();              //очистка поля
        for(auto i : arrayCoordinates)
        {
            ui->textEdit->insertPlainText("line = " + QString::number(i.first) + ", andle =  " + QString::number(i.second) + '\n');
        }



        in_frame = in_frame + tmp;  //добавление квадратов к изображению

        // Mat image2;                                               //калибровка
        // remap(in_frame, image2, map1, map2, cv::INTER_CUBIC);     //
        // imshow("New", image2);                                   //
        imshow("image", in_frame);

        imshow("win2", in_frame2);

        if (cv::waitKey (1000/30) >= 0)
        {
            break;
        }
    }
}

void MainWindow::CalibBright(cv::Mat &frame)
{

//    for(int x = 130; x < 180; x++)
//    {
//        for(int y = 40; y < 90; y++)
//        {
//           frame.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(255, 255, 255);      //перекрашиваем
//        }
//    }


    const int rM = 140;         //значения каналов макета
    const int gM = 48;          //
    const int bM = 44;          //

    int R = 0;
    int G = 0;
    int B = 0;
    int i = 0;

    for(int x = 130; x < 180; x++)
    {
        for(int y = 40; y < 90; y++)
        {
            cv::Vec3b pValVec = frame.at<cv::Vec3b>(cv::Point(x, y));

            B += pValVec[0];
            G += pValVec[1];
            R += pValVec[2];
            i++;
        }
    }
    B /= i;
    G /= i;
    R /= i;

    if(R > G && R > B)
    {
        koefR = (double)R/rM;
        koefG = (double)G/gM;
        koefB = (double)B/bM;
    }
    else
    {
      koefR = 1;
      koefG = 1;
      koefB = 1;
    }



//    ui->Bb->setText(QString::number(koefB));
//    ui->Gg->setText(QString::number(koefG));
//    ui->Rr->setText(QString::number(koefR));
    ui->Bb->setText(QString::number(B));
    ui->Gg->setText(QString::number(G));
    ui->Rr->setText(QString::number(R));


}

void MainWindow::CalibCamera()
{
    using namespace cv;
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

void testCom()
{
    while(true)
    {
        if(!serial.atEnd())
        {
            QByteArray hypBuffer = serial.readAll();
            //hypBuffer.size();
            QString temp = QString::fromStdString(hypBuffer.toStdString());
            qDebug() << hypBuffer[0];
        }
    }
}

std::pair<int, int> MainWindow::Coordinates(cv::Rect &boundRect, cv::Mat &frame, cv::Mat &tmp)
{
    int centre = boundRect.width/2 + boundRect.x;     //получаем координ центра по x
    int y = boundRect.height/2 + boundRect.y;         //получаем координ центра по y

    int deltaX = frame.cols/2 - centre + 15;         // 8 отклонение от центра камеры
    int deltaY =  y -238;                          //значение длины (260 - значение что б получить 0 робота)

    double angle = (double)deltaX/(deltaY + 260); //260 min радиус робота
    angle = atan(angle) * (-180.0 / PI);                //находим угол поворота

    double line =(abs(deltaX) / sin(abs(angle) * PI/180)) - 260;    //считаем необходимое перемещение
    line /= 1.32;

    angle /= 1.15;                  //костыли
    if (angle < -30) line -=20;     //

    cv::Point center(centre, y);                                //присваевоем координаты точке center
    circle(frame, center, 5, cv::Scalar(0, 0, 255), 3, 8, 0);                                //выводим центер
    rectangle( tmp, boundRect.tl(), boundRect.br(), cv::Scalar(0, 0, 255), 2, 8, 0 ); //выводим круг

    PrintValues(0, deltaX, deltaY, angle, line);

    cv::Point center2(frame.cols/2 + 15, -240);
    circle(frame, center2, 260 + 238, cv::Scalar(0, 255, 255), 2, 8, 0);
    circle(frame, center2, 260 + 238 +185 * 1.32, cv::Scalar(0, 255, 255), 2, 8, 0);

    return std::pair<int, int> (line, angle);
}

void MainWindow::PrintValues(const double area, const int Y, const int X, const double angle, const double test)
{
    ui->PrintArea->setText(QString::number(area));
    ui->PrintX->setText(QString::number(Y));
    ui->PrintY->setText(QString::number(X));
    ui->PrintAngle->setText(QString::number(angle));
    ui->PrintAngle_2->setText(QString::number(test));

}

void MainWindow::PrintValues(std::pair<int, int> Coordinates)
{
    int line = Coordinates.first;
    int angle = Coordinates.second;
    ui->PrintAngle->setText(QString::number(angle));
    ui->PrintX->setText(QString::number(line));
}

void MainWindow::Move(pair<int, int> Coordinates)
{
    if(ui->startWork->isChecked())
    {
        if(!serial.atEnd())
        {
            QByteArray hypBuffer = serial.readAll();
            //hypBuffer.size();
            QString temp = QString::fromStdString(hypBuffer.toStdString());
            qDebug() << hypBuffer;
            if(temp == "v100")
            {
                qDebug() << "true";

                char messenger[13];
                int line = Coordinates.first;
                int angle = Coordinates.second;

                sprintf(messenger, "a%03db%03dc%03d", angle, line, line +angle);   //кодировка координат a030b102c132

                serial.write(messenger);
                cout << messenger <<endl;
            }
            else
            {
                qDebug() << "false";
            }
        }

    }
}

void MainWindow::on_Start_Work_clicked()
{
    //            if(!serial.atEnd())
    //            {
    //                QByteArray hypBuffer = serial.readAll();
    //                //hypBuffer.size();
    //                QString temp = QString::fromStdString(hypBuffer.toStdString());
    //                qDebug() << hypBuffer;
    //            }
    if(ui->Start_Work->isChecked())
    {
        char messenger[5] = {"s111"};
        serial.write(messenger);
        qDebug() << "true";
    }
    else
    {
        char messenger[5] = {"s100"};
        serial.write(messenger);
        qDebug() << "false";
    }
}

void MainWindow::on_Reset_pos_clicked()
{
    char messenger[5] = {"s200"};
    serial.write(messenger);
    qDebug() << "Reset_pos";
}

