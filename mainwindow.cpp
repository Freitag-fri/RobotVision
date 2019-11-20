#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "vision.cpp"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    CalibCamera();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    SetPort();
    Video();
    //CalibCamera2();
}
