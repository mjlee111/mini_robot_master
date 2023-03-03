#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#ifndef Q_MOC_RUN
// basic header
#include <ros/ros.h>
#include <ros/network.h>
#include <iostream>
#include <thread>

// QT header
#include <QtGui>
#include <QThread>
#include <QStringListModel>
#include <QObject>
#include <QtNetwork/QUdpSocket>
#include <QtNetwork/QHostAddress>
#include <QDialog>
#include <QDebug>
#include <QString>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>

// OpenCV header
#include <opencv2/opencv.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgcodecs.hpp"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>

// rviz header
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/default_plugin/map_display.h"


// msgs header
#include <mini_serial/encoder_msg.h>
#include <mini_serial/motor_msg.h>
#include <sensor_msgs/Joy.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/OccupancyGrid.h"


// additional header
#include "../crc/crc.h"
#include <string>
#include <math.h>

#endif

namespace Ui {
using namespace cv;
using namespace std;
using namespace Qt;

class MainWindow;

class Display;
class RenderPanel;
class VisualizationManager;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    virtual ~MainWindow();
    bool init();

    //////////////////////////////////////////////////////////////////////////////////////
    void udp_write(QString text);
    void udp_write_(QByteArray text, uint16_t port, QUdpSocket &socket);
    bool crc8_check(QByteArray *data);
    void crc8_input(QByteArray *data);
    //////////////////////////////////////////////////////////////////////////////////////

    cv::Mat *Cam_img=NULL;
    cv::Mat Original;
    std::vector<uchar> usb1_buff;

    void run();
    bool ui_check = false;


    float leftStickY = 0, leftStickX = 0, rightStickY = 0, rightStickX = 0, l2 = 0, r2 = 0;
    int arrowX = 0, arrowY = 0, buttonSq = 0, buttonX = 0, buttonO = 0, buttonTr = 0, l1 = 0, r1 = 0, buttonShare = 0, buttonOption = 0, buttonTouch = 0;

Q_SIGNALS:
    void rosShutdown();
    void view_SIGNAL(void);
    


public Q_SLOTS:
    void udp_read();
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_controller_mode_clicked();
    void motor_L_publish(int val);
    void motor_R_publish(int val);
    
    
private:
    bool connection = false;
    bool controller_stat = false;
    
    
    Ui::MainWindow *ui;
    //##################################################################################//
    rviz::VisualizationManager* manager_;
    rviz::RenderPanel* render_panel_;
    rviz::Display* grid_;
    rviz::Display* map_;
    //##################################################################################//
    QUdpSocket *socket;
    QUdpSocket *m_pUdpSocket;
    QUdpSocket *lidar_socket;
    QUdpSocket *map_socket;

    QString text_data = 0;
    QByteArray buffer;
    quint8 image_cnt_past=0;
    quint8 image_cnt_now=0;

    int encoder_val_L = 0;
    int encoder_val_R = 0;
    //##################################################################################//
    QTimer *_5ms_Timer, *_1s_Timer;
    //##################################################################################//
    ros::Subscriber Cam_sub;
    ros::Subscriber Lidar_sub;
    ros::Subscriber Map_sub;
    ros::Subscriber Encoder_sub;
    ros::Subscriber Joy_sub;
    //##################################################################################//
    ros::Publisher Motor_pub;
    //##################################################################################//
    mini_serial::motor_msg motor_msgs;
    //##################################################################################//
    void Cam_Callback(const sensor_msgs::ImageConstPtr& msg_img);
    void scan_data_send(const sensor_msgs::LaserScan& scan);
    void map_data_send(const nav_msgs::OccupancyGrid::ConstPtr& map);
    void Encoder_Callback(const mini_serial::encoder_msgPtr &msg);
    void Joystick_Callback(const sensor_msgs::Joy::ConstPtr &joy);
    //##################################################################################//
    int init_argc;
    char** init_argv;
    std::thread th;
    //##################################################################################//
private slots:
};

#endif // MAINWINDOW_H
