#include "../include/mini_robot_master/mainwindow.h"
#include "ui_mainwindow.h"


#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>



using namespace std;
using namespace Qt;

string STR_USB_CAM = "USB CAM";


QHostAddress RO = QHostAddress("192.168.188.100");
QHostAddress OP = QHostAddress("192.168.188.253");
//QHostAddress RO = QHostAddress("192.168.0.60");
//QHostAddress OP = QHostAddress("192.168.0.130");
//QHostAddress RO = QHostAddress("192.168.0.66");
//QHostAddress OP = QHostAddress("192.168.0.77");

uint16_t ROBOT_PORT = 9999; //RX, TX
uint16_t other_PORT = 8888;
uint16_t LIDAR_PORT = 4055;
uint16_t MAP_PORT = 4033;

bool isRecved = false;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowIcon(QIcon(":/images/icon.jpg"));
    ros::init(init_argc,init_argv,"mini_robot_master");
    if ( ! ros::master::check() ) return;
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    //##################################################################################//
    // Subscriber
    Cam_sub = n.subscribe("/usb_cam/image_raw",1, &MainWindow::Cam_Callback,this);
    //Lidar_sub = n.subscribe("/scan", 1, &MainWindow::scan_data_send, this);
    Map_sub = n.subscribe("/map", 1, &MainWindow::map_data_send, this);
    Encoder_sub = n.subscribe("/encoder_state", 1, &MainWindow::Encoder_Callback, this);
    
    //##################################################################################//
    // Publisher
    Motor_pub = n.advertise<mini_serial::motor_msg>("/motor_val", 1);
   
    //##################################################################################//
    // Socket
    socket = new QUdpSocket(this);
    m_pUdpSocket = new QUdpSocket(this);
    //lidar_socket = new QUdpSocket(this);
    map_socket = new QUdpSocket(this);

    if(socket->bind(RO, ROBOT_PORT, QUdpSocket::ShareAddress)){
        qDebug()<<"text message socket bind success"<<endl;
        ui->connection->setText("OK");
        ui->ip->setText(RO.toString());
        connect(socket, SIGNAL(readyRead()), this, SLOT(udp_read()));
        connection = true;
    }
    m_pUdpSocket->bind(RO, other_PORT, QUdpSocket::ShareAddress);
    //lidar_socket->bind(RO, LIDAR_PORT, QUdpSocket::ShareAddress);
    map_socket->bind(RO, MAP_PORT, QUdpSocket::ShareAddress);

    qDebug()<<"setup ready running..."<<endl;
    th=std::thread(&MainWindow::run,this);

    //##################################################################################//
    //rviz
    // Construct and lay out render panel.
    render_panel_ = new rviz::RenderPanel();
    ui->rviz_layout->addWidget(render_panel_);

    manager_ = new rviz::VisualizationManager(render_panel_);
    render_panel_->initialize(manager_->getSceneManager(), manager_);
    manager_->initialize();
    manager_->startUpdate();
    map_ = manager_->createDisplay("rviz/Map", "adjustable map", true);
    map_ -> subProp("Topic")->setValue("/map");


    // Create a Grid display.
    grid_ = manager_->createDisplay("rviz/Grid", "adjustable grid", true);
    ROS_ASSERT( grid_ != NULL );

    // Configure the GridDisplay the way we like it.
    grid_->subProp("Line Style")->setValue("Billboards");
    grid_->subProp( "Line Style" )->subProp( "Line Width" )->setValue( 40 / 100.0f );
    grid_->subProp( "Cell Size" )->setValue( 100 / 10.0f );

    //##################################################################################//
    return;
}

MainWindow::~MainWindow(){
    if(ros::isStarted()) {
        ROS_INFO("exiting");
        ros::shutdown(); // explicitly needed since we use ros::start();
    }
    th.join();
    delete ui;
    delete manager_;
}

void MainWindow::udp_write(QString text){
    QByteArray packet;
    packet.append(text);
    //qDebug() << "Message from: udp_write";
    socket->writeDatagram(packet, OP, ROBOT_PORT);
}

void MainWindow::udp_write_(QByteArray data, uint16_t port, QUdpSocket &socket){
    QByteArray packet;
    packet.append(data);
    socket.writeDatagram(packet, OP, port);
    usleep(1);
}

void MainWindow::run(){
    ros::Rate loop_rate(33);
    while ( ros::ok() ) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MainWindow::udp_read(){
    QByteArray buffer;
    buffer.resize(socket->pendingDatagramSize());
    socket->readDatagram(buffer.data(), buffer.size(), &OP, &ROBOT_PORT);
    ui->log->append("----------------------");
    ui->log->append(OP.toString());
    ui->log->append("Message port: 9999");
    ui->log->append("Message: "+buffer);
    ui->log->append("----------------------");
}


bool MainWindow::crc8_check(QByteArray *data){
    uint8_t end = data->at(data->size()-1);
    *data->remove(data->size()-1,1);
    if(end != CRC8(true).calculate(*data)) return false;
    else return true;
}

void MainWindow::crc8_input(QByteArray *data){
    uint8_t crc_result = CRC8(true).calculate(*data);
    data->push_back(crc_result);
}


void MainWindow::Cam_Callback(const sensor_msgs::ImageConstPtr& msg_img){
    if(Cam_img == NULL && !isRecved){
        Cam_img = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::BGR8)->image);
        if(Cam_img !=NULL){
            isRecved =true;
            Original = Cam_img->clone();
            cv::resize(Original, Original, cv::Size(320, 240));

            vector<int> param = vector<int>(2);
            param[0]=CV_IMWRITE_JPEG_QUALITY;
            param[1]=70;
            imencode(".jpg",Original, usb1_buff, param);

            QImage usb_image((const unsigned char*)(Original.data), Original.cols, Original.rows, QImage::Format_RGB888);
            ui->robot_cam->setPixmap(QPixmap::fromImage(usb_image.rgbSwapped()));
            QByteArray array;
            for(auto val: usb1_buff) array.push_back(val);
            int sendok;
            sendok = m_pUdpSocket->writeDatagram(array.data(),array.size(),OP, other_PORT);
            while (sendok==-1) sendok = m_pUdpSocket->writeDatagram(array.data(),array.size(),OP, other_PORT);
            udp_write("image from" + RO.toString());
            usleep(1);
            isRecved=false;
            delete Cam_img;
            if(Cam_img!=NULL)Cam_img=NULL;
        }
    }
}

void MainWindow::scan_data_send(const sensor_msgs::LaserScan& scan){
    ROS_INFO("recieved laser scan data");
    QByteArray scan_array;
    sensor_msgs::LaserScan _scan;
    _scan = scan;
    uint8_t data[4];
    for(int i = 0 ; i <scan.ranges.size() ; i++){
        std::memcpy(&data, &_scan.ranges[i], 4);
        for(int j = 0 ; j < 4 ; j++){
            scan_array.push_back((char)data[j]);
        }
    }
    crc8_input(&scan_array);
    udp_write_(scan_array, LIDAR_PORT, *lidar_socket);
    scan_array.clear();
    ROS_INFO("send scan data");
}

void MainWindow::map_data_send(const nav_msgs::OccupancyGrid::ConstPtr& map){
    QByteArray map_array;
    uint8_t data[8];

    std::memcpy(&data, &map->info.width,8);
    for(int i=0;i<8;i++) map_array.push_back((char)data[i]);
    std::memcpy(&data, &map->info.height,8);
    for(int i=0;i<8;i++) map_array.push_back((char)data[i]);
    std::memcpy(&data, &map->info.origin.position.x,8);
    for(int i=0;i<8;i++) map_array.push_back((char)data[i]);
    std::memcpy(&data, &map->info.origin.position.y,8);
    for(int i=0;i<8;i++) map_array.push_back((char)data[i]);
    std::memcpy(&data, &map->info.origin.position.z,8);
    for(int i=0;i<8;i++) map_array.push_back((char)data[i]);
    std::memcpy(&data, &map->info.origin.orientation.x,8);
    for(int i=0;i<8;i++) map_array.push_back((char)data[i]);
    std::memcpy(&data, &map->info.origin.orientation.y,8);
    for(int i=0;i<8;i++) map_array.push_back((char)data[i]);
    std::memcpy(&data, &map->info.origin.orientation.z,8);
    for(int i=0;i<8;i++) map_array.push_back((char)data[i]);
    std::memcpy(&data, &map->info.origin.orientation.w,8);
    for(int i=0;i<8;i++) map_array.push_back((char)data[i]);

    //data size
    int data_size=map->data.size();
    std::memcpy(&data, &data_size,8);
    for(int i=0;i<8;i++) map_array.push_back((char)data[i]);

    //data//
    for(int i=0; i<map->data.size(); i++){
        map_array.push_back(map->data[i]);
    }

    map_array = qCompress(map_array, 9);
    crc8_input(&map_array);
    udp_write_(map_array, MAP_PORT, *map_socket);
    map_array.clear();
    ROS_INFO("send map data");
}

void MainWindow::Encoder_Callback(const mini_serial::encoder_msgPtr &msg){
    encoder_val_L = msg->encoder_L;
    encoder_val_R = msg->encoder_R;
    // ui->encoder_value_L->setText(encoder_val_L.toString());
    // ui->encoder_value_R->setText(encoder_val_R.toString());
}

void MainWindow::on_pushButton_clicked(){
    text_data.clear();
    text_data.append("//"+ui->send_text->text());
    udp_write(text_data);
    ui->send_log->append("ROBOT : "+ui->send_text->text());
    ui->send_text->clear();
}

void MainWindow::on_pushButton_2_clicked(){
    ui->send_log->clear();
}
