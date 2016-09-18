#include <QtCore>
#include <QtGui>
#include <QDebug>
#include <QLayout>
#include <iostream>

#include "mainwindow.h"

using namespace std;

MainWindow::MainWindow(QWidget* parent)
  : QMainWindow(parent)
{
    //Button to launch a user-defined app
    pushButton_24 = new QPushButton("Hello");
    connect(pushButton_24, SIGNAL(clicked()),
            this, SLOT(on_pushButton_24_clicked()));

    //Button to launch ROS launch
    pushButtonRos_ = new QPushButton("Launch ROS Launch");
    connect(pushButtonRos_, SIGNAL(clicked()),
            this, SLOT(on_pushButtonRos_clicked()));

    //Button to kill the ROS launch process
    pushButtonKill_ = new QPushButton("Kill ROS Launch");
    connect(pushButtonKill_, SIGNAL(clicked()),
            this, SLOT(on_pushButtonKill_clicked()));

    lineEdit_4   = new QLineEdit;
    lineEditRos_ = new QLineEdit;
    lineEditKill_= new QLineEdit;


    QWidget* central = new QWidget;
    QLayout* layout = new QVBoxLayout();
    layout->addWidget(pushButton_24);
    layout->addWidget(lineEdit_4);
    layout->addWidget(pushButtonRos_);
    layout->addWidget(lineEditRos_);
    layout->addWidget(pushButtonKill_);
    layout->addWidget(lineEditKill_);

    central->setLayout(layout);
    setCentralWidget(central);
}

MainWindow::~MainWindow()
{
}

void MainWindow::on_pushButton_24_clicked()
{
    helloProcess_ = new QProcess(this);
    connect(helloProcess_, SIGNAL(readyReadStandardOutput()),
            this, SLOT(outlog()));
    helloProcess_->start("../helloworld");
    rosProcess_->kill();

    // For debugging: Wait until the process has finished.
    if(!helloProcess_->waitForFinished())
        qDebug() << "Process HELLO error code:" <<
                    helloProcess_->error();
}

void MainWindow::on_pushButtonRos_clicked()
{
    rosProcess_ = new QProcess;
    QProcess * minProcess = new QProcess(this);
//    rosProcess_->setStandardOutputFile("./output.txt");
    connect(rosProcess_, SIGNAL(readyReadStandardOutput()),
            this, SLOT(outlogRos()));
    QStringList rosCmd;
    rosCmd << "source ~/hawaii_ws/devel/setup.bash"
           << "roslaunch carmen_navigation dis_navigation.launch";

//    rosProcess_->start("rosrun", QStringList()<<"rviz" <<"rviz");
//    rosProcess_->start("gnome-terminal --geometry=0x0+0+0 -x bash -c \"roslaunch carmen_sim navigation.launch\" ");

/*
   https://github.com/jordansissel/xdotool.git
   http://www.semicomplete.com/projects/xdotool/ The xdotool is needed to make the terminal minimized
   you could also install xdotool following: sudo apt-get install xdotool
   it works: xdotool windowminimize $(xdotool search --class 'gnome-terminal' |sort|tail -1)
   For details see: http://askubuntu.com/questions/488356/how-can-i-start-a-script-in-a-minimised-gnome-terminal
   To minimize the terminal: xdotool windowminimize $(xdotool search --class 'gnome-terminal' |sort|tail -1)
   (gnome-terminal --geometry=0x0+0+0 --command 'roslaunch carmen_sim navigation.launch' &) && sleep 0.5 && xdotool windowminimize $(xdotool search --class 'gnome-terminal' |sort|tail -1)
*/

//    rosProcess_->start("gnome-terminal --geometry=0x0+0+0 -x bash -c \"roslaunch carmen_sim navigation.launch\" ");
    rosProcess_->start("gnome-terminal --geometry=0x0+0+0 --command \" roslaunch carmen_sim navigation.launch\" ");
//    rosProcess_->start("gnome-terminal --geometry=0x0+0+0 --command 'roslaunch carmen_sim navigation.launch' ");

/*
   Try to minimize the terminal window, but all fail to do this. Why?
 */
//    rosProcess_->start("gnome-terminal --geometry=0x0+0+0 --command \"roslaunch carmen_sim navigation.launch\" && \"sleep 1\" && \"xdotool windowminimize $(xdotool search --class 'gnome-terminal' |sort|tail -1)\" ");
//    rosProcess_->start("gnome-terminal --geometry=0x0+0+0 --command ", QStringList()<<"roslaunch carmen_sim navigation.launch" << "sleep 1" << "xdotool windowminimize $(xdotool search --class 'gnome-terminal' |sort|tail -1)");
//    rosProcess_->start("gnome-terminal --geometry=0x0+0+0 --command \"roslaunch carmen_sim navigation.launch\" ");
    rosLaunchPid_ = rosProcess_->pid();

    qDebug() << "processId = " << rosProcess_->processId();
    qDebug() << "PID =" << rosLaunchPid_;


//    rosProcess_->terminate();
//    rosProcess_->kill();
    if(rosProcess_->waitForStarted())
    {
        qDebug() << "STARTED!";
//        minProcess->start("gnome-terminal --geometry=10x10+20+0 -x bash -c \"xdotool windowminimize $(xdotool search --class \"gnome-terminal\" \" ");

    }
    if(!rosProcess_->waitForFinished(-1))
        qDebug() << "Process ROS error code:" << rosProcess_->error();
//    if(!minProcess->waitForFinished(-1))
//        qDebug() << "Process Min error code:" << minProcess->error();
}

void MainWindow::on_pushButtonKill_clicked( )
{
//    rosProcess_->terminate();
//    rosProcess_->kill();
//    rosProcess_->close();

/*
 * Don't know why a bias exists between the actual ROS launch PID (get by 'ps -e') and the QProcess.pid reads.
 * The bias might be different depending on different ROS launchs.
*/

    int bias = 4;
    QProcess* killRosProcess = new QProcess;
    QString correctPid, cmd;

    cmd = QString("gnome-terminal --geometry=0x0+0+0 -x kill ")
            + correctPid.setNum(rosLaunchPid_ + bias);

    killRosProcess->start(cmd);

    qDebug() << "Kill the ROS launch process!";
}

void MainWindow::outlog()
{
    QString abc = helloProcess_->readAllStandardOutput();
//    emit outlogtext(abc);
    lineEdit_4->setText(abc);
    qDebug() << abc;
}

void MainWindow::outlogRos()
{
    QString str = rosProcess_->readAllStandardOutput();
//    emit outlogtext(abc);
    lineEditRos_->setText(str);
    qDebug() << str;
}
