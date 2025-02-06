#include "road_sweeper_gui.hpp"
#include <QApplication>
#include <QProcess>

RoadSweeperGui::RoadSweeperGui(QWidget *parent)
    : QMainWindow(parent),
      startButton(new QPushButton("Start Launch File", this)),
      controlButton(new QPushButton("Control Auto Sweep", this))
{
    // 设置窗口标题
    setWindowTitle("Road Sweeper GUI");
    
    // 设置窗口大小
    resize(400, 300);
    
    // Set up the layout
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(startButton);
    layout->addWidget(controlButton);

    QWidget *centralWidget = new QWidget(this);
    centralWidget->setLayout(layout);
    setCentralWidget(centralWidget);

    // Connect buttons to slots
    connect(startButton, &QPushButton::clicked, this, &RoadSweeperGui::startLaunchFile);
    connect(controlButton, &QPushButton::clicked, this, &RoadSweeperGui::controlAutoSweep);

    // Create service client
    nh = ros::NodeHandle();
    client = nh.serviceClient<std_srvs::SetBool>("enable_auto_sweep");
}

RoadSweeperGui::~RoadSweeperGui()
{
}

void RoadSweeperGui::startLaunchFile()
{
    QProcess *process = new QProcess(this);
    QStringList arguments;
    arguments << "-e" << "roslaunch road_sweeper setup.launch";
    process->start("gnome-terminal", arguments);
}

void RoadSweeperGui::controlAutoSweep()
{
    std_srvs::SetBool srv;
    srv.request.data = controlButton->isChecked();
    if (client.call(srv))
    {
        ROS_INFO("Service call succeeded: %s", srv.response.message.c_str());
    }
    else
    {
        ROS_ERROR("Service call failed");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "road_sweeper_gui");

    QApplication app(argc, argv);

    RoadSweeperGui gui;
    gui.show();

    std::thread spin_thread([]()
                            { ros::spin(); });
    spin_thread.detach();

    int result = app.exec();
    ros::shutdown();
    return result;
}