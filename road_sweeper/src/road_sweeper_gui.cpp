#include "road_sweeper_gui.hpp"
#include <QApplication>

RoadSweeperGui::RoadSweeperGui(QWidget *parent)
    : QMainWindow(parent),
      startButton(new QPushButton("Start Launch File", this)),
      controlButton(new QPushButton("Control Auto Sweep", this)),
      launchProcess(nullptr)
{
    // Set window title
    setWindowTitle("Road Sweeper GUI");
    
    // Set window size
    resize(400, 300);
    
    // Set up the layout
    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(startButton);
    layout->addWidget(controlButton);

    QWidget *centralWidget = new QWidget(this);
    centralWidget->setLayout(layout);
    setCentralWidget(centralWidget);

    // Connect buttons to slots
    connect(startButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleLaunchFile);
    connect(controlButton, &QPushButton::clicked, this, &RoadSweeperGui::controlAutoSweep);

    // Create service client
    nh = ros::NodeHandle();
    client = nh.serviceClient<std_srvs::SetBool>("enable_auto_sweep");

    // Initialize variables
    autoSweepEnabled = false;
    setupLaunched = false;
}

RoadSweeperGui::~RoadSweeperGui()
{
    if (launchProcess)
    {
        launchProcess->terminate();
        launchProcess->waitForFinished();
        delete launchProcess;
    }
}

void RoadSweeperGui::toggleLaunchFile()
{
    if (setupLaunched)
    {
        QProcess killProcess;
        killProcess.start("bash", QStringList() << "-c" << "pkill -f 'roslaunch.*setup.launch'");
        killProcess.waitForFinished();

        // 终止gnome-terminal
        if (launchProcess) {
            launchProcess->terminate();
            launchProcess->waitForFinished();
            delete launchProcess;
            launchProcess = nullptr;
        }

        startButton->setStyleSheet("");
        setupLaunched = false;
    }
    else
    {
        launchProcess = new QProcess(this);
        QStringList arguments;
        arguments << "-e" << "roslaunch road_sweeper setup.launch";
        launchProcess->start("gnome-terminal", arguments);

        startButton->setStyleSheet("background-color: green");
        setupLaunched = true;
    }
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