#include "road_sweeper_gui.hpp"
#include <QApplication>
#include <QDebug>

RoadSweeperGui::RoadSweeperGui(QWidget *parent)
    : QMainWindow(parent)
{
    /*====== Widget setup ======*/
    /*=== Control tab ===*/
    // Buttons
    setupButton = new QPushButton("Setup", this);
    mapButton = new QPushButton("Map", this);
    localizationButton = new QPushButton("Localization", this);
    globalPlanningButton = new QPushButton("Global Planning", this);
    perceptionButton = new QPushButton("Perception", this);
    localPlanningButton = new QPushButton("Local Planning", this);
    cmdOutputButton = new QPushButton("Cmd Output", this);
    canNodeButton = new QPushButton("Cmd To Can", this);
    sweepNodeButton = new QPushButton("Auto Sweep Node", this);
    // Checkboxes
    autoSweepCheckBox = new QCheckBox("Auto Sweep", this);

    /*=== Display tab ===*/
    // Buttons
    rvizButton = new QPushButton("RViz", this);

    /*====== Layout setup ======*/
    QHBoxLayout *layout = new QHBoxLayout;
    
    /*=== Control tab ===*/
    QVBoxLayout *controlLayout = new QVBoxLayout;
    controlLayout->addWidget(setupButton);
    controlLayout->addWidget(mapButton);
    controlLayout->addWidget(localizationButton);
    controlLayout->addWidget(globalPlanningButton);
    controlLayout->addWidget(perceptionButton);
    controlLayout->addWidget(localPlanningButton);
    controlLayout->addWidget(cmdOutputButton);
    controlLayout->addWidget(canNodeButton);

    QHBoxLayout *sweepLayout = new QHBoxLayout;
    sweepLayout->addWidget(sweepNodeButton);
    sweepLayout->addWidget(autoSweepCheckBox);
    controlLayout->addLayout(sweepLayout);

    layout->addLayout(controlLayout);

    /*=== Display tab ===*/
    QVBoxLayout *displayLayout = new QVBoxLayout;
    displayLayout->addWidget(rvizButton);

    layout->addLayout(displayLayout);

    /*=== Central widget ===*/
    QWidget *centralWidget = new QWidget(this);
    centralWidget->setLayout(layout);
    setCentralWidget(centralWidget);

    /*====== Signal/slot connections ======*/
    /*=== Control tab ===*/
    // Buttons
    connect(setupButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleSetupLaunch);
    connect(mapButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleMapLaunch);
    connect(localizationButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleLocalizationLaunch);
    connect(globalPlanningButton, &QPushButton::clicked, this, &RoadSweeperGui::togglePlanningLaunch);
    connect(perceptionButton, &QPushButton::clicked, this, &RoadSweeperGui::togglePerceptionLaunch);
    connect(localPlanningButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleLocalPlanningLaunch);
    connect(cmdOutputButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleCmdOutputLaunch);
    connect(canNodeButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleCanNodeLaunch);
    connect(sweepNodeButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleSweepNodeLaunch);
    // Checkboxes
    connect(autoSweepCheckBox, &QCheckBox::clicked, this, &RoadSweeperGui::controlAutoSweep);

    /*=== Display tab ===*/
    // Buttons
    connect(rvizButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleRvizLaunch);

    /*====== ROS setup ======*/
    // Create service client
    nh = ros::NodeHandle();
    client = nh.serviceClient<std_srvs::SetBool>("enable_auto_sweep");

    /*====== Process setup ======*/
    /*=== Control tab ===*/
    setupProcess = nullptr;
    mapProcess = nullptr;
    localizationProcess = nullptr;
    globalPlanningProcess = nullptr;
    perceptionProcess = nullptr;
    localPlanningProcess = nullptr;
    cmdOutputProcess = nullptr;
    canNodeProcess = nullptr;
    sweepNodeProcess = nullptr;
    /*=== Display tab ===*/
    rvizProcess = nullptr;

    /*====== Launch status setup ======*/
    /*=== Control tab ===*/
    setupLaunched = false;
    mapLaunched = false;
    localizationLaunched = false;
    globalPlanningLaunched = false;
    perceptionLaunched = false;
    localPlanningLaunched = false;
    cmdOutputLaunched = false;
    canNodeLaunched = false;
    sweepNodeLaunched = false;
    /*=== Display tab ===*/
    rvizLaunched = false;
}

RoadSweeperGui::~RoadSweeperGui()
{
    bool launched[] = {setupLaunched, mapLaunched, localizationLaunched, globalPlanningLaunched, perceptionLaunched, localPlanningLaunched, cmdOutputLaunched, canNodeLaunched, sweepNodeLaunched, rvizLaunched};
    QProcess* processes[] = {setupProcess, mapProcess, localizationProcess, globalPlanningProcess, perceptionProcess, localPlanningProcess, cmdOutputProcess, canNodeProcess, sweepNodeProcess, rvizProcess};
    QString launchFiles[] = {SETUP_LAUNCH_FILE, MAP_LAUNCH_FILE, LOCALIZATION_LAUNCH_FILE, GLOBAL_PLANNING_LAUNCH_FILE, PERCEPTION_LAUNCH_FILE, LOCAL_PLANNING_LAUNCH_FILE, CMD_OUTPUT_LAUNCH_FILE, CAN_NODE_LAUNCH_FILE, SWEEP_NODE_LAUNCH_FILE, RVIZ_LAUNCH_FILE};
    for (int i = 0; i < 10; i++)
    {
        if (launched[i])
        {
            QProcess killProcess;
            killProcess.start("pkill", QStringList() << "-f" << launchFiles[i]);
            killProcess.waitForFinished();

            if (processes[i])
            {
                processes[i]->terminate();
                processes[i]->waitForFinished();
                delete processes[i];
                processes[i] = nullptr;
            }
        }
    }
}

void RoadSweeperGui::toggleSetupLaunch()
{
    toggleLaunch(setupProcess, setupButton, setupLaunched, SETUP_LAUNCH_FILE);
}

void RoadSweeperGui::toggleMapLaunch()
{
    toggleLaunch(mapProcess, mapButton, mapLaunched, MAP_LAUNCH_FILE);
}

void RoadSweeperGui::toggleLocalizationLaunch()
{
    toggleLaunch(localizationProcess, localizationButton, localizationLaunched, LOCALIZATION_LAUNCH_FILE);
}

void RoadSweeperGui::togglePlanningLaunch()
{
    toggleLaunch(globalPlanningProcess, globalPlanningButton, globalPlanningLaunched, GLOBAL_PLANNING_LAUNCH_FILE);
}

void RoadSweeperGui::togglePerceptionLaunch()
{
    toggleLaunch(perceptionProcess, perceptionButton, perceptionLaunched, PERCEPTION_LAUNCH_FILE);
}

void RoadSweeperGui::toggleLocalPlanningLaunch()
{
    toggleLaunch(localPlanningProcess, localPlanningButton, localPlanningLaunched, LOCAL_PLANNING_LAUNCH_FILE);
}

void RoadSweeperGui::toggleCmdOutputLaunch()
{
    toggleLaunch(cmdOutputProcess, cmdOutputButton, cmdOutputLaunched, CMD_OUTPUT_LAUNCH_FILE);
}

void RoadSweeperGui::toggleCanNodeLaunch()
{
    toggleLaunch(canNodeProcess, canNodeButton, canNodeLaunched, CAN_NODE_LAUNCH_FILE);
}

void RoadSweeperGui::toggleSweepNodeLaunch()
{
    toggleLaunch(sweepNodeProcess, sweepNodeButton, sweepNodeLaunched, SWEEP_NODE_LAUNCH_FILE);
}

void RoadSweeperGui::toggleRvizLaunch()
{
    toggleLaunch(rvizProcess, rvizButton, rvizLaunched, RVIZ_LAUNCH_FILE);
}

void RoadSweeperGui::toggleLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile)
{
    if (launched)
    {
        QProcess killProcess;
        killProcess.start("pkill", QStringList() << "-f" << launchFile);
        killProcess.waitForFinished();
        QDebug deb = qDebug();

        if (process)
        {
            process->terminate();
            process->waitForFinished();
            delete process;
            process = nullptr;
        }
        button->setStyleSheet("");
        launched = false;
    }
    else
    {
        process = new QProcess(this);
        QStringList arguments;
        arguments << "-e" << QString("roslaunch road_sweeper %1").arg(launchFile);
        process->start("gnome-terminal", arguments);
        button->setStyleSheet("background-color: green");
        launched = true;
    }
}

void RoadSweeperGui::controlAutoSweep()
{
    std_srvs::SetBool srv;
    srv.request.data = autoSweepCheckBox->isChecked();
    client.call(srv);
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