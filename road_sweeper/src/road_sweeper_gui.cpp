#include "road_sweeper_gui.hpp"
#include <QApplication>
#include <QDebug>

RoadSweeperGui::RoadSweeperGui(QWidget *parent)
    : QMainWindow(parent)
{
    setupButton = new QPushButton("Setup", this);
    mapButton = new QPushButton("Map", this);
    localizationButton = new QPushButton("Localization", this);
    globalPlanningButton = new QPushButton("Global Planning", this);
    perceptionButton = new QPushButton("Perception", this);
    localPlanningButton = new QPushButton("Local Planning", this);
    cmdOutputButton = new QPushButton("Cmd Output", this);
    canNodeButton = new QPushButton("Cmd To Can", this);
    sweepNodeButton = new QPushButton("Auto Sweep Node", this);
    
    autoSweepCheckBox = new QCheckBox("Auto Sweep", this);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(setupButton);
    layout->addWidget(mapButton);
    layout->addWidget(localizationButton);
    layout->addWidget(globalPlanningButton);
    layout->addWidget(perceptionButton);
    layout->addWidget(localPlanningButton);
    layout->addWidget(cmdOutputButton);
    layout->addWidget(canNodeButton);
    layout->addWidget(sweepNodeButton);
    layout->addWidget(autoSweepCheckBox);

    QWidget *centralWidget = new QWidget(this);
    centralWidget->setLayout(layout);
    setCentralWidget(centralWidget);

    connect(setupButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleSetupLaunch);
    connect(mapButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleMapLaunch);
    connect(localizationButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleLocalizationLaunch);
    connect(globalPlanningButton, &QPushButton::clicked, this, &RoadSweeperGui::togglePlanningLaunch);
    connect(perceptionButton, &QPushButton::clicked, this, &RoadSweeperGui::togglePerceptionLaunch);
    connect(localPlanningButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleLocalPlanningLaunch);
    connect(cmdOutputButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleCmdOutputLaunch);
    connect(canNodeButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleCanNodeLaunch);
    connect(sweepNodeButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleSweepNodeLaunch);
    connect(autoSweepCheckBox, &QCheckBox::clicked, this, &RoadSweeperGui::controlAutoSweep);

    setupProcess = mapProcess = localizationProcess = globalPlanningProcess = perceptionProcess = nullptr;
    setupLaunched = mapLaunched = localizationLaunched = globalPlanningLaunched = perceptionLaunched = false;

    // Create service client
    nh = ros::NodeHandle();
    client = nh.serviceClient<std_srvs::SetBool>("enable_auto_sweep");

    // Initialize variables
    setupLaunched = false;
    mapLaunched = false;
    localizationLaunched = false;
    globalPlanningLaunched = false;
    perceptionLaunched = false;
    localPlanningLaunched = false;
    cmdOutputLaunched = false;
    canNodeLaunched = false;
    sweepNodeLaunched = false;
}

RoadSweeperGui::~RoadSweeperGui()
{
    bool launched[] = {setupLaunched, mapLaunched, localizationLaunched, globalPlanningLaunched, perceptionLaunched, localPlanningLaunched, cmdOutputLaunched, canNodeLaunched, sweepNodeLaunched};
    QProcess* processes[] = {setupProcess, mapProcess, localizationProcess, globalPlanningProcess, perceptionProcess, localPlanningProcess, cmdOutputProcess, canNodeProcess, sweepNodeProcess};
    QString launchFiles[] = {"setup.launch", "map.launch", "localization.launch", "global_planning.launch", "perception.launch", "local_planning.launch", "cmd_output.launch", "cmd_to_can.launch", "auto_sweep.launch"};

    for (int i = 0; i < 9; i++)
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
    toggleLaunch(setupProcess, setupButton, setupLaunched, "setup.launch");
}

void RoadSweeperGui::toggleMapLaunch()
{
    toggleLaunch(mapProcess, mapButton, mapLaunched, "map.launch");
}

void RoadSweeperGui::toggleLocalizationLaunch()
{
    toggleLaunch(localizationProcess, localizationButton, localizationLaunched, "localization.launch");
}

void RoadSweeperGui::togglePlanningLaunch()
{
    toggleLaunch(globalPlanningProcess, globalPlanningButton, globalPlanningLaunched, "global_planning.launch");
}

void RoadSweeperGui::togglePerceptionLaunch()
{
    toggleLaunch(perceptionProcess, perceptionButton, perceptionLaunched, "perception.launch");
}

void RoadSweeperGui::toggleLocalPlanningLaunch()
{
    toggleLaunch(localPlanningProcess, localPlanningButton, localPlanningLaunched, "local_planning.launch");
}

void RoadSweeperGui::toggleCmdOutputLaunch()
{
    toggleLaunch(cmdOutputProcess, cmdOutputButton, cmdOutputLaunched, "cmd_output.launch");
}

void RoadSweeperGui::toggleCanNodeLaunch()
{
    toggleLaunch(canNodeProcess, canNodeButton, canNodeLaunched, "cmd_to_can.launch");
}

void RoadSweeperGui::toggleSweepNodeLaunch()
{
    toggleLaunch(sweepNodeProcess, sweepNodeButton, sweepNodeLaunched, "auto_sweep.launch");
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