#include "road_sweeper_gui.hpp"
#include <QApplication>

RoadSweeperGui::RoadSweeperGui(QWidget *parent)
    : QMainWindow(parent)
{
    setupButton = new QPushButton("Setup", this);
    mapButton = new QPushButton("Map", this);
    localizationButton = new QPushButton("Localization", this);
    planningButton = new QPushButton("Planning", this);
    perceptionButton = new QPushButton("Perception", this);
    controlButton = new QPushButton("Auto Sweep", this);
    controlButton->setCheckable(true);

    QVBoxLayout *layout = new QVBoxLayout;
    layout->addWidget(setupButton);
    layout->addWidget(mapButton);
    layout->addWidget(localizationButton);
    layout->addWidget(planningButton);
    layout->addWidget(perceptionButton);
    layout->addWidget(controlButton);

    QWidget *centralWidget = new QWidget(this);
    centralWidget->setLayout(layout);
    setCentralWidget(centralWidget);

    connect(setupButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleSetupLaunch);
    connect(mapButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleMapLaunch);
    connect(localizationButton, &QPushButton::clicked, this, &RoadSweeperGui::toggleLocalizationLaunch);
    connect(planningButton, &QPushButton::clicked, this, &RoadSweeperGui::togglePlanningLaunch);
    connect(perceptionButton, &QPushButton::clicked, this, &RoadSweeperGui::togglePerceptionLaunch);
    connect(controlButton, &QPushButton::clicked, this, &RoadSweeperGui::controlAutoSweep);

    setupProcess = mapProcess = localizationProcess = planningProcess = perceptionProcess = nullptr;
    setupLaunched = mapLaunched = localizationLaunched = planningLaunched = perceptionLaunched = false;

    // Create service client
    nh = ros::NodeHandle();
    client = nh.serviceClient<std_srvs::SetBool>("enable_auto_sweep");

    // Initialize variables
    autoSweepEnabled = false;
    setupLaunched = false;
    mapLaunched = false;
    localizationLaunched = false;
    planningLaunched = false;
    perceptionLaunched = false;
}

RoadSweeperGui::~RoadSweeperGui()
{
    QProcess *processes[] = {setupProcess, mapProcess, localizationProcess, planningProcess, perceptionProcess};
    for (auto process : processes)
    {
        if (process)
        {
            process->terminate();
            process->waitForFinished();
            delete process;
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
    toggleLaunch(planningProcess, planningButton, planningLaunched, "global_planning.launch");
}

void RoadSweeperGui::togglePerceptionLaunch()
{
    toggleLaunch(perceptionProcess, perceptionButton, perceptionLaunched, "perception.launch");
}

void RoadSweeperGui::toggleLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile)
{
    if (launched)
    {
        QProcess killProcess;
        killProcess.start("pkill", QStringList() << "-f" << launchFile);
        killProcess.waitForFinished();

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