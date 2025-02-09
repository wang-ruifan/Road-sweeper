#include "road_sweeper_gui.hpp"
#include <QApplication>

RoadSweeperGui::RoadSweeperGui(QWidget *parent)
    : QMainWindow(parent)
    , launchComponents{
        {"setup.launch", "Setup", PanelName::CONTROL},
        {"map.launch", "Map", PanelName::CONTROL},
        {"localization.launch", "Localization", PanelName::CONTROL},
        {"global_planning.launch", "Global Planning", PanelName::CONTROL},
        {"perception.launch", "Perception", PanelName::CONTROL},
        {"local_planning.launch", "Local Planning", PanelName::CONTROL},
        {"cmd_output.launch", "Cmd Output", PanelName::CONTROL},
        {"cmd_to_can.launch", "Cmd To Can", PanelName::CONTROL},
        {"auto_sweep.launch", "Auto Sweep Node", PanelName::CONTROL},
        {"rviz.launch", "RViz", PanelName::DISPLAY}
    }
    , nh()
{
    initializeWidgets();
    setupLayouts();
    connectSignalsAndSlots();
    setupROS();
}

RoadSweeperGui::~RoadSweeperGui()
{
    for(auto& component : launchComponents) {
        if(component.launched) {
            closeLaunch(component.process, component.button, component.launched, component.launchFile);
        }
    }
}

void RoadSweeperGui::initializeWidgets()
{
    /*====== Widget initialization ======*/
    // Buttons
    for(auto& component : launchComponents) {
        component.button = new QPushButton(component.buttonName, this);
    }
    // Checkboxes
    autoSweepCheckBox = new QCheckBox("Auto Sweep", this);
}

void RoadSweeperGui::setupLayouts()
{
    /*====== Layout setup ======*/
    QHBoxLayout *mainLayout = new QHBoxLayout;
    QVBoxLayout *controlPanel = new QVBoxLayout;
    QVBoxLayout *displayPanel = new QVBoxLayout;
    
    for(const auto& component : launchComponents) {
        if (component.panel == PanelName::CONTROL) {
            /*=== Control panel ===*/
            if (component.buttonName == "Auto Sweep Node") {
                // Special case for auto sweep node
                QHBoxLayout *sweepLayout = new QHBoxLayout;
                sweepLayout->addWidget(component.button);
                sweepLayout->addWidget(autoSweepCheckBox);
                controlPanel->addLayout(sweepLayout);
            } else {
                controlPanel->addWidget(component.button);
            }
        } else {
            /*=== Display panel ===*/
            displayPanel->addWidget(component.button);
        }
    }

    /*=== Main layout ===*/
    mainLayout->addLayout(controlPanel);
    mainLayout->addLayout(displayPanel);

    /*=== Central widget ===*/
    QWidget *centralWidget = new QWidget(this);
    centralWidget->setLayout(mainLayout);
    setCentralWidget(centralWidget);
}

void RoadSweeperGui::connectSignalsAndSlots()
{
    /*====== Signal/slot connections ======*/
    // Buttons
    for(size_t i = 0; i < launchComponents.size(); ++i) {
        // Special case for auto sweep node
        if (launchComponents[i].buttonName == "Auto Sweep Node") {
            connect(launchComponents[i].button, &QPushButton::clicked, 
                [this, i]() { 
                    toggleAutoSweep(launchComponents[i]);
                });
        } else {
            connect(launchComponents[i].button, &QPushButton::clicked, 
                [this, i]() { 
                    toggleLaunch(launchComponents[i].process, 
                               launchComponents[i].button,
                               launchComponents[i].launched, 
                               launchComponents[i].launchFile);
                });
        }
    }
    
    // Checkboxes
    connect(autoSweepCheckBox, &QCheckBox::clicked, this, &RoadSweeperGui::controlAutoSweep);
}

void RoadSweeperGui::setupROS()
{
    /*====== ROS setup ======*/
    // Create service client
    nh = ros::NodeHandle();
    client = nh.serviceClient<std_srvs::SetBool>("enable_auto_sweep");
}

void RoadSweeperGui::closeLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile)
{
    // Kill node
    QProcess killProcess;
    killProcess.start("pkill", QStringList() << "-f" << launchFile);
    if (!killProcess.waitForFinished(5000)) {
        qWarning() << "Kill node timeout for:" << launchFile;
    }

    // Terminate process
    if (process) {
        process->terminate();
        if (!process->waitForFinished(5000)) {
            process->kill();
        }
        delete process;
        process = nullptr;
    }
    
    updateLaunchStatus(button, LaunchStatus::DEFAULT);
    launched = false;
}

void RoadSweeperGui::startLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile)
{
    process = new QProcess(this);
    
    connect(process, &QProcess::errorOccurred, 
        [this, button](QProcess::ProcessError error) {
            updateLaunchStatus(button, LaunchStatus::ERROR);
            qWarning() << "Process error occurred:" << error;
        });

    QStringList arguments;
    arguments << "-e" << QString("roslaunch road_sweeper %1").arg(launchFile);
    process->start("gnome-terminal", arguments);
    
    if (!process->waitForStarted(3000)) {
        updateLaunchStatus(button, LaunchStatus::ERROR);
        throw std::runtime_error("Process failed to start: " + launchFile.toStdString());
    }
    
    updateLaunchStatus(button, LaunchStatus::ACTIVE);
    launched = true;
}

void RoadSweeperGui::toggleLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile)
{
    try {
        if (launched) {
            closeLaunch(process, button, launched, launchFile);
        } else {
            startLaunch(process, button, launched, launchFile);
        }
    } catch (const std::exception& e) {
        qWarning() << "Error in toggleLaunch:" << e.what();
        launched = false;
        if (process) {
            delete process;
            process = nullptr;
        }
        updateLaunchStatus(button, LaunchStatus::ERROR);
    }
}

// 添加新的辅助函数用于更新启动状态
void RoadSweeperGui::updateLaunchStatus(QPushButton* button, LaunchStatus status)
{
    switch (status) {
        case LaunchStatus::ACTIVE:
            button->setStyleSheet(ACTIVE);
            break;
        case LaunchStatus::ERROR:
            button->setStyleSheet(ERROR);
            break;
        case LaunchStatus::DEFAULT:
        default:
            button->setStyleSheet(DEFAULT);
            break;
    }
}

void RoadSweeperGui::toggleAutoSweep(LaunchComponent &component)
{
    autoSweepCheckBox->setChecked(false);
    toggleLaunch(component.process, component.button, component.launched, component.launchFile);
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