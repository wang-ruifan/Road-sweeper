#include "road_sweeper_gui.hpp"
#include <QApplication>

using namespace ButtonStyle;

RoadSweeperGui::RoadSweeperGui(QWidget *parent)
    : QMainWindow(parent)
    , launchComponents{
        {"setup.launch", "Setup", PanelName::CONTROL},
        {"map.launch", "Map", PanelName::CONTROL},
        {"lidar_imu.launch", "Lidar and IMU", PanelName::CONTROL},
        {"localization.launch", "Localization", PanelName::CONTROL},
        {"global_planning.launch", "Global Planning", PanelName::CONTROL},
        {"perception.launch", "Perception", PanelName::CONTROL},
        {"local_planning.launch", "Local Planning", PanelName::CONTROL},
        {"cmd_output.launch", "Cmd Output", PanelName::CONTROL},
        {"cmd_to_can.launch", "Cmd To Can", PanelName::CONTROL},
        {"auto_sweep.launch", "Auto Sweep Node", PanelName::CONTROL},
        {"rviz.launch", "RViz", PanelName::DISPLAY},
        {"rqt.launch", "RQT", PanelName::DISPLAY}
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

void RoadSweeperGui::initializeSpeedDisplay()
{
    speedLabel = new QLabel("Speed:", this);
    speedLabel->setStyleSheet("QLabel { font-size: 14pt; }");
    speedDisplay = new QLCDNumber(this);
    speedDisplay->setDigitCount(2);
    speedDisplay->setSmallDecimalPoint(false);
    speedDisplay->setSegmentStyle(QLCDNumber::Flat);
    speedDisplay->setStyleSheet("QLCDNumber { background-color: white; color: green; }");
    speedDisplay->setMinimumSize(150, 50);
    unitLabel = new QLabel("km/h", this);
    unitLabel->setStyleSheet("QLabel { font-size: 14pt; }");
}

void RoadSweeperGui::initializeBatteryDisplay()
{
    batteryLabel = new QLabel("Battery:", this);
    batteryLabel->setStyleSheet("QLabel { font-size: 14pt; }");
    batteryBar = new QProgressBar(this);
    batteryBar->setRange(0, 100);
    batteryBar->setTextVisible(true);
    batteryBar->setFormat("%p%");
    batteryBar->setStyleSheet(
        "QProgressBar {"
        "   border: 2px solid grey;"
        "   border-radius: 5px;"
        "   text-align: center;"
        "}"
        "QProgressBar::chunk {"
        "   background-color: #05B8CC;"
        "   width: 20px;"
        "}"
    );
}

void RoadSweeperGui::initializeCarWidget()
{
    carWidget = new CarWidget(this);
    carWidget->setLineLength(50);

    QString execPath = QApplication::applicationDirPath();    
    QString imagePath = execPath + "/../../share/road_sweeper/resources/car.png";
    
    if(!QFile::exists(imagePath)) {
        qDebug() << "Car image not found at:" << imagePath;
    }
    carWidget->setCarImage(imagePath);
}

void RoadSweeperGui::initializeResourceMonitor()
{
    resourceMonitor = new ResourceMonitor(this);
}

void RoadSweeperGui::initializeWidgets()
{
    /*====== Widget initialization ======*/
    // Buttons
    for(auto& component : launchComponents) {
        component.button = new QPushButton(component.buttonName, this);
    }
    // Checkboxes
    sweepCheckBox = new QCheckBox("Sweep", this);

    // Speed Display
    initializeSpeedDisplay();

    // Battery Display
    initializeBatteryDisplay();

    // Car Widget
    initializeCarWidget();

    // Resource Monitor
    initializeResourceMonitor();
    
    currentSpeed = 0.00;
    batteryLevel = 100;
    currentAngle = 0.0;

    updateBatteryFlag = updateAngleFlag = updateSpeedFlag = false;
    isPaintRunning = false;
}

void RoadSweeperGui::setupLayouts()
{
    /*====== Layout setup ======*/
    QHBoxLayout *mainLayout = new QHBoxLayout;
    QVBoxLayout *controlPanel = new QVBoxLayout;
    QVBoxLayout *displayPanel = new QVBoxLayout;

    // Battery layout setup
    QHBoxLayout *batteryLayout = new QHBoxLayout;
    batteryLayout->addWidget(batteryLabel);
    batteryLayout->addWidget(batteryBar);
    
    // Speed layout setup
    QHBoxLayout *speedLayout = new QHBoxLayout;
    speedLayout->addWidget(speedLabel);
    speedLayout->addWidget(speedDisplay);
    speedLayout->addWidget(unitLabel);

    // Add speed，car and battery to display panel
    displayPanel->addLayout(batteryLayout, 1);
    displayPanel->addWidget(carWidget, 3);
    displayPanel->addLayout(speedLayout, 3);
    displayPanel->setSpacing(10);

    // Add buttons to control panel
    for(const auto& component : launchComponents) {
        if (component.panel == PanelName::CONTROL) {
            /*=== Control panel ===*/
            if (component.buttonName == "Auto Sweep Node") {
                // Special case for auto sweep node
                QHBoxLayout *sweepLayout = new QHBoxLayout;
                sweepLayout->addWidget(component.button);
                sweepLayout->addWidget(sweepCheckBox);
                controlPanel->addLayout(sweepLayout);
            } else {
                controlPanel->addWidget(component.button);
            }
        } else {
            /*=== Display panel ===*/
            displayPanel->addWidget(component.button, 1);
        }
    }

    // Add resource monitor to control panel
    controlPanel->addWidget(resourceMonitor);

    /*=== Main layout ===*/
    mainLayout->addLayout(controlPanel);
    mainLayout->addLayout(displayPanel);

    /*=== Central widget ===*/
    QWidget *centralWidget = new QWidget(this);
    centralWidget->setLayout(mainLayout);
    setCentralWidget(centralWidget);

    updateDisplays(UpdateType::ALL);
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
    connect(sweepCheckBox, &QCheckBox::clicked, this, &RoadSweeperGui::controlSweep);

    // Timers
    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, [this]() {
        updateBatteryFlag = updateAngleFlag = updateSpeedFlag = true;
    });
    timer->start(500);
}

void RoadSweeperGui::setupROS()
{
    /*====== ROS setup ======*/
    nh = ros::NodeHandle();
    // Create service client
    client = nh.serviceClient<std_srvs::SetBool>("enable_auto_sweep");
    // Create subscriber
    canSubscriber = nh.subscribe("/received_messages", 10, 
                               &RoadSweeperGui::canCallback, this);
    // Create publisher
    sweepPublisher = nh.advertise<std_msgs::Bool>("sweep_control", 1);
}

 void RoadSweeperGui::canCallback(const can_msgs::Frame::ConstPtr& msg)
{
    if ((!updateBatteryFlag && !updateAngleFlag && !updateSpeedFlag)|| isPaintRunning) {
        return;
    }

    // Speed data message ID: 0x503
    if(msg->id == 0x503 && updateSpeedFlag)
    {
        updateSpeedFlag = false;
        int speed = msg->data[2];
        if (speed >= 12) {
            speed -= 12;
            speed = -speed;
        }
        if(speed != currentSpeed) {
            currentSpeed = speed;
            updateDisplays(UpdateType::SPEED);
        }
    }
    // Angle data message ID: 0x502
    else if(msg->id == 0x502 && updateAngleFlag)
    {
        isPaintRunning = true;
        updateAngleFlag = false;
        double angle = (msg->data[1] << 8) | msg->data[2];
        angle -= 1024.0;
        angle /= 15.0;
        angle = -angle;
        if(angle != currentAngle) {
            currentAngle = angle;
            updateDisplays(UpdateType::ANGLE);
        }
        isPaintRunning = false;
    }
    // Battery data message ID: 0x504
    else if(msg->id == 0x504 && updateBatteryFlag)
    {
        updateBatteryFlag = false;
        int level = msg->data[1];
        if(level != batteryLevel) {
            batteryLevel = level;
            updateDisplays(UpdateType::BATTERY);
        }
    }
}

void RoadSweeperGui::updateSpeed()
{
    speedDisplay->display(QString::number(currentSpeed));
}

void RoadSweeperGui::updateBattery()
{
    batteryBar->setValue(batteryLevel);

    QString barStyle = 
        "QProgressBar {"
        "   border: 2px solid grey;"
        "   border-radius: 5px;"
        "   text-align: center;"
        "}"
        "QProgressBar::chunk {";
        
    if(batteryLevel <= 20) {
        barStyle += "background-color: red;";
    } else if(batteryLevel <= 50) {
        barStyle += "background-color: orange;";
    } else {
        barStyle += "background-color:rgb(5, 204, 38);";
    }
    
    barStyle += "width: 20px;}";
    batteryBar->setStyleSheet(barStyle);
}

void RoadSweeperGui::updateCarWidget()
{
    carWidget->setAngle(currentAngle);
}
void RoadSweeperGui::updateDisplays(UpdateType type)
{
    switch(type) {
        case UpdateType::SPEED:
            updateSpeed();
            break;
        case UpdateType::ANGLE:
            updateCarWidget();
            break;
        case UpdateType::BATTERY:
            updateBattery();
            break;
        case UpdateType::ALL:
            updateSpeed();
            updateBattery();
            updateCarWidget();
            break;
    }
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

void RoadSweeperGui::updateLaunchStatus(QPushButton* button, LaunchStatus status)
{
    switch (status) {
        case LaunchStatus::ACTIVE:
            button->setStyleSheet(ButtonStyle::ACTIVE);
            break;
        case LaunchStatus::ERROR:
            button->setStyleSheet(ButtonStyle::ERROR);
            break;
        case LaunchStatus::DEFAULT:
        default:
            button->setStyleSheet(ButtonStyle::DEFAULT);
            break;
    }
}

void RoadSweeperGui::toggleAutoSweep(LaunchComponent &component)
{
    toggleLaunch(component.process, component.button, component.launched, component.launchFile);
    changeSweepCheckBox(component.launched);
}

void RoadSweeperGui::changeSweepCheckBox(bool state)
{
    sweepCheckBox->setChecked(false);
    if (state) {
        // change to "Auto Sweep"
        controlSweep();
        sweepCheckBox->setText("Auto Sweep");
        disconnect(sweepCheckBox, &QCheckBox::clicked, this, &RoadSweeperGui::controlSweep);
        connect(sweepCheckBox, &QCheckBox::clicked, this, &RoadSweeperGui::controlAutoSweep);
    } else {
        // change to "Sweep"
        sweepCheckBox->setText("Sweep");
        disconnect(sweepCheckBox, &QCheckBox::clicked, this, &RoadSweeperGui::controlAutoSweep);
        connect(sweepCheckBox, &QCheckBox::clicked, this, &RoadSweeperGui::controlSweep);
    }
}

void RoadSweeperGui::controlSweep()
{
    std_msgs::Bool msg;
    msg.data = sweepCheckBox->isChecked();
    sweepPublisher.publish(msg);
}

void RoadSweeperGui::controlAutoSweep()
{
    std_srvs::SetBool srv;
    srv.request.data = sweepCheckBox->isChecked();
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