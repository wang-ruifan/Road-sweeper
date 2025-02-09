#ifndef ROAD_SWEEPER_GUI_HPP
#define ROAD_SWEEPER_GUI_HPP

#include <QMainWindow>
#include <QPushButton>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QLabel>
#include <QLCDNumber>
#include <QProgressBar>
#include <QProcess>
#include <QString>
#include <QDebug>
#include <QFile>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <thread>
#include <unistd.h>
#include <sys/types.h>
#include <vector>
#include <memory>
#include <can_msgs/Frame.h>
#include "car_widget.hpp"

namespace ButtonStyle {
        const QString DEFAULT = "";
        const QString ACTIVE = "background-color: green";
        const QString ERROR = "background-color: red";
}

class RoadSweeperGui : public QMainWindow
{
	Q_OBJECT

public:
	RoadSweeperGui(QWidget *parent = nullptr);
	~RoadSweeperGui();

private slots:
	// Control tab;
	void controlAutoSweep();

private:
    enum class PanelName {
    	CONTROL,
    	DISPLAY
	};

    enum class LaunchStatus
	{
		ACTIVE,
		ERROR,
		DEFAULT
	};

    enum class UpdateType
    {
        SPEED,
        ANGLE,
        BATTERY,
        ALL
    };

    struct LaunchComponent {
        QPushButton* button;			// Button to launch the process
        QProcess* process;				// Process to launch
        const char* launchFile;			// Launch file name
        bool launched;					// Launched status
        QString buttonName;
        PanelName panel;
        
        LaunchComponent(const char* file, QString name, PanelName p) 
            : button(nullptr)
            , process(nullptr)
            , launchFile(file)
            , launched(false)
            , buttonName(std::move(name))
            , panel(p) {}
    };

    void initializeSpeedDisplay();
    void initializeBatteryDisplay();
    void initializeCarWidget();

	void initializeWidgets();
    void setupLayouts();
    void connectSignalsAndSlots();
    void setupROS();

	void toggleLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile);

	void closeLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile);
	void startLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile);

  	void updateLaunchStatus(QPushButton* button, LaunchStatus status);

    void toggleAutoSweep(LaunchComponent &component);

    void canCallback(const can_msgs::Frame::ConstPtr& msg);
    void updateDisplays(UpdateType type);
    void updateSpeed();
    void updateBattery();
    void updateCarWidget();

	std::vector<LaunchComponent> launchComponents;

	QCheckBox *autoSweepCheckBox;

    QLCDNumber *speedDisplay;
    QProgressBar *batteryBar;
    QLabel *speedLabel;
    QLabel *unitLabel;
    QLabel *batteryLabel;
    double currentSpeed;
    int batteryLevel;
    double currentAngle;

    CarWidget* carWidget;

	/*=== ROS ===*/
	ros::NodeHandle nh;
	ros::ServiceClient client;
    ros::Subscriber canSubscriber;
};

#endif // ROAD_SWEEPER_GUI_HPP