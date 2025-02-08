#ifndef ROAD_SWEEPER_GUI_HPP
#define ROAD_SWEEPER_GUI_HPP

#include <QMainWindow>
#include <QPushButton>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QProcess>
#include <QString>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <thread>
#include <unistd.h>
#include <sys/types.h>
#include <vector>
#include <memory>

#define SETUP_LAUNCH_FILE "setup.launch"
#define MAP_LAUNCH_FILE "map.launch"
#define LOCALIZATION_LAUNCH_FILE "localization.launch"
#define GLOBAL_PLANNING_LAUNCH_FILE "global_planning.launch"
#define PERCEPTION_LAUNCH_FILE "perception.launch"
#define LOCAL_PLANNING_LAUNCH_FILE "local_planning.launch"
#define CMD_OUTPUT_LAUNCH_FILE "cmd_output.launch"
#define CAN_NODE_LAUNCH_FILE "cmd_to_can.launch"
#define SWEEP_NODE_LAUNCH_FILE "auto_sweep.launch"
#define RVIZ_LAUNCH_FILE "rviz.launch"


class RoadSweeperGui : public QMainWindow
{
	Q_OBJECT

public:
	RoadSweeperGui(QWidget *parent = nullptr);
	~RoadSweeperGui();

private slots:
	// Control tab
	void toggleSetupLaunch();
	void toggleMapLaunch();
	void toggleLocalizationLaunch();
	void togglePlanningLaunch();
	void togglePerceptionLaunch();
	void toggleLocalPlanningLaunch();
	void toggleCmdOutputLaunch();
	void toggleCanNodeLaunch();
	void toggleSweepNodeLaunch();
	void controlAutoSweep();

	// Display tab
	void toggleRvizLaunch();

private:
	void toggleLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile);
	/*=== Widgets ===*/
	// Control tab
	QPushButton *setupButton;
	QPushButton *mapButton;
	QPushButton *localizationButton;
	QPushButton *globalPlanningButton;
	QPushButton *perceptionButton;
	QPushButton *localPlanningButton;
	QPushButton *cmdOutputButton;
	QPushButton *canNodeButton;
	QPushButton *sweepNodeButton;
	QCheckBox *autoSweepCheckBox;

	// Display tab
	QPushButton *rvizButton;

	/*=== Processes ===*/
	// Control tab
	QProcess *setupProcess;
	QProcess *mapProcess;
	QProcess *localizationProcess;
	QProcess *globalPlanningProcess;
	QProcess *perceptionProcess;
	QProcess *localPlanningProcess;
	QProcess *cmdOutputProcess;
	QProcess *canNodeProcess;
	QProcess *sweepNodeProcess;
	// Display tab
	QProcess *rvizProcess;

	/*=== ROS ===*/
	ros::NodeHandle nh;
	ros::ServiceClient client;

	// Display tab

	/*=== Launched flags ===*/
	// Control tab
	bool setupLaunched;
	bool mapLaunched;
	bool localizationLaunched;
	bool globalPlanningLaunched;
	bool perceptionLaunched;
	bool localPlanningLaunched;
	bool cmdOutputLaunched;
	bool canNodeLaunched;
	bool sweepNodeLaunched;

	// Display tab
	bool rvizLaunched;
	
};

#endif // ROAD_SWEEPER_GUI_HPP