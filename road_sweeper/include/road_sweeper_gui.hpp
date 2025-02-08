#ifndef ROAD_SWEEPER_GUI_HPP
#define ROAD_SWEEPER_GUI_HPP

#include <QMainWindow>
#include <QPushButton>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QProcess>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <thread>
#include <unistd.h>
#include <sys/types.h>


class RoadSweeperGui : public QMainWindow
{
	Q_OBJECT

public:
	RoadSweeperGui(QWidget *parent = nullptr);
	~RoadSweeperGui();

private slots:
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

private:
	void toggleLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile);

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

	QProcess *setupProcess;
	QProcess *mapProcess;
	QProcess *localizationProcess;
	QProcess *globalPlanningProcess;
	QProcess *perceptionProcess;
	QProcess *localPlanningProcess;
	QProcess *cmdOutputProcess;
	QProcess *canNodeProcess;
	QProcess *sweepNodeProcess;

	ros::NodeHandle nh;
	ros::ServiceClient client;

	bool setupLaunched;
	bool mapLaunched;
	bool localizationLaunched;
	bool globalPlanningLaunched;
	bool perceptionLaunched;
	bool localPlanningLaunched;
	bool cmdOutputLaunched;
	bool canNodeLaunched;
	bool sweepNodeLaunched;
};

#endif // ROAD_SWEEPER_GUI_HPP