#ifndef ROAD_SWEEPER_GUI_HPP
#define ROAD_SWEEPER_GUI_HPP

#include <QMainWindow>
#include <QPushButton>
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
	void controlAutoSweep();

private:
	void toggleLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile);

	QPushButton *setupButton;
	QPushButton *mapButton;
	QPushButton *localizationButton;
	QPushButton *planningButton;
	QPushButton *perceptionButton;
	QPushButton *controlButton;

	QProcess *setupProcess;
	QProcess *mapProcess;
	QProcess *localizationProcess;
	QProcess *planningProcess;
	QProcess *perceptionProcess;

	ros::NodeHandle nh;
	ros::ServiceClient client;

	bool setupLaunched;
	bool mapLaunched;
	bool localizationLaunched;
	bool planningLaunched;
	bool perceptionLaunched;

	bool autoSweepEnabled;
};

#endif // ROAD_SWEEPER_GUI_HPP