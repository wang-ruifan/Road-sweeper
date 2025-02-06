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
	void toggleLaunchFile();
	void controlAutoSweep();

private:
	QPushButton *startButton;
	QPushButton *controlButton;

	QTextEdit *outputDisplay;

	QProcess *launchProcess;

	ros::NodeHandle nh;
	ros::ServiceClient client;

	bool autoSweepEnabled;
	bool setupLaunched;
};

#endif // ROAD_SWEEPER_GUI_HPP