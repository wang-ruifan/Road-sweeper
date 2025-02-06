#ifndef ROAD_SWEEPER_GUI_HPP
#define ROAD_SWEEPER_GUI_HPP

#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QTextEdit>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <thread>

class RoadSweeperGui : public QMainWindow
{
	Q_OBJECT

public:
	RoadSweeperGui(QWidget *parent = nullptr);
	~RoadSweeperGui();

private slots:
	void startLaunchFile();
	void controlAutoSweep();

private:
	QPushButton *startButton;
	QPushButton *controlButton;
	ros::NodeHandle nh;
	ros::ServiceClient client;
};

#endif // ROAD_SWEEPER_GUI_HPP