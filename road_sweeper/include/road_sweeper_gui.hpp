#ifndef ROAD_SWEEPER_GUI_HPP
#define ROAD_SWEEPER_GUI_HPP

#include <QMainWindow>
#include <QPushButton>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QProcess>
#include <QString>
#include <QDebug>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <thread>
#include <unistd.h>
#include <sys/types.h>
#include <vector>
#include <memory>

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

	void initializeWidgets();
    void setupLayouts();
    void connectSignalsAndSlots();
    void setupROS();

	void toggleLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile);

	void closeLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile);
	void startLaunch(QProcess *&process, QPushButton *button, bool &launched, const QString &launchFile);

    void toggleAutoSweep(LaunchComponent &component);

	std::vector<LaunchComponent> launchComponents;

	QCheckBox *autoSweepCheckBox;

	/*=== ROS ===*/
	ros::NodeHandle nh;
	ros::ServiceClient client;
	
	static constexpr const char* DEFAULT = "";
    static constexpr const char* ACTIVE = "background-color: green";
    static constexpr const char* ERROR = "background-color: red";

	enum class LaunchStatus
	{
		ACTIVE,
		ERROR,
		DEFAULT
	};

	void updateLaunchStatus(QPushButton* button, LaunchStatus status);

};

#endif // ROAD_SWEEPER_GUI_HPP