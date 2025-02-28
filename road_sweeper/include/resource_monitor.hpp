#ifndef RESOURCEMONITOR_HPP
#define RESOURCEMONITOR_HPP

#include <QWidget>
#include <QTimer>
#include <QProgressBar>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFrame>
#include <QPalette>
#include <QVector>
#include <string>

class ResourceMonitor : public QWidget
{
	Q_OBJECT

public:
	explicit ResourceMonitor(QWidget *parent = nullptr);
	~ResourceMonitor();

private slots:
	void updateStats();

private:
	// UI elements
	QProgressBar *cpuProgressBar;
	QLabel *cpuTempLabel;
	QTimer *updateTimer;

	// Helper functions
	float getCpuUsage();
	float getCpuTemperature();
	void setProgressColor(QProgressBar *progressBar, float value);

	// CPU usage tracking
	unsigned long long lastTotalUser, lastTotalUserLow, lastTotalSys, lastTotalIdle;
};

#endif // RESOURCEMONITOR_HPP