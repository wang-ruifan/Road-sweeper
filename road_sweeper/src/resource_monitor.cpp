#include "resource_monitor.hpp"
#include <QDebug>
#include <QColor>
#include <QPalette>
#include <signal.h>
#include <fcntl.h>
#include <time.h>
#include <unistd.h>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <array>
#include <dirent.h>
#include <unistd.h>
#include <regex>

ResourceMonitor::ResourceMonitor(QWidget *parent)
    : QWidget(parent),
      lastTotalUser(0),
      lastTotalUserLow(0),
      lastTotalSys(0),
      lastTotalIdle(0)
{
    // 创建主布局
    QVBoxLayout *mainLayout = new QVBoxLayout(this);

    // CPU监控部分
    QHBoxLayout *cpuLayout = new QHBoxLayout();
    QLabel *cpuLabel = new QLabel(tr("CPU:"));
    cpuProgressBar = new QProgressBar();
    cpuProgressBar->setRange(0, 100);
    cpuTempLabel = new QLabel("N/A");
    cpuTempLabel->setMinimumWidth(80);

    cpuLayout->addWidget(cpuLabel);
    cpuLayout->addWidget(cpuProgressBar);
    cpuLayout->addWidget(cpuTempLabel);

    // 只添加CPU布局到主布局
    mainLayout->addLayout(cpuLayout);

    // 初始化CPU使用率统计
    updateStats();

    // 设置定时器，每秒更新一次
    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &ResourceMonitor::updateStats);
    updateTimer->start(1000); // 1000ms = 1s
}

ResourceMonitor::~ResourceMonitor()
{
    if (updateTimer)
    {
        updateTimer->stop();
    }
}

void ResourceMonitor::updateStats()
{
    // 更新CPU使用率
    float cpuPercent = getCpuUsage();
    cpuProgressBar->setValue(static_cast<int>(cpuPercent));
    setProgressColor(cpuProgressBar, cpuPercent);
    
    // 更新CPU温度
    float cpuTemp = getCpuTemperature();
    if (cpuTemp > 0) {
        cpuTempLabel->setText(tr("%1°C").arg(cpuTemp, 0, 'f', 1));
    } else {
        cpuTempLabel->setText(tr("N/A"));
    }
}

float ResourceMonitor::getCpuUsage()
{
    std::ifstream statFile("/proc/stat");
    if (!statFile.is_open())
    {
        qDebug() << "Failed to open /proc/stat";
        return 0.0f;
    }

    std::string line;
    std::getline(statFile, line);
    statFile.close();

    std::istringstream ss(line);
    std::string cpu;
    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal, guest, guest_nice;

    ss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal >> guest >> guest_nice;

    // 计算总和
    unsigned long long totalUser = user - guest;
    unsigned long long totalUserLow = nice - guest_nice;
    unsigned long long totalSys = system + irq + softirq;
    unsigned long long totalIdle = idle + iowait;

    // 计算总值
    unsigned long long total = totalUser + totalUserLow + totalSys + totalIdle;

    // 计算CPU使用率
    float percent = 0.0f;
    if (lastTotalUser > 0 || lastTotalUserLow > 0 || lastTotalSys > 0 || lastTotalIdle > 0)
    {
        // 计算使用的差值
        unsigned long long diffIdle = totalIdle - lastTotalIdle;
        unsigned long long diffTotal = total - (lastTotalUser + lastTotalUserLow + lastTotalSys + lastTotalIdle);

        if (diffTotal > 0)
        {
            percent = 100.0f * (diffTotal - diffIdle) / diffTotal;
        }
    }

    // 保存当前值供下次使用
    lastTotalUser = totalUser;
    lastTotalUserLow = totalUserLow;
    lastTotalSys = totalSys;
    lastTotalIdle = totalIdle;

    return percent;
}

float ResourceMonitor::getCpuTemperature()
{
    // 在Jetson上通过thermal_zone读取CPU温度
    // 尝试多个可能的thermal_zone文件
    for (int i = 0; i < 10; i++)
    {
        std::string path = "/sys/devices/virtual/thermal/thermal_zone" + std::to_string(i) + "/temp";
        std::ifstream file(path);
        if (file.is_open())
        {
            int temp;
            file >> temp;
            file.close();
            
            // 通常值是以毫摄氏度为单位的，需要除以1000转换为摄氏度
            return temp / 1000.0f;
        }
    }
    
    return -1.0f; // 无法获取温度
}

void ResourceMonitor::setProgressColor(QProgressBar *progressBar, float value)
{
    QPalette palette = progressBar->palette();
    QColor color;

    if (value >= 80)
    {
        // 红色 - 高使用率
        color = QColor(200, 0, 0);
    }
    else if (value >= 60)
    {
        // 黄色 - 中等使用率
        color = QColor(200, 200, 0);
    }
    else
    {
        // 绿色 - 低使用率
        color = QColor(0, 200, 0);
    }

    palette.setColor(QPalette::Highlight, color);
    progressBar->setPalette(palette);
}