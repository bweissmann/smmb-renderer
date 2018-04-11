#ifndef STATUSLOGGER_H
#define STATUSLOGGER_H

#include <vector>
#include <QMutex>

class StatusLogger
{
public:
    StatusLogger(int parallel_range, int image_width, int output_height);

    static void initInstance(int parallel_range, int image_width, int output_height);
    static void deleteInstance();

    static StatusLogger *getInstance();

    void updateStatus(int x, int y);

    static const bool SHOULD_PRINT_STATUS = true;

private:
    static StatusLogger *instance;

    void printStatus();

    QMutex status_mutex;

    int m_parallel_range;
    int m_image_width;
    int m_output_height;

    std::vector<bool> finished_sections;
};

#endif // STATUSLOGGER_H
