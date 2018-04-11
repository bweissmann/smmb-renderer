#ifndef STATUSLOGGER_H
#define STATUSLOGGER_H

#include <vector>
#include <QMutex>

class StatusLogger
{
public:
    StatusLogger();

    static void resizeStatus(int size, int parallel_range, int image_width, int output_height);
    static void updateStatus(int x, int y);
    static void printStatus();

    static const bool SHOULD_PRINT_STATUS = true;
    static QMutex status_mutex;

    static int m_parallel_range;
    static int m_image_width;
    static int m_output_height;

    static std::vector<bool> status;

};

#endif // STATUSLOGGER_H
