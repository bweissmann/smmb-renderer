#include "statuslogger.h"

#include <iostream>

QMutex StatusLogger::status_mutex;

int StatusLogger::m_parallel_range;
int StatusLogger::m_image_width;
int StatusLogger::m_output_height;

std::vector<bool> StatusLogger::status;

StatusLogger::StatusLogger()
{
}

void StatusLogger::updateStatus(int x, int y) {
    StatusLogger::status_mutex.lock();
    int x_index = x / StatusLogger::m_parallel_range;
    int y_index = y / StatusLogger::m_parallel_range;
    StatusLogger::status[x_index + y_index * (StatusLogger::m_image_width / StatusLogger::m_parallel_range)] = true;
    StatusLogger::status_mutex.unlock();
    StatusLogger::printStatus();
}

void StatusLogger::printStatus() {
    if (!SHOULD_PRINT_STATUS) {
        return; // Turn off print statements if they are annoying
    }
    StatusLogger::status_mutex.lock();

    for(int y = 0; y < StatusLogger::m_output_height / StatusLogger::m_parallel_range; y ++) {
        for(int x = 0; x < StatusLogger::m_image_width / StatusLogger::m_parallel_range; x++) {
            std::cout << "|";
            char out[2] = " "; // Used to print the next character
            if (StatusLogger::status[x + (y * StatusLogger::m_image_width / StatusLogger::m_parallel_range)]) out[0] = 'X';
            std:: cout << out;
        }
        std::cout << "|" << std::endl;
    }
    std::cout << std::endl;
    StatusLogger::status_mutex.unlock();
}


void StatusLogger::resizeStatus(int size, int parallel_range, int image_width, int output_height) {
    StatusLogger::status.resize(size);
    m_parallel_range = parallel_range;
    m_image_width = image_width;
    m_output_height = output_height;

    for(unsigned int i = 0; i < status.size(); i++) {
        status[i] = false;
    }
}
