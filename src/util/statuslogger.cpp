#include "statuslogger.h"

#include <iostream>

StatusLogger* StatusLogger::instance = nullptr;

StatusLogger::StatusLogger(int parallel_range, int image_width, int output_height) :
    m_parallel_range(parallel_range), m_image_width(image_width), m_output_height(output_height)
{
    // Setup boolean array
    int size = m_image_width * m_output_height / (m_parallel_range * m_parallel_range);
    finished_sections.resize(size);

    // Initialize all sections to not finished
    for(unsigned int i = 0; i < finished_sections.size(); i++) {
        finished_sections[i] = false;
    }

}

void StatusLogger::initInstance(int parallel_range, int image_width, int output_height) {
    instance = new StatusLogger(parallel_range, image_width, output_height);
}

void StatusLogger::deleteInstance() {
    delete instance;
}

StatusLogger *StatusLogger::getInstance() {
    if (instance == nullptr) {
        std::cerr << "Status Logger is uninitialized" << std::endl;
        exit(1);
    }

    return instance;
}

void StatusLogger::updateStatus(int x, int y) {
    status_mutex.lock();
    int x_index = x / m_parallel_range;
    int y_index = y / m_parallel_range;
    finished_sections[x_index + y_index * (m_image_width / m_parallel_range)] = true;
    if (SHOULD_PRINT_STATUS)
        std::cout << "(" << y_index << ", " << x_index << ")" << std::endl;
    status_mutex.unlock();
    printStatus();

}

void StatusLogger::printStatus() {
    if (!SHOULD_PRINT_STATUS) return; // Turn off print statements if they are annoying

    status_mutex.lock();
    for(int y = 0; y < m_output_height / m_parallel_range; y ++) {
        for(int x = 0; x < m_image_width / m_parallel_range; x++) {
            std::cout << "|";
            char out[2] = " "; // Used to print the next character
            if (finished_sections[x + (y * m_image_width / m_parallel_range)]) out[0] = 'X';
            std:: cout << out;
        }
        std::cout << "|" << std::endl;
    }
    std::cout << std::endl;
    status_mutex.unlock();
}
