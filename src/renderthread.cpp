#include "renderthread.h"

#include <QThread>
#include <QDebug>
#include <QCoreApplication>
#include <iostream>

#include "pathtracer.h"
#include "util/statuslogger.h"

using namespace Eigen;

void RenderThread::run()
{
    /* Now that we have a new thread, we can tell the scene to render itself. */
    for (int x = 0; x < m_range; x ++) {
        for (int y = 0; y < m_range; y++) {
            m_pathTracer->tracePixel(m_x + x, m_y + y, m_scene, m_intensityValues, *m_invViewMatrix);
        }
    }
    std::cout << "(" << m_x / m_range << ", " << m_y / m_range << ")" << std::endl;
    StatusLogger::getInstance()->updateStatus(m_x, m_y); // Tell the path tracer that our square is done for logging
}

void RenderThread::setData(PathTracer *p, Vector3f *intensityValues, const Scene &scene, int x, int y,
                           int range, Matrix4f *invViewMatrix) {
    m_pathTracer = p;
    m_intensityValues = intensityValues;
    m_scene = scene;
    m_x = x;
    m_y = y;
    m_range = range;
    m_invViewMatrix = invViewMatrix;
}
