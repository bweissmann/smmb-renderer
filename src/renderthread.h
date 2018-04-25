#ifndef RENDERTHREAD_H
#define RENDERTHREAD_H

#include <QRunnable>
#include <QObject>

#include <QThread>

#include "scene/scene.h"
#include "pathtracer.h"

class PathTracer;

class RenderThread : public QRunnable
{
public:
    /* Set all the data we need to calculate rays */
    void setData(PathTracer *p, Eigen::Vector3f *intensityValues, const Scene &scene, int x, int y,
                 int range, Eigen::Matrix4f *invViewMatrix, RenderType render_type);

private:
    void run();

    /* Not a perfect solution to store a ton of member variables
     * to do our computation but it gets the job done. */
    PathTracer *m_pathTracer;
    Eigen::Vector3f *m_intensityValues;
    Scene m_scene;
    int m_x;
    int m_y;
    int m_range;
    Eigen::Matrix4f *m_invViewMatrix;
    RenderType m_render_type;
};

#endif // RENDERTHREAD_H
