#ifndef DENOISER_H
#define DENOISER_H

#include <QImage>
#include <Eigen/Dense>
#include <iostream>
#include "util/sampleinfo.h"

/** Top class for the Denoiser
 * Interface
 *  - take in an image - array of intensiy values (Eigen::Vector3f *intensityValues)
 *  - take in the scene (const Scene &scene)
 *  - take in a pointer to a QRgb (QRgb *imageData)
 * Mode of operation
 *  - modify the QRgb array
 */

class Denoiser {
public:
    Denoiser(int width, int height, int radius, QString name);
    ~Denoiser();
    void init(Eigen::Vector3f *intensityValues, int *numSamples,
              Eigen::Vector3f **samples, Eigen::Vector3f **colours,
              Eigen::Vector3f **normals, float** depths);
    void init(std::vector<PixelInfo>& pixelInfos);
    void run();

private:
    void toneMap(Eigen::Vector3f *buf, QRgb *pixels);
    void toneMapToBuf(Eigen::Vector3f* ibuf, Eigen::Vector3f* obuf);
    void filter();

    void splitIntoBuffers();
    void calculateVariances();
    int getIndex(int cols, int row, int col);
    void getCoords(int index, int cols, int* row, int* col);
    bool outOfBufferBounds(int X, int Y, int x, int y);
    void filterBufferVariances_RKZ12();
    template <class T>
    void filterBufferVariances_RMZ13(T* in, T** samples, T* variance_out, T* in_A, T* in_B, T init);
    void prefilterFeatures();
    void NL_means_filter(int c_r, int c_f, float c_k, Eigen::Vector3f *in, Eigen::Vector3f *out, Eigen::Vector3f *variance, float **weight_buf, bool saveWeights);

    // Other stuff
    template <class T>
    void splitBuffer(int i, int num_A, int num_B,
                     T** source, T** a_dest, T* a_avg, T** b_dest, T* b_avg);
    template <class T>
    void calculateBufferVariance(int num_pixels, T** samples, int* num_samples, T* average, T* out, T init);
    void squaredDifference(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f *difference);
    void squaredDifference(float a, float b, float *difference);
    void ratioValues(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c, Eigen::Vector3f *ratio);
    void ratioValues(float a, float b, float c, float *ratio);
    void saveImage(Eigen::Vector3f *buf, QString nameMod);
    void saveImage(float *buf, QString nameMod);
    void saveImageNoToneMap(Eigen::Vector3f* buf, QString nameMod);

    // original image dimensions
    int m_height;
    int m_width;

    // radius of our filters
    int m_radius;

    // filename
    QString m_name;

    // scene data
    Eigen::Vector3f *m_intensityValues;
    int *m_numSamples;
    Eigen::Vector3f **m_samples;
    Eigen::Vector3f *m_colourValues;
    Eigen::Vector3f **m_colours;
    Eigen::Vector3f *m_normalValues;
    Eigen::Vector3f **m_normals;
    float *m_depthValues;
    float **m_depths;
    Eigen::Affine3f m_invViewMatrix;

    // buffer A
    int *m_numSamplesA;
    Eigen::Vector3f **m_samplesA;
    Eigen::Vector3f **m_coloursA;
    Eigen::Vector3f **m_normalsA;
    float **m_depthsA;
    Eigen::Vector3f *m_intensityValuesA;
    Eigen::Vector3f *m_colourValuesA;
    Eigen::Vector3f *m_normalValuesA;
    float *m_depthValuesA;
    Eigen::Vector3f *m_variancesA;
    Eigen::Vector3f *m_colour_variancesA;
    Eigen::Vector3f *m_normal_variancesA;
    float *m_depth_variancesA;

    // buffer B
    int *m_numSamplesB;
    Eigen::Vector3f **m_samplesB;
    Eigen::Vector3f **m_coloursB;
    Eigen::Vector3f **m_normalsB;
    float **m_depthsB;
    Eigen::Vector3f *m_intensityValuesB;
    Eigen::Vector3f *m_colourValuesB;
    Eigen::Vector3f *m_normalValuesB;
    float *m_depthValuesB;
    Eigen::Vector3f *m_variancesB;
    Eigen::Vector3f *m_colour_variancesB;
    Eigen::Vector3f *m_normal_variancesB;
    float *m_depth_variancesB;

    // other buffers
    Eigen::Vector3f* m_dual_buffer_variances;
    Eigen::Vector3f* m_variances;
    Eigen::Vector3f* m_colour_variances;
    Eigen::Vector3f* m_normal_variances;
    float* m_depth_variances;
};

#endif // DENOISER_H
