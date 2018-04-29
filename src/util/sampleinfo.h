#ifndef SAMPLEINFO_H
#define SAMPLEINFO_H

#include "Eigen/Dense"

struct SampleInfo {

    //initialize denoise info to all infinity except for radiance
    SampleInfo() : sample_radiance(Eigen::Vector3f(0, 0, 0)), sample_normal(Eigen::Vector3f(INFINITY, INFINITY, INFINITY)),
        sample_color(Eigen::Vector3f(0, 0, 0)), sample_depth(INFINITY) {}

    Eigen::Vector3f sample_radiance;
    Eigen::Vector3f sample_normal;
    Eigen::Vector3f sample_color;
    float sample_depth;
};

struct PixelInfo {

    PixelInfo() : num_samples(0), samplesPerPixel(std::vector<SampleInfo>(0)), radiance(Eigen::Vector3f(0, 0, 0)) {}

    PixelInfo(int num_samples) : num_samples(num_samples), samplesPerPixel(std::vector<SampleInfo>(num_samples)),
        radiance(Eigen::Vector3f(0, 0, 0)) {}

    void addEmptySample() {
        samplesPerPixel.push_back(SampleInfo());
        num_samples += 1;
    }

    void addInfo(const PixelInfo &infoToAdd) {
        samplesPerPixel.insert(samplesPerPixel.end(), infoToAdd.samplesPerPixel.begin(), infoToAdd.samplesPerPixel.end());
        radiance += infoToAdd.radiance;
        num_samples += infoToAdd.num_samples;
    }

    int num_samples;
    std::vector<SampleInfo> samplesPerPixel;
    Eigen::Vector3f radiance;
};

#endif // SAMPLEINFO_H
