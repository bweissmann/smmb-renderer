#include "denoiser.h"

/**
 * This denoiser is an implementation of two papers, "Adaptive Rendering with Non-Local Means Filtering" (RKZ12) and
 * "Robust Denoising using Feature and Color Information" (RMZ13)
 */

Denoiser::Denoiser(int width, int height, int radius, QString name) :
    m_height(height),
    m_width(width),
    m_radius(radius),
    m_name(name)
{

}

Denoiser::~Denoiser() {

}

void Denoiser::init(Eigen::Vector3f *intensityValues, int *numSamples,
                    Eigen::Vector3f **samples, Eigen::Vector3f **colours,
                    Eigen::Vector3f **normals, float** depths) {
    // if our monte carlo renderer returns the six basic sets of data, we simply set
    // our pointers to point to them.
    m_intensityValues = intensityValues;
    m_numSamples = numSamples;
    m_samples = samples;
    m_colours = colours;
    m_normals = normals;
    m_depths = depths;
}

void Denoiser::init(PixelInfo* pixelInfos) {
    // if we get give a pixel info vector instead, then we need to decompose it into
    // the six basic sets of data. Our denoising functions operate on arrays of vectors or
    // arrays of floats only, so decomposing our data as such helps us write cleaner and
    // more abstract code.
    int num_pixels = m_width * m_height;
    m_intensityValues = new Eigen::Vector3f[num_pixels];
    m_candidate_1 = new Eigen::Vector3f[num_pixels];
    m_candidate_2 = new Eigen::Vector3f[num_pixels];
    m_candidate_3 = new Eigen::Vector3f[num_pixels];
    m_numSamples = new int[num_pixels];
    m_samples = new Eigen::Vector3f*[num_pixels];
    m_colours = new Eigen::Vector3f*[num_pixels];
    m_normals = new Eigen::Vector3f*[num_pixels];
    m_depths = new float*[num_pixels];
    for (int i = 0; i < num_pixels; i++) {
        m_intensityValues[i] = pixelInfos[i].radiance;
        int num_samples = pixelInfos[i].num_samples;
        m_numSamples[i] = num_samples;
        m_samples[i] = new Eigen::Vector3f[num_samples];
        m_colours[i] = new Eigen::Vector3f[num_samples];
        m_normals[i] = new Eigen::Vector3f[num_samples];
        m_depths[i] = new float[num_samples];
        for (int j = 0; j < num_samples; j++) {
            m_samples[i][j] = pixelInfos[i].samplesPerPixel[j].sample_radiance;
            m_colours[i][j] = pixelInfos[i].samplesPerPixel[j].sample_color;
            if (m_colours[i][j] == Eigen::Vector3f(INFINITY, INFINITY, INFINITY)) {
                m_colours[i][j] = Eigen::Vector3f(0.0, 0.0, 0.0);
            }
            m_normals[i][j] = pixelInfos[i].samplesPerPixel[j].sample_normal;
            if (m_normals[i][j] == Eigen::Vector3f(INFINITY, INFINITY, INFINITY)) {
                m_normals[i][j] = Eigen::Vector3f(0.0, 1.0, 0.0);
            }
            absVal(&m_normals[i][j]);
            m_depths[i][j] = pixelInfos[i].samplesPerPixel[j].sample_depth;
            if (m_depths[i][j] == INFINITY) {
                m_depths[i][j] = 0.5;
            }
        }
    }
}

void Denoiser::run() {
    // split into the dual buffers and do a variance calculation for each of them
    int num_pixels = m_width * m_height;
    splitIntoBuffers();
    calculateVariances();

    // calculate the variances for the colour space stuff
    filterBufferVariances_RMZ13(m_intensityValues, m_samples, m_variances, m_intensityValuesA, m_intensityValuesB, Eigen::Vector3f(0.0, 0.0, 0.0), "intensity");

    // calculate our feature variances like we did for the first colour filter.
    filterBufferVariances_RMZ13(m_colourValues, m_colours, m_colour_variances, m_colourValuesA, m_colourValuesB, Eigen::Vector3f(0.0, 0.0, 0.0), "colour");
    filterBufferVariances_RMZ13(m_normalValues, m_normals, m_normal_variances, m_normalValuesA, m_normalValuesB, Eigen::Vector3f(0.0, 0.0, 0.0), "normal");
    filterBufferVariances_RMZ13(m_depthValues, m_depths, m_depth_variances, m_depthValuesA, m_depthValuesB, 0.f, "depth");

    // TODO: Pre-filter the feature buffers in the way described by the paper
    prefilterFeatures();

    // TODO: Calculate the gradient of the feature buffers
    m_colour_gradient = new Eigen::Vector3f[num_pixels];
    m_normal_gradient = new Eigen::Vector3f[num_pixels];
    m_depth_gradient = new float[num_pixels];

    sobelFilterSquared(m_colourValues, m_colour_gradient);
    sobelFilterSquared(m_normalValues, m_normal_gradient);
    Eigen::Vector3f *depth_Vec = new Eigen::Vector3f[num_pixels];
    Eigen::Vector3f *depth_Grad = new Eigen::Vector3f[num_pixels];
    for (int i = 0; i < num_pixels; i++) {
        depth_Vec[i] = Eigen::Vector3f(m_depthValues[i], m_depthValues[i], m_depthValues[i]);
    }
    sobelFilterSquared(depth_Vec, depth_Grad);
    for (int i = 0; i < num_pixels; i++) {
        m_depth_gradient[i] = depth_Grad[i](0);
    }


    // DEBUG: Save the noisy image and feature buffers!
    saveImage(m_intensityValues, "undenoised");
    saveImage(m_colourValues, "colours");
    saveImage(m_normalValues, "normals");
    saveImage(depth_Vec, "depths");
    // CANDIATE FILTER ONE
    // do the first candidate filter
    // c_f = 1, c_k = 0.45, c_kf = 0.6
    float** bilateral_weights1 = new float*[num_pixels];
    float** NL_weights1 = new float*[num_pixels];
    float** final_weights1 = new float*[num_pixels];
    NL_means_filter(m_radius, 1, 0.45, m_intensityValues, m_variances, NL_weights1);
    bilateral_means_filter(m_radius, 0.6, bilateral_weights1, 1e-3);
    minWeights(m_radius, NL_weights1, bilateral_weights1, final_weights1);
    filterWithWeights(m_radius, m_intensityValues, m_candidate_1, final_weights1, Eigen::Vector3f(0.0, 0.0, 0.0));
    saveImage(m_candidate_1, "candidate-1");

    // CANDIDATE FILTER TWO
    // do the second candidate filter, which is done in the same way as the first. Same TODOs
    // c_f = 3, c_k = 0.45, c_kf = 0.6
    float** NL_weights2 = new float*[num_pixels];
    float** final_weights2 = new float*[num_pixels];
    NL_means_filter(m_radius, 3, 0.45, m_intensityValues, m_variances, NL_weights2);
    minWeights(m_radius, NL_weights2, bilateral_weights1, final_weights2);
    filterWithWeights(m_radius, m_intensityValues, m_candidate_2, final_weights2, Eigen::Vector3f(0.0, 0.0, 0.0));
    saveImage(m_candidate_2, "candidate-2");

    // CANDIDATE FILTER THREE
    // do the third candidate filter
    // c_f = *, c_k = +inf, c_kf = 0.6 (in other words, we don't need to NL_means_filter here
    float** bilateral_weights3 = new float*[num_pixels];
    bilateral_means_filter(m_radius, 0.6, bilateral_weights3, 1e-4);
    filterWithWeights(m_radius, m_intensityValues, m_candidate_3, bilateral_weights3, Eigen::Vector3f(0.0, 0.0, 0.0));
    saveImage(m_candidate_3, "candidate-3");

    average_candidates();

    // post-processing
    saveImage(m_intensityValues, "denoised");
    delete[] depth_Vec;
    delete[] depth_Grad;
}


// ============
// COLOUR FEATURE FUNCTIONS
// For filtering in the image-space using only colour/intensity information
// ============

// The NL-means filter is improved by calculating pixel variance when the samples are spread
// over two buffers. My understanding of this is that we obtain an error of order 3 when doing so.
// RKZ12 does not mention this as being the rationale, but my intuition leads me to this conclusion.
// This function takes the individual samples contributing to the image and puts them into two different
// buffers, each of which contains half of the samples.
void Denoiser::splitIntoBuffers() {
    // Malloc all of our buffers
    int num_pixels = m_height * m_width;
    m_colourValues = new Eigen::Vector3f[num_pixels];
    m_normalValues = new Eigen::Vector3f[num_pixels];
    m_depthValues = new float[num_pixels];
    m_numSamplesA = new int[num_pixels];
    m_numSamplesB = new int[num_pixels];
    m_intensityValuesA = new Eigen::Vector3f[num_pixels];
    m_intensityValuesB = new Eigen::Vector3f[num_pixels];
    m_colourValuesA = new Eigen::Vector3f[num_pixels];
    m_colourValuesB = new Eigen::Vector3f[num_pixels];
    m_normalValuesA = new Eigen::Vector3f[num_pixels];
    m_normalValuesB = new Eigen::Vector3f[num_pixels];
    m_depthValuesA = new float[num_pixels];
    m_depthValuesB = new float[num_pixels];
    m_samplesA = new Eigen::Vector3f*[num_pixels];
    m_samplesB = new Eigen::Vector3f*[num_pixels];
    m_coloursA = new Eigen::Vector3f*[num_pixels];
    m_coloursB = new Eigen::Vector3f*[num_pixels];
    m_normalsA = new Eigen::Vector3f*[num_pixels];
    m_normalsB = new Eigen::Vector3f*[num_pixels];
    m_depthsA = new float*[num_pixels];
    m_depthsB = new float*[num_pixels];
    // for each pixel, split the complete buffers into two
    for (int i = 0; i < num_pixels; i++) {
        // get the number of samples in each
        int num_samples = m_numSamples[i];
        int num_samplesA = num_samples / 2;
        int num_samplesB = num_samples - num_samplesA;
        m_numSamplesA[i] = num_samplesA;
        m_numSamplesB[i] = num_samplesB;
        // split the intensities and average them correctly
        m_intensityValuesA[i] = Eigen::Vector3f(0.0, 0.0, 0.0);
        m_intensityValuesB[i] = Eigen::Vector3f(0.0, 0.0, 0.0);
        splitBuffer(i, num_samplesA, num_samplesB, m_samples, m_samplesA, m_intensityValuesA,
                    m_samplesB, m_intensityValuesB, false);
        // split the colours and average them correctly
        m_colourValuesA[i] = Eigen::Vector3f(0.0, 0.0, 0.0);
        m_colourValuesB[i] = Eigen::Vector3f(0.0, 0.0, 0.0);
        splitBuffer(i, num_samplesA, num_samplesB, m_colours, m_coloursA, m_colourValuesA,
                     m_coloursB, m_colourValuesB, false);
        // split the normals and average them correctly
        m_normalValuesA[i] = Eigen::Vector3f(0.0, 0.0, 0.0);
        m_normalValuesB[i] = Eigen::Vector3f(0.0, 0.0, 0.0);
        splitBuffer(i, num_samplesA, num_samplesB, m_normals, m_normalsA, m_normalValuesA,
                    m_normalsB, m_normalValuesB, false);
        // split the depths and average them correctly
        m_depthValuesA[i] = 0.0;
        m_depthValuesB[i] = 0.0;
        splitBuffer(i, num_samplesA, num_samplesB, m_depths, m_depthsA, m_depthValuesA,
                    m_depthsB, m_depthValuesB, false);
        m_colourValues[i] = 0.5 * (m_colourValuesA[i] + m_colourValuesB[i]);
//        absVal(&m_colourValues[i]);
        m_normalValues[i] = 0.5 * (m_normalValuesA[i] + m_normalValuesB[i]);
//        absVal(&m_mormalValues[i]);
        m_depthValues[i] = 0.5 * (m_depthValuesA[i] + m_depthValuesB[i]);
    }
}

// Calculate the per-pixel variances of each pixel in both buffers
void Denoiser::calculateVariances() {
    // malloc our variances buffers
    int num_pixels = m_height * m_width;
    m_variances = new Eigen::Vector3f[num_pixels];
    m_variancesA = new Eigen::Vector3f[num_pixels];
    m_variancesB = new Eigen::Vector3f[num_pixels];
    m_colour_variancesA = new Eigen::Vector3f[num_pixels];
    m_normal_variancesA = new Eigen::Vector3f[num_pixels];
    m_depth_variancesA = new float[num_pixels];
    m_colour_variancesB = new Eigen::Vector3f[num_pixels];
    m_normal_variancesB = new Eigen::Vector3f[num_pixels];
    m_depth_variancesB = new float[num_pixels];
    m_colour_variances = new Eigen::Vector3f[num_pixels];
    m_normal_variances = new Eigen::Vector3f[num_pixels];
    m_depth_variances = new float[num_pixels];

    // calculate sample variances for each pixel in buffer A
    calculateBufferVariance(num_pixels, m_samplesA, m_numSamplesA, m_intensityValuesA, m_variancesA, Eigen::Vector3f(0.0, 0.0, 0.0));

    // calculate sample variances for each pixel in buffer B
    calculateBufferVariance(num_pixels, m_samplesB, m_numSamplesB, m_intensityValuesB, m_variancesB, Eigen::Vector3f(0.0, 0.0, 0.0));

    // calcuate feature variances for each pixel in buffer A
    calculateBufferVariance(num_pixels, m_coloursA, m_numSamplesA, m_colourValuesA, m_colour_variancesA, Eigen::Vector3f(0.0, 0.0, 0.0));
    calculateBufferVariance(num_pixels, m_normalsA, m_numSamplesA, m_normalValuesA, m_normal_variancesA, Eigen::Vector3f(0.0, 0.0, 0.0));
    calculateBufferVariance(num_pixels, m_depthsA, m_numSamplesA, m_depthValuesA, m_depth_variancesA, 0.f);

    // calcuate feature variances for each pixel in buffer B
    calculateBufferVariance(num_pixels, m_coloursB, m_numSamplesB, m_colourValuesB, m_colour_variancesB, Eigen::Vector3f(0.0, 0.0, 0.0));
    calculateBufferVariance(num_pixels, m_normalsB, m_numSamplesB, m_normalValuesB, m_normal_variancesB, Eigen::Vector3f(0.0, 0.0, 0.0));
    calculateBufferVariance(num_pixels, m_depthsB, m_numSamplesB, m_depthValuesB, m_depth_variancesB, 0.f);
}

// calculate variances in a different manner. This is described by RMZ13. I do not know if this
// or if the RKZ12 paper is better. We shall see...
// RMZ13 says to compute the sample variance for each pixel, and the empirical variance between the
// two buffers. Then, we blur these with a 21x21 box filter. Then, we compute the ratio between these
// on a pixel-by-pixel basis. After that, we multiply this ratio on the initial unfiltered sample variance.
// This is our variance that we will use in the NL-means filter.
template <class T>
void Denoiser::filterBufferVariances_RMZ13(T* in, T** samples, T* variance_out, T* in_A, T* in_B, T init, QString mod) {
    // calculate the sample variance and dual buffer empirical variance
    std::cout << "Calculating variances..." << std::endl;
    int num_pixels = m_height * m_width;
    T* temp_variances = new T[num_pixels];
    T* temp_buffer_variances = new T[num_pixels];
    T* dual_buffer_variances = new T[num_pixels];
    for (int i = 0; i < num_pixels; i++) {
        T variance = init;
        int num_samples = m_numSamples[i];
        for (int j = 0; j < num_samples; j++) {
            T curr_var;\
            squaredDifference(in[i], samples[i][j], &curr_var);
            variance += curr_var;
        }
        variance = variance / (float)(num_samples - 1);
        temp_variances[i] = variance;
        T curr_var;
        squaredDifference(in_A[i], in_B[i], &curr_var);
        temp_buffer_variances[i] = 0.5 * curr_var;
        variance_out[i] = init;
        dual_buffer_variances[i] = init;
    }
    saveImage(temp_variances, mod + "-" + "sample-variance-unfiltered");
    saveImage(temp_buffer_variances, mod + "-" + "buffer-variance-unfiltered");
    // filter with a 21x21 box filter
    int r = 10;
    for (int i = 0; i < num_pixels; i++) {
        T filter_value = init;
        T buffer_filter_value = init;
        int normalizer = ((2 * r) + 1) * ((2 * r) + 1);
        int row_centre, col_centre;
        getCoords(i, m_width, &row_centre, &col_centre);
        for (int col = col_centre - r; col <= col_centre + r; col++) {
            for (int row = row_centre - r; row <= row_centre + r; row++) {
                if (outOfBufferBounds(m_width, m_height, row, col)) {
                   normalizer--;
                   continue;
                }
                int index = getIndex(m_width, row, col);
                filter_value += temp_variances[index];
                buffer_filter_value += temp_buffer_variances[index];
            }
        }
        variance_out[i] = filter_value / (float)normalizer;
        dual_buffer_variances[i] = buffer_filter_value / (float)normalizer;
    }
    saveImage(variance_out, mod + "-" + "sample-variance-filtered");
    saveImage(dual_buffer_variances, mod + "-" + "buffer-variance-filtered");
    // compute ratio on a per-pixel basis and then apply it to the original unblurred sample variances
    for (int i = 0; i < num_pixels; i++) {
        T normalizedVal;
        ratioValues(dual_buffer_variances[i], variance_out[i], temp_variances[i], &normalizedVal);
        temp_variances[i] = normalizedVal;
    }
    // copy over into our m_variances buffer. We are done... according to RMZ13.
    for (int i = 0; i < num_pixels; i++) {
        variance_out[i] = temp_variances[i];
    }
    saveImage(variance_out, mod + "-" + "final-variance");
    // cleanup
    delete[] temp_variances;
    delete[] temp_buffer_variances;
}

// prefiltering for the feature buffers
void Denoiser::prefilterFeatures() {
    std::cout << "Prefiltering features..." << std::endl;
    int c_r = 5;
    int c_f = 3;
    int c_k = 1.f;
    int pixels = m_height * m_width;
    float** colour_prefilter_weights = new float*[pixels];
    float** normal_prefilter_weights = new float*[pixels];
    float** depth_prefilter_weights = new float*[pixels];
    Eigen::Vector3f* depth_vec = new Eigen::Vector3f[pixels];
    Eigen::Vector3f* depth_var = new Eigen::Vector3f[pixels];
    // fill out our depth_vec
    for (int i = 0; i < pixels; i++) {
        depth_vec[i] = Eigen::Vector3f(m_depthValues[i], m_depthValues[i], m_depthValues[i]);
        depth_var[i] = Eigen::Vector3f(m_depth_variances[i], m_depth_variances[i], m_depth_variances[i]);
    }
    // get the weights for everyone
    NL_means_filter(c_r, c_f, c_k, m_colourValues, m_colour_variances, colour_prefilter_weights);
    std::cout << "Colours done" << std::endl;
    NL_means_filter(c_r, c_f, c_k, m_normalValues, m_normal_variances, normal_prefilter_weights);
    std::cout << "Normals done" << std::endl;
    NL_means_filter(c_r, c_f, c_k, depth_vec, depth_var, depth_prefilter_weights);
    std::cout << "Depths done" << std::endl;
    // filter the A and B dual buffers with these weights
    filterWithWeights(c_r, m_colourValuesA, m_colourValuesA, colour_prefilter_weights, Eigen::Vector3f(0.0, 0.0, 0.0));
    filterWithWeights(c_r, m_colourValuesB, m_colourValuesB, colour_prefilter_weights, Eigen::Vector3f(0.0, 0.0, 0.0));
    filterWithWeights(c_r, m_normalValuesA, m_normalValuesA, normal_prefilter_weights, Eigen::Vector3f(0.0, 0.0, 0.0));
    filterWithWeights(c_r, m_normalValuesB, m_normalValuesB, normal_prefilter_weights, Eigen::Vector3f(0.0, 0.0, 0.0));
    filterWithWeights(c_r, m_depthValuesA, m_depthValuesA, depth_prefilter_weights, 0.f);
    filterWithWeights(c_r, m_depthValuesB, m_depthValuesB, depth_prefilter_weights, 0.f);
    // do a squared difference of the two buffers and set this to be the variance as it stands.
    for (int i = 0; i < pixels; i++) {
        Eigen::Vector3f colourDiff;
        Eigen::Vector3f normalDiff;
        float depthDiff;
        Eigen::Vector3f depthDiff_vec;
        squaredDifference(m_colourValuesA[i], m_colourValuesB[i], &colourDiff);
        squaredDifference(m_normalValuesA[i], m_normalValuesB[i], &normalDiff);
        squaredDifference(m_depthValuesA[i], m_depthValuesB[i], &depthDiff);
        depthDiff_vec = Eigen::Vector3f(depthDiff, depthDiff, depthDiff);
        m_colour_variances[i] = colourDiff;
        m_normal_variances[i] = normalDiff;
        depth_vec[i] = depthDiff_vec;
    }
    // we need to filter each of these variance buffers with a Gaussian with standard deviation of
    // 0.5 pixels. Then we are done.
    gaussianBlur(0.5f, m_colour_variances, m_colour_variances);
    gaussianBlur(0.5f, m_normal_variances, m_normal_variances);
    gaussianBlur(0.5f, depth_vec, depth_vec);
    for (int i = 0; i < pixels; i++) {
        m_depth_variances[i] = depth_vec[i](0);
    }
}


// Finally perform an NL-means filtering on the output intensities.
// The method outlined for this is identical between RKZ12 and RMZ13. It is long,
// and contains many nested for loops. Perhaps I can figure out a better way of doing this,
// but for now, this should work... just be slow.
// when filtering the depth feature, we turn it into Vector3fs with the depth duplicated
// in each entry. This makes our code a little easier to understand even if a little slower.
void Denoiser::NL_means_filter(int c_r, int c_f, float c_k, Eigen::Vector3f* in,
                               Eigen::Vector3f* variance, float** weight_buf) {
    int image_pixels = m_width * m_height;
    int neighbour_pixels = ((2 * c_r) + 1) * ((2 * c_r) + 1);
    int patch_pixels = ((2 * c_f) + 1) * ((2 * c_f) + 1);
    // we need to allocate a few buffers to store the results of patchwise method calculations
    // for the (2r + 1) x (2r + 1) neighbourhood
    Eigen::Vector3f* N              = new Eigen::Vector3f[neighbour_pixels];
    float *N_weights                = new float[neighbour_pixels];
    bool *N_valid                   = new bool[neighbour_pixels];
    // for the (2f + 1) x (2f + 1) centre patch
    Eigen::Vector3f* P              = new Eigen::Vector3f[patch_pixels];
    Eigen::Vector3f* P_variances    = new Eigen::Vector3f[patch_pixels];
    bool* P_valid                   = new bool[patch_pixels];
    // for the (2f + 1) x (2f + 1) neighbour patch
    Eigen::Vector3f* Q              = new Eigen::Vector3f[patch_pixels];
    Eigen::Vector3f* Q_variances    = new Eigen::Vector3f[patch_pixels];
    bool* Q_valid                   = new bool[patch_pixels];

    // we also need a temporary buffer to store the results of the NL-means filtering, so
    // we don't clobber the original buffer.
    // now we can start to go through our image.
    for (int pixel = 0; pixel < image_pixels; pixel++) { // FOR EACH PIXEL IN IMAGE
        // if we are saving weights, we need to malloc space for our weights
        weight_buf[pixel] = new float[neighbour_pixels];
        float C = 0.f; // running total of the neigbour weights
        int row, col;
        getCoords(pixel, m_width, &row, &col); // get the current coordinate
        std::cout << col  << ", " << row << std::endl;
        // we can get the centre patch at this point. It does not change.
        for (int row_P = row - c_f; row_P <= row + c_f; row_P++) { // FOR EACH PIXEL
            for (int col_P = col - c_f; col_P <= col + c_f; col_P++) { // IN CENTRE PATCH
                int patch_index = getIndex((2 * c_f) + 1, row_P - (row - c_f), col_P - (col - c_f));
                // if we are out of bounds on the actual image, then we can't include this pixel
                // and we must mark it as invalid, and then continue on
                if (outOfBufferBounds(m_height, m_width, row_P, col_P)) {
                    P_valid[patch_index] = false;
                    continue;
                }
                int image_index = getIndex(m_width, row_P, col_P);
                P[patch_index] = in[image_index]; // get the pixel value from the input buffer
                P_valid[patch_index] = true; // valid
                P_variances[patch_index] = variance[image_index]; // get the pixel variance
            }
        }
        // we can now iterate through the neighbourhood
        for (int row_N = row - c_r; row_N <= row + c_r; row_N++) {
            for (int col_N = col - c_r; col_N <= col + c_r; col_N++) {
                int neighbourhood_index = getIndex((2 * c_r) + 1, row_N - (row - c_r), col_N - (col - c_r));
                // if we are out of bounds on the actual image, then we can't include this pixel
                // and we must mark it as invalid, and then continue on
                if (outOfBufferBounds(m_height, m_width, row_N, col_N)) {
                    N_valid[neighbourhood_index] = false;
                    continue;
                }
                // otherwise, we can gather our data
                int image_index = getIndex(m_width, row_N, col_N);
                N[neighbourhood_index] = in[image_index];
                N_weights[neighbourhood_index] = 0.f;
                // now we should get the patch surrounding the current neigbour
                for (int row_Q = row_N - c_f; row_Q <= row_N + c_f; row_Q++) {
                    for (int col_Q = col_N - c_f; col_Q <= col_N + c_f; col_Q++) {
                        int patch_index = getIndex((2 * c_f) + 1, row_Q - (row_N - c_f), col_Q - (col_N - c_f));
                        // if we are out of bounds on the actual image, then we can't include this pixel
                        // and we must mark it as invalid, and then continue on
                        if (outOfBufferBounds(m_width, m_height, row_Q, col_Q)) {
                            Q_valid[patch_index] = false;
                            continue;
                        }
                        // otherwise, we can gather our data
                        int image_index = getIndex(m_width, row_Q, col_Q);
                        Q[patch_index] = in[image_index];
                        Q_valid[patch_index] = true;
                        Q_variances[patch_index] = variance[image_index];
                    }
                }
                // so we now have the two patches, P and Q. We can use them to generate the weight for
                // the current neighbour which is the centre of Q.
                int normalizer = patch_pixels;
                float d2 = 0.f;
                for (int q = 0; q < patch_pixels; q++) {
                    // check if the entry in our patch is valid. If it is not, then we decrement the
                    // normalizer and continue.
                    if (!(P_valid[q] && Q_valid[q])) {
                        normalizer--;
                        continue;
                    }
                    // calculate the squared difference of the current patch pixel
                    Eigen::Vector3f PQ2(0.0, 0.0, 0.0);
                    squaredDifference(P[q], Q[q], &PQ2);
                    // find the minimum of P_variance and Q_variance
                    Eigen::Vector3f QP_variance;
                    QP_variance(0) = fmin(P_variances[q](0), Q_variances[q](0));
                    QP_variance(1) = fmin(P_variances[q](1), Q_variances[q](1));
                    QP_variance(2) = fmin(P_variances[q](2), Q_variances[q](2));
                    // get the numerator of our expression for the pixel distance
                    Eigen::Vector3f num = PQ2 - (P_variances[q] + QP_variance);
                    // small epsilon to prevent division by zero
                    Eigen::Vector3f epsilon(0.0001, 0.0001, 0.0001);
                    // get the denominator
                    Eigen::Vector3f den = epsilon + (c_k * c_k)*(P_variances[q] + Q_variances[q]);
                    // calculate the delta2
                    Eigen::Vector3f delta2 = num.cwiseQuotient(den);
                    d2 = d2 + delta2(0) + delta2(1) + delta2(2);
                }
                d2 = d2 / (3.f * (float)normalizer);
                d2 = fmax(0.f, d2);
                float w = std::exp(-d2);
                N_weights[neighbourhood_index] = w;
                N_valid[neighbourhood_index] = true;
                C += w;
            }
        }
        for (int n = 0; n < neighbour_pixels; n++) {
            weight_buf[pixel][n] = N_weights[n];
        }
    }
    delete[] P;
    delete[] P_variances;
    delete[] P_valid;
    delete[] Q;
    delete[] Q_variances;
    delete[] Q_valid;
    delete[] N;
    delete[] N_weights;
    delete[] N_valid;
}

void Denoiser::bilateral_means_filter(int c_r, float c_k, float** weight_buf, float tau) {
    // we need to make some buffers to store the final results and our intermediaries
    int image_pixels = m_width * m_height;
    int neighbour_pixels = ((2 * c_r) + 1) * ((2 * c_r) + 1);
    Eigen::Vector3f N_colour[neighbour_pixels];
    Eigen::Vector3f N_colour_variance[neighbour_pixels];
    Eigen::Vector3f N_normal[neighbour_pixels];
    Eigen::Vector3f N_normal_variance[neighbour_pixels];
    float N_depth[neighbour_pixels];
    float N_depth_variance[neighbour_pixels];
    bool N_valid[neighbour_pixels];
    for (int pixel = 0; pixel < image_pixels; pixel++) {
        weight_buf[pixel] = new float[neighbour_pixels];
        Eigen::Vector3f P_colour = m_colourValues[pixel];
        Eigen::Vector3f P_colour_variance = m_colour_variances[pixel];
        Eigen::Vector3f P_colour_gradient = m_colour_gradient[pixel];
        Eigen::Vector3f P_normal = m_normalValues[pixel];
        Eigen::Vector3f P_normal_variance = m_normal_variances[pixel];
        Eigen::Vector3f P_normal_gradient = m_normal_gradient[pixel];
        float P_depth = m_depthValues[pixel];
        float P_depth_variance = m_depth_variances[pixel];
        float P_depth_gradient = m_depth_gradient[pixel];
        int row, col;
        getCoords(pixel, m_width, &row, &col);
        for (int row_N = row - c_r; row_N <= row + c_r; row_N++) {
            for (int col_N = col + c_r; col_N <= col + c_r; col_N++) {
                int neighbour_index = getIndex((2 * c_r) + 1, row_N - (row - c_r), col_N - (col - c_r));
                if (outOfBufferBounds(m_width, m_height, row_N, col_N)) {
                    N_valid[neighbour_index] = false;
                    continue;
                }
                int image_index = getIndex(m_width, row_N, col_N);
                N_colour[neighbour_index] = m_colourValues[image_index];
                N_colour_variance[neighbour_index] = m_colour_variances[image_index];
                N_normal[neighbour_index] = m_normalValues[image_index];
                N_normal_variance[neighbour_index] = m_normal_variances[image_index];
                N_depth[neighbour_index] = m_depthValues[image_index];
                N_depth_variance[neighbour_index] = m_depth_variances[image_index];
            }
        }
        // now we can calculate the weights
        for (int n = 0; n < neighbour_pixels; n++) {
                if (N_valid[n]) {
                float colour_dist = bilateralCalc(P_colour, N_colour[n], P_colour_variance, N_colour_variance[n], P_colour_gradient, tau, c_k);
                float normal_dist = bilateralCalc(P_normal, N_normal[n], P_normal_variance, N_normal_variance[n], P_normal_gradient, tau, c_k);
                float depth_dist = bilateralCalc(P_depth, N_depth[n], P_depth_variance, N_depth_variance[n], P_depth_gradient, tau, c_k);
//                float maxDist = fmax(colour_dist, fmax(normal_dist, depth_dist));
                float maxDist = fmax(colour_dist, normal_dist);
                weight_buf[pixel][n] = std::exp(-maxDist);
            }
        }
    }
}

void Denoiser::SURE_calc(int pixels, Eigen::Vector3f sigma2, Eigen::Vector3f* candidate, float* SURE, float* SURE_deriv) {
    for (int i = 0; i < pixels; i++) {
        Eigen::Vector3f SURE_vec;
        squaredDifference(candidate[i], m_intensityValues[i], &SURE_vec);
        SURE_vec = SURE_vec - sigma2;
        Eigen::Vector3f deriv;
        deriv = candidate[i].cwiseQuotient(m_intensityValues[i]);
        SURE_deriv[i] = deriv.norm();
        SURE_vec = SURE_vec + 2.f*sigma2.cwiseProduct(deriv);
        SURE[i] = (1.f/3.f) * (SURE_vec(0) + SURE_vec(1) + SURE_vec(2));
    }
}

void Denoiser::average_candidates() {
    // calculate the true variance of the actual image
    int num_pixels = m_width * m_height;
    Eigen::Vector3f mean_intensity(0.0, 0.0, 0.0);
    for (int i = 0; i < num_pixels; i++) {
        mean_intensity += m_intensityValues[i];
    }
    mean_intensity = mean_intensity / (float)num_pixels;
    Eigen::Vector3f sigma2(0.0, 0.0, 0.0);
    for (int i = 0; i < num_pixels; i++) {
        Eigen::Vector3f squaredDiff;
        squaredDifference(m_intensityValues[i], mean_intensity, &squaredDiff);
        sigma2 += squaredDiff;
    }
    // do the two SURE error estimates
    sigma2 = sigma2 / (float)num_pixels;
    float* SURE1 = new float[num_pixels];
    float* SURE2 = new float[num_pixels];
    float* SURE3 = new float[num_pixels];
    float* SURE1_deriv = new float[num_pixels];
    float* SURE2_deriv = new float[num_pixels];
    float* SURE3_deriv = new float[num_pixels];
    SURE_calc(num_pixels, sigma2, m_candidate_1, SURE1, SURE1_deriv);
    SURE_calc(num_pixels, sigma2, m_candidate_2, SURE2, SURE2_deriv);
    SURE_calc(num_pixels, sigma2, m_candidate_3, SURE3, SURE3_deriv);
    // smooth the SURE error estimates
    Eigen::Vector3f* SURE1_vec = new Eigen::Vector3f[num_pixels];
    Eigen::Vector3f* SURE2_vec = new Eigen::Vector3f[num_pixels];
    Eigen::Vector3f* SURE3_vec = new Eigen::Vector3f[num_pixels];
    for (int i = 0; i < num_pixels; i++) {
        SURE1_vec[i] = Eigen::Vector3f(SURE1[i], SURE1[i], SURE1[i]);
        SURE2_vec[i] = Eigen::Vector3f(SURE2[i], SURE2[i], SURE2[i]);
        SURE3_vec[i] = Eigen::Vector3f(SURE3[i], SURE3[i], SURE3[i]);
    }
    float** SURE1_filter_w = new float*[num_pixels];
    float** SURE2_filter_w = new float*[num_pixels];
    float** SURE3_filter_w = new float*[num_pixels];
    NL_means_filter(1, 1, 1.f, SURE1_vec, m_variances, SURE1_filter_w);
    NL_means_filter(1, 1, 1.f, SURE2_vec, m_variances, SURE2_filter_w);
    NL_means_filter(1, 1, 1.f, SURE3_vec, m_variances, SURE3_filter_w);
    filterWithWeights(1, SURE1, SURE1, SURE1_filter_w, 0.f);
    filterWithWeights(1, SURE2, SURE2, SURE2_filter_w, 0.f);
    filterWithWeights(1, SURE3, SURE3, SURE3_filter_w, 0.f);
    delete[] SURE1_vec;
    delete[] SURE2_vec;
    delete[] SURE3_vec;
    delete[] SURE1_filter_w;
    delete[] SURE2_filter_w;
    delete[] SURE3_filter_w;
    // create binary maps
    Eigen::Vector3f* binaryMap1 = new Eigen::Vector3f[num_pixels];
    Eigen::Vector3f* binaryMap2 = new Eigen::Vector3f[num_pixels];
    Eigen::Vector3f* binaryMap3 = new Eigen::Vector3f[num_pixels];
    for (int i = 0; i < num_pixels; i++) {
        binaryMap1[i] = Eigen::Vector3f(0.f, 0.f, 0.f);
        binaryMap2[i] = Eigen::Vector3f(0.f, 0.f, 0.f);
        binaryMap3[i] = Eigen::Vector3f(0.f, 0.f, 0.f);
    }
    for (int i = 0; i < num_pixels; i++) {
        float min = fmin(SURE1[i], fmin(SURE2[i], SURE3[i]));
        if (min == SURE1[i]) {
            if (SURE1_deriv[i] < SURE2_deriv[i]) binaryMap1[i] = Eigen::Vector3f(1.f, 1.f, 1.f);
            else binaryMap2[i] = Eigen::Vector3f(1.f, 1.f, 1.f);
        }
//        else if (min == SURE2[i]) {
//            binaryMap2[i] = Eigen::Vector3f(1.f, 1.f, 1.f);
//        }
//        else binaryMap3[i] = Eigen::Vector3f(1.f, 1.f, 1.f);
        else binaryMap2[i] = Eigen::Vector3f(1.f, 1.f, 1.f);
    }
    delete[] SURE1;
    delete[] SURE2;
    delete[] SURE3;
    delete[] SURE1_deriv;
    delete[] SURE2_deriv;
    delete[] SURE3_deriv;
    // filter binary maps
    float** binaryMap1_w = new float*[num_pixels];
    float** binaryMap2_w = new float*[num_pixels];
    float** binaryMap3_w = new float*[num_pixels];
//    NL_means_filter(5, 1, 1.f, binaryMap1, m_variances, binaryMap1_w);
//    NL_means_filter(5, 1, 1.f, binaryMap2, m_variances, binaryMap2_w);
//    NL_means_filter(5, 1, 1.f, binaryMap3, m_variances, binaryMap3_w);
//    filterWithWeights(1, binaryMap1, binaryMap1, binaryMap1_w, Eigen::Vector3f(0.0, 0.0, 0.0));
//    filterWithWeights(1, binaryMap2, binaryMap2, binaryMap2_w, Eigen::Vector3f(0.0, 0.0, 0.0));
//    filterWithWeights(1, binaryMap3, binaryMap3, binaryMap3_w, Eigen::Vector3f(0.0, 0.0, 0.0));
    for (int i = 0; i < num_pixels; i++) {
//        m_intensityValues[i] = binaryMap1[i](0) * m_candidate_1[i] + binaryMap2[i](0) * m_candidate_2[i] + binaryMap3[i](0) * m_candidate_3[i];
        m_intensityValues[i] = binaryMap1[i](0) * m_candidate_1[i] + binaryMap2[i](0) * m_candidate_2[i];

    }
    delete[] binaryMap1;
    delete[] binaryMap2;
    delete[] binaryMap3;
    delete[] binaryMap1_w;
    delete[] binaryMap2_w;
    delete[] binaryMap3_w;
}

// ============
// HELPER FUNCTIONS
// ============
template <class T>
void Denoiser::splitBuffer(int i, int num_A, int num_B,
                           T** source, T** a_dest, T* a_avg, T** b_dest, T* b_avg, bool abs) {
    a_dest[i] = new T[num_A];
    b_dest[i] = new T[num_B];
    for (int j = 0; j < num_A; j++) {
        a_avg[i] += source[i][j];
        a_dest[i][j] = source[i][j];
    }
    a_avg[i] = a_avg[i] / (float)num_A;
    if (abs) absVal(&a_avg[i]);
    for (int j = 0; j < num_B; j++) {
        b_avg[i] += source[i][num_A + j];
        b_dest[i][j] = source[i][num_A + j];
    }
    b_avg[i] = b_avg[i] / (float)num_B;
    if (abs) absVal(&b_avg[i]);
}

template <class T>
void Denoiser::calculateBufferVariance(int num_pixels, T** samples, int* num_samples, T* average, T* out, T init) {
    // calculate variances for each pixel in buffer B
    for (int i = 0; i < num_pixels; i++) {
        int pix_samples = num_samples[i];
        // get the variance for B
        T variance = init;
        for (int j = 0; j < pix_samples; j++) {
            T curr_var;
            squaredDifference(average[i], samples[i][j], &curr_var);
            variance += curr_var;
        }
        variance = variance/(float)pix_samples/* * (1.f)/((float)pix_samples - 1.f)*/;
        out[i] = variance;
    }
}


template <class T>
void Denoiser::filterWithWeights(int c_r, T* in, T* out, float** weights, T init) {
    int pixels = m_height * m_width;
    T* temp = new T[pixels];
    for (int i = 0; i < pixels; i++) {
        int row, col;
        getCoords(i, m_width, &row, &col);
        float weight_total = 0.f;
        T accumulator = init;
        std::cout << row << ", " << col << std::endl;
        for (int col_N = col - c_r; col_N <= col + c_r; col_N++) {
            for (int row_N = row - c_r; row_N <= row + c_r; row_N++) {
                if (outOfBufferBounds(m_height, m_width, row_N, col_N)) continue;
                int neighbourhood_index = getIndex((2 * c_r) + 1, row_N - (row - c_r), col_N - (col - c_r));
                int image_index = getIndex(m_width, row_N, col_N);
                weight_total += weights[i][neighbourhood_index];
                accumulator += weights[i][neighbourhood_index] * in[image_index];
            }
        }
        temp[i] = accumulator / weight_total;
    }
    for (int i = 0; i < pixels; i++) {
        out[i] = temp[i];
    }
    delete[] temp;
}

void Denoiser::gaussianBlur(float std_dev, Eigen::Vector3f* in, Eigen::Vector3f* out) {
    // make a temporary array
    int num_pixels = m_height * m_width;
    Eigen::Vector3f temp_buf[num_pixels];
    // generate Gaussian kernel
    int radius = std::ceil(std_dev * 3.f); // we don't care about being >3sigma away
    int kernel_size = ((2 * radius) + 1) * ((2 * radius) + 1);
    float kernel[kernel_size];
    float normalizer = 0.f;
    for (int i = 0; i < kernel_size; i++) {
        int row, col;
        getCoords(i, (2 * radius) + 1, &row, &col);
        int x = col - radius;
        int y = row - radius;
        float weight = std::exp(-((x * x) + (y * y))/(2.f * (std_dev * std_dev)));
        weight = weight * (1.f / (2.f * M_PI * (std_dev * std_dev)));
        kernel[i] = weight;
        normalizer += weight;
    }
    // iterate over all pixels and perform filter
    // for the (2f + 1) x (2f + 1) centre patch
    Eigen::Vector3f P[kernel_size];
    bool P_valid[kernel_size];
    for (int i = 0; i < num_pixels; i++) {
        int row, col;
        getCoords(i, m_width, &row, &col);
        for (int col_P = col - radius; col_P <= col + radius; col_P++) { // FOR EACH PIXEL
            for (int row_P = row - radius; row_P <= row + radius; row_P++) { // IN CENTRE PATCH
                int kernel_index = getIndex((2 * radius) + 1, row_P - (row - radius), col_P - (col - radius));
                // if we are out of bounds on the actual image, then we can't include this pixel
                // and we must mark it as invalid, and then continue on
                if (outOfBufferBounds(m_height, m_width, row_P, col_P)) {
                    P_valid[kernel_index] = false;
                    continue;
                }
                int image_index = getIndex(m_width, row_P, col_P);
                P[kernel_index] = in[image_index]; // get the pixel value from the input buffer
                P_valid[kernel_index] = true; // valid
            }
        }
        Eigen::Vector3f filtered_value(0.0, 0.0, 0.0);
        float curr_normalizer = normalizer;
        for (int K = 0; K < kernel_size; K++) {
            if (!P_valid[K]) {
                normalizer -= kernel[K];
                continue;
            }
            filtered_value += P[K] * kernel[K];
        }
        filtered_value = filtered_value / curr_normalizer;
        temp_buf[i] = filtered_value;
    }
    // copy into our output buffer
    for (int i = 0; i < num_pixels; i++) {
        out[i] = temp_buf[i];
    }
}

// perform a sobel filter (edge detection) on a buffer. Improves results for the feature denoising
// significantly around edges.
void Denoiser::sobelFilterSquared(Eigen::Vector3f* in_buf, Eigen::Vector3f* out_buf) {
    // create temporary buffer
    int num_pixels = m_height * m_width;
    Eigen::Vector3f *temp_buf = new Eigen::Vector3f[num_pixels];
    // create Kx, Ky
    float Kx[9] = {1.0, 0.0, -1.0, 2.0, 0.0, -2.0, 1.0, 0.0, -1.0};
    float Ky[9] = {1.0, 2.0, 1.0, 0.0, 0.0, 0.0, -1.0, -2.0, -1.0};
    Eigen::Vector3f P[9]; // our pixels that we are convolving with
    // for each pixel, calculate Gx, Gy and the gradient G.
    for (int i = 0; i < num_pixels; i++) {
        int row, col;
        getCoords(i, m_width, &row, &col);
        Eigen::Vector3f Gx(0.0, 0.0, 0.0);
        Eigen::Vector3f Gy(0.0, 0.0, 0.0);
        for (int row_P = row - 1; row_P <= row + 1; row_P++) {
            for (int col_P = col - 1; col_P <= col + 1; col_P++) {
                // check if we are out of bounds. if we are, we need to get
                // a mirrored pixel so our sobel filter does not create fake edges
                int kernel_index = getIndex(3, row_P - (row - 1), col_P - (col - 1));
                int image_index;
                if (outOfBufferBounds(3, 3, row_P - (row - 1), col_P - (col - 1))) {
                    int new_row, new_col;
                    // if the row is out of bounds, we need to reflect it across the kernel fulcrum
                    if (row_P >= m_height || row_P < 0) {
                        int row_dist = row_P - row;
                        new_row = row - row_dist;
                    } else new_row = row_P;
                    // if the col is out of bounds, then we need to reflect it too
                    if (col_P >= m_width || col_P < 0) {
                        int col_dist = col_P - col;
                        new_col = col - col_dist;
                    } else new_col = col_P;
                    image_index = getIndex(m_width, new_row, new_col);
                } else image_index = getIndex(m_width, row_P, col_P);
                Gx += Kx[kernel_index] * in_buf[image_index];
                Gy += Ky[kernel_index] * in_buf[image_index];
            }
        }
        // square Gx and Gy
        Gx = Gx.cwiseQuotient(Gx);
        Gy = Gy.cwiseQuotient(Gy);
        // calculate G squared, this is what we want.
        Eigen::Vector3f G2 = (Gx + Gy);
//        Eigen::Vector3f G(std::sqrt(G2(0)), std::sqrt(G2(1)), std::sqrt(G2(2)));
//        temp_buf[i] = G;
        temp_buf[i] = G2;
    }
    // copy into our output buffer
    for (int i = 0; i < num_pixels; i++) {
        out_buf[i] = temp_buf[i];
    }
    delete[] temp_buf;
}

float Denoiser::bilateralCalc(Eigen::Vector3f fp, Eigen::Vector3f fq, Eigen::Vector3f varp, Eigen::Vector3f varq, Eigen::Vector3f gradp, float tau, float c_k) {
    Eigen::Vector3f dist;
    squaredDifference(fp, fq, &dist);
    Eigen::Vector3f varPQ;
    varPQ(0) = fmin(varp(0), varq(0));
    varPQ(1) = fmin(varp(1), varq(1));
    varPQ(2) = fmin(varp(2), varq(2));
    Eigen::Vector3f num = dist - (fp + varPQ);
    Eigen::Vector3f maxVarGrad;
    maxVarGrad(0) = fmax(varp(0), gradp(0));
    maxVarGrad(1) = fmax(varp(1), gradp(1));
    maxVarGrad(2) = fmax(varp(2), gradp(2));
    Eigen::Vector3f maxVarGradTau;
    maxVarGradTau(0) = fmax(maxVarGrad(0), tau);
    maxVarGradTau(1) = fmax(maxVarGrad(1), tau);
    maxVarGradTau(2) = fmax(maxVarGrad(2), tau);
    Eigen::Vector3f den = (c_k * c_k) * maxVarGradTau;
    Eigen::Vector3f result = num.cwiseQuotient(den);
    return (1.f/3.f) * (result(0) + result(1) + result(2));
}

float Denoiser::bilateralCalc(float fp, float fq, float varp, float varq, float gradp, float tau, float c_k) {
    float dist;
    squaredDifference(fp, fq, &dist);
    float varPQ = fmin(varp, varq);
    float num = dist - (varp + varPQ);
    float maxVarGradTau = fmax(tau, fmax(varp, gradp));
    float den = (c_k * c_k) * maxVarGradTau;
    return num / den;
}

void Denoiser::minWeights(int c_r, float** wc, float** wf, float** w) {
    int num_pixels = m_width * m_height;
    int neighbour_pixels = ((2 * c_r) + 1) * ((2 * c_r) + 1);
    for (int i = 0; i < num_pixels; i++) {
        w[i] = new float[neighbour_pixels];
        for (int j = 0; j < neighbour_pixels; j++) {
            w[i][j] = fmin(wc[i][j], wf[i][j]);
        }
    }
}

int Denoiser::getIndex(int cols, int row, int col) {
    return cols * row + col;
}

void Denoiser::getCoords(int index, int cols, int* row, int* col) {
    *col = index % cols;
    *row = index / cols;
}

bool Denoiser::outOfBufferBounds(int rows, int cols, int row, int col) {
    if (col < 0 || col >= cols) return true;
    else if (row < 0 || row >= rows) return true;
    return false;
}

void Denoiser::squaredDifference(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f *difference) {
    Eigen::Vector3f d = b - a;
    *difference = d.cwiseProduct(d);
}

void Denoiser::squaredDifference(float a, float b, float *difference) {
    float d = b - a;
    *difference = d * d;
}

void Denoiser::ratioValues(Eigen::Vector3f a, Eigen::Vector3f b, Eigen::Vector3f c, Eigen::Vector3f *ratio) {
    if (a == Eigen::Vector3f(0.0, 0.0, 0.0) && b == Eigen::Vector3f(0.0, 0.0, 0.0)) {
        *ratio = Eigen::Vector3f(0.0, 0.0, 0.0);
    } else {
        *ratio = a.cwiseQuotient(b);
        *ratio = (*ratio).cwiseProduct(c);
    }
}

void Denoiser::ratioValues(float a, float b, float c, float *ratio) {
    if (a == 0.0 && b == 0.0) {
        *ratio = 0.0;
    } else {
        *ratio = a / b;
        *ratio = (*ratio) * c;
    }
}

void Denoiser::absVal(float *a) {
    *a = fabs(*a);
}

void Denoiser::absVal(Eigen::Vector3f *a) {
    (*a)(0) = fabs((*a)(0));
    (*a)(1) = fabs((*a)(1));
    (*a)(2) = fabs((*a)(2));
}

// ============
// IMAGE-CREATING FUNCTIONS
// For saving images/tone mapping nicely
// ============
void Denoiser::toneMap(Eigen::Vector3f* buf, QRgb* pixels) {
    for(int y = 0; y < m_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            Eigen::Vector3f hdr_intensity = buf[offset];
            Eigen::Vector3f ones(1, 1, 1);
            Eigen::Vector3f out_intensity = hdr_intensity.cwiseQuotient(ones + hdr_intensity) * 255;
            pixels[offset] = qRgb(out_intensity.x(), out_intensity.y(), out_intensity.z());
        }
    }
}

void Denoiser::toneMapToBuf(Eigen::Vector3f* ibuf, Eigen::Vector3f* obuf) {
    int num_pixels = m_height * m_width;
    for (int i = 0; i < num_pixels; i++) {
        Eigen::Vector3f ones(1.f, 1.f, 1.f);
        obuf[i] = ibuf[i].cwiseQuotient(ones + ibuf[i]) * 255;
    }
}

void Denoiser::saveImage(Eigen::Vector3f* buf, QString nameMod) {
    QImage image(m_width, m_height, QImage::Format_RGB32);
    QRgb *data = reinterpret_cast<QRgb *>(image.bits());
    toneMap(buf, data);
    bool success = image.save(m_name + "-" + nameMod + ".png");
    if(!success) {
        success = image.save(m_name + "-" + nameMod + ".png", "PNG");
    }
    if(success) {
        std::cout << "Wrote rendered image to " << (m_name + "-" + nameMod + ".png").toStdString() << std::endl;
    } else {
        std::cerr << "Error: failed to write image to " << (m_name + "-" + nameMod + ".png").toStdString() << std::endl;
    }
}

void Denoiser::saveImage(float* buf, QString nameMod) {
    QImage image(m_width, m_height, QImage::Format_RGB32);
    QRgb *data = reinterpret_cast<QRgb *>(image.bits());
    int pixels = m_height * m_width;
    Eigen::Vector3f* tempBuf = new Eigen::Vector3f[pixels];
    for (int i = 0; i < pixels; i++) {
        tempBuf[i] = Eigen::Vector3f(buf[i], buf[i], buf[i]);
    }
    toneMap(tempBuf, data);
    bool success = image.save(m_name + "-" + nameMod + ".png");
    if(!success) {
        success = image.save(m_name + "-" + nameMod + ".png", "PNG");
    }
    if(success) {
        std::cout << "Wrote rendered image to " << (m_name + "-" + nameMod + ".png").toStdString() << std::endl;
    } else {
        std::cerr << "Error: failed to write image to " << (m_name + "-" + nameMod + ".png").toStdString() << std::endl;
    }
    delete[] tempBuf;
}

void Denoiser::saveImageNoToneMap(Eigen::Vector3f* buf, QString nameMod) {
    QImage image(m_width, m_height, QImage::Format_RGB32);
    QRgb *data = reinterpret_cast<QRgb *>(image.bits());
    int num_pixels = m_width * m_height;
    for (int i = 0; i < num_pixels; i++) {
        int x, y;
        getCoords(i, m_width, &x, &y);
        data[i] = qRgb(buf[i].x(), buf[i].y(), buf[i].z());
        std::cout << "Coord: " << x << ", " << y << std::endl;
        std::cout << buf[i].x() << " " << buf[i].y() << " " << buf[i].z() << std::endl;
        std::cout << ((data[i] & 0x00FF0000) >> 16) << " " << ((data[i] & 0x0000FF00) >> 8) << " " << (data[i] & 0x000000FF) << std::endl;
    }
    bool success = image.save(m_name + "-" + nameMod + ".png");
    if(!success) {
        success = image.save(m_name + "-" + nameMod + ".png", "PNG");
    }
    if(success) {
        std::cout << "Wrote rendered image to " << (m_name + "-" + nameMod + ".png").toStdString() << std::endl;
    } else {
        std::cerr << "Error: failed to write image to " << (m_name + "-" + nameMod + ".png").toStdString() << std::endl;
    }
}

// ============
// GRAVEYARD
// Functions that are not to be used but kept here for archival purposes
// ============

// In order to have a nicely denoised variance, we perform an NL-means filter on the two buffer variance.
// Strategy:
//      1) For each pixel, get the centre patch weights (these do not change)
//      2) For each pixel in the r-patch we are averaging over, we get the f-patch
//      3) We use the f-patches to get the weights of each pixel in them. We average this
//      4) We save this averaged weight. This is the NL-means weight for that pixel in the r-patch
//      5) We compute the averaged value for the individual pixel in the new variances buffer.
void Denoiser::filterBufferVariances_RKZ12() {
    // parameters for NL-means filter
    int r = 1;
    int f = 3;
    int c_a = 4;
    float c_k = 0.45;
    int num_pixels = m_width * m_height;
    m_variances = new Eigen::Vector3f[num_pixels];
    std::vector<bool> r_bools;
    std::vector<bool> centrePatchBools;
    std::vector<Eigen::Vector3f> centrePatchSamples;
    std::vector<Eigen::Vector3f> weights;
    std::vector<Eigen::Vector3f> samples;
    centrePatchBools.resize(((2*f) + 1)*((2*f) + 1));
    centrePatchSamples.resize(((2*f) + 1)*((2*f) + 1));
    std::vector<Eigen::Vector3f> patchSamples;
    std::vector<bool> patchBools;
    patchBools.resize(((2*f) + 1)*((2*f) + 1));
    patchSamples.resize(((2*f) + 1)*((2*f) + 1));
    r_bools.resize(((2*r) + 1)*((2*r) + 1));
    weights.resize(((2*r) + 1)*((2*r) + 1));
    samples.resize(((2*r) + 1)*((2*r) + 1));
    for (int i = 0; i < num_pixels; i++) {
        // for each pixel in the region we filter within (2r + 1 x 2r + 1), we need to compute NL-means weights
        int row, col;
        getCoords(i, m_width, &row, &col);
        // get centre patch samples (these don't change)
        for (int j = col - f; j <= col + f; j++) {
            for (int k = row - f; k <= row + f; k++) {
                int patch_index = getIndex((2 * f) + 1, k - (row - f), j - (col - f));
                if (outOfBufferBounds(m_width, m_height, k, r)) {
                    centrePatchBools[patch_index] = false;
                    continue;
                }
                int index = getIndex(m_width, k, r);
                // keep aligned within our patch (out of bounds entries are just ignored)
                centrePatchBools[patch_index] = true;
                Eigen::Vector3f sqD;
                squaredDifference(m_intensityValuesA[index], m_intensityValuesB[index], &sqD);
                centrePatchSamples[patch_index] = 0.5 * sqD;
            }
        }
        for (int j = col - r; j <= col + r; j++) {
            for (int k = row - r; k <= row + r; k++) {
                // if we are out of bounds, skip this
                int r_index = getIndex((2 * r) + 1, k - (row - r), j - (col - r));
                if (outOfBufferBounds(m_width, m_height, k, r)) {
                    r_bools[r_index] = false;
                    continue;
                }
                // if not, we can use patches to compute a good weight for each pixel
                for (int l = j - f; l <= j + f; l++) {
                    for (int m = k - f; m <= k + f; m++) {
                        int patch_index = getIndex((2 * f) + 1, m - (k - f), l - (j - f));
                        if (outOfBufferBounds(m_width, m_height, m, l)) {
                            patchBools[patch_index] = false;
                            continue;
                        }
                        int index = getIndex(m_width, m, l);
                        // keep aligned within our patch (invalid entries are just ignored)
                        patchBools[patch_index] = true;
                        Eigen::Vector3f sqD;
                        squaredDifference(m_intensityValuesA[index], m_intensityValuesB[index], &sqD);
                        patchSamples[patch_index] = 0.5 * sqD;
                    }
                }
                // TODO: Include Buades et al. 2005 extension for the patchwise method
                int normalizer = 0;
                Eigen::Vector3f d2 = Eigen::Vector3f(0.0, 0.0, 0.0);
                // sum over all non-uniform variance distances squared
                for (int l = 0; l < (2 * f) + 1; l++) {
                    for (int m = 0; m < (2 * f) + 1; m++) {
                        int patch_index = getIndex((2 * f) + 1, m, l);
                        // if we have a valid entry in both patches, we can compute the distances
                        if (patchBools[patch_index] && centrePatchBools[patch_index]) {
                            int Pindex = getIndex(m_width, row + (m - f), col + (l - f));
                            int Qindex = getIndex(m_width, k + (m - f), j + (l - f));
                            Eigen::Vector3f distance2;
                            squaredDifference(patchSamples[patch_index], centrePatchSamples[patch_index], &distance2);
                            Eigen::Vector3f VarP;
                            squaredDifference(m_variancesA[Pindex], m_variancesB[Pindex], &VarP);
                            Eigen::Vector3f VarQ;
                            squaredDifference(m_variancesA[Qindex], m_variancesB[Qindex], &VarQ);
                            Eigen::Vector3f VarQP;
                            VarQP(0) = std::fmin(VarP(0), VarQ(0));
                            VarQP(1) = std::fmin(VarP(1), VarQ(1));
                            VarQP(2) = std::fmin(VarP(2), VarQ(2));
                            Eigen::Vector3f epsilon = Eigen::Vector3f(1e-10, 1e-10, 1e-10);
                            Eigen::Vector3f num = distance2 - c_a*(VarP + VarQP);
                            Eigen::Vector3f dom = epsilon + (c_k*c_k)*(VarP+VarQ);
                            d2 += num.cwiseQuotient(dom);
                            normalizer++;
                        }
                    }
                }
                // compute the final weight by taking the negative exponential of each distance squared
                d2 = d2 / (float)normalizer;
                Eigen::Vector3f W;
                W(0) = std::exp(-std::fmax(0.f, d2(0)));
                W(1) = std::exp(-std::fmax(0.f, d2(1)));
                W(2) = std::exp(-std::fmax(0.f, d2(2)));
                // get centre sample of patchSamples as the sample we need to keep
                // store all necessary data
                int sample_index = getIndex((2 * f) + 1, f, f);
                r_bools[r_index] = true;
                weights[r_index] = W;
                samples[r_index] = patchSamples[sample_index];
            }
        }
        // now we can compute the actual variance that we will use for denoising the actual image.
        // sum over all of the weights and windowed vi
        Eigen::Vector3f variance = Eigen::Vector3f(0.0, 0.0, 0.0);
        int normalizer = 0;
        for (int j = 0; j < (2 * r) + 1; j++) {
            for (int k = 0; k < (2 * r) + 1; k++) {
                int r_index = getIndex((2*r) + 1, k, j);
                if (r_bools[r_index]) {
                    variance += samples[r_index].cwiseProduct(weights[r_index]);
                    normalizer++;
                }
            }
        }
        variance = variance / (float)normalizer;
        m_variances[i] = variance;
    }
    saveImage(m_variances, "variance");
}
