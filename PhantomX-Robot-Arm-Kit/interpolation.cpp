#include "interpolation.h"

std::vector<std::valarray<double>> linearInterpolation (std::vector<std::valarray<double>> &x, double timeSlice) {
    if (timeSlice < 0.001) timeSlice = 0.001;
    if (x.size() < 2) return x;

    unsigned int n = x.size () - 1;

    std::vector<double> h(n);
    for (unsigned int i=0; i<n; i++) {
        h[i] = x[i+1][0] - x[i][0];
        if (h[i] < timeSlice) h[i] = timeSlice;
    }

    std::vector<std::valarray<double>> s;
    s.reserve ((int)(x[n][0] / timeSlice + 10));

    double t = x[0][0];
    for (unsigned int i=0; i<n; i++) {
        std::valarray<double> m = (x[i+1] - x[i])/h[i];

        for (; t<=x[i+1][0]; t+=timeSlice) {
            std::valarray<double> p = m*(t - x[i][0]) + x[i];
            p[0] = t;
            s.push_back (p);
        }
    }
    s.push_back (x[n]);

    return s;
}
