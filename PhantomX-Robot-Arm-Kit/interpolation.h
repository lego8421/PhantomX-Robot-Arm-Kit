#ifndef INTERPOLATION_H
#define INTERPOLATION_H

#include <vector>
#include <valarray>

#include <QDebug>

std::vector<std::valarray<double>> linearInterpolation (std::vector<std::valarray<double>> &x, double timeSlice);

#endif // INTERPOLATION_H
