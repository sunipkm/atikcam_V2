
/**
 * @file BesselFilter.hpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief Bessel filter library, applies Bessel filter on data in a Ring Buffer
 * @version 0.1
 * @date 2021-12-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _BESSEL_FILTER_HPP_
#define _BESSEL_FILTER_HPP_

#include <stdexcept>

#include "RingBuf.hpp"

template <class T>
class BesselFilter
{
private:
    T *bessel_coeff;
    int size;

    T factorial(int i)
    {
        T result = 1;
        do
        {
            result *= i;
        } while (--i > 0);
        return result;
    }

public:
    BesselFilter() : bessel_coeff(nullptr)
    {
    }
    BesselFilter(int size, int order = 3, float freq_cutoff = 5)
    {
        if (order < 1)
            throw std::invalid_argument("Bessel filter order can not be < 1.");
        if (order > 10)
            throw std::invalid_argument("Bessel filter order can not be > 10.");
        T *coeff = new T[order + 1];
        if (size < 2)
            throw std::invalid_argument("Bessel filter size can not be < 2.");
        bessel_coeff = new T[size];
        this->size = size;
        // evaluate coefficients for order
        for (int i = 0; i < order + 1; i++)
        {
            coeff[i] = 0;
            coeff[i] = factorial(2 * order - i) / (((int)(1 << (order - i))) * factorial(i) * factorial(order - i)); // https://en.wikipedia.org/wiki/Bessel_filter
        }
        // evaluate transfer function coefficients
        for (int j = 0; j < size; j++)
        {
            bessel_coeff[j] = 0; // initiate value to 0
            double pow_num = 1;  // (j/w_0)^0 is the start
            for (int i = 0; i < order + 1; i++)
            {
                bessel_coeff[j] += coeff[i] * pow_num; // add the coeff
                pow_num *= j / freq_cutoff;            // multiply by (j/w_0) to avoid power function call
            }
            bessel_coeff[j] = coeff[0] / bessel_coeff[j]; // H(s) = T_n(0)/T_n(s/w_0)
        }
        delete[] coeff;
    }
    template <class U>
    U ApplyFilter(RingBuf<U> &buf)
    {
        if (!buf.HasData())
        {
            throw std::invalid_argument("Ring buffer not initialized");
        }
        U val = 0;
        U coeff_sum = 0;
        int max_lim = buf.GetSize() > size ? size : buf.GetSize();
        for (int i = 0;;)
        {
            val += bessel_coeff[i] * buf[i];
            coeff_sum += bessel_coeff[i];
            i++;
            if (bessel_coeff[i] < 0.001 || i >= size || i > buf.GetSize())
                break;
        }
        return val / coeff_sum;
    }
};

#endif // _BESSEL_FILTER_HPP_