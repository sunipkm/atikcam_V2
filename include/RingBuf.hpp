/**
 * @file RingBuf.hpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief Ring buffer implementation
 * @version 0.1
 * @date 2021-12-09
 *
 * @copyright Copyright (c) 2021
 *
 */

#ifndef _RING_BUF_HPP_
#define _RING_BUF_HPP_

#include <stdexcept>

template <class T>
/**
 * @brief Ring buffer class for any data of numeric type. The ring buffer is not
 * internally MT-safe. The reason behind the design choice being, ring buffer
 * access may be time-critical and thread safety may be specific to the use-case.
 *
 */
class RingBuf
{
public:
    /**
     * @brief Construct a new empty ring buffer.
     *
     */
    RingBuf() : data(nullptr), idx(-1), full(false), size(0)
    {
    }

    /**
     * @brief Construct a new ring buffer.
     *
     * @param size Size of ring buffer.
     */
    RingBuf(int size) : data(nullptr), idx(-1), full(false)
    {
        this->size = size;
        Initialize(size);
    }

    /**
     * @brief Destroy the ring buffer.
     *
     */
    ~RingBuf()
    {
        if (HasData())
            delete[] data;
        data = nullptr;
    }

    /**
     * @brief Initialize an empty ring buffer.
     *
     * @param size Size of ring buffer.
     */
    void Initialize(int size)
    {
        if (size < 1)
        {
            std::invalid_argument("Buffer size can not be negative or zero.");
        }
        this->size = size;
        if (data != nullptr)
            delete[] data;
        data = nullptr;
        data = new T[size];
        for (int i = 0; i < size; i++)
            data[i] = 0;
        pushed = 0;
        idx = -1;
    }

    /**
     * @brief Check if ring buffer has been initialized.
     *
     * @return bool
     */
    bool HasData() const { return size > 0; }

    /**
     * @brief Check if ring buffer is full.
     * Ring buffer is considered full if number of elements
     * pushed in is at least equal to the size of the ring buffer.
     *
     * @return bool
     */
    bool IsFull() const { return full; }

    /**
     * @brief Get the index at data was pushed in last.
     *
     * @return int Current index of ring buffer.
     */
    int GetIndex() const { return idx; }

    /**
     * @brief Get data at index i relative to the current index.
     * i = 0 returns the element at current index.
     * i = 1 returns the element at the previous index etc.
     *
     * @param i Ring buffer index
     * @return T& Reference to the element at index idx - i.
     */
    T &operator[](int i)
    {
        if (!HasData())
        {
            throw std::invalid_argument("Buffer not initialized.");
        }
        if (i < 0)
        {
            throw std::invalid_argument("Index can not be negative.");
        }
        if (i >= size)
        {
            i = i % size;
        }
        int index = (idx - i + size) % size;
        return data[index];
    }

    /**
     * @brief Get data at the absolute index i
     *
     * @param i Absolute array index
     * @return T& Reference to the element at the absolute index i
     */
    T &at(int i)
    {
        if (!HasData())
        {
            throw std::invalid_argument("Buffer not initialized.");
        }
        if (i < 0)
        {
            throw std::invalid_argument("Index can not be negative.");
        }
        if (i >= size)
        {
            throw std::invalid_argument("Index > size, invalid.");
        }
        return data[i];
    }

    /**
     * @brief Push data into the ring buffer.
     * This process iterates the index.
     *
     * @param data Data to be pushed into the buffer.
     */
    void push(const T &data)
    {
        if (!HasData())
        {
            throw std::invalid_argument("Buffer not initialized.");
        }
        if (idx == size - 1)
            full = true;
        idx = (idx + 1) % size;
        (this->data)[idx] = data;
        pushed++;
    }

    /**
     * @brief Clear data in the ring buffer.
     *
     */
    void clear()
    {
        if (!HasData())
        {
            throw std::invalid_argument("Buffer not initialized.");
        }
        idx = -1;
        full = false;
        for (int i = 0; i < size; i++)
            data[i] = 0;
        pushed = 0;
    }

    /**
     * @brief Get the total number of elements pushed into the array
     *
     * @return const int Number of elements pushed into the ring buffer
     */
    const int GetPushed() const { return pushed; }

    /**
     * @brief Get the size of the ring buffer
     *
     * @return const int
     */
    const int GetSize() const { return size; }

    /**
     * @brief Linear regression of data in this ring buffer
     * 
     * @tparam U Output data type class
     * @param m Slope
     * @param c Intercept
     * @param r Correlation
     * @param size Dataset size
     * @return true on success, false on failure
     */
    template <class U>
    bool LinearRegression(U &m, U &c, U &r, int size = -1) const
    {
        T S_x = 0,
          S_x2 = 0,
          S_xy = 0,
          S_y = 0,
          S_y2 = 0;
        RingBuf<T> buf = (*this);

        if (size <= 0) // act on all available data
        {
            size = pushed % this->size;
        }
        else
        {
            size = (size % pushed) % (this->size); // limit size to available data
        }
        if (size < 2)
            return false;
        for (int i = 0; i < size; i++)
        {
            S_xy += i * buf[i];
            S_y += buf[i];
            S_y2 += buf[i] * buf[i];
        }
        S_x = (size * (size - 1)) / 2;                 // Sum of 0 .. (n-1)
        S_x2 = size * (size - 1) * (2 * size - 1) / 6; // sum of 0 .. (n - 1)^2
        T denom = (size * S_x2 - S_x * S_x);
        if (fabs(denom) < 1e-30)
        {
            return false;
        }
        m = (size * S_xy - S_x * S_y) / denom;
        c = (S_y - S_x2 - S_x * S_xy) / denom;
        r = (S_xy - S_x * S_y / size) /
            sqrt((S_x2 - S_x * S_x / size) *
                 (S_y2 - S_y * S_y / size));
        return true;
    }

    /**
     * @brief Linear regression of data in this ring buffer
     * 
     * @tparam U Output and X axis buffer data type class
     * @param xaxis X Axis data ring buffer
     * @param m Slope
     * @param c Intercept
     * @param r Correlation
     * @param size Dataset size
     * @return true on success, false on failure
     */
    template <class U>
    bool LinearRegression(RingBuf<U> &xaxis, U &m, U &c, U &r, int size = -1) const
    {
        T S_x = 0,
          S_x2 = 0,
          S_xy = 0,
          S_y = 0,
          S_y2 = 0;
        RingBuf<T> buf = (*this);

        int pushed = buf.GetPushed() > xaxis.GetPushed() ? xaxis.GetPushed() : buf.GetPushed();
        int _size = buf.GetSize() > xaxis.GetSize() ? xaxis.GetSize() : buf.GetSize();

        if (size <= 0) // act on all available data
        {
            size = pushed % _size;
        }
        else
        {
            size = (size % pushed) % (_size); // limit size to available data
        }
        if (size < 2)
            return false;
        for (int i = 0; i < size; i++)
        {
            S_x += xaxis[i];
            S_x2 += xaxis[i] * xaxis[i];
            S_xy += xaxis[i] * buf[i];
            S_y += buf[i];
            S_y2 += buf[i] * buf[i];
        }
        T denom = (size * S_x2 - S_x * S_x);
        if (fabs(denom) < 1e-30)
        {
            return false;
        }
        m = (size * S_xy - S_x * S_y) / denom;
        c = (S_y - S_x2 - S_x * S_xy) / denom;
        r = (S_xy - S_x * S_y / size) /
            sqrt((S_x2 - S_x * S_x / size) *
                 (S_y2 - S_y * S_y / size));
        return true;
    }

private:
    T *data;
    int idx;
    bool full;
    int size;
    int pushed;
};

#endif // _RING_BUF_HPP_