/**
 * @file ProtQueue.hpp
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief Protected Queue Implementation for thread-safe operations.
 * @version 0.1
 * @date 2021-12-08
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef _PROT_QUEUE_HPP_
#define _PROT_QUEUE_HPP_

#include <mutex>
#include <queue>

/**
 * @brief Wrapper around std::queue (FIFO) to ensure thread safety. Standard std::queue functions apply.
 * 
 * @tparam T Template type
 */
template <class T>
class ProtQueue
{
public:
    /**
     * @brief Construct a new empty protected queue.
     * 
     */
    ProtQueue()
    {
    }
    /**
     * @brief Construct a new protected queue from another protected queue.
     * 
     * @param other 
     */
    ProtQueue(const ProtQueue &other)
    {
        q_ = other.q_;
    }
    /**
     * @brief Construct a new protected queue from reference of another protected queue.
     * 
     * @param other 
     */
    ProtQueue(ProtQueue &&other)
    {
        q_ = other->q_;
    }
    /**
     * @brief Construct a new protected queue from a standard queue.
     * 
     * @param other 
     */
    ProtQueue(std::queue<T> &other)
    {
        q_ = other;
    }
    /**
     * @brief Construct a new protected queue from a reference to a standard queue.
     * 
     * @param other 
     */
    ProtQueue(std::queue<T> &&other)
    {
        q_ = other;
    }
    /**
     * @brief Check if the queue is empty.
     * 
     * @return bool
     */
    bool empty()
    {
        std::lock_guard<std::mutex> lock(cs_);
        return q_.empty();
    }
    /**
     * @brief Return current size of the queue
     * 
     * @return size_t 
     */
    size_t size()
    {
        std::lock_guard<std::mutex> lock(cs_);
        return q_.size();
    }
    /**
     * @brief Return a reference to the object at the front of the queue (FIFO)
     * 
     * @return const T& 
     */
    const T &front() const
    {
        std::lock_guard<std::mutex> lock(cs_);
        return q_.front();
    }
    /**
     * @brief Return a reference to the object at the rear of the queue (FIFO)
     * 
     * @return const T& 
     */
    const T &back() const
    {
        std::lock_guard<std::mutex> lock(cs_);
        return q_.back();
    }
    /**
     * @brief Push a new object into the queue
     * 
     * @param val 
     */
    void push(const T &val)
    {
        std::lock_guard<std::mutex> lock(cs_);
        q_.push(val);
    }
    /**
     * @brief Push a new object into the queue using a reference
     * 
     * @param val 
     */
    void push(T &&val)
    {
        std::lock_guard<std::mutex> lock(cs_);
        q_.push(val);
    }
    /**
     * @brief Create a new object using the provided arguments and push it
     * into the queue.
     * 
     */
    template <typename... _Args>
#if __cplusplus > 201402L
    decltype(auto) void emplace(_Args &&...__args)
#else
    void emplace(_Args &&...__args)
#endif
    {
        std::lock_guard<std::mutex> lock(cs_);
        q_.emplace(std::forward<_Args>(__args)...);
    }
    /**
     * @brief Remove the element at the front of the queue
     * 
     */
    void pop()
    {
        std::lock_guard<std::mutex> lock(cs_);
        q_.pop();
    }
    /**
     * @brief Swap the contents of two protected queues
     * 
     * @param x ProtQueue to swap with
     */
    void swap(ProtQueue &x) noexcept
    {
        std::unique_lock<std::mutex> l1(cs_, std::defer_lock);
        std::unique_lock<std::mutex> l2(x.cs_, std::defer_lock);
        std::lock(l1, l2);
        this->swap(x);
    }
    /**
     * @brief Acquire a relockable lock on the protected queue for extended operations.
     * 
     * @return std::unique_lock<std::mutex>& Lock on the queue.
     */
    std::unique_lock<std::mutex> &GetLock()
    {
        std::unique_lock<std::mutex> lock(cs_);
        return lock;
    }

private:
    std::queue<T> q_;
    std::mutex cs_;
};
#endif // _PROT_QUEUE_HPP_