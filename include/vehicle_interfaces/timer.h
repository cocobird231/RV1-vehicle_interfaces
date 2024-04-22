#pragma once

#include <pthread.h>
#include <sched.h>

#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>

#include <deque>
#include <memory>

#include "vehicle_interfaces/utils.h"

namespace vehicle_interfaces
{

class TimerPool;



/**
 * @brief LiteTimer. Pass a callback function and period to the constructor to create a timer, and call start() to start the timer.
 * @note This class support TiemrPool, the timer can be added to TimerPool by addTimer() and controlled by TimerPool.
 * @note If the timer is added to TimerPool, it can't be controlled by its own functions.
 */
class LiteTimer
{
private:
    std::chrono::high_resolution_clock::duration period_;

    std::condition_variable cv_;
    std::mutex mtx_;
    unique_thread tmTh_;
    std::atomic<bool> setTmThPriF_;
    unique_thread tickTh_;
    std::function<void()> cbFunc_;

    std::atomic<bool> enableF_;
    std::atomic<bool> exitF_;

    friend class TimerPool;
    std::atomic<bool> pooledF_;// Set by TimerPool.
    std::atomic<bool> noPoolF_;// Set by LiteTimer.

private:
    /**
     * @brief Timer thread function.
     * @param[in] st Start time of the timer.
     * @note The timer thread will notify the callback thread when the timer is ticked.
     * @note The timer thread will be stopped when the timer is stopped or destroyed.
     */
    void _tmTh(std::chrono::high_resolution_clock::time_point st) noexcept
    {
        std::unique_lock<std::mutex> lock(this->mtx_, std::defer_lock);
        while (!this->exitF_ && this->enableF_)
        {
            auto ts = st + this->period_;
            st = ts;
            while (std::chrono::high_resolution_clock::now() < ts && !this->exitF_ && this->enableF_)
                std::this_thread::yield();
            // std::this_thread::sleep_until(ts);
            lock.lock();
            this->cv_.notify_all();
            lock.unlock();
        }
    }

    /**
     * @brief Callback thread function.
     * @note The callback thread will call the callback function when the timer is ticked.
     * @note The callback thread will be stopped when the timer is destroyed.
     */
    void _tickTh() noexcept
    {
        std::unique_lock<std::mutex> lock(this->mtx_, std::defer_lock);
        while (!this->exitF_)
        {
            lock.lock();
            this->cv_.wait(lock);
            lock.unlock();
            if (!this->exitF_)
                this->cbFunc_();
        }
    }

    /**
     * @brief Notify the callback thread.
     * @note This funciton will only be called by TimerPool.
     */
    void _notify() noexcept
    {
        std::unique_lock<std::mutex> lock(this->mtx_);
        this->cv_.notify_all();
    }

public:
    /**
     * @brief LiteTimer constructor.
     * @param[in] period_ms Period of the timer in milliseconds.
     * @param[in] cbFunc Callback function.
     * @param[in] MAX_SCHED_RR Set the timer thread priority to SCHED_RR.
     * @warning The `MAX_SCHED_RR` option needs sudo permission and may cause the system to be unresponsive.
     */
    LiteTimer(double period_ms, const std::function<void()>& cbFunc, bool MAX_SCHED_RR = false) : 
        period_(std::chrono::high_resolution_clock::duration(std::chrono::nanoseconds(static_cast<uint64_t>(period_ms * 1000000)))), 
        setTmThPriF_(MAX_SCHED_RR), 
        enableF_(false), 
        exitF_(false), 
        pooledF_(false), 
        noPoolF_(false)
    {
        this->cbFunc_ = cbFunc;
        this->tickTh_ = make_unique_thread(&LiteTimer::_tickTh, this);
    }

    ~LiteTimer()
    {
        this->destroy();
    }

    /**
     * @brief Start the timer.
     * @note The timer will not start if it is already started.
     * @note The timer will not start if it is added to TimerPool.
     */
    void start()
    {
        if (this->pooledF_)
            return;
        this->noPoolF_ = true;// Can't be added to TimerPool.
        this->enableF_ = true;
        if (!this->tmTh_)
        {
            this->tmTh_ = make_unique_thread(&LiteTimer::_tmTh, this, std::chrono::high_resolution_clock::now());
            if (this->setTmThPriF_)
            {
                sched_param sch;
                int policy;
                pthread_getschedparam(this->tmTh_->native_handle(), &policy, &sch);
                sch.sched_priority = sched_get_priority_max(SCHED_RR);
                if (pthread_setschedparam(this->tmTh_->native_handle(), SCHED_RR, &sch))
                    std::cerr << "[LiteTimer] Failed to setschedparam\n";
            }
        }
    }

    /**
     * @brief Stop the timer.
     * @note The timer will not stop if it is added to TimerPool.
     */
    void stop()
    {
        if (this->pooledF_)
            return;
        this->noPoolF_ = false;// Can be added to TimerPool.
        this->enableF_ = false;
        this->tmTh_.reset();// Destruct the thread.
    }

    /**
     * @brief Set the period of the timer.
     * @param[in] period_ms Period of the timer in milliseconds.
     * @note The timer will not be changed if it is added to TimerPool.
     */
    void setPeriod(double period_ms)
    {
        if (this->pooledF_)
            return;
        bool preState = this->enableF_;
        this->stop();
        this->period_ = std::chrono::high_resolution_clock::duration(std::chrono::nanoseconds(static_cast<uint64_t>(period_ms * 1000000)));
        if (preState)
            this->start();
    }

    /**
     * @brief Destroy the timer.
     * @note The timer will not be destroyed if it is added to TimerPool.
     */
    void destroy()
    {
        if (this->pooledF_)
            return;
        this->exitF_ = true;
        this->stop();
        {
            std::unique_lock<std::mutex> lock(this->mtx_);
            this->cv_.notify_all();
        }
        if (this->tickTh_ && this->tickTh_->joinable())
            this->tickTh_->join();
    }
};



/**
 * @brief Timer pool. Pass a timer by pointer to addTimer() to add a timer to the pool, and call start() to start the pool.
 * @note The timer cannot be controlled by its own functions after being added to the pool.
 * @note The pooled timer will not be destroyed when the pool is released or destructed, it should be destroyed by the user.
 */
class TimerPool
{
private:
    std::deque<std::shared_ptr<LiteTimer> > tms_;// Pooled timers.
    std::deque<std::chrono::high_resolution_clock::time_point> tmSts_;// Record next start time of each pooled timer.

    unique_thread pollingTh_;
    std::atomic<bool> startF_;
    std::atomic<bool> exitF_;
    std::mutex mtx_;// For public functions.

private:
    /**
     * @brief Polling thread function.
     * @param[in] st Start time of the pool.
     * @note The polling thread will check each pooled timer and call its callback function when the timer is triggered.
     * @note The polling thread will be stopped when the pool is stopped or released.
     */
    void _pollingTh(std::chrono::high_resolution_clock::time_point st) noexcept
    {
        for (const auto tm : this->tms_)
            this->tmSts_.push_back(st + tm->period_);// Record next start time of each pooled timer.

        while (!this->exitF_ && this->startF_)
        {
            for (size_t i = 0; i < this->tms_.size(); i++)
            {
                if (std::chrono::high_resolution_clock::now() >= this->tmSts_[i])
                {
                    this->tms_[i]->_notify();
                    this->tmSts_[i] = this->tmSts_[i] + this->tms_[i]->period_;// Update next start time.
                }
            }
            std::this_thread::yield();
        }
    }

public:
    /**
     * @brief Timer pool constructor.
     */
    TimerPool() : startF_(false), exitF_(false) {}

    ~TimerPool() { this->release(); }

    /**
     * @brief Add a timer to the pool.
     * @param[in] tm Timer to be added.
     * @note The timer will be added to the pool only if it is not started and not pooled.
     * @note If the timer is started, call stop() before adding it to the pool.
     */
    void addTimer(std::shared_ptr<LiteTimer> tm)
    {
        std::lock_guard<std::mutex> lock(this->mtx_);
        if (this->startF_ || tm->noPoolF_ || tm->pooledF_)
            return;
        tm->pooledF_ = true;
        this->tms_.push_back(tm);
    }

    /**
     * @brief Start the pool.
     * @note The pool will start polling the pooled timers.
     * @note The pool will not start if it is already started or no timer is added.
     */
    void start()
    {
        std::lock_guard<std::mutex> lock(this->mtx_);
        if (this->startF_ || this->tms_.empty())
            return;
        this->startF_ = true;
        this->pollingTh_ = make_unique_thread(&TimerPool::_pollingTh, this, std::chrono::high_resolution_clock::now());
    }

    /**
     * @brief Stop the pool.
     * @note The pool will stop polling the pooled timers.
     */
    void stop()
    {
        std::lock_guard<std::mutex> lock(this->mtx_);
        this->startF_ = false;
        if (this->pollingTh_ && this->pollingTh_->joinable())
        {
            this->pollingTh_->join();
        }
    }

    /**
     * @brief Release the pool.
     * @note The pool will stop polling the pooled timers and clear the timer pointers.
     * @note The timers will not be destroyed when the pool is released, they should be destroyed by the user.
     */
    void release()
    {
        this->exitF_ = true;
        this->stop();
        std::lock_guard<std::mutex> lock(this->mtx_);
        for (auto tm : this->tms_)
            tm->pooledF_ = false;
        this->tms_.clear();
    }
};



class Timer
{
private:
    std::chrono::high_resolution_clock::duration interval_;
    std::chrono::high_resolution_clock::time_point st_;

    std::atomic<bool> activateF_;
    std::atomic<bool> exitF_;
    std::atomic<bool> funcCallableF_;
    std::function<void()> func_;

    std::thread timerTH_;
    std::thread callbackTH_;

private:
    void _timer_fixedRate()
    {
        while (!this->exitF_)
        {
            try
            {
                if (!this->exitF_ && this->activateF_)
                {
                    auto tickTimePoint = this->st_ + this->interval_;
                    this->st_ = tickTimePoint;

                    while (!this->exitF_ && this->activateF_ && (std::chrono::high_resolution_clock::now() < tickTimePoint))
                        std::this_thread::yield();
                    if (!this->exitF_ && this->activateF_ && this->funcCallableF_)
                    {
                        if (this->callbackTH_.joinable())
                            this->callbackTH_.join();
                        this->callbackTH_ = std::thread(&Timer::_tick, this);
                    }
                }
                std::this_thread::yield();
            }
            catch (const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        if (this->callbackTH_.joinable())
            this->callbackTH_.join();
    }

    void _tick()
    {
        this->funcCallableF_ = false;
        this->func_();
        this->funcCallableF_ = true;
    }

public:
    Timer(double period_ms, const std::function<void()>& callback, bool MAX_SCHED_RR = false) : activateF_(false), exitF_(false), funcCallableF_(true)
    {
        this->interval_ = std::chrono::high_resolution_clock::duration(std::chrono::nanoseconds(static_cast<uint64_t>(period_ms * 1000000)));
        this->func_ = callback;
        this->st_ = std::chrono::high_resolution_clock::now();
        this->timerTH_ = std::thread(&Timer::_timer_fixedRate, this);

        if (MAX_SCHED_RR)// Increase thread priority
        {
            sched_param sch;
            int policy;
            pthread_getschedparam(this->timerTH_.native_handle(), &policy, &sch);
            sch.sched_priority = sched_get_priority_max(SCHED_RR);
            if (pthread_setschedparam(this->timerTH_.native_handle(), SCHED_RR, &sch))
                std::cerr << "[Timer] Failed to setschedparam:\n";
        }
    }

    ~Timer()
    {
        this->destroy();
    }

    void start()
    {
        this->st_ = std::chrono::high_resolution_clock::now();
        this->activateF_ = true;
    }

    void stop() { this->activateF_ = false; }

    void setPeriod(double period_ms)
    {
        bool preState = this->activateF_;
        this->stop();
        this->interval_ = std::chrono::high_resolution_clock::duration(std::chrono::nanoseconds(static_cast<uint64_t>(period_ms * 1000000)));
        if (preState)
            this->start();
    }

    void destroy()
    {
        this->activateF_ = false;
        this->exitF_ = true;
        if (this->timerTH_.joinable())
            this->timerTH_.join();
    }
};

std::shared_ptr<LiteTimer> make_shared_timer(double period_ms, const std::function<void()>& func, bool MAX_SCHED_RR = false)
{
    return std::make_shared<LiteTimer>(period_ms, func, MAX_SCHED_RR);
}

std::unique_ptr<LiteTimer> make_unique_timer(double period_ms, const std::function<void()>& func, bool MAX_SCHED_RR = false)
{
    return std::make_unique<LiteTimer>(period_ms, func, MAX_SCHED_RR);
}

}