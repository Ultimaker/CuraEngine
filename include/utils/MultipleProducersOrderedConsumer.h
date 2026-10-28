// Copyright (c) 2026 UltiMaker
// CuraEngine is released under the terms of the AGPLv3 or higher.

#pragma once

#include <memory>
#include <type_traits>

#include "utils/ThreadPool.h"

namespace curaengine
{

template<typename Producer, typename Consumer>
class MultipleProducersOrderedConsumer
{
    using item_t = std::invoke_result_t<Producer, ptrdiff_t>;
    using lock_t = ThreadPool::lock_t;

public:
    /*!
     * \see run_multiple_producers_ordered_consumer
     * \param max_pending Number of allocated slots for items waiting to be consumed.
     */
    template<typename P, typename C>
    MultipleProducersOrderedConsumer(ptrdiff_t first, ptrdiff_t last, P&& producer, C&& consumer, size_t max_pending)
        : producer_(std::forward<P>(producer))
        , consumer_(std::forward<C>(consumer))
        , max_pending_(max_pending)
        , queue_(std::make_unique<item_t[]>(max_pending))
        , last_idx_(last)
        , write_idx_(first)
        , read_idx_(first)
        , consumer_wait_idx_(first)
    {
    }

    //! Schedules the tasks on thread_pool, then run one on the main thread until completion.
    void run(ThreadPool& thread_pool)
    {
        if (write_idx_ >= last_idx_)
        {
            return;
        }
        workers_count_ = thread_pool.thread_count() + 1;
        // Start thread_pool.thread_count() workers on the thread pool
        auto lock = thread_pool.get_lock();
        for (size_t i = 1; i < workers_count_; i++)
        {
            thread_pool.push(
                lock,
                [this](lock_t& th_lock)
                {
                    worker(th_lock);
                });
        }
        // Run a worker on the main thread
        worker(lock);
        // Wait for completion of all workers
        if (workers_count_ > 0)
        {
            work_done_cond_.wait(lock);
        }
    }

protected:
    //! Waits for free space in the ring. Returns false when work is completed.
    bool wait(lock_t& lock)
    {
        while (true)
        {
            if (write_idx_ >= last_idx_)
            { // Work completed: stop worker
                return false;
            }
            if (write_idx_ - read_idx_ < max_pending_)
            { // Continue as a producer
                return true;
            }
            else
            { // Queue is full, wait for consumer signal
                free_slot_cond_.wait(lock); // Signaled by consume_many() and worker() completion
            }
        }
    }

    //! Produces an item and store in in the ring buffer. Assumes that there is items to produce and free space in the ring
    ptrdiff_t produce(lock_t& lock)
    {
        ptrdiff_t produced_idx = write_idx_++;
        item_t* slot = &queue_[(produced_idx + max_pending_) % max_pending_];
        assert(produced_idx < last_idx_);

        // Unlocks global mutex while producing an item
        lock.unlock();
        item_t item = producer_(produced_idx);
        lock.lock();

        assert(! *slot);
        *slot = std::move(item);
        assert(*slot);

        return produced_idx;
    }

    //! Consumes items, until an empty slot (not yet produced) is found.
    void consume_many(lock_t& lock)
    {
        assert(read_idx_ < write_idx_);
        for (item_t* slot = &queue_[(read_idx_ + max_pending_) % max_pending_]; *slot; slot = &queue_[(read_idx_ + max_pending_) % max_pending_])
        {
            // Unlocks global mutex while consuming an item
            lock.unlock();
            consumer_(std::move(*slot));
            *slot = {};
            lock.lock();

            // Increment read index and signal a waiting worker if there is one
            bool queue_was_full = write_idx_ - read_idx_ >= max_pending_;
            read_idx_++;

            // Notify producers that are waiting for a queue slot
            if (queue_was_full)
            {
                free_slot_cond_.notify_one();
            }
        }
        consumer_wait_idx_ = read_idx_; // The producer filling this slot will resume consumption
    }

    //! Task pushed on the ThreadPool
    void worker(lock_t& lock)
    {
        while (wait(lock)) // While there is work to do
        {
            ptrdiff_t produced_idx = produce(lock);
            if (produced_idx == consumer_wait_idx_)
            { // This thread just produced the item that was waited for by the consumer
                consume_many(lock); // Consume a contiguous block starting at consumer_wait_idx
            }
        }

        // Notify eventual workers waiting for a free slot but never got one during the interval of producing the last items
        free_slot_cond_.notify_all();

        if (--workers_count_ == 0)
        { // Last worker exiting: signal run() about workers completion
            work_done_cond_.notify_one();
        }
    }

    // Tracks worker completion
    size_t workers_count_;
    std::condition_variable work_done_cond_;

    Producer producer_;
    Consumer consumer_;
    const ptrdiff_t max_pending_; // Number of produced items that can wait in the queue
    const std::unique_ptr<item_t[]> queue_; // Ring buffer mapping each intermediary result to a slot
    const ptrdiff_t last_idx_;

    ptrdiff_t write_idx_; // Next slot to produce
    ptrdiff_t read_idx_; // Next slot to consume
    ptrdiff_t consumer_wait_idx_; // First slot that is waited for by the consumer
    std::condition_variable free_slot_cond_; // Condition to wait for available space in the buffer
};

//! \private Template deduction guide: defaults to inlining closures into the class layout
template<typename P, typename C>
MultipleProducersOrderedConsumer(ptrdiff_t, ptrdiff_t, P, C, size_t) -> MultipleProducersOrderedConsumer<P, C>;

} // namespace curaengine
