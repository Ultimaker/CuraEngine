//Copyright (c) 2021 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#ifndef THREADPOOL_H
#define THREADPOOL_H

#include "../Application.h" // accessing singleton's Application::thread_pool
#include "../utils/math.h" // round_up_divide

#include <cassert>
#include <condition_variable>
#include <deque>
#include <functional> // std::function<>
#include <mutex>
#include <thread>
#include <vector>

namespace cura
{

/*!
 * \brief Very minimal and low level thread pool.
 *
 * Consider using `parallel_for()` instead, interfacing directly with this class should be reserved to concurrency primitives.
 * ThreadPool can be described as a synchronized FIFO queue shared by a fleet of `std::thread`s.
 * Tasks have the responsibility of unlocking the queue's lock passed as an argument while they do asynchronous work.
 */
class ThreadPool
{
  public:
    using lock_t = std::unique_lock<std::mutex>;
    using task_t = std::function<void(lock_t&)>;

    //! Spawns a thread pool with `nthreads` threads
    ThreadPool(size_t nthreads);

    ~ThreadPool() { join(); }

    //! Returns the number of threads
    size_t thread_count() const { return threads.size(); }

    //! Gets a lock on the queue, stopping the queuing or execution of new tasks while held
    lock_t get_lock() { return lock_t(mutex); }

    /*!
     * \brief Pushes a new task while the queue is locked.
     * \param func Closure that unlocks the local worker's lock passed as an argument while
     * doing asynchronous work.
     */
    template<typename F>
    void push(const lock_t& lock [[maybe_unused]], F&& func)
    {
        assert(lock);
        tasks.push_back(std::forward<F>(func));
        condition.notify_one();
    }
    /*!
     * \brief Executes pending tasks while the predicates returns true
     * This method doesn't wait unless predicate does (like implementation of ThreadPool::worker())
     */
    template<typename P> void work_while(lock_t& lock, P predicate)
    {
        assert(lock);
        while(predicate() && !tasks.empty()) // Order is important: predicate() might wait on an empty queue
        {
            assert(!tasks.empty());
            task_t task = std::move(tasks.front());
            tasks.pop_front();

            task(lock);
            assert(lock);
        }
    }

  private:
    void worker();

    void join();

    std::mutex mutex;
    std::condition_variable condition;
    std::deque<task_t> tasks;
    std::vector<std::thread> threads;
    bool wait_for_new_tasks;
};


/// `std::make_signed_t` fails for non integral types in a way that doesn't allows SFINAE fallbacks. This alias solves that.
template<typename T> using make_signed_if_integral_t = typename std::enable_if_t<std::is_integral_v<T>, std::make_signed<T>>::type;

/// Overloads `std::distance()` to work on integral types
template<typename Int, typename Signed=make_signed_if_integral_t<Int>>
inline Signed distance(const Int& first, const Int& last)
{
    return static_cast<Signed>(last) - static_cast<Signed>(first);
}


/*! An implementation of parallel for.
 * There are still a lot of compilers that claim to be fully C++17 compatible, but don't implement the Parallel Execution TS of the accompanying standard library.
 * This means that we mostly have to fall back to the things that C++11/14 provide when it comes to threading/parallelism/etc.
 *
 * The range of items is divided in chunks such that there is a maximum number of `chunks_per_worker` and such that
 * chunk size is a multiple of `chunk_size_factor`.
 *
 * \param from, to: The [inclusive, exclusive) range of iteration. Integers or random access iterators
 * \param body The loop-body, as a closure. Receives the index on invocation.
 * \param chunk_size_factor Chunk size will be a multiple of this number.
 * \param chunks_per_worker Maximum number of tasks that are queue at once (defaults to 4 times the number of workers).
 */
template<typename T, typename F>
void parallel_for(T first, T last, F&& loop_body, size_t chunk_size_factor=1, const size_t chunks_per_worker=8)
{
    using lock_t = ThreadPool::lock_t;

    // Computes the number of items (early out if needed)
    const auto dist = distance(first, last);
    if (dist <= 0)
    {
        return;
    }
    const size_t nitems = dist;

    ThreadPool* const thread_pool = Application::getInstance().thread_pool;
    assert(thread_pool);
    const size_t nworkers = thread_pool->thread_count() + 1; // One task per std::thread + 1 for main thread

    size_t blocks; // Number of indivisible units of work (sized by chunk_size_factor)
    if (chunk_size_factor <= 1)
    {
        chunk_size_factor = 1;
        blocks = nitems;
    }
    else
    {   // User wants to divide the work in blocks of chunk_size_factor items
        blocks = round_up_divide(nitems, chunk_size_factor);
    }

    // With the maximum number of chunks, computes the chunk size, and then the number of chunks
    const size_t max_chunks = std::min(chunks_per_worker * nworkers, blocks);
    const size_t chunk_size = chunk_size_factor * round_up_divide(blocks, max_chunks);
    const size_t chunks = round_up_divide(nitems, chunk_size);
    const auto chunk_increment = static_cast<decltype(dist)>(chunk_size);
    assert(chunks * chunk_size >= nitems && (chunks - 1) * chunk_size < nitems);
    assert(chunks <= chunks_per_worker * nworkers && chunks <= blocks);

    // Packs state variables such that they can be referenced by the task closure through a single reference
    struct
    {
        std::decay_t<F> loop_body; // User's closure data
        size_t chunks_remaining;
        std::condition_variable work_done = {};
    } shared_state = { std::forward<F>(loop_body), chunks };

    // Schedules a task per chunk on the thread pool
    lock_t lock = thread_pool->get_lock();
    T chunk_last;
    for (T chunk_first = first ; chunk_first < last ; chunk_first = chunk_last)
    {
        if (distance(chunk_first, last) > chunk_increment)
        {   // Full size chunk
            chunk_last = chunk_first + chunk_increment;
        }
        else
        {   // Adjust for the size of the last chunk
            chunk_last = last;
        }

        thread_pool->push(lock, [&shared_state, chunk_first, chunk_last](lock_t& th_lock)
            {
                th_lock.unlock(); // Enter unsynchronized region
                for (T i = chunk_first ; i < chunk_last ; ++i)
                {
                    shared_state.loop_body(i);
                }
                th_lock.lock();
                if (--shared_state.chunks_remaining == 0)
                {
                    shared_state.work_done.notify_one();
                }
            });
    }

    // Do work while parallel_for's tasks are running
    thread_pool->work_while(lock, [&]{ return shared_state.chunks_remaining > 0; });
    while(shared_state.chunks_remaining > 0) // Wait until all the task are completed
    {
        shared_state.work_done.wait(lock);
    }
}

/*!
 *  \brief An implementation of parallel for.
 *  Overload for iterating over containers with random access iterators.
 */
template<typename Container, typename F>
auto parallel_for(Container& container, F&& loop_body, size_t chunk_size_factor=1, size_t chunks_per_worker=8)
  -> std::void_t<decltype(container.end() - container.begin())>
{
    parallel_for(container.begin(), container.end(), std::forward<F>(loop_body), chunk_size_factor, chunks_per_worker);
}


//! \private Internal state for run_multiple_producers_ordered_consumer()
template<typename Producer, typename Consumer> class MultipleProducersOrderedConsumer;

/*!
 * \brief Runs parallel producers and buffers the results to be consumed serially in indices order.
 *
 * Producers run while there is available space in the buffer.
 * Only 0 or 1 consumer runs at any time.
 *
 * When a thread produces the item waited for consumption, it turns itself into a consumer.
 * When the consumer thread completes an item, it notifies an eventual producer waiting on a full buffer.
 * When the consumer thread encounter an item not yet produced, it becomes a producer again.
 *
 * Produced items are stored into a shared ring buffer.
 * The item type must be nullable in order to differentiate free (not yet produced) slots.
 *
 * \param fist,last The numerical range of elements to produce.
 * \param producer Given an index/iterator, produces a non-null value of a nullable type (eg pointer, optional, etc...).
 * \param consumer Consumes an item produced by `producer`.
 * \param max_pending_per_worker Number of allocated slots per worker for items waiting to be consumed.
 */
template<typename P, typename C>
void run_multiple_producers_ordered_consumer(ptrdiff_t first, ptrdiff_t last, P&& producer, C&& consumer, size_t max_pending_per_worker=8)
{
    ThreadPool* thread_pool = Application::getInstance().thread_pool;
    assert(thread_pool);
    assert(max_pending_per_worker > 0);
    const size_t max_pending = max_pending_per_worker * (thread_pool->thread_count() + 1);
    MultipleProducersOrderedConsumer<P, C>(first, last, std::forward<P>(producer), std::forward<C>(consumer), max_pending).run(*thread_pool);
}

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
      : producer(std::forward<P>(producer)), consumer(std::forward<C>(consumer)),
        max_pending(max_pending),
        queue(std::make_unique<item_t[]>(max_pending)),
        last_idx(last), write_idx(first), read_idx(first), consumer_wait_idx(first)
    {}

    //! Schedules the tasks on thread_pool, then run one on the main thread until completion.
    void run(ThreadPool& thread_pool)
    {
        if (write_idx >= last_idx)
        {
            return;
        }
        workers_count = thread_pool.thread_count() + 1;
        // Start thread_pool.thread_count() workers on the thread pool
        auto lock = thread_pool.get_lock();
        for (size_t i = 1 ; i < workers_count ; i++)
        {
            thread_pool.push(lock, [this](lock_t& th_lock){ worker(th_lock); });
        }
        // Run a worker on the main thread
        worker(lock);
        // Wait for completion of all workers
        if (workers_count > 0)
        {
            work_done_cond.wait(lock);
        }
    }

  protected:
    //! Waits for free space in the ring. Returns false when work is completed.
    bool wait(lock_t& lock)
    {
        while(true)
        {
            if (write_idx >= last_idx)
            {   // Work completed: stop worker
                return false;
            }
            if (write_idx - read_idx < max_pending)
            {   // Continue as a producer
                return true;
            }
            else
            {   // Queue is full, wait for consumer signal
                free_slot_cond.wait(lock); // Signaled by consume_many() and worker() completion
            }
        }
    }

    //! Produces an item and store in in the ring buffer. Assumes that there is items to produce and free space in the ring
    ptrdiff_t produce(lock_t& lock)
    {
        ptrdiff_t produced_idx = write_idx++;
        item_t* slot = &queue[(produced_idx + max_pending) % max_pending];
        assert(produced_idx < last_idx);

        // Unlocks global mutex while producing an item
        lock.unlock();
        item_t item = producer(produced_idx);
        lock.lock();

        assert(!*slot);
        *slot = std::move(item);
        assert(*slot);

        return produced_idx;
    }

    //! Consumes items, until an empty slot (not yet produced) is found.
    void consume_many(lock_t& lock)
    {
        assert(read_idx < write_idx);
        for (item_t* slot = &queue[(read_idx + max_pending) % max_pending]; *slot ; slot = &queue[(read_idx + max_pending) % max_pending])
        {
            // Unlocks global mutex while consuming an item
            lock.unlock();
            consumer(std::move(*slot));
            *slot = {};
            lock.lock();

            // Increment read index and signal a waiting worker if there is one
            bool queue_was_full = write_idx - read_idx >= max_pending;
            read_idx++;

            // Notify producers that are waiting for a queue slot
            if (queue_was_full)
            {
                free_slot_cond.notify_one();
            }
        }
        consumer_wait_idx = read_idx; // The producer filling this slot will resume consumption
    }

    //! Task pushed on the ThreadPool
    void worker(lock_t& lock)
    {
        while(wait(lock)) // While there is work to do
        {
            ptrdiff_t produced_idx = produce(lock);
            if (produced_idx == consumer_wait_idx)
            {   // This thread just produced the item that was waited for by the consumer
                consume_many(lock); // Consume a contiguous block starting at consumer_wait_idx
            }
        }

        // Notify eventual workers waiting for a free slot but never got one during the interval of producing the last items
        free_slot_cond.notify_all();

        if (--workers_count == 0)
        {   // Last worker exiting: signal run() about workers completion
            work_done_cond.notify_one();
        }
    }

    // Tracks worker completion
    size_t workers_count;
    std::condition_variable work_done_cond;

    Producer producer;
    Consumer consumer;
    const ptrdiff_t max_pending; // Number of produced items that can wait in the queue
    const std::unique_ptr<item_t[]> queue; // Ring buffer mapping each intermediary result to a slot
    const ptrdiff_t last_idx;

    ptrdiff_t write_idx; // Next slot to produce
    ptrdiff_t read_idx; // Next slot to consume
    ptrdiff_t consumer_wait_idx; // First slot that is waited for by the consumer
    std::condition_variable free_slot_cond; // Condition to wait for available space in the buffer
};

//! \private Template deduction guide: defaults to inlining closures into the class layout
template<typename P, typename C>
MultipleProducersOrderedConsumer(ptrdiff_t, ptrdiff_t, P , C, size_t) -> MultipleProducersOrderedConsumer<P, C>;

} //Cura namespace.
#endif // THREADPOOL_H
