#ifndef GCODE_LAYER_THREADER_H
#define GCODE_LAYER_THREADER_H

#include <queue> // priority_queue
#include <functional> // function
#include <thread> // sleep
#include <chrono> // milliseconds

#include "utils/logoutput.h"
#include "utils/optional.h"
#include "utils/Lock.h"


/*!
 * Producer Consumer construct for when:
 * - production can occur in parallel
 * - consumption must be ordered and not multithreaded
 * 
 * A layer_nr index is passed to the item producer in order to produce the different items.
 * 
 * Each thread does production and consumption, giving priority to consumption if some is available.
 * 
 * If there is only one thread, it consumes every time it has produced one item.
 * 
 * \warning This class is only adequate when the expected production time of an item is more than (n_threads - 1) times as much as the expected consumption time of an item
 */
template <typename T>
class GcodeLayerThreader
{
public:
    /*!
     * \param start_item_argument_index The first value with which to produce an item
     * \param end_item_argument_index The last value with which to produce an item
     * \param produce_item The function with which to produce an item
     * \param consume_item The function with which to consume an item
     * \param max_task_count The maximum number of items (being) produced without having been consumed
     */
    GcodeLayerThreader(
        int start_item_argument_index,
        int end_item_argument_index,
        const std::function<T* (int)>& produce_item,
        const std::function<void (T*)>& consume_item,
        const unsigned int max_task_count
    )
    : start_item_argument_index(start_item_argument_index)
    , end_item_argument_index(end_item_argument_index)
    , item_count(end_item_argument_index - start_item_argument_index)
    , max_task_count(max_task_count)
    , produce_item(produce_item)
    , consume_item(consume_item)
    , last_produced_argument_index(start_item_argument_index - 1)
    {
        produced.resize(item_count, nullptr);
    }

    /*!
     * Produce all items and consume them.
     */
    void run()
    {
        #pragma omp parallel
        {
            #pragma omp master
            #pragma omp critical
#ifdef ENABLE_OPENMP
            log("Multithreading GcodeLayerThreader with %i threads.\n", omp_get_num_threads());
#endif // ENABLE_OPENMP
            while (true)
            {
                if (finished())
                {
                    break;
                }
                act();
            }
        }
    }
private:
    /*!
     * Produce an item and put it in \ref GcodeLayerThreader::produced
     * 
     * \param item_argument_index The parameter with which to call \ref GcodeLayerThreader::produce_item
     */
    void produce(int item_argument_index)
    {
        T* produced_item = produce_item(item_argument_index);
        int item_idx = item_argument_index - start_item_argument_index;
        #pragma omp critical
        {
            produced[item_idx] = produced_item;
            if (!to_be_consumed_item_idx && item_idx == last_consumed_idx + 1) // TODO: did lijkt dubbel op
            {
                to_be_consumed_item_idx = item_idx;
            }
        }
    }

    /*!
     * Consume an item from \ref GcodeLayerThreader::produced
     * 
     * \param item_idx The index into \ref GcodeLayerThreader::produced
     */
    void consume(int item_idx)
    {
        consume_item(produced[item_idx]);
        produced[item_idx] = nullptr;
        #pragma omp critical
        {
            assert(item_idx == last_consumed_idx + 1);
            last_consumed_idx = item_idx;
            if (produced[last_consumed_idx + 1])
            {
                to_be_consumed_item_idx = last_consumed_idx + 1;
            }
            active_task_count--;
        }
    }

    /*!
     * Consume if possible, otherwise
     * Produce if possible, otherwise
     * wait half a second
     */
    void act()
    {
        {
            int item_idx = -1;
            #pragma omp critical
            {
                if (to_be_consumed_item_idx && consume_lock.test_lock())
                {
                    item_idx = *to_be_consumed_item_idx;
                    to_be_consumed_item_idx = nullptr;
                    
                }
            }
            if (item_idx >= 0)
            {
                consume(item_idx);
                consume_lock.unlock();
                return;
            }
        }
        
        {
            std::optional<int> item_argument_index;
            #pragma omp critical
            {
                if (active_task_count < max_task_count)
                {
                    item_argument_index = ++last_produced_argument_index;
                    active_task_count++;
                }
            }
            if (item_argument_index && *item_argument_index < end_item_argument_index)
            {
                produce(*item_argument_index);
                return;
            }
        }
        // thread is blocked by too many items being processed
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    /*!
     * Check whether no tasks are left for a thread to pick up
     */
    bool finished()
    {
        bool finished;
        #pragma omp critical
        {
            finished = last_produced_argument_index >= end_item_argument_index - 1
                && !to_be_consumed_item_idx;
        }
        return finished;
    }

private:
    // algorithm paramters
    const int start_item_argument_index; //!< The first index with which \ref GcodeLayerThreader::produce_item will be called
    const int end_item_argument_index; //!< The end index with which \ref GcodeLayerThreader::produce_item will not be called any more
    const unsigned int item_count; //!< The number of items to produce and consume

    const unsigned int max_task_count; //!< The maximum amount of items active in the system

    const std::function<T* (int)>& produce_item; //!< The function to produce an item
    const std::function<void (T*)>& consume_item; //!< The function to consume an item

    // variables which change throughout the computation of the algorithm
    std::vector<T*> produced; //!< ordered list for every item to be produced; contains pointers to produced items which aren't consumed yet; rest is nullptr
    int last_produced_argument_index; //!< Counter to see which item next to produce

    std::optional<int> to_be_consumed_item_idx; //!< The index into \ref GcodeLayerThreader::produced where to find the next item ready to be consumed (if any)
    Lock consume_lock; //!< Lock to make sure no two threads consume at the same time
    int last_consumed_idx = -1; //!< The index into \ref GcodeLayerThreader::produced for the last item consumed

    // statistics
    int active_task_count = 0; //!< Number of items active in this system.

};

#endif // GCODE_LAYER_THREADER_H