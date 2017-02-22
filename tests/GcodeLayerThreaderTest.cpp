//Copyright (c) 2015 Ultimaker B.V.
//CuraEngine is released under the terms of the AGPLv3 or higher.

#include "GcodeLayerThreaderTest.h"

#include <iomanip>
#include <sstream> // ostringstream
#include <list>

#include <../src/utils/intpoint.h>
#include <../src/GcodeLayerThreader.h>



namespace cura
{
    CPPUNIT_TEST_SUITE_REGISTRATION(GcodeLayerThreaderTest);

void GcodeLayerThreaderTest::setUp()
{
    //Do nothing.
}

void GcodeLayerThreaderTest::tearDown()
{
    //Do nothing.
}

void GcodeLayerThreaderTest::test1()
{
    test(1000, 100, 10);
}
void GcodeLayerThreaderTest::test2()
{
    test(100, 1000, 10);
}
void GcodeLayerThreaderTest::test3()
{
    test(1000, 20, 1);
}
void GcodeLayerThreaderTest::test4()
{
    test(10000, 1000, 100);
}



void GcodeLayerThreaderTest::test(int avg_computation_time, int layer_count, int max_task_count)
{
//     srand(1);
    int starting_layer_nr = -10;
    
    std::vector<int> times_consumed;
    std::vector<int> times_produced;
    times_consumed.resize(layer_count, 0);
    times_produced.resize(layer_count, 0);
    
    const std::function<LayerPlan* (int)>& produce_item =
        [&times_produced, &starting_layer_nr, &avg_computation_time](int layer_nr)
        {
            // simulate work:
            std::this_thread::sleep_for(std::chrono::milliseconds(rand() % (avg_computation_time * 2 * 2 / 3)));
            times_produced[layer_nr - starting_layer_nr]++;
            return new LayerPlan(layer_nr);
        };
    
    const std::function<void (LayerPlan*)>& consume_item =
        [&times_consumed, &starting_layer_nr, &avg_computation_time](LayerPlan* item)
        {
            int item_idx = item->layer_nr - starting_layer_nr;
            times_consumed[item_idx]++;
            // simulate work:
            std::this_thread::sleep_for(std::chrono::milliseconds(rand() % (avg_computation_time * 2 * 9 / 30)));
            delete item;
        };
    unsigned int buffer_size = 5;
    std::list<LayerPlan*> buffer;
    const std::function<void (LayerPlan*)>& zip_item =
        [&buffer, consume_item, buffer_size, &avg_computation_time](LayerPlan* plan_in)
        {
            { // perform zipping / buffering comand
                for (LayerPlan* plan : buffer)
                {
                    plan->buffered_items.push_back(plan_in->layer_nr);
                }
                
                // simulate work:
                std::this_thread::sleep_for(std::chrono::milliseconds(rand() % (avg_computation_time * 2 * 1 / 30)));
            }
            buffer.push_back(plan_in);
            if (buffer.size() > buffer_size)
            {
                LayerPlan* layer = buffer.front();
                buffer.pop_front();
                consume_item(layer);
            }
            else
            {
                return -1;
            }
        };
    GcodeLayerThreader<LayerPlan> threader(starting_layer_nr, layer_count + starting_layer_nr, produce_item, zip_item, max_task_count);
    
    
    double time_before = omp_get_wtime();
    
    threader.run();
    
    // flush
    while (!buffer.empty())
    {
        LayerPlan* plan = buffer.front();
        consume_item(plan);
        buffer.pop_front();
    }
    
    double time_diff = omp_get_wtime() - time_before;

    for (unsigned int idx = 0; idx < times_produced.size(); idx++)
    {
        CPPUNIT_ASSERT_MESSAGE(std::string("Items are being produced more than once!"), times_produced[idx] == 1);
        CPPUNIT_ASSERT_MESSAGE(std::string("Items are being consumed more than once!"), times_consumed[idx] == 1);
    }
    std::cerr << "computed in " << time_diff << "s.\n";
}

}