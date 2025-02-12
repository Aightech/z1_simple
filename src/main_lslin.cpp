#include <iostream>
#include <stdint.h>
#include <lsl_cpp.h>
#include <unistd.h>

int main()
{
    //find lsl inlet named z1_cmd
    std::cout << "Looking for a stream named \"z1_cmd\"..." << std::endl;
    std::vector<lsl::stream_info> results = lsl::resolve_stream("name", "z1_cmd", 1, 5);
    std::cout << "Found " << results.size() << " streams." << std::endl;
    if (results.empty())
    {
        std::cout << "No streams found with the name \"z1_cmd\"." << std::endl;
        return 0;
    }
    lsl::stream_inlet inlet(results[0]);

    std::cout << "Now receiving data from \"z1_cmd\"..." << std::endl;

    while (1)
    {
        std::vector<float> sample(6);
        double timestamp = inlet.pull_sample(sample);
        std::cout << "Received sample: ";
        for (int i = 0; i < 6; i++)
            std::cout << sample[i] << " ";
        std::cout << "at time " << timestamp << std::endl;
        //sleep for 1 second
        usleep(10000);
    }
    
    return 0;
}