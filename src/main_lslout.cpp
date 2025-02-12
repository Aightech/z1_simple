#include <stdint.h>
#include <iostream>
#include <lsl_cpp.h>
#include <unistd.h>

int main()
{
    //create lsl outlet named z1_cmd
    lsl::stream_info info("z1_cmd", "Control commands for Z1", 6, 100, lsl::cf_float32, "myuniqueid23443");
    lsl::stream_outlet outlet(info);

    std::cout << "Now sending data to \"z1_cmd\"..." << std::endl;

    while (1)
    {
        //send a sample
        std::vector<float> sample(6);
        sample[0] = 0;
        sample[1] = 0.0;
        sample[2] = 0.0;
        sample[3] = 0.5;
        sample[4] = 0;
        sample[5] = 0;
        outlet.push_sample(sample);
        //sleep for 1 second
        usleep(10000);
    }
    
    return 0;
}