#include "mission.hpp"


int main(){
    setenv("DISPLAY",":0", 1);

    std::thread mission1_thread(mission1);
    std::thread mission2_thread(mission2);

    mission1_thread.join();
    mission2_thread.join();

    return 0;
}