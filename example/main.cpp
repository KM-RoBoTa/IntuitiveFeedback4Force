#include "force_sensors.hpp"
#include <unistd.h>     // Sleep function
#include <sstream> 
#include <fstream> 

using namespace std;

int main()
{
    ForceSensors forceSensors("/dev/ttyACM0");
    sleep(1);

    vector<ForceSensorStruct> forces(4);

    cout << "Starting sensor reading" << endl;

    int ctr = 0;
    while(ctr < 1000) {
        forceSensors.getForces(forces);

        for (int i=0; i<forces.size(); i++) {
            cout << "Sensor " << i+1 << endl;
            cout << "\tFx = " << forces[i].x << " N" << endl;
            cout << "\tFy = " << forces[i].y << " N" << endl;
            cout << "\tFz = " << forces[i].z << " N" << endl;
        }
        cout << endl;

        ctr++;
        usleep(1000);

    }

    return 1;
}
