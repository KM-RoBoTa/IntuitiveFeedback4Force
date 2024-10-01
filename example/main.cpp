#include "force_sensors.hpp"
#include <unistd.h>     // Sleep function
#include <sstream> 
#include <fstream> 

using namespace std;

int main()
{
    ForceSensors forceSensors("/dev/ttyACM0");
    sleep(1);

    vector<ForceSensorStruct> forces(NBR_SENSORS);

    forceSensors.calibrate();

    cout << "Starting sensor reading" << endl;
    int ctr = 0;
    const int max = 3;
    while(ctr < max) {
        float freq = forceSensors.getForces(forces);

        cout << "Sensor freq: " << freq << " Hz" << endl;
        for (int i=0; i<forces.size(); i++) {
            cout << "Sensor " << i+1 << endl;
            cout << "\tFx = " << forces[i].Fx << " N" << endl;
            cout << "\tFy = " << forces[i].Fy << " N" << endl;
            cout << "\tFz = " << forces[i].Fz << " N" << endl;
        }
        cout << endl;

        ctr++;
        usleep(1000);

    }

    return 1;
}
