/**
 ****************************************************************************
 * @file        force_sensors.cpp
 * @brief       Handle force sensors reading in a thread
 ****************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  katarina.lichardova@km-robota.com, 10/2023
 * @authors  kamilo.melo@km-robota.com, 10/2023
 ****************************************************************************
 */

#include <iostream>
#include <fcntl.h>      // Control over the open file referenced by file descriptor 
#include <string> 
#include <termios.h>    // POSIX terminal control fefinitions   
#include <unistd.h>     // Sleep function

#include "force_sensors.hpp"

#define BUFFER_SIZE		256
#define BAUDRATE		230400
#define TARE_SAMPLES	100 // Number of samples taken for the tare

using namespace std;


/**
 * @brief       Create a Sensors object
 * @param[in]	sensors_portname Port handling the sensors, in the form of /dev/ttyACMx
 */
ForceSensors::ForceSensors(const char* sensors_portname)
{
	m_forces = vector<ForceSensorStruct>(NBR_SENSORS);
	m_tareOffsets = vector<ForceSensorStruct>(NBR_SENSORS);

    cout << "Creating the force sensors thread..." << endl;
    m_thread = thread(&ForceSensors::forceSensorsLoop, this, sensors_portname);
}

/**
 * @brief	Class destructor. Takes care of safely stopping the thread
 */
ForceSensors::~ForceSensors()
{
	// Set the loop-stopping variable
	{
		scoped_lock lock(m_mutex);
 		m_stopThread = true;
	}	
   
    cout << "Safely stopping the force sensors' thread...." << endl;

	// Block the main thread until the sensors' thread finishes
    if (m_thread.joinable())
        m_thread.join();
}


/**
 * @brief       Read and save the current sensors values. Called in the thread
 * @param[in]   sensors_portname Port handling the sensors
 */
void ForceSensors::forceSensorsLoop(const char* sensors_portname)
{
	// Open port
	int fd = openPort(sensors_portname);

    char read_buffer[BUFFER_SIZE];	// Buffer to store the data received              
	int  bytes_read = 0;     		// Number of bytes read by the read() system call 

	// -----  Start of the main loop -----
	cout << "Starting sensor reading" << endl;
	bool stopThread = 0;
    while(!stopThread) {

		tcflush(fd, TCIOFLUSH);   // Discards old data in the rx and tx buffers 
		bytes_read = read(fd, &read_buffer, BUFFER_SIZE);   // Read the data

		// Print read values
 		/*printf("\n\n");    
		cout << "Bytes read: " << bytes_read << endl;    
		for(int i=0; i<bytes_read; i++)	 // printing only the received characters
			cout << read_buffer[i];*/

		if (bytes_read > 0)
			interpretData(read_buffer, bytes_read);

		// Thread sleep for scheduling
		std::this_thread::sleep_for(chrono::microseconds(50));

		// Check if internal stop received (= destructor called)
		{
			scoped_lock lock(m_mutex);
			stopThread = m_stopThread;	
		}	
    }

	// Close the serial port 
	// Important! The output buffer also needs to be flushed, else the port takes 30s (!!) to close
	tcflush(fd, TCIOFLUSH);  
    close(fd); 
    cout << "Force sensors' serial port closed successfully" << endl; 
}

/**
 * @brief       Open and setup the port handling the sensors
 * @param[in]   sensors_portname Port handling the sensors
 * @return		File descriptor of the port
 */
int ForceSensors::openPort(const char* sensors_portname)
{
    // O_RDWR   - Read/Write access to serial port 
    // O_NOCTTY - No terminal will control the process 
    // Open in blocking mode, read will wait
	int fd = open(sensors_portname, O_RDWR | O_NOCTTY);

	if (fd < 0) {
		cout << "Error in opening the sensors' port! Exiting" << endl;
		exit(0);
	}
	else
		cout << "Force sensors port open successfully" << endl;

	// ----- Setting the attributes of the serial port using termios structure -----

	struct termios SerialPortSettings;	                      
	tcgetattr(fd, &SerialPortSettings);	// Get the current attributes of the serial port 

	// Setting the Baud rate 
	cfsetispeed(&SerialPortSettings, BAUDRATE ); // Set read  speed                       
	cfsetospeed(&SerialPortSettings, BAUDRATE ); // Set write speed                       

	/* 8N1 Mode */
	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity   
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit 
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size             
	SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8                                 

	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control                  
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines

	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p
	SerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non-canonical mode                           

	SerialPortSettings.c_oflag &= ~OPOST;   // No output processing

	// Setting timeout parameters for the non-canonical mode
	SerialPortSettings.c_cc[VMIN] = 40;  // Read at least 40 characters 
	SerialPortSettings.c_cc[VTIME] = 10; // Intercharacter timeout in deciseconds

    // Set the new attributes to the termios structure
	if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) {
	    cout << "ERROR in setting serial port attributes!" << endl;
        sleep(1);
    }
	
	return fd;
}


/**
 * @brief       Interpret the data in the received buffer and internally save them
 * @param[in]	read_buffer Input buffer containing raw data from the sensors
 * @param[in]   bytes_read Number of read bytes from the port 
 */
void ForceSensors::interpretData(char* read_buffer, int bytes_read)
{
	string value_c;
	vector<float> values;

	// First, transform the array of chars (the buffer) into real values
	int offset = 0, nbr_digits = 0;
	for (int i=0; i<bytes_read; i++) {
		char c = read_buffer[i];

		if (c == 'd' || c == 't' || c == '=' ) {
			nbr_digits = 0;
			offset++;
		}
		else if (c != ',' && c != 'H') // The "H" from "Hz" is also a delimiter
			nbr_digits++;
		else {
			value_c.clear();

			for (int j=0; j<nbr_digits; j++) 
				value_c.push_back(read_buffer[offset + j]);

			// Convert the string to a float	
			float value = atof(value_c.c_str());
			values.push_back(value);

			// Update the offset
			offset = i+1;
			nbr_digits = 0;
		}
	}
 
	// Store those values into a comprehensive structure
	scoped_lock lock(m_mutex);

	for (int i=0; i<values.size()/NBR_SENSORS+1; i++) {
		ForceSensorStruct sensor;
		sensor.Fx = values[i*3 + 0];
		sensor.Fy = values[i*3 + 1];
		sensor.Fz = values[i*3 + 2];

		m_forces[i] = sensor;
	}

	m_freq = values.back();
}


/**
 * @brief       Get the sensors values from the main thread (interface function)
 * @param[out]	forces Vector that will hold the output values
 * @return		Sensor frequency refresh rate
 */
float ForceSensors::getForces(vector<ForceSensorStruct>& forces)
{
	// Grab raw values
	{
		scoped_lock lock(m_mutex);

		for (int i=0; i<NBR_SENSORS; i++) {
			forces[i].Fx = m_forces[i].Fx;
			forces[i].Fy = m_forces[i].Fy;
			forces[i].Fz = m_forces[i].Fz;
		}
	}

	// Apply the calibration
	for (int i=0; i<NBR_SENSORS; i++) {
		forces[i].Fx -= m_tareOffsets[i].Fx;
		forces[i].Fy -= m_tareOffsets[i].Fy;
		forces[i].Fz -= m_tareOffsets[i].Fz;
	}

	return m_freq;
}



/**
 * @brief	Calibrate the sensors
 * @note	Make sure the sensors are not moving during the calibration!
 */
void ForceSensors::calibrate()
{
	cout << "Force sensors tare in progress..." << endl;

	vector<ForceSensorStruct> forces(NBR_SENSORS);

	for (int i=0; i<TARE_SAMPLES; i++) {
		{
			scoped_lock lock(m_mutex);
			for (int j=0; j<NBR_SENSORS; j++)
				forces[i] = m_forces[i];
		}

		for (int j=0; j<NBR_SENSORS; j++) {
			// Update values
			m_tareOffsets[j].Fx += forces[j].Fx; 
			m_tareOffsets[j].Fy += forces[j].Fy; 
			m_tareOffsets[j].Fz += forces[j].Fz;
		}

		usleep(30000);
	}

	for (int j=0; j<NBR_SENSORS; j++) {
		m_tareOffsets[j].Fx = m_tareOffsets[j].Fx/(float)TARE_SAMPLES;
		m_tareOffsets[j].Fy = m_tareOffsets[j].Fy/(float)TARE_SAMPLES;
		m_tareOffsets[j].Fz = m_tareOffsets[j].Fz/(float)TARE_SAMPLES;

		cout << "\nTare offsets for force sensor: " << j+1 << endl;
		cout << "Fx: " << m_tareOffsets[j].Fx << " N" << endl;
		cout << "Fy: " << m_tareOffsets[j].Fy << " N" << endl;
		cout << "Fz: " << m_tareOffsets[j].Fz << " N" << endl;
	}
	cout << "Force sensors calibrated!" << endl;
}