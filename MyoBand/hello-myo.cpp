#define _USE_MATH_DEFINES
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include "SerialPort.h"
#include <myo/myo.hpp>
#include <chrono>


std::string DEBUG;
char output[MAX_DATA_LENGTH];
char incomming[MAX_DATA_LENGTH];
//set port used by arduino if 7 then keep as below if another number replace 7.
char port[] = "\\\\.\\COM7";

using namespace std;

//datacollector class handels all comunication with myoband.
class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
    : onArm(false), isUnlocked(false), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
    {
    }

   //checks if myoband has been unpaired and resets all values if so.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
       
        roll_w = 0;
        pitch_w = 0;
        yaw_w = 0;
        onArm = false;
        isUnlocked = false;
    }
    //interpits accelerometer data
    void onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3< float >& accel) 
    {
        x = accel.x();
        y = accel.y();
        z = accel.z();

    }
    //interprits gyroscope data
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;

        
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                        1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));

        
        roll_w = static_cast<int>(180+(roll*((float)180/((float)M_PI))));
        pitch_w = static_cast<int>(180 + (pitch * ((float)180 / ((float)M_PI))));
        yaw_w = static_cast<int>(180 + (yaw * ((float)180 / ((float)M_PI))));

        
       

    }

    //interprits gestural events.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;

        if (pose != myo::Pose::unknown && pose != myo::Pose::rest) {
            
            myo->unlock(myo::Myo::unlockHold);

            
            myo->notifyUserAction();
        } else {
            
            myo->unlock(myo::Myo::unlockTimed);
        }
    }

    
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                   myo::WarmupState warmupState)
    {
        onArm = true;
        whichArm = arm;
    }

    
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
    }

    //checks if the armband is unlocked
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = true;
    }

    //checks if armband is locked
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = false;
    }

    
    
    //prints gyro data, accellerometer data, arm the armband is worn on and wheter it is unlocked or locked
    void print()
    {
       std::cout << '\r';
        

        std::cout <<   '[' << roll_w<< ']'
            << '[' << pitch_w << ']'
            << '[' << yaw_w<< ']'
            << '[' << x << ']'
            << '[' << y << ']'
            << '[' << z << ']';

        if (onArm) {

            std::string poseString = currentPose.toString();

            std::cout << '[' << (isUnlocked ? "unlocked" : "locked  ") << ']'
                << '[' << (whichArm == myo::armLeft ? "L" : "R") << ']'
                << '[' << poseString << std::string(14 - poseString.size(), ' ') << ']';

            
          
            
        }
        else {

            std::cout << '[' << std::string(8, ' ') << ']' << "[?]" << '[' << std::string(14, ' ') << ']';
        }

        std::cout << std::flush;
    }


    //changes gestural events from strings to integers for easy transfer between applcation and arduino
    void posenr()
    {
        string poseString = currentPose.toString();

        if (poseString == "rest")
        {
            pose = 1;
        }
        if (poseString == "fist")
        {
            pose = 2;
        }
        if (poseString == "fingersSpread")
        {
            pose = 3;
        }
        if (poseString == "waveOut")
        {
            pose = 4;
        }
        if (poseString == "waveIn")
        {
            pose = 5;
        }
        if (poseString == "doubleTap")
        {
            pose = 6;
        }
        if (poseString == "unknown")
        {
            pose = 1;
        }
        

        


    }

    
   
    bool onArm;
    myo::Arm whichArm;

   
    bool isUnlocked;

    
    
    float x, y, z;
    int roll_w, pitch_w, yaw_w, pose, w;
    myo::Pose currentPose;
};

int main()
{


    DataCollector collector;

    SerialPort arduino(port);
    try {

   
    //checks if an arduino is connected.
    if (arduino.isConnected())
    {
        std::cout << "connection is established \n";
    }
    else
    {
        std::cout << "Error in port name";
    }
    
    //sets a name for the hub, the name can be anything.
    myo::Hub hub("com.362.myo-controls");
    
    //simple print message informing the user of what is happening in the code
    std::cout << "Attempting to find a Myo..." << std::endl;

    //waits for myoband connection for 10 seconds
    myo::Myo* myo = hub.waitForMyo(10000);

    //if no connection is formed the application sends a throw message and shuts down the application
    if (!myo) {
        throw std::runtime_error("Unable to find a Myo!");
    }

    //informs user that a myoband has been found
    std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

    
    
    DataCollector collector;
    hub.addListener(&collector);
    
    

   
    while (arduino.isConnected()) 
    {
        std::string DATA;
        //the hub checks for myoband events 20 times a second
        hub.run(1000/20);
        
        

        //can be outcommented if you want to see the myoband data
        //collector.print();
        collector.posenr();
      
        //the roll pitch and yaw from the myoband are given new names.
        int a = collector.roll_w;

        int b = collector.pitch_w;

        int c = collector.yaw_w;
        
        int d = collector.pose;

        
        //a string is formed using roll pitch and yaw.
        string A = to_string(a);

        string B = to_string(b);

        string C = to_string(c);

        string D = to_string(d);

       
        //0's are added to make the values into 3 digit numbers making the easier to pull out of an array.
        if (a < 100)
        {
            A = "0" + A;
            if (a < 10)
            {
                A = "0" + A;
            }
        }

        if (b < 100)
        {
            B = "0" + B;
            if (b < 10)
            {
                B = "0" + B;
            }
        }
        
        if (c < 100)
        {
            C = "0" + C;
            if (c < 10)
            {
                C = "0" + C;
            }
        }

        
        
        //the strings are combind and a 1 is added at the end to signefi to the arduino that a connection is established
        DATA = A + B + C + D + "1";
        
        //a char array is made using DATA.
        char* charArray = new char[DATA.size() + 1];
        copy(DATA.begin(), DATA.end(), charArray);
        charArray[DATA.size()] = '\n';


        //the char array is writen onto the serial port for the arduino to pick up the size of the message is set to the lenght,
        //of the array to avoid sending any more information than nessecary.
        arduino.writeSerialPort(charArray, DATA.size() + 1);
        //the serial port is read and any information found is saved as output.
        arduino.readSerialPort(output, MAX_DATA_LENGTH);
        
        
        //output is printed onto the consol
        cout << output;
        cout << '\r';


        delete[] charArray;
       
         
        

    }
    //the throw that was sent earlier is caught and the code is shut down
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
    return 0;
}
