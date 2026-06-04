#pragma once


// Includes
#include "math.h"
#include "MPU6500.h"



// Setup
// #define HAS_MAGNETOMETER
// #define HAS_BAROMETER

// Filter Tuning
#define MAHONY_KP                       .5               
#define MAHONY_KI                       .4              
#define MAHONY_KACC                     1               
#define MAHONY_KMAG                     2               
#define MADGWICK_BETA                   .8              

// Control Frequency
#define IMU_ANGLE_MODE_FREQ             250
#define IMU_RATE_MODE_FREQ              500
#define IMU_ANGLE_MODE_PERIOD           float(1.0f / IMU_ANGLE_MODE_FREQ)
#define IMU_RATE_MODE_PERIOD            float(1.0f / IMU_RATE_MODE_FREQ)
#define IMU_ANGLE_MODE_COUNT            IMU_RATE_MODE_FREQ/IMU_ANGLE_MODE_FREQ                 






template<typename T>
class Quaternion
{
public:
    T w=1, x=0, y=0, z=0;
    void Normalize();

public:
    Quaternion();
    Quaternion(T w, T x, T y, T z);
    ~Quaternion();
    
    // Operations
    Quaternion<T> operator*(Quaternion<T> Other);
    Quaternion<T> & operator*=(Quaternion<T> Other);        // Return by reference and not pointer
    Quaternion<T> operator*(T Value);
    Quaternion<T> & operator*=(T Value);                    // Return by reference and not pointer
    Quaternion<T> operator+(Quaternion<T> Other);
    Quaternion<T> operator-(Quaternion<T> Other);
    Quaternion<T> inverse();

    // Filters
    void Init(IMUData<T> *S);
    void UpdateMahony(IMUData<T> *S);
    void UpdateMadgwick(IMUData<T> *S);
};