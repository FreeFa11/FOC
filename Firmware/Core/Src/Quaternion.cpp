#include <Quaternion.h>


// Objects
template class Quaternion<float>;



// Quaternion
template<typename T>
Quaternion<T>::Quaternion()
{
  w = 1; x = 0; y = 0; z = 0;
}
template<typename T>
Quaternion<T>::Quaternion(T w, T x, T y, T z)
{
    this->w = w; this->x = x; this->y = y; this->z = z;
}
template<typename T>
Quaternion<T>::~Quaternion(){}


template<typename T>
void Quaternion<T>::Normalize()
{
  T recp_magnitude = 1 / sqrt(w*w + x*x + y*y + z*z);
  w *= recp_magnitude;
  x *= recp_magnitude;
  y *= recp_magnitude;
  z *= recp_magnitude;
}

template<typename T>
Quaternion<T> Quaternion<T>::operator*(Quaternion<T> Other)
{
  Quaternion<T> result;
  
  // Multiplication
  result.w = w*Other.w - x*Other.x - y*Other.y - z*Other.z;
  result.x = x*Other.w + w*Other.x - z*Other.y + y*Other.z;
  result.y = y*Other.w + z*Other.x + w*Other.y - x*Other.z;
  result.z = z*Other.w - y*Other.x + x*Other.y + w*Other.z;

  return result;
}

template<typename T>
Quaternion<T> & Quaternion<T>::operator*=(Quaternion<T> Other)
{
  // Multiplication
  w = w*Other.w - x*Other.x - y*Other.y - z*Other.z;
  x = x*Other.w + w*Other.x - z*Other.y + y*Other.z;
  y = y*Other.w + z*Other.x + w*Other.y - x*Other.z;
  z = z*Other.w - y*Other.x + x*Other.y + w*Other.z;

  return *this;
}

template<typename T>
Quaternion<T> Quaternion<T>::operator*(T Value)
{
  Quaternion result;

  result.w = this->w * Value;
  result.x = this->x * Value;
  result.y = this->y * Value;
  result.z = this->z * Value;

  return result;
}

template<typename T>
Quaternion<T> & Quaternion<T>::operator*=(T Value)
{
  this->w *= Value;
  this->x *= Value;
  this->y *= Value;
  this->z *= Value;

  return *this;
}

template<typename T>
Quaternion<T> Quaternion<T>::operator+(Quaternion<T> Other)
{
  Other.w = this->w + Other.w;
  Other.x = this->x + Other.x;
  Other.y = this->y + Other.y;
  Other.z = this->z + Other.z;

  return Other;
}

template<typename T>
Quaternion<T> Quaternion<T>::operator-(Quaternion<T> Other)
{
  Other.w = this->w - Other.w;
  Other.x = this->x - Other.x;
  Other.y = this->y - Other.y;
  Other.z = this->z - Other.z;

  return Other;
}

template<typename T>
Quaternion<T> Quaternion<T>::inverse()
{
  Quaternion inv(this->w, -this->x, -this->y, -this->z);
  
  return inv;
}

template<typename T>
void Quaternion<T>::Init(IMUData<T> *S)
{
  // Reference Gravity vector
  T vx = 0, vy = 0, vz = 1;

  // Cross Product and Dot Product
  T cx = vy * S->AZ - vz * S->AY;             
  T cy = vz * S->AX - vx * S->AZ;
  T cz = vx * S->AY - vy * S->AX;
  T dot = vx * S->AX + vy * S->AY + vz * S->AZ;

  // if (dot < -0.9999) // Edge Case
  // {
  //   // 180° rotation
  //   this->w = 0;
  //   this->x = 1;
  //   this->y = 0;
  //   this->z = 0;
  //   return;
  // }

  // Compute Quaternion
  T s = std::sqrt((1 + dot) * 2);
  T invs = 1 / s;
  this->w = s * 0.5;
  this->x = -cx * invs;
  this->y = -cy * invs;
  this->z = -cz * invs;
}

template<typename T>
void Quaternion<T>::UpdateMahony(IMUData<T> *S)
{
  #ifndef HAS_MAGNETOMETER
  
  
  static T WX, WY, WZ;                                                  // Innovation Vector
  static T wxInt=0, wyInt=0, wzInt=0;
  static Quaternion<T> tempQuatA;             
  
  tempQuatA = (this->inverse())*Quaternion<T>(0,0,0,1)*(*this);         // Estimated  Vector

  WX = S->AY * tempQuatA.z - S->AZ * tempQuatA.y;
  WY = S->AZ * tempQuatA.x - S->AX * tempQuatA.z; 
  WZ = S->AX * tempQuatA.y - S->AY * tempQuatA.x;

  wxInt += WX * 0.004;
  wyInt += WY * 0.004;
  wzInt += WZ * 0.004;
  S->GX += MAHONY_KP * WX + MAHONY_KI * wxInt;                                         // PI Correction
  S->GY += MAHONY_KP * WY + MAHONY_KI * wyInt;
  S->GZ += MAHONY_KP * WZ + MAHONY_KI * wzInt;
  
  tempQuatA = ((*this) * Quaternion<T>(0,S->GX,S->GY,S->GZ))*0.5;          // Quat  Derivative
  (*this) = (*this) + tempQuatA * IMU_ANGLE_MODE_PERIOD;                 // Euler Integration
  this->Normalize();
  

  #else


  static T WX, WY, WZ;                                                  // Innovation Vector
  static T wxInt=0, wyInt=0, wzInt=0;
  static Quaternion<T> tempQuatA, tempQuatM;             
  
  tempQuatM = (*this)*Quaternion<T>(0, S->MX, S->MY, S->MZ)*(this->inverse());   // Local to Global
  tempQuatA = tempQuatM;                                                // A is Measured
  tempQuatM.x = sqrt(pow(tempQuatM.x,2) + pow(tempQuatM.y,2));          // M is Estimated
  tempQuatM.y = 0;
  WX = (tempQuatA.y * tempQuatM.z - tempQuatA.z * tempQuatM.y);         // Innovation Global
  WY = (tempQuatA.z * tempQuatM.x - tempQuatA.x * tempQuatM.z);
  WZ = (tempQuatA.x * tempQuatM.y - tempQuatA.y * tempQuatM.x);
  // WZ = sqrt(WX*WX + WY*WY + WZ*WZ)
  tempQuatM = (this->inverse()) * Quaternion<T>(0,WX,WY,WZ) * (*this);  // Magnetic Innovation


  tempQuatA = (this->inverse()) * Quaternion<T>(0, 0, 0, 1) * (*this);  // Estimated Gravity

  WX = (S->AY * tempQuatA.z - S->AZ * tempQuatA.y)*Kacc   +   tempQuatM.x*Kmag; 
  WY = (S->AZ * tempQuatA.x - S->AX * tempQuatA.z)*Kacc   +   tempQuatM.y*Kmag; 
  WZ = (S->AX * tempQuatA.y - S->AY * tempQuatA.x)*Kacc   +   tempQuatM.z*Kmag;
  
  wxInt += WX * 0.004;
  wyInt += WY * 0.004;
  wzInt += WZ * 0.004;
  S->GX += Kp * WX + Ki * wxInt;                                           // PI Correction
  S->GY += Kp * WY + Ki * wyInt;
  S->GZ += Kp * WZ + Ki * wzInt;

  tempQuatA = ((*this) * Quaternion<T>(0, S->GX, S->GY, S->GZ))*0.5;       // Quat  Derivative
  (*this)   = (*this) + tempQuatA * TimerAngleModePeriod;               // Euler Integration
  this->Normalize();


  #endif
}

template<typename T>
void Quaternion<T>::UpdateMadgwick(IMUData<T> *S)
{
  #ifndef HAS_MAGNETOMETER


  static T costX, costY, costZ;
  static Quaternion<T> tempQuatA;

  costX = -S->AX - 2*(w*y - x*z);                                        // q^-1*(0,0,0,1)*q - a
  costY = -S->AY + 2*(w*x + y*z);
  costZ = -S->AZ - 2*(x*x + y*y) + 1;

  tempQuatA.w = -2*y*costX + 2*x*costY;                                 // J^T * (q^-1*(0,0,0,1)*q - a)
  tempQuatA.x =  2*z*costX + 2*w*costY - 4*x*costZ;
  tempQuatA.y = -2*w*costX + 2*z*costY - 4*y*costZ;
  tempQuatA.z =  2*x*costX + 2*y*costY;

  tempQuatA = ((*this)*Quaternion<T>(0,S->GX,S->GY,S->GZ))*0.5 - tempQuatA*MADGWICK_BETA;
  (*this)   = (*this) + tempQuatA * IMU_ANGLE_MODE_PERIOD;
  this->Normalize();


  #else


  static T costAX, costAY, costAZ;
  static T costMX, costMY, costMZ;
  static Quaternion<T> tempQuatA, tempQuatM;
  
  costAX = -S->AX - 2*(w*y - x*z);                                       // q^-1*(0,0,0,1)*q - a
  costAY = -S->AY + 2*(w*x + y*z);
  costAZ = -S->AZ - 2*(x*x + y*y) + 1;
                                                                        // q*(0,x,y,z)*q^-1
  tempQuatM = (*this)*Quaternion<T>(0, S->MX, S->MY, S->MZ)*(this->inverse());
  tempQuatM.x = sqrt(pow(tempQuatM.x,2) + pow(tempQuatM.y,2));
  tempQuatM.y = 0;                                                      // q^-1*(0,x,0,z)*q - m
  costMX = 2*tempQuatM.x*(0.5-y*y-z*z) + 2*tempQuatM.z*(x*z-w*y)     - S->MX; 
  costMY = 2*tempQuatM.x*(x*y-w*z)     + 2*tempQuatM.z*(w*x+y*z)     - S->MY;
  costMZ = 2*tempQuatM.x*(w*y+x*z)     + 2*tempQuatM.z*(0.5-x*x-y*y) - S->MZ;
  
  tempQuatA.w = -2*y*costAX + 2*x*costAY                - 2*tempQuatM.z*y*costMX                   + 2*(tempQuatM.z*x-tempQuatM.x*z)*costMY + 2*tempQuatM.x*y*costMZ;      
  tempQuatA.x =  2*z*costAX + 2*w*costAY - 4*x*costAZ   + 2*tempQuatM.z*z*costMX                   + 2*(tempQuatM.x*y+tempQuatM.z*w)*costMY + 2*(tempQuatM.x*z-2*tempQuatM.z*x)*costMZ;
  tempQuatA.y = -2*w*costAX + 2*z*costAY - 4*y*costAZ   - 2*(2*tempQuatM.x*y+tempQuatM.z*w)*costMX + 2*(tempQuatM.x*x+tempQuatM.z*z)*costMY + 2*(tempQuatM.x*w-2*tempQuatM.z*y)*costMZ;
  tempQuatA.z =  2*x*costAX + 2*y*costAY                + 2*(tempQuatM.z*x-2*tempQuatM.x*z)*costMX + 2*(tempQuatM.z*y-tempQuatM.x*w)*costMY + 2*tempQuatM.x*x*costMZ;
  
  tempQuatA = ((*this)*Quaternion<T>(0, S->GX, S->GY, S->GZ))*0.5 - tempQuatA*Beta;
  (*this)   = (*this) + tempQuatA * TimerAngleModePeriod;
  this->Normalize();  


  #endif
}