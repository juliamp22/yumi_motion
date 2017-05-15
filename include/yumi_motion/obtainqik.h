/* In the .h file is class yumi_dialbox implementation.  In the constructor we've have redefined the names of each yumi joints and the time duration */
#include <ctime>
#include <iostream>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <ros/ros.h>


// MT LIBRARY HEADERS
#include <mt/exception.h>
#include <mt/unit3.h>
#include <mt/matrix3x3.h>
#include <mt/plane3.h>
#include <mt/quaternion.h>


class obtainq{

	private:

	public:
		const Scalar half_yaw;
		const Scalar half_pitch;
  		const Scalar half_roll;
		

inline void obtainq::setYpr(const Scalar& yaw,
                             const Scalar& pitch,
                             const Scalar& roll)
{

  // Normalizes yaw, pitch and roll angles to the interval [0, 2pi)
  const Scalar yaw_n  (mt::normalize(yaw,   Scalar(0.0), TWO_PI));
  const Scalar pitch_n(mt::normalize(pitch, Scalar(0.0), TWO_PI));
  const Scalar roll_n (mt::normalize(roll,  Scalar(0.0), TWO_PI));

  	half_yaw   = yaw_n   * Scalar(0.5);
        half_pitch = pitch_n * Scalar(0.5);
 	Scalar half_roll  = roll_n  * Scalar(0.5);

  const Scalar cy = cos(half_yaw);
  const Scalar sy = sin(half_yaw);

  const Scalar cp = cos(half_pitch);
  const Scalar sp = sin(half_pitch);

  const Scalar cr = cos(half_roll);
  const Scalar sr = sin(half_roll);

  setValue(sr * cp * cy  -  cr * sp * sy,
           cr * sp * cy  +  sr * cp * sy,
           cr * cp * sy  -  sr * sp * cy,
           cr * cp * cy  +  sr * sp * sy);

}
