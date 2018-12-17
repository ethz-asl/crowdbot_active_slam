/**
 *  tracked_object.cpp
 *
 *  Created on: 17.12.2018
 *      Author: Dario Mammolo
 */

 #include <tracked_object.h>

TrackedObject::TrackedObject(double x, double y){
  // Init state mean
  x_ = x;
  y_ = y;
  x_vel_ = 0;
  y_vel_ = 0;
  state_mean << x_, y_, x_vel_, y_vel_;

  // Init state var
  state_var << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

  // Init counter for not seen
  counter_not_seen = 0;
}

TrackedObject::~TrackedObject(){}
