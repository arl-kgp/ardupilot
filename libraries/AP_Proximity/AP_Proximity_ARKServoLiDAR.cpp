/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity_ARKServoLiDAR.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <stdio.h>

extern const AP_HAL::HAL& hal;

AP_Proximity_ARKServoLiDAR::AP_Proximity_ARKServoLiDAR(AP_Proximity &_frontend,
                                                       AP_Proximity::Proximity_State &_state) :
    AP_Proximity_Backend(_frontend, _state)
{
    _sign = 1.0;
    _angle_range = 45; // TODO: Read from Params
    _angle_increment = 5; // TODO: Read from Params
    _keep_sector_min = true; // TODO: Read from Params
    _read_rate = (uint8_t)40; //TODO: Read from Params
    _time_delay = ((uint32_t)1000)/((uint32_t)_read_rate);
    _servo_angle = -_angle_range;
    _prev_sector = 10;
    _last_called = (uint32_t)0;
    _min_sector_dist = distance_max();
    SRV_Channels::move_servo(SRV_Channel::k_ark_servo_lidar, _servo_angle, -_angle_range, _angle_range);

    _found_lidar = _add_backend(AP_RangeFinder_PulsedLightLRF::detect(1, _lidar_state, RangeFinder::RangeFinder_TYPE_PLI2C));
    if (!_found_lidar) {
        _found_lidar = _add_backend(AP_RangeFinder_PulsedLightLRF::detect(0, _lidar_state, RangeFinder::RangeFinder_TYPE_PLI2C));
    }
}

// update the state of the sensor
void AP_Proximity_ARKServoLiDAR::update(void)
{
    if (AP_HAL::millis()-_last_called < _time_delay) return;
    if (!_found_lidar) return; // In case the LiDAR is not available
    if (_driver == nullptr || _lidar_state.type == RangeFinder::RangeFinder_TYPE_NONE) {
        _lidar_state.status = RangeFinder::RangeFinder_NotConnected;
        _lidar_state.range_valid_count = 0;
            return;
    }

    //Read LiDAR
    float distance_m = read_lidar();
    if (distance_m < distance_min()) return;

    //Move Servo
    _servo_angle += _sign*_angle_increment;
    if(_servo_angle > _angle_range) {
        _servo_angle = _angle_range-_angle_increment;
        _sign = -1.0;
    }
    else if(_servo_angle < -_angle_range) {
        _servo_angle = -(_angle_range-_angle_increment);
        _sign = 1.0;
    }
    //gcs().send_text(MAV_SEVERITY_INFO, "Test Print Angle %f", _servo_angle);
    SRV_Channels::move_servo(SRV_Channel::k_ark_servo_lidar, _servo_angle, -_angle_range, _angle_range);
    
    // Update Data
    float angle_non_negative = _servo_angle;
    if (_servo_angle<0){
        angle_non_negative = 360.0+_servo_angle;
    }
    uint8_t sector;
    if (convert_angle_to_sector(angle_non_negative, sector)) {
        float distance_for_update = distance_m;
        if(_keep_sector_min){
            if(sector == _prev_sector)
            {
                if (distance_m < _min_sector_dist) _min_sector_dist = distance_m;
            }
            else _min_sector_dist = distance_m;
            distance_for_update = _min_sector_dist;
        }
        
        _angle[sector] = angle_non_negative;
        _distance[sector] = distance_for_update;
        _distance_valid[sector] = is_positive(distance_for_update);
        _last_called = AP_HAL::millis();
        // update boundary used for avoidance
        update_boundary_for_sector(sector);
        _prev_sector = sector;
        
    }
    set_status(AP_Proximity::Proximity_Good);
}

// get maximum and minimum distances (in meters) of primary sensor
float AP_Proximity_ARKServoLiDAR::distance_max() const
{
    return 100.0f;
}
float AP_Proximity_ARKServoLiDAR::distance_min() const
{
    return 0.20f;
}

float AP_Proximity_ARKServoLiDAR::read_lidar()
{
    _driver->update();
    float distance = ((float)_lidar_state.distance_cm)/100.0;
    return distance;
}


bool AP_Proximity_ARKServoLiDAR::_add_backend(AP_RangeFinder_Backend *backend)
{
    if (!backend) {
        return false;
    }
    _driver = backend;
    return true;
}