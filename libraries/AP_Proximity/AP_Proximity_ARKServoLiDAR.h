#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>

#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>
#include <AP_RangeFinder/AP_RangeFinder_PulsedLightLRF.h>

class AP_Proximity_ARKServoLiDAR : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_ARKServoLiDAR(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // update state
    void update(void);

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const;
    float distance_min() const;

private:

    enum RequestType {
        RequestType_None = 0,
        RequestType_Health,
        RequestType_MotorSpeed,
        RequestType_MotorDirection,
        RequestType_ForwardDirection,
        RequestType_DistanceMeasurement
    };
    float _sign;
    float _servo_angle, _angle_range;
    bool _found_lidar;
    bool _keep_sector_min;
    float _angle_increment;
    uint8_t _prev_sector;
    uint8_t _read_rate;
    uint32_t _last_called;
    uint32_t _time_delay;
    float _min_sector_dist;
    
    AP_RangeFinder_Backend *_driver;
    RangeFinder::RangeFinder_State _lidar_state;
    float read_lidar();
    bool _add_backend(AP_RangeFinder_Backend *driver);
    
};
