#pragma once

#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>
#include <AP_RangeFinder/AP_RangeFinder_PulsedLightLRF.h>

class ARK_Lidar {

public:
    ARK_Lidar();
    
    void update();
    void detect_instance(uint8_t instance, uint8_t& serial_instance);
    bool _add_backend(AP_RangeFinder_Backend *driver);
    uint16_t pwm;
    uint8_t chan;
    AP_RangeFinder_Backend *driver;
    RangeFinder::RangeFinder_State state;
    bool init_once;
};
