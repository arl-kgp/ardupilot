#include "ARK_Lidar.h"

extern const AP_HAL::HAL &hal;

ARK_Lidar::ARK_Lidar(){
	pwm = (uint16_t)1000;
	init_once = true;
}

void ARK_Lidar::update()
{
    pwm = pwm + (uint16_t)60;
    if(pwm > (uint16_t)1980) pwm = (uint16_t)1000;
    gcs().send_text(MAV_SEVERITY_INFO, "ARK Test Print %d", pwm);
    SRV_Channels::set_output_pwm_chan((uint8_t)9, pwm);
    SRV_Channels::set_output_pwm_chan((uint8_t)10, 1500);

    if(init_once){
    	for (uint8_t i=0, serial_instance = 0; i<RANGEFINDER_MAX_INSTANCES; i++) {
	    	detect_instance(i, serial_instance);
	    }
	    init_once = false;
    }
    else{
    	driver->update();
    	gcs().send_text(MAV_SEVERITY_INFO, "ARK Lidar Distance %d", state.distance_cm);
	    // if (driver != nullptr) {
	    //      if (state.type == RangeFinder::RangeFinder_TYPE_NONE) {
	    //         // allow user to disable a rangefinder at runtime
	    //         state.status = RangeFinder::RangeFinder_NotConnected;
	    //         state.range_valid_count = 0;
	    //         gcs().send_text(MAV_SEVERITY_INFO, "Driver None");
	    //         return;
	    //     }
	    	
	    // 	gcs().send_text(MAV_SEVERITY_INFO, "ARK Lidar Distance %d", state.distance_cm);
	    // }
	    // else gcs().send_text(MAV_SEVERITY_INFO, "Driver Null");
	}
}

void ARK_Lidar::detect_instance(uint8_t instance, uint8_t& serial_instance)
{
	bool found = _add_backend(AP_RangeFinder_PulsedLightLRF::detect(1, state, RangeFinder::RangeFinder_TYPE_PLI2C));
	if (!found) {
        found = _add_backend(AP_RangeFinder_PulsedLightLRF::detect(0, state, RangeFinder::RangeFinder_TYPE_PLI2C));
    }
	if(found) gcs().send_text(MAV_SEVERITY_INFO, "Found LiDAR");
	else gcs().send_text(MAV_SEVERITY_INFO, "LiDAR not Found");
}

bool ARK_Lidar::_add_backend(AP_RangeFinder_Backend *backend)
{
    if (!backend) {
        return false;
    }
    driver = backend;
    return true;
}