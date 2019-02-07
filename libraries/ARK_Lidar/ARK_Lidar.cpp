#include "ARK_Lidar.h"

ARK_Lidar::ARK_Lidar(){}

void ARK_Lidar::update()
{
    gcs().send_text(MAV_SEVERITY_INFO, "ARK Test Print");
}
