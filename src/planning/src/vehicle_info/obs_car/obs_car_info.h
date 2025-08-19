#ifndef OBS_CAR_INFO_H_
#define OBS_CAR_INFO_H_

#include "vehicle_info_base.h"

namespace Planning
{

class ObsCar : public VehicleBase  // 障碍物
{
public:
    ObsCar(const int &id);
};

}   // namespace Planning

#endif  // OBS_CAR_INFO_H_