#ifndef MAIN_CAR_INFO_H_
#define MAIN_CAR_INFO_H_

#include "vehicle_info_base.h"

namespace Planning
{

class MainCar : public VehicleBase  // 主车  
{
public:
    MainCar();

    void vehicle_cartesian_to_frenet(const Referline &refer_line) override; // 重写
};

}   // namespace Planning

#endif  // MAIN_CAR_INFO_H_