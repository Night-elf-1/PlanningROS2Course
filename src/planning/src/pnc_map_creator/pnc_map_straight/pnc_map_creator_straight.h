#ifndef PNC_MAP_CREATOR_STRAIGHT_H_
#define PNC_MAP_CREATOR_STRAIGHT_H_

#include "pnc_map_creator_base.h"

namespace Planning
{

    class PNCMapCreatorStraight : public PNCMapCreatorBase // 直道地图
    {
    public:
        PNCMapCreatorStraight();
        PNCMap creat_pnc_map() override; // 重写 生成地图
    private:
        void init_pnc_map(); // 初始化地图
        void draw_straight_x(const double &length, const double &plus_flag,
                             const double &ratio = 1.0); // 沿着x轴画直线
    };

} // namespace Planning

#endif // PNC_MAP_CREATOR_STRAIGHT_H_