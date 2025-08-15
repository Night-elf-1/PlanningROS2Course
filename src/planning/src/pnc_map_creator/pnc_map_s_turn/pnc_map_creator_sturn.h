#ifndef PNC_MAP_CREATOR_STURN_H_
#define PNC_MAP_CREATOR_STURN_H_

#include "pnc_map_creator_base.h"

namespace Planning
{

    class PNCMapCreatorSTrun : public PNCMapCreatorBase // sturn地图
    {
    public:
        PNCMapCreatorSTrun();
        PNCMap creat_pnc_map() override; // 重写基类 生成地图
    private:
        void init_pnc_map(); // 初始化地图
        void draw_straight_x(const double &length, const double &plus_flag,
                             const double &ratio = 1.0); // 沿着x轴画直线
        void draw_arc(const double &angle, const double &plus_flag,
                      const double &ratio = 1.0);   // 画弧线，逆时针为正方向 顺时针为负方向 angle为总角度
    };

} // namespace Planning

#endif // PNC_MAP_CREATOR_STURN_H_