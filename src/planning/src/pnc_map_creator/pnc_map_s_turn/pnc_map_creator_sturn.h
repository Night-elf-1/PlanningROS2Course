#ifndef PNC_MAP_CREATOR_STURN_H_
#define PNC_MAP_CREATOR_STURN_H_

#include "pnc_map_creator_base.h"

namespace Planning
{

class PNCMapCreatorSTrun : public PNCMapCreatorBase  // sturn地图
{
public:
    PNCMapCreatorSTrun();
    PNCMap creat_pnc_map() override;    // 重写基类 生成地图
private:
    
};

}   // namespace Planning

#endif  // PNC_MAP_CREATOR_STURN_H_