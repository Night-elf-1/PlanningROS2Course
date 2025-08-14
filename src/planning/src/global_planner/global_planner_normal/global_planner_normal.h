#ifndef GLOBAL_PLANNER_NORMAL_H_
#define GLOBAL_PLANNER_NORMAL_H_

#include "global_planner_base.h"

namespace Planning
{

    class GlobalPlannerNormal : public GlobalPlannerBase // 规划算法的基类
    {
    public:
        GlobalPlannerNormal();
        Path search_global_path(const PNCMap &pnc_map) override; // 重写 搜索全局路径
    private:
    };
}
#endif