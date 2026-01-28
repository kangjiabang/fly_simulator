import os
import sys


# Add project root to sys.path
proj_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if proj_root not in sys.path:
    sys.path.insert(0, proj_root)

from fastapi import HTTPException, Query
from src.core.fly_path_collision_analyse import collision_analyse, analyze_nofly_zone_risk, visualize_nofly_zone_risk
from src.service.load_nofly_zones import get_nofly_zones
from src.tool.profiler import profile_each_line

@profile_each_line
def draw_pic(drone_id, path_data, result):
    output_filename = f"risk_analysis_{drone_id}.png"
    output_path = os.path.join(proj_root, output_filename)
    saved_path = visualize_nofly_zone_risk(path_data["path"], result, output_path=output_path)
    
    print(f"Analysis complete. Image saved to: {saved_path}")
    print(f"Risk Level: {result.get('risk_level')}")
    return saved_path



    
@profile_each_line
def start_analyze_drone_risk(fly_path: dict,drone_id: str, show_pic: bool = False):
    """
    分析无人机飞行风险：调用飞行轨迹和禁飞区接口，计算风险并画图。
    """
    try:

        # 2. 获取禁飞区 (调用 get_nofly_zones)
        zones = get_nofly_zones()
        
        # 3. 分析风险
        result = analyze_nofly_zone_risk(fly_path["path"], zones)

        #del result["zone_info"]
        
        # 4. 画图 (保存到本地)
        saved_path = None
        if show_pic:
            # 使用绝对路径保存，确保能找到
            saved_path = draw_pic(drone_id, fly_path, result)
        
        # # 5. 清理结果中的不可序列化对象
        sanitized_result = result.copy()
        if "zone_info" in sanitized_result:
            # geometry is not serializable
            # We can remove it or convert it if needed. For now, remove it to match previous behavior
            del sanitized_result["zone_info"]
            
        sanitized_result["image_path"] = saved_path
        
        return sanitized_result
        
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"风险分析失败: {e}")

if __name__ == "__main__":
    # 手动调用一次，确保 kernprof 能抓到
    start_analyze_drone_risk("drone_test_id")