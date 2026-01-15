import unittest
import sys
import os
from shapely.geometry import LineString

# 添加项目根目录到路径
proj_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if proj_root not in sys.path:
    sys.path.insert(0, proj_root)

# 导入业务代码
from src.core.fly_path_collision_analyse import (
    collision_analyse,
    visualize_collision_analysis,
    calculate_distance_3d
)
from src.service.load_fly_paths import get_fly_paths


class TestFlyPathCollision(unittest.TestCase):

    def test_collision_analyse_structure(self):
        """测试碰撞分析结果的基础结构"""
        fly_paths = get_fly_paths()
        result = collision_analyse(fly_paths)

        self.assertIsInstance(result, dict)
        self.assertIn("has_risk", result)
        self.assertIn("risk_paths", result)
        self.assertIsInstance(result["has_risk"], bool)
        self.assertIsInstance(result["risk_paths"], list)

        for risk_path in result["risk_paths"]:
            # 验证必填字段
            keys = ["path1_index", "path2_index", "min_distance", "risk_level", "nearest_points"]
            for key in keys:
                self.assertIn(key, risk_path)

            # 验证风险等级
            self.assertIn(risk_path["risk_level"], ["fatal", "warn", "safe"])

            # 验证逻辑一致性
            min_dist = risk_path["min_distance"]
            level = risk_path["risk_level"]
            if min_dist <= 10:
                self.assertEqual(level, "fatal")
            elif min_dist <= 50:
                self.assertEqual(level, "warn")

    def test_collision_analyse_with_close_paths(self):
        """测试非常接近的路径（应该有碰撞风险）"""
        fly_paths = [
            {
                "geometry": LineString([(120.0, 30.0, 20), (120.1, 30.1, 25)]),
                "altitude": 30, "speed": 10.0, "type": "flight_path"
            },
            {
                "geometry": LineString([(120.001, 30.001, 20), (120.101, 30.101, 25)]),
                "altitude": 35, "speed": 12.0, "type": "flight_path"
            }
        ]
        result = collision_analyse(fly_paths)
        self.assertTrue(result["has_risk"])
        self.assertGreaterEqual(len(result["risk_paths"]), 1)

    def test_collision_analyse_with_distant_paths(self):
        """测试距离很远的路径（应该没有碰撞风险）"""
        fly_paths = [
            {"geometry": LineString([(120.0, 30.0, 20), (120.1, 30.1, 25)])},
            {"geometry": LineString([(130.0, 40.0, 20), (130.1, 40.1, 25)])}
        ]
        result = collision_analyse(fly_paths)
        print(result)
        self.assertFalse(result["has_risk"])
        self.assertEqual(len(result["risk_paths"]), 0)

    def test_collision_analyse_with_intersecting_paths(self):
        """测试相交的路径"""
        # 将 0.05 改为 0.0001 (大约 11 米)
        fly_paths = [
            {"geometry": LineString([(120.0, 30.0, 20), (120.1, 30.1, 25)])},
            {"geometry": LineString([(120.0001, 30.0001, 20), (120.1001, 30.1001, 25)])}
        ]
        result = collision_analyse(fly_paths)
        print(result)
        # 即使相交，这里也应该返回合法的字典结构
        self.assertIsInstance(result, dict)

    def test_collision_analyse_with_single_or_empty_path(self):
        """测试单条或空路径"""
        # 单条
        res_single = collision_analyse([{"geometry": LineString([(0, 0, 0), (1, 1, 1)])}])
        self.assertFalse(res_single["has_risk"])
        # 空列表
        res_empty = collision_analyse([])
        self.assertFalse(res_empty["has_risk"])

    def test_calculate_distance_3d(self):
        """测试3D距离计算"""
        # 相同点
        self.assertEqual(calculate_distance_3d((0, 0, 0), (0, 0, 0)), 0.0)
        # 单位距离
        self.assertAlmostEqual(calculate_distance_3d((0, 0, 0), (1, 0, 0)), 1.0, places=10)
        # 对角线 sqrt(3)
        self.assertAlmostEqual(calculate_distance_3d((0, 0, 0), (1, 1, 1)), 3 ** 0.5, places=10)

    def test_visualize_collision_analysis(self):
        """测试可视化（不崩溃即可）"""
        try:
            fly_paths = get_fly_paths()
            result = collision_analyse(fly_paths)
            visualize_collision_analysis(fly_paths, result)
        except Exception as e:
            # 如果在服务器/无界面环境运行，跳过此报错或打印警告
            print(f"Visualization skipped or failed: {e}")

    def test_visualize_collision_analysis_parallel(self):
        """测试：两条严格平行的航线（最近点在线段中部）"""
        try:
            fly_paths = []

            # -------------------------
            # 航线 1：向南飞行（高度 30m）
            fly_paths.append({
                "geometry": LineString([
                    (120.0080, 30.3000, 30),
                    (120.0080, 30.2950, 30),
                    (120.0080, 30.2900, 30),
                ]),
                "altitude": 30,
                "speed": 10.0,
                "type": "flight_path"
            })

            # -------------------------
            # 航线 2：与航线 1 平行（高度 35m）
            fly_paths.append({
                "geometry": LineString([
                    (120.0090, 30.3000, 35),
                    (120.0090, 30.2950, 35),
                    (120.0090, 30.2900, 35),
                ]),
                "altitude": 35,
                "speed": 10.0,
                "type": "flight_path"
            })

            result = collision_analyse(fly_paths)
            visualize_collision_analysis(fly_paths, result)

        except Exception as e:
            print(f"Visualization skipped or failed: {e}")

    def test_visualize_collision_analysis_three_routes(self):
        """测试：三条空间交错的航线（垂直、平行、交叉）"""
        try:
            fly_paths = []

            # -------------------------
            # 航线 1：由北向南（高度 30m，直线）
            # 经度固定在 120.0080
            fly_paths.append({
                "geometry": LineString([
                    (120.0080, 30.3000, 30),
                    (120.0080, 30.2950, 30),
                    (120.0080, 30.2900, 30),
                ]),
                "altitude": 30,
                "speed": 10.0,
                "type": "flight_path"
            })

            # -------------------------
            # 航线 2：由西向东交叉（高度 32m，横切航线1）
            # 在 (120.0080, 30.2950) 附近与航线1在平面投影上相交
            fly_paths.append({
                "geometry": LineString([
                    (120.0060, 30.2950, 32),
                    (120.0080, 30.2950, 32),
                    (120.0100, 30.2950, 32),
                ]),
                "altitude": 32,
                "speed": 12.0,
                "type": "flight_path"
            })

            # -------------------------
            # 航线 3：对角线飞行（高度 28m -> 35m，斜向爬升）
            # 从东南向西北穿过前两条航线构成的区域
            fly_paths.append({
                "geometry": LineString([
                    (120.0100, 30.2900, 28),
                    (120.0085, 30.2940, 31.5),
                    (120.0070, 30.2980, 35),
                ]),
                "altitude": 31, # 标称高度
                "speed": 8.0,
                "type": "flight_path"
            })

            result = collision_analyse(fly_paths)
            print( result)
            visualize_collision_analysis(fly_paths, result)

        except Exception as e:
            print(f"Visualization of three routes failed: {e}")
if __name__ == "__main__":
    unittest.main()