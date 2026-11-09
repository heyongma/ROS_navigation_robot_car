#!/usr/bin/env python3
import yaml
import subprocess
import time
import csv
import os

# ========== 配置区 ==========
BASE_YAML_PATH = "/home/ma/demo03_ws/src/lianxi_slam/param/teb_local_planner_params.yaml"
LAUNCH_GAZEBO = ["roslaunch", "lianxi_gazebo", "car_to.launch"]
LAUNCH_NAV = ["roslaunch", "lianxi_slam", "test_to01.launch", "planner:=teb"]
OBS_NODE = ["rosrun", "lianxi_slam", "obs_auto"]
CSV_OUT = "/home/ma/demo03_ws/teb_compare_result.csv"

TEST_CASES = [
# 0 基准
    {"max_global_plan_lookahead_dist":0.8, "weight_viapoint":10.0, "weight_optimaltime":0.3, "weight_goal_point":15.0},
# 1 降低viapoint，允许横向偏移避让，减少路径拉扯
    {"max_global_plan_lookahead_dist":0.8, "weight_viapoint":8.0, "weight_optimaltime":0.3, "weight_goal_point":15.0},
# 2 降低平滑权重，抑制超大圆弧，保留避让自由度
    {"max_global_plan_lookahead_dist":0.8, "weight_viapoint":10.0, "weight_optimaltime":0.2, "weight_goal_point":15.0},
# 3 提高目标朝向权重，提前对准目标，减少来回摇摆
    {"max_global_plan_lookahead_dist":0.8, "weight_viapoint":10.0, "weight_optimaltime":0.3, "weight_goal_point":18.0},
# 4 前瞻加大到1.0，更早预判动态障碍，减少突发轨迹跳变
    {"max_global_plan_lookahead_dist":1.0, "weight_viapoint":10.0, "weight_optimaltime":0.3, "weight_goal_point":15.0},
# 5 viapoint=5.0，min_obstacle_dist=0.3，折中，兼顾贴路径+减少拉扯
    {"max_global_plan_lookahead_dist":0.8, "weight_viapoint":5.0, "weight_optimaltime":0.3, "weight_goal_point":15.0, "min_obstacle_dist":0.3},
# 6 viapoint=4.0，min_obstacle_dist=0.3，适度放宽，约束力还在，不容易反向乱跑
    {"max_global_plan_lookahead_dist":0.8, "weight_viapoint":4.0, "weight_optimaltime":0.3, "weight_goal_point":15.0, "min_obstacle_dist":0.3},
# 7 保守避让：压低时间权重，加大安全距离，不抢通行窗口，优先空间绕行
    {"max_global_plan_lookahead_dist":0.8, "weight_viapoint":10.0, "weight_optimaltime":0.1, "weight_goal_point":15.0, "min_obstacle_dist":0.4}
]

WAIT_GAZEBO = 6
WAIT_OBS = 2
WAIT_NAV = 6
# =====================================

def load_yaml(filepath):
    with open(filepath, 'r') as f:
        return yaml.safe_load(f)

def save_yaml(data, filepath):
    with open(filepath, 'w') as f:
        yaml.dump(data, f, sort_keys=False)

def kill_all():
    subprocess.run(["pkill", "-f", "gazebo"], capture_output=True)
    subprocess.run(["pkill", "-f", "roslaunch"], capture_output=True)
    subprocess.run(["pkill", "-f", "obs_auto"], capture_output=True)
    time.sleep(2)

def run_test(case_name, param_dict):
    cfg = load_yaml(BASE_YAML_PATH)
    for k,v in param_dict.items():
        cfg["TebLocalPlannerROS"][k] = v
    save_yaml(cfg, BASE_YAML_PATH)
    print(f"\n==== 【开始】测试用例 {case_name} | 参数: {param_dict} ====")

    p_gazebo = subprocess.Popen(LAUNCH_GAZEBO)
    time.sleep(WAIT_GAZEBO)

    p_obs = subprocess.Popen(OBS_NODE)
    time.sleep(WAIT_OBS)

    p_nav = subprocess.Popen(LAUNCH_NAV)
    time.sleep(WAIT_NAV)

    print("\n---------- 检测局部规划器 ----------")
    check_cmd = ["rosparam", "get", "/move_base/base_local_planner"]
    res = subprocess.run(check_cmd, capture_output=True, text=True)
    if res.returncode == 0:
        planner_name = res.stdout.strip()
        print(f"当前加载的规划器：{planner_name}")
        if "TebLocalPlannerROS" in planner_name:
            print("✅ 确认是TEB，可以开始测试")
        else:
            print("❌ 是DWA！本组作废，不要记录")
    else:
        print("⚠️ 获取规划器参数失败，move_base还没就绪")
    print("------------------------------------")

    print("👉 等动态障碍物回到起始位置，再在RVIZ下发2D Nav Goal")
    print("👉 观察完毕，按回车结束本组")
    try:
        input()
    except EOFError:
        print("\n⚠️ 输入流被打断")

    kill_all()

    note = input("请输入本组note记录：")
    row = {
        "case_id": case_name,
        **param_dict,
        "note": note
    }

    header = ["case_id"] + list(param_dict.keys()) + ["note"]
    if not os.path.exists(CSV_OUT):
        with open(CSV_OUT, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=header)
            writer.writeheader()
    with open(CSV_OUT, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=header)
        writer.writerow(row)
    print(f"✅ 数据已经写入csv，case_id={case_name}")

if __name__ == "__main__":
    bak_data = load_yaml(BASE_YAML_PATH)
    total = len(TEST_CASES)
    print(f"可选组别：0~{total-1}")
    select_idx = int(input("输入你要跑的组别编号："))
    if select_idx < 0 or select_idx >= total:
        print("编号非法，退出")
        exit(1)
    try:
        run_test(select_idx, TEST_CASES[select_idx])
    finally:
        save_yaml(bak_data, BASE_YAML_PATH)
        kill_all()
        print("\n==== 本组结束，yaml恢复原始参数 ====")
