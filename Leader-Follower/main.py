"""
无人机编队控制（领航-跟随法）

本实现展示了一种用于多无人机的分布式领航-跟随编队控制算法。领航无人机沿预定义路径飞行，
跟随无人机则使用PID控制保持相对位置。

作者：罗嘉豪
日期：2025-07-02
BITkunkun
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time

class DroneState:
    """表示无人机状态，包含位置和速度
    
    属性:
        x (float): X位置坐标（米）
        y (float): Y位置坐标（米）
        vx (float): X方向速度（米/秒）
        vy (float): Y方向速度（米/秒）
    """
    def __init__(self, x=0, y=0, vx=0, vy=0):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy

class LeaderDrone:
    """领航无人机，定义编队轨迹
    
    参数:
        id (int): 无人机唯一标识符
    """
    def __init__(self, id):
        self.id = id
        self.state = DroneState()
        self.history = []  # 存储历史位置
    
    def update_position(self, new_pos):
        """更新无人机位置并记录历史"""
        self.state.x, self.state.y = new_pos
        self.history.append((self.state.x, self.state.y))
        
    def get_position(self):
        """获取当前位置的 numpy 数组"""
        return np.array([self.state.x, self.state.y])

class FollowerDrone:
    """跟随无人机，保持与领航无人机的相对位置
    
    参数:
        id (int): 无人机唯一标识符
        leader (LeaderDrone): 领航无人机的引用
        offset (np.array): 相对于领航无人机的编队偏移量 [x, y]（米）
    """
    def __init__(self, id, leader, offset):
        self.id = id
        self.leader = leader
        self.offset = offset  # 编队偏移量(x,y)
        self.state = DroneState()
        self.history = []  # 存储历史位置
        
    def update_target(self):
        """根据领航无人机位置更新目标位置"""
        leader_pos = self.leader.get_position() # 领航无人机实时位置
        self.target_pos = leader_pos + self.offset # 跟随无人机目标位置
        
    def pid_control(self):
        """使用PID控制（仅比例项）更新无人机状态"""
        self.update_target()
        # 计算位置误差
        error = self.target_pos - np.array([self.state.x, self.state.y])
        
        # 简单PID控制 (只有比例项)
        kp = 0.8  # 比例系数
        self.state.vx = kp * error[0]
        self.state.vy = kp * error[1]
        
        # 更新位置（时间步长0.05秒）
        self.state.x += self.state.vx * 0.05
        self.state.y += self.state.vy * 0.05
        self.history.append((self.state.x, self.state.y))

def quadcopter_dynamics(state, u, dt):
    """简化的2D四旋翼动力学模型（含空气阻力）
    
    参数:
        state (DroneState): 无人机当前状态
        u (tuple): 控制输入 (ax, ay) 单位m/s²
        dt (float): 时间步长（秒）
        
    返回:
        DroneState: 应用动力学后的更新状态
        
    动力学方程:
        v̇ = u - βv  （β = 空气阻力系数）
        ṗ = v
    """
    x, y, vx, vy = state.x, state.y, state.vx, state.vy
    ax, ay = u
    
    # 含空气阻尼项的速度更新
    new_vx = vx + ax*dt - 0.1*vx  
    new_vy = vy + ay*dt - 0.1*vy
    # 位置更新
    new_x = x + new_vx*dt
    new_y = y + new_vy*dt
    
    return DroneState(new_x, new_y, new_vx, new_vy)

def animate(i, leader, followers, scatters, lines, dt):
    """动画更新函数，每帧调用一次
    
    参数:
        i (int): 帧数
        leader (LeaderDrone): 领航无人机实例
        followers (list): 跟随无人机实例列表
        scatters (list): matplotlib散点对象（表示无人机当前位置）
        lines (list): matplotlib线对象（表示飞行轨迹）
    """
    # 示例：直线运动 领航无人机直线匀速运动
    leader_pos = leader.get_position() + np.array([0.1, 0]) # 直线匀速向右飞行运动
    leader.update_position(leader_pos)

    # 领航无人机变速运动
    # u = (3, 1)  # 加速度控制输入 (ax, ay)
    
    # # 使用动力学模型更新领航者状态
    # new_state = quadcopter_dynamics(leader.state, u, dt)
    # leader.state = new_state
    # leader.update_position((new_state.x, new_state.y))

    # 更新所有跟随者状态
    for follower in followers:
        follower.pid_control()
    
    # 更新无人机位置的散点图数据
    positions = [leader.get_position()] + [np.array([f.state.x, f.state.y]) for f in followers]
    for scatter, pos in zip(scatters, positions):
        scatter.set_offsets(pos)
    
    # 更新轨迹线数据
    for line, drone in zip(lines, [leader] + followers):
        if len(drone.history) > 1:
            x, y = zip(*drone.history)
            line.set_data(x, y)
    
    return scatters + lines

def main():
    """主函数：设置并运行编队控制仿真

    创建:
    - 1架领航无人机（红色）
    - 3架跟随无人机组成三角形编队（蓝色、绿色、紫色）
    - 实时动画显示位置和轨迹
    """
    # 设置中文字体
    plt.rcParams['font.sans-serif'] = ['SimHei']  # 设置中文显示
    plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题
    
    # 创建编队配置
    leader = LeaderDrone(0) # Leader无人机向右飞行
    followers = [
        FollowerDrone(1, leader, offset=np.array([0, 5])),   # 上侧无人机
        FollowerDrone(2, leader, offset=np.array([5, 0])),   # 右方无人机
        FollowerDrone(3, leader, offset=np.array([-5, 0]))  # 左侧无人机
    ]
    
    # 设置绘图
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(-10, 30)   # X轴范围
    ax.set_ylim(-10, 20)   # Y轴范围
    ax.grid(True)          # 显示网格
    ax.set_title('无人机编队控制（领航-跟随法）')  # 标题
    
    # 创建散点图（无人机当前位置）
    colors = ['red', 'blue', 'green', 'purple']
    scatters = [ax.scatter([], [], color=colors[i], s=100, label=f'无人机 {i}') 
               for i in range(len(followers)+1)]
    
    # 创建线图（无人机轨迹）
    lines = [ax.plot([], [], color=colors[i], alpha=0.5)[0] 
            for i in range(len(followers)+1)]
    
    ax.legend()  # 显示图例
    
    dt = 0.05

    # 创建动画
    ani = FuncAnimation(fig, animate, frames=200, 
                       fargs=(leader, followers, scatters, lines, dt),
                       interval=50, blit=True)
    
    plt.show()

if __name__ == "__main__":
    main()
