import numpy as np
import asyncio
import matplotlib.pyplot as plt
from mavsdk import System
from mavsdk import action

class DroneController:
    def __init__(self, drone_id):
        self.id = drone_id
        self.current_pos = np.array([0.0, 0.0])  # 当前坐标(X,Y)
        self.target_pos = np.array([0.0, 0.0])   # 目标坐标
        self.drone = System()  # MAVSDK无人机实例

    async def connect(self):
        # 使用外部mavsdk_server连接
        await self.drone.connect(system_address=f"udp://:14540")
        print(f"无人机 {self.id} 连接成功")

    async def update_position(self):
        # 简化位置更新逻辑，添加超时处理
        try:
            async for position in self.drone.telemetry.position():
                # 转换为简单的X-Y坐标（实际应用中应使用经纬度转换）
                self.current_pos = np.array([position.latitude * 1e-7, 
                                           position.longitude * 1e-7])  # 更新实时位置
                break  # 仅获取一次位置用于示例
        except asyncio.TimeoutError:
            print(f"无人机 {self.id} 获取位置超时")

    async def move_to_target(self):
        if np.linalg.norm(self.target_pos - self.current_pos) < 0.1:
            return  # 位置足够接近时不发送指令
            
        error = self.target_pos - self.current_pos
        # PID控制算法实现（仅比例项）
        kp = 0.6  # 比例系数
        vx = kp * error[0]
        vy = kp * error[1]
        
        try:
            # 发送速度指令 (兼容新版mavsdk)
            await self.drone.action.set_velocity_ned(
                action.VelocityBodyYawspeed(vx, vy, 0, 0))
        except Exception as e:
            print(f"无人机 {self.id} 发送指令错误: {e}")

class Visualizer:
    def __init__(self):
        plt.ion()  # 开启交互模式
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_title('无人机编队虚拟结构')
        self.ax.grid(True)
        self.drone_plots = []
        self.structure_plot = None
        
    def update(self, drones, structure):
        # 清除之前的绘图
        for plot in self.drone_plots:
            plot.remove()
        if self.structure_plot:
            self.structure_plot.remove()
            
        # 绘制虚拟结构
        if structure.shape == 'rectangle':
            width, height = structure.size
            rect = plt.Rectangle((structure.center[0]-width/2, structure.center[1]-height/2), 
                                width, height, fill=False, color='blue')
            self.structure_plot = self.ax.add_patch(rect)
        
        # 绘制无人机位置
        self.drone_plots = []
        for drone in drones:
            plot = self.ax.plot(drone.current_pos[0], drone.current_pos[1], 'ro')[0]
            self.drone_plots.append(plot)
            self.ax.text(drone.current_pos[0], drone.current_pos[1], f'Drone {drone.id}')
        
        plt.pause(0.01)  # 短暂暂停以更新图形

class VirtualStructure:
    def __init__(self, shape='rectangle', size=(8, 5)):
        self.shape = shape
        self.size = size
        self.center = np.array([0.0, 0.0])  # 虚拟结构中心位置
        self.members = []  # 无人机成员列表
    
    def assign_positions(self):
        # 为不同形状分配位置（以矩形为例）
        if self.shape == 'rectangle':
            width, height = self.size
            # 矩形四个顶点位置（相对于中心）
            positions = [
                np.array([width/2, height/2]),    # 右上
                np.array([width/2, -height/2]),   # 右下
                np.array([-width/2, -height/2]),  # 左下
                np.array([-width/2, height/2])    # 左上
            ]
            # 分配位置给各无人机
            for i, drone in enumerate(self.members):
                drone.target_pos = self.center + positions[i]
                print(f"无人机 {i} 目标位置: {drone.target_pos}")
    
    def update_drone_target(self, drone):
        # 根据虚拟结构中心更新无人机目标位置
        # 找到无人机在成员列表中的索引
        idx = self.members.index(drone)
        # 初始位置（相对于中心）
        if self.shape == 'rectangle':
            width, height = self.size
            initial_positions = [
                np.array([width/2, height/2]),
                np.array([width/2, -height/2]),
                np.array([-width/2, -height/2]),
                np.array([-width/2, height/2])
            ]
        else:
            initial_positions = [np.array([0, 0])]  # 默认为中心点
        
        # 计算新目标位置 = 中心位置 + 初始相对位置
        new_target = self.center + initial_positions[idx]
        return new_target

async def formation_control():
    # 创建虚拟结构与4架无人机
    vs = VirtualStructure(shape='rectangle', size=(8, 5))
    visualizer = Visualizer()  # 创建可视化实例
    drones = [DroneController(i) for i in range(4)]
    vs.members = drones
    
    # 初始化连接与位置分配
    connect_tasks = [drone.connect() for drone in drones]
    await asyncio.gather(*connect_tasks)
    
    vs.assign_positions()
    
    # 主控制循环（10Hz更新频率）
    try:
        while True:
            # 虚拟结构中心移动（示例：直线运动）
            vs.center += np.array([0.01, 0])  # 每秒向东移动0.01单位
            
            # 更新各无人机目标位置
            tasks = []
            for drone in vs.members:
                # 先更新位置
                await drone.update_position()
                # 设置新目标位置
                drone.target_pos = vs.update_drone_target(drone)
                # 移动到目标位置
                tasks.append(drone.move_to_target())
            
            # 等待所有任务完成
            await asyncio.gather(*tasks)
            
            # 更新可视化
            visualizer.update(vs.members, vs)
            
            await asyncio.sleep(0.1)  # 控制周期100ms
    except KeyboardInterrupt:
        print("编队控制程序停止")

def main():
    # Windows下运行异步程序需要特别处理
    if sys.platform == 'win32':
        # Python 3.8+ 支持的Windows异步事件循环策略
        if sys.version_info >= (3, 8) and sys.platform == 'win32':
            asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    
    # 运行异步函数
    asyncio.run(formation_control())

if __name__ == "__main__":
    import sys
    main()
