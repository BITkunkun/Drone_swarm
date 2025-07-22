import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

"""
四旋翼SE(3)空间控制器
实现基于Lee et al. "Geometric tracking control of a quadrotor UAV on SE(3)"的控制算法
"""

# 四旋翼物理参数
m = 4.34  # kg
J = np.diag([0.0820, 0.0845, 0.1377])  # kg·m²
g = 9.81
e3 = np.array([0, 0, 1])

# 控制器参数
kx = 16 * m
kv = 5.6 * m
kR = 8.81
kOmega = 2.54

def hat(v):
    """向量到反对称矩阵"""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def vee(M):
    """反对称矩阵到向量"""
    return np.array([M[2, 1], M[0, 2], M[1, 0]])

def compute_desired_thrust_and_Rd(x, v, R, Omega, xd, xd_dot, xd_ddot, b1d):
    """
    计算期望推力和姿态控制器
    基于几何控制理论，在SE(3)空间实现四旋翼轨迹跟踪
    
    输入参数：
        x: 当前位置 (3D向量) [m]
        v: 当前速度 (3D向量) [m/s]
        R: 当前姿态 (3x3旋转矩阵)
        Omega: 当前角速度 (体坐标系) [rad/s]
        xd: 期望位置 (3D向量) [m]
        xd_dot: 期望速度 (3D向量) [m/s]
        xd_ddot: 期望加速度 (3D向量) [m/s²]
        b1d: 期望航向方向 (机体x轴方向)
        
    输出：
        f: 总推力 (标量) [N]
        M: 控制力矩 (3D向量) [N·m]
        Rd: 期望姿态 (3x3旋转矩阵)
        
    算法步骤：
        1. 计算位置和速度误差
        2. 计算期望推力方向(b3d)
        3. 构造期望姿态矩阵Rd
        4. 计算总推力
        5. 计算姿态误差和控制力矩
    """
    ex = x - xd
    ev = v - xd_dot

    # 期望推力方向
    A = -kx * ex - kv * ev - m * g * e3 + m * xd_ddot
    b3d = -A / np.linalg.norm(A)

    # 构造期望姿态矩阵 Rd
    b1d = b1d / np.linalg.norm(b1d)
    b2d = np.cross(b3d, b1d)
    b2d /= np.linalg.norm(b2d)
    b1d = np.cross(b2d, b3d)  # 正交化
    Rd = np.column_stack((b1d, b2d, b3d))

    # 总推力
    f = -A @ (R @ e3)

    # 姿态误差
    eR = 0.5 * vee(Rd.T @ R - R.T @ Rd)
    Omega_d = np.zeros(3)  # 期望角速度（可扩展）
    eOmega = Omega - R.T @ Rd @ Omega_d

    # 控制力矩
    M = -kR * eR - kOmega * eOmega + np.cross(Omega, J @ Omega) \
        - J @ (hat(Omega) @ R.T @ Rd @ Omega_d - R.T @ Rd @ np.zeros(3))

    return f, M, Rd

if __name__ == "__main__":
    """
    控制器测试用例 - 椭圆螺旋轨迹跟踪
    无人机跟踪一个椭圆螺旋上升轨迹：
    x = 2*cos(t), y = 1*sin(t), z = 0.1*t
    """
    # 椭圆螺旋参数
    a, b, c = 2.0, 1.0, 0.1  # x/y半径和上升速率
    
    def desired_trajectory(t):
        """计算期望轨迹的位置、速度、加速度"""
        xd = np.array([a*np.cos(t), b*np.sin(t), c*t])
        xd_dot = np.array([-a*np.sin(t), b*np.cos(t), c])
        xd_ddot = np.array([-a*np.cos(t), -b*np.sin(t), 0])
        return xd, xd_dot, xd_ddot

    # 初始状态
    x = np.array([0, 0, 0])
    v = np.array([0, 0, 0])
    R = np.eye(3)
    Omega = np.array([0, 0, 0])
    b1d = np.array([1, 0, 0])  # 航向沿x轴

    # 准备可视化
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 生成完整期望轨迹用于绘图
    t_vals = np.linspace(0, 5, 100)
    xd_vals = np.array([desired_trajectory(t)[0] for t in t_vals])
    ax.plot(xd_vals[:,0], xd_vals[:,1], xd_vals[:,2], 'b-', label='期望轨迹')
    
    # 初始化无人机位置和姿态指示器
    drone_pos, = ax.plot([], [], [], 'ro', label='无人机位置')
    drone_orient = [ax.quiver(0,0,0,0,0,0, color='r', length=0.5) for _ in range(3)]
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('四旋翼椭圆螺旋轨迹跟踪')
    ax.legend()
    ax.set_xlim(-3, 3)
    ax.set_ylim(-2, 2)
    ax.set_zlim(0, 0.6)
    
    # 存储历史数据
    history = {'t': [], 'x': [], 'f': [], 'M': []}
    
    def update(frame):
        t = frame * 0.1
        xd, xd_dot, xd_ddot = desired_trajectory(t)
        f, M, Rd = compute_desired_thrust_and_Rd(x, v, R, Omega, xd, xd_dot, xd_ddot, b1d)
        
        # 简单模拟无人机运动 (实际应用中应该用动力学模型)
        x = xd.copy()
        
        # 更新绘图
        drone_pos.set_data([x[0]], [x[1]])
        drone_pos.set_3d_properties([x[2]])
        
        for i, vec in enumerate(Rd.T):
            drone_orient[i].remove()
            drone_orient[i] = ax.quiver(x[0], x[1], x[2], 
                                      vec[0], vec[1], vec[2],
                                      color=['r','g','b'][i], length=0.5)
        
        # 记录数据
        history['t'].append(t)
        history['x'].append(x.copy())
        history['f'].append(f)
        history['M'].append(M.copy())
        
        return [drone_pos] + drone_orient
    
    # 创建动画
    ani = FuncAnimation(fig, update, frames=50, interval=100, blit=True)
    
    print("正在显示轨迹跟踪动画...")
    plt.tight_layout()
    plt.show()
    
    # 打印最后一步的控制输出
    print(f"\n最终控制输出 (t={history['t'][-1]:.1f}s):")
    print(f"位置: [{history['x'][-1][0]:.2f}, {history['x'][-1][1]:.2f}, {history['x'][-1][2]:.2f}]")
    print(f"总推力: {history['f'][-1]:.2f} N")
    print(f"控制力矩: [{history['M'][-1][0]:.2f}, {history['M'][-1][1]:.2f}, {history['M'][-1][2]:.2f}] N·m")
