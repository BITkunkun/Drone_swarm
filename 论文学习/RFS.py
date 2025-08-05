"""
随机有限集(Random Finite Set, RFS)生成器
该模块实现了一个简单的随机有限集生成算法，用于从给定的状态空间中随机选择元素
"""

import random  # 导入random模块用于生成随机数

# 定义状态空间：0到99的整数列表
# 状态空间表示所有可能被选择的元素集合
state_space = list(range(100))

def generate_rfs(state_space):
    """
    生成随机有限集(RFS)的函数
    
    参数:
    state_space -- 包含所有可能元素的状态空间列表
    
    返回值:
    从状态空间中随机选择的元素子集(可能为空集)
    """
    # 随机确定要选择的元素数量(0到状态空间大小的随机整数)
    num_elements = random.randint(0, len(state_space))
    
    # 从状态空间中随机选择指定数量的不重复元素
    rfs = random.sample(state_space, num_elements)
    
    return rfs  # 返回生成的随机有限集

# 测试代码：生成并打印一个随机有限集示例
if __name__ == "__main__":
    print("生成的随机有限集:", generate_rfs(state_space))
