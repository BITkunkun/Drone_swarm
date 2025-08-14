import math

def euclidean_distance(a, b):
    """
    计算两个n维点之间的欧几里得距离
    :param a: 第一个点（列表或元组，如[1,2,3]）
    :param b: 第二个点（列表或元组，如[4,5,6]）
    :return: 欧几里得距离（浮点数）
    """
    if len(a) != len(b):
        raise ValueError("两个点的维度必须相同")
    
    # 计算每个维度差值的平方和
    squared_diff_sum = sum((x - y) **2 for x, y in zip(a, b))
    # 开平方得到距离
    return math.sqrt(squared_diff_sum)

# 示例
point1 = [1, 2]
point2 = [4, 5]
print(euclidean_distance(point1, point2))  # 输出欧式距离