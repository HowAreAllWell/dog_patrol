import torch

from RL2Path.nav.cost_map_2d import Costmap_2d

# 世界坐标
p = torch.tensor([[0.00, 0.00],      # 原点
                  [1.92, 1.92],      # 接近右上角
                  [-1.99, -1.99]])   # 接近左下角
cm = Costmap_2d()        # 使用默认参数

print(cm._world_to_map(p))