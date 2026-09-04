# obstacle.py

import math
from numpy import ndarray
from typing import Tuple

from dataclasses import dataclass

@dataclass(eq=None)
class Obstacle:
    position: ndarray

    def get_bounding_box(self) -> Tuple[float, float, float, float]:
        """
        获取障碍物的边界框 (x_min, x_max, y_min, y_max)。
        """
        raise NotImplementedError

    def is_collision(self, point: Tuple[float, float], safety_distance: float = 0.0) -> bool:
        """
        判断给定点是否与障碍物发生碰撞，考虑安全距离。
        """
        raise NotImplementedError
    
    def get_radius(self) -> float:
        """
        返回用于邻近查询的一个“近似半径”。
        子类需要重写。
        """
        raise NotImplementedError

@dataclass(eq=None)
class RectangleObstacle(Obstacle):
    width: float = 1.0
    height: float = 1.0

    def get_bounding_box(self) -> Tuple[float, float, float, float]:
        x, y = self.position
        return (
            x - self.width / 2,
            x + self.width / 2,
            y - self.height / 2,
            y + self.height / 2
        )

    def is_collision(self, point: Tuple[float, float], safety_distance: float = 0.0) -> bool:
        x, y = point
        x_min, x_max, y_min, y_max = self.get_bounding_box()

        # 计算点到矩形水平边界的最短距离
        if x < x_min:
            distance_x = x_min - x
        elif x > x_max:
            distance_x = x - x_max
        else:
            distance_x = 0  # 点在矩形水平范围内

        # 计算点到矩形垂直边界的最短距离
        if y < y_min:
            distance_y = y_min - y
        elif y > y_max:
            distance_y = y - y_max
        else:
            distance_y = 0  # 点在矩形垂直范围内

        # 计算点到矩形边界的最短欧几里得距离
        distance = (distance_x ** 2 + distance_y ** 2) ** 0.5

        # 判断最短距离是否小于安全距离
        return distance <= safety_distance

    
    def get_radius(self) -> float:
        """
        将矩形近似成圆形，用对角线的一半作为“近似半径”。
        """
        return math.hypot(self.width / 2, self.height / 2)

@dataclass(eq=None)
class CircleObstacle(Obstacle):
    radius: float = 0.5

    def get_bounding_box(self) -> Tuple[float, float, float, float]:
        x, y = self.position
        return (
            x - self.radius,
            x + self.radius,
            y - self.radius,
            y + self.radius
        )

    def is_collision(self, point: Tuple[float, float], safety_distance: float = 0.0) -> bool:
        x, y = point
        dx = x - self.position[0]
        dy = y - self.position[1]
        distance = math.hypot(dx, dy)
        return distance <= (self.radius + safety_distance)
    
    def get_radius(self) -> float:
        """
        圆形本身的半径就可以直接作为“近似半径”。
        """
        return self.radius
