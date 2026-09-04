# grid.py

import math
import random
import numpy as np
from tqdm import tqdm
from collections import defaultdict
from typing import List, Tuple

from isaaclab.utils import configclass
from RL2Path.nav.obstacle import *  

@configclass
class GridCfg:
    x_min = -10.0
    x_max = 10.0
    y_min = -10.0
    y_max = 10.0
    grid_size = 1.0  # Default grid size
    grid_inflated_distance = 1.0  # Default inflation distance
    wall_thickness = 0.2  # Wall thickness
    num_large_rect = 5
    num_large_circ = 5
    num_small_rect = 20
    num_small_circ = 20
    large_rect_size = (1.0, 3.0)  # Width and height range for large rectangles
    large_circ_radius = (0.5, 1.5)  # Radius range for large circles
    small_rect_size = (0.2, 1.0)  # Width and height range for small rectangles
    small_circ_radius = (0.2, 0.6)  # Radius range for small circles
    min_distance = 0.5

class Grid:
    def __init__(self, config: GridCfg):
        """
        初始化网格。

        :param config: 配置对象，包含网格参数。
        """
        self.cfg = config
        self.x_min = config.x_min
        self.x_max = config.x_max
        self.y_min = config.y_min
        self.y_max = config.y_max
        self.grid_size = config.grid_size
        self.grid_inflated_distance = config.grid_inflated_distance

        # 每个网格索引对应一个障碍物集合
        self.grid = defaultdict(set)
        # 每个障碍物对应它所覆盖的所有网格索引
        self.obstacle_map = defaultdict(set)
        
        self.obstacles = []
        
        
    def _in_bounds(self, gx: int, gy: int) -> bool:
        """
        判断网格索引 (gx, gy) 是否在整个网格范围内。
        """
        return 0 <= gx < (self.x_max - self.x_min) / self.grid_size and 0 <= gy < (self.y_max - self.y_min) / self.grid_size
    
    def get_potential_collisions(self, point: Tuple[float, float]) -> List[Obstacle]:
        """
        获取指定点及其邻近网格单元的所有障碍物，增加碰撞检测的全面性。

        :param point: 点的坐标 (x, y)。
        :return: 障碍物列表。
        """
        grid_x, grid_y = self._get_single_grid_coordinate(point[0], point[1])
        neighbors = [
            (grid_x + dx, grid_y + dy)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
        ]
        obstacles = set()
        for cell in neighbors:
            obstacles.update(self.grid.get(cell, []))
        return list(obstacles)
    
    def get_obstacles_near(self, x: float, y: float, radius: float) -> List[Obstacle]:
        """
        获取指定位置和半径内的所有障碍物。

        :param x, y: 中心位置。
        :param radius: 半径。
        :return: 障碍物列表。
        """
        x_min = x - radius
        x_max = x + radius
        y_min = y - radius
        y_max = y + radius

        grid_x_min, grid_x_max, grid_y_min, grid_y_max = self._get_grid_coordinates_range(x_min, y_min, x_max, y_max)

        nearby_obstacles = set()
        for gx in range(grid_x_min, grid_x_max + 1):
            for gy in range(grid_y_min, grid_y_max + 1):
                if self._in_bounds(gx, gy):
                    nearby_obstacles.update(self.grid.get((gx, gy), set()))
        return list(nearby_obstacles)

    def add_obstacle(self, obstacle: Obstacle):
        """
        添加障碍物到网格中。

        :param obstacle: 待添加的障碍物对象。
        """
        # 获取障碍物的(扩展)边界框
        bounding_box = self._get_obstacle_bounding_box(obstacle)
        
        x_min, x_max, y_min, y_max = bounding_box
        x_min -= self.grid_inflated_distance
        x_max += self.grid_inflated_distance
        y_min -= self.grid_inflated_distance
        y_max += self.grid_inflated_distance
        
        # 将该边界框映射到网格索引区间
        grid_x_min, grid_x_max, grid_y_min, grid_y_max = self._get_grid_coordinates_range(x_min, y_min, x_max, y_max)

        # 遍历所有覆盖到的网格单元，将障碍物加入
        for gx in range(grid_x_min, grid_x_max + 1):
            for gy in range(grid_y_min, grid_y_max + 1):
                self.grid[(gx, gy)].add(obstacle)
                self.obstacle_map[obstacle].add((gx, gy))
                # self.obstacles.append(obstacle)
        self.obstacles.append(obstacle)

    def remove_obstacle(self, obstacle: Obstacle):
        """
        从网格中移除障碍物。

        :param obstacle: 待移除的障碍物对象。
        """
        # 先从 obstacle_map 中找到该障碍物占据的所有网格
        grid_cells = self.obstacle_map.get(obstacle, set())
        for cell in grid_cells:
            # 从对应的网格集合中移除
            self.grid[cell].discard(obstacle)
            # 若该网格已无障碍物，则删除该键以保持干净
            if not self.grid[cell]:
                del self.grid[cell]

        # 最后删掉 obstacle_map 里该障碍物的记录
        if obstacle in self.obstacle_map:
            del self.obstacle_map[obstacle]

    def update_obstacle(self, obstacle: Obstacle, new_position: Tuple[float, float]):
        """
        更新障碍物的位置。

        :param obstacle: 待更新的障碍物对象。
        :param new_position: 新的位置坐标 (x, y)。
        """
        # 先移除再添加
        self.remove_obstacle(obstacle)
        obstacle.position = new_position
        self.add_obstacle(obstacle)

    def get_obstacles_in_cell(self, x: float, y: float) -> List[Obstacle]:
        """
        获取指定点所在网格单元的所有障碍物。

        :param x, y: 全局坐标。
        :return: 障碍物列表。
        """
        grid_x, grid_y = self._get_single_grid_coordinate(x, y)
        return list(self.grid.get((grid_x, grid_y), []))
    
    def generate_random_start_end_points(self, num_points, max_distance, min_distance, safety_distance_start=0.25, safety_distance_end=0.185, offset=0.65, rl_or_test=False):
        points = []
        if rl_or_test:
            while len(points) < num_points:
                # 随机选择起点和终点
                start_x = random.uniform(self.x_min, self.x_max)
                start_y = random.uniform(self.y_min, self.y_max)
                
                end_x, end_y = np.inf, np.inf
                
                while end_x < self.x_min + self.grid_inflated_distance or end_x > self.x_max - self.grid_inflated_distance or \
                        end_y < self.y_min + self.grid_inflated_distance or end_y > self.y_max - self.grid_inflated_distance:
                    end_x = start_x + random.uniform(-offset, offset)
                    end_y = start_y + random.uniform(-offset, offset)
                
                # 检查起点和终点是否在障碍物内
                if self.is_in_obstacle(start_x, start_y, safety_distance=safety_distance_start) or \
                self.is_in_obstacle(end_x, end_y, safety_distance=safety_distance_end):
                    continue
                
                # 检查直线距离是否符合要求
                distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
                if min_distance <= distance <= max_distance:
                    points.append(((start_x, start_y), (end_x, end_y)))
        else:
            with tqdm(total=num_points, desc="Generating points") as pbar:
                while len(points) < num_points:
                    # 随机选择起点和终点
                    start_x = random.uniform(self.x_min, self.x_max)
                    start_y = random.uniform(self.y_min, self.y_max)
                    
                    end_x, end_y = np.inf, np.inf
                    
                    while end_x < self.x_min + self.grid_inflated_distance or end_x > self.x_max - self.grid_inflated_distance or \
                            end_y < self.y_min + self.grid_inflated_distance or end_y > self.y_max - self.grid_inflated_distance:
                        end_x = start_x + random.uniform(-offset, offset)
                        end_y = start_y + random.uniform(-offset, offset)
                    
                    # 检查起点和终点是否在障碍物内
                    if self.is_in_obstacle(start_x, start_y, safety_distance=safety_distance_start) or \
                    self.is_in_obstacle(end_x, end_y, safety_distance=safety_distance_end):
                        continue
                    
                    # 检查直线距离是否符合要求
                    distance = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
                    if min_distance <= distance <= max_distance:
                        points.append(((start_x, start_y), (end_x, end_y)))
                        pbar.update(1)
        
        return np.array(points)
    
    def sample_free_points(
        self,
        n: int,
        safety_distance: float = 0.0,
        max_trials_per_point: int = 2000,
    ) -> np.ndarray:
        """随机采样 n 个不落在障碍物中的自由点。

        参数
        ----
        n : int
            需要采样的点数量。
        safety_distance : float, optional
            点到障碍物边缘的最小安全距离；默认为 0（仅要求不在障碍物内部）。
        max_trials_per_point : int, optional
            为每个点可尝试的最大采样次数；超过后若仍失败将抛出异常。

        返回
        ----
        np.ndarray
            形状为 (n, 2) 的坐标数组。
        """
        points = np.empty((n, 2), dtype=float)
        half = self.grid_inflated_distance  # 为了少试无效点，可把边界略收缩
        
        for i in range(n):
            found = False
            for _ in range(max_trials_per_point):
                x = random.uniform(self.x_min + half, self.x_max - half)
                y = random.uniform(self.y_min + half, self.y_max - half)

                if not self.is_in_obstacle(x, y, safety_distance=safety_distance):
                    points[i] = (x, y)
                    found = True
                    break
            if not found:
                raise RuntimeError(
                    f"采样第 {i} 个自由点时在 {max_trials_per_point} 次尝试内均失败；"
                    "请放宽安全距离、减少障碍物密度或提高 max_trials_per_point。"
                )
        return points
    
    def sample_goal_points(
        self,
        robot_positions: np.ndarray,
        yaws: np.ndarray, 
        distance_range: Tuple[float, float] = (0.5, 1.0),
        safety_distance_goal: float = 0.2,
        max_trials: int = 500,
    ) -> np.ndarray:
        """根据多个机器人当前位置批量生成终点。

        参数
        ----
        robot_positions : np.ndarray
            形状为 (N, 2) 的起点坐标。
        distance_range : Tuple[float, float]
            起点与终点之间允许的欧氏距离区间 (min_d, max_d)。
        safety_distance_goal : float
            终点与障碍物之间的安全距离。
        max_trials : int
            为每个机器人尝试采样的最大次数，超过则抛出异常。

        返回
        ----
        np.ndarray
            形状为 (N, 2) 的终点坐标。
        """
        robot_positions = np.asarray(robot_positions, dtype=float)
        if robot_positions.ndim == 1:
            robot_positions = robot_positions.reshape(1, 2)
        num_bots = robot_positions.shape[0]
        goals = np.empty_like(robot_positions)

        min_d, max_d = distance_range
        assert 0 < min_d <= max_d, "distance_range 必须满足 0 < min_d <= max_d"

        for i in range(num_bots):
            sx, sy = robot_positions[i]
            found = False
            for _ in range(max_trials):             
                angle_offset = random.uniform(-math.pi, math.pi)
                theta = yaws[i] + angle_offset
                dist = random.uniform(min_d, max_d)
                gx = sx + dist * math.cos(theta)
                gy = sy + dist * math.sin(theta)

                # 边界检查
                if not (
                    self.x_min + self.cfg.wall_thickness
                    <= gx
                    <= self.x_max - self.cfg.wall_thickness
                    and self.y_min + self.cfg.wall_thickness
                    <= gy
                    <= self.y_max - self.cfg.wall_thickness
                ):
                    # print(f"Robot {i} sampled goal ({gx:.2f}, {gy:.2f}) 超出边界，重新采样。")
                    continue

                # 障碍物碰撞检查
                if self.is_in_obstacle(gx, gy, safety_distance=safety_distance_goal):
                    # print(f"Robot {i} sampled goal ({gx:.2f}, {gy:.2f}) 在障碍物内，重新采样。")
                    continue

                goals[i] = (gx, gy)
                found = True
                break

            if not found:
                raise RuntimeError(
                    f"无法为机器人 {i} 在 {max_trials} 次尝试内采样到合法终点，" "请尝试放宽 distance_range 或减少障碍物密度。"
                )
        return goals
    
    def generate_obstacles(self) -> Tuple[List[RectangleObstacle], List[CircleObstacle]]:
        """
        生成场景中的所有障碍物，包括墙壁、大型障碍物和小型障碍物
        """
        seed = 3407
        random.seed(seed)
        np.random.seed(seed)
        
        # 生成墙壁
        print("Generating walls...")
        walls = self.create_walls()
        
        # 生成大型障碍物
        print("Generating large obstacles...")
        large_rects, large_circs = self.generate_large_obstacles(
            num_rect=self.cfg.num_large_rect,
            num_circ=self.cfg.num_large_circ,
            large_rect_size=self.cfg.large_rect_size,
            large_circ_radius=self.cfg.large_circ_radius
        )
        
        # 生成小型障碍物
        print("Generating small obstacles...")
        small_rects, small_circs = self.generate_small_obstacles(
            num_rect=self.cfg.num_small_rect,
            num_circ=self.cfg.num_small_circ,
            small_rect_size=self.cfg.small_rect_size,
            small_circ_radius=self.cfg.small_circ_radius
        )
        
        return walls + large_rects + large_circs + small_rects + small_circs
    
    def create_walls(self) -> List[RectangleObstacle]:
        """
        创建场景四周的墙壁
        """
        walls = [
            RectangleObstacle(position=((self.x_min + self.x_max) / 2, self.y_min + self.cfg.wall_thickness / 2),
                              width=self.x_max - self.x_min, height=self.cfg.wall_thickness),  # Bottom wall
            RectangleObstacle(position=((self.x_min + self.x_max) / 2, self.y_max - self.cfg.wall_thickness / 2),
                              width=self.x_max - self.x_min, height=self.cfg.wall_thickness),  # Top wall
            RectangleObstacle(position=(self.x_min + self.cfg.wall_thickness / 2, (self.y_min + self.y_max) / 2),
                              width=self.cfg.wall_thickness, height=self.y_max - self.y_min),  # Left wall
            RectangleObstacle(position=(self.x_max - self.cfg.wall_thickness / 2, (self.y_min + self.y_max) / 2),
                              width=self.cfg.wall_thickness, height=self.y_max - self.y_min),  # Right wall
        ]
        # Add each wall to the grid
        for wall in walls:
            self.add_obstacle(wall)

        return walls
    
    def generate_large_obstacles(self, num_rect: int, num_circ: int, large_rect_size: Tuple[float, float], large_circ_radius: Tuple[float, float]) -> Tuple[List[RectangleObstacle], List[CircleObstacle]]:
        """
        生成大型的矩形和圆形障碍物，加入最小间距和碰撞检测。
        """
        large_rects = []
        large_circs = []
        attempts = 0
        max_attempts = 2000 * self.cfg.num_large_rect + 2000 * self.cfg.num_large_circ

        # 生成矩形
        while len(large_rects) < num_rect and attempts < max_attempts:
            attempts += 1
            width = random.uniform(*large_rect_size)
            height = random.uniform(*large_rect_size)
            position = np.array(
                [random.uniform(self.x_min + width / 2, self.x_max - width / 2),
                random.uniform(self.y_min + height / 2, self.y_max - height / 2)]
            )
            rect = RectangleObstacle(position=position, width=width, height=height)

            # 检查该位置是否与其他障碍物碰撞并满足最小间距
            if self.is_in_obstacle(position[0], position[1], safety_distance=self.cfg.min_distance + rect.get_radius()):
                continue

            self.add_obstacle(rect)
            large_rects.append(rect)

        if attempts >= max_attempts:
            print("Warning: failed to generate all large rectangular obstacles.")

        attempts = 0
        # 生成圆形
        while len(large_circs) < num_circ and attempts < max_attempts:
            attempts += 1
            radius = random.uniform(*large_circ_radius)
            position = np.array(
                [random.uniform(self.x_min + radius, self.x_max - radius),
                random.uniform(self.y_min + radius, self.y_max - radius)]
            )
            circ = CircleObstacle(position=position, radius=radius)

            # 检查该位置是否与其他障碍物碰撞并满足最小间距
            if self.is_in_obstacle(position[0], position[1], safety_distance=self.cfg.min_distance + circ.get_radius()):
                continue

            self.add_obstacle(circ)
            large_circs.append(circ)

        if attempts >= max_attempts:
            print("Warning: failed to generate all large circular obstacles.")

        return large_rects, large_circs

    def generate_small_obstacles(self, num_rect: int, num_circ: int, small_rect_size: Tuple[float, float], small_circ_radius: Tuple[float, float]) -> Tuple[List[RectangleObstacle], List[CircleObstacle]]:
        """
        在场景内随机生成多个小型矩形和圆形障碍物，加入最小间距和碰撞检测。
        """
        small_rects = []
        small_circs = []
        attempts = 0
        max_attempts = 10000 * self.cfg.num_small_rect + 10000 * self.cfg.num_small_circ

        # 生成小型矩形
        while len(small_rects) < num_rect and attempts < max_attempts:
            attempts += 1
            width = random.uniform(*small_rect_size)
            height = random.uniform(*small_rect_size)
            position = np.array(
                [random.uniform(self.x_min + width / 2, self.x_max - width / 2),
                random.uniform(self.y_min + height / 2, self.y_max - height / 2)]
            )
            rect = RectangleObstacle(position=position, width=width, height=height)

            # 检查该位置是否与其他障碍物碰撞并满足最小间距
            if self.is_in_obstacle(position[0], position[1], safety_distance=self.cfg.min_distance + rect.get_radius()):
                continue

            self.add_obstacle(rect)
            small_rects.append(rect)

        if attempts >= max_attempts:
            print("Warning: failed to generate all small rectangular obstacles.")

        attempts = 0
        # 生成小型圆形
        while len(small_circs) < num_circ and attempts < max_attempts:
            attempts += 1
            radius = random.uniform(*small_circ_radius)
            position = np.array(
                [random.uniform(self.x_min + radius, self.x_max - radius),
                random.uniform(self.y_min + radius, self.y_max - radius)]
            )
            circ = CircleObstacle(position=position, radius=radius)

            # 检查该位置是否与其他障碍物碰撞并满足最小间距
            if self.is_in_obstacle(position[0], position[1], safety_distance=self.cfg.min_distance + circ.get_radius()):
                continue

            self.add_obstacle(circ)
            small_circs.append(circ)

        if attempts >= max_attempts:
            print("Warning: failed to generate all small circular obstacles.")

        return small_rects, small_circs

    def _get_obstacle_bounding_box(self, obstacle: Obstacle) -> Tuple[float, float, float, float]:
        """
        获取障碍物的扩展边界框，考虑安全距离。

        :param obstacle: 障碍物对象。
        :return: 扩展后的边界框 (x_min, x_max, y_min, y_max)。
        """
        x_min, x_max, y_min, y_max = obstacle.get_bounding_box()

        return x_min, x_max, y_min, y_max

    def _get_single_grid_coordinate(self, x: float, y: float) -> Tuple[int, int]:
        """
        将 (x, y) 坐标转换为网格索引 (gx, gy)。

        :param x, y: 全局坐标。
        :return: 网格索引 (gx, gy)。
        """
        grid_x = int(math.floor((x - self.x_min) / self.grid_size))
        grid_y = int(math.floor((y - self.y_min) / self.grid_size))
        return grid_x, grid_y

    def _get_grid_coordinates_range(self,
                                    x_min: float, y_min: float,
                                    x_max: float, y_max: float
                                    ) -> Tuple[int, int, int, int]:
        """
        获取覆盖指定范围的网格索引范围。

        :param x_min, y_min, x_max, y_max: 全局坐标范围。
        :return: 网格索引范围 (grid_x_min, grid_x_max, grid_y_min, grid_y_max)。
        """
        grid_x_min = int(math.floor((x_min - self.x_min) / self.grid_size))
        grid_x_max = int(math.floor((x_max - self.x_min) / self.grid_size))
        grid_y_min = int(math.floor((y_min - self.y_min) / self.grid_size))
        grid_y_max = int(math.floor((y_max - self.y_min) / self.grid_size))

        return grid_x_min, grid_x_max, grid_y_min, grid_y_max

    def clear(self):
        """
        清空网格中的所有障碍物。
        """
        self.grid.clear()
        self.obstacle_map.clear()

    def is_in_obstacle(self, x: float, y: float, safety_distance: float = 0.0) -> bool:
        """
        判断点是否位于障碍物内，考虑安全距离。

        :param x, y: 点的全局坐标。
        :param safety_distance: 安全距离。
        :return: 是否在障碍物内的布尔值。
        """
        obstacles_in_cell = self.get_potential_collisions((x, y))
        for obstacle in obstacles_in_cell:
            if obstacle.is_collision((x, y), safety_distance=safety_distance):
                return True
        return False

    def query_range(self, x_min: float, y_min: float, x_max: float, y_max: float) -> List[Obstacle]:
        """
        查询指定范围内的所有障碍物。

        :param x_min, y_min, x_max, y_max: 查询范围的全局坐标。
        :return: 障碍物列表。
        """
        grid_x_min, grid_x_max, grid_y_min, grid_y_max = self._get_grid_coordinates_range(
            x_min, y_min, x_max, y_max
        )
        obstacles = set()
        for gx in range(grid_x_min, grid_x_max + 1):
            for gy in range(grid_y_min, grid_y_max + 1):
                obstacles.update(self.grid.get((gx, gy), set()))
        return list(obstacles)
