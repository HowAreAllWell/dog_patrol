import os
import pickle
import trimesh
import numpy as np

from trimesh.transformations import translation_matrix

THICKNESS = 1.5

def generate_obj(obstacles, ouput_dir="./outputs"):
    meshes = []

    for obs in obstacles:
        if hasattr(obs, 'width') and hasattr(obs, 'height'):
            # RectangleObstacle
            cx, cy = obs.position
            w = obs.width
            h = obs.height

            # 创建箱体模型: extents = [width, height, thickness]
            box = trimesh.creation.box(extents=[w, h, THICKNESS])

            # 平移到指定位置 (cx, cy, 0)
            transform = translation_matrix([cx, cy, THICKNESS / 2.0])
            box.apply_transform(transform)
            meshes.append(box)

        elif hasattr(obs, 'radius'):
            # CircleObstacle
            cx, cy = obs.position
            r = obs.radius

            # 创建圆柱模型: radius = r, height = thickness
            cylinder = trimesh.creation.cylinder(radius=r, height=THICKNESS)

            # 平移到指定位置 (cx, cy, 0)
            transform = translation_matrix([cx, cy, THICKNESS / 2.0])
            cylinder.apply_transform(transform)
            meshes.append(cylinder)

        else:
            # 未定义类型的Obstacle，可根据需要添加其他类型的处理逻辑
            pass


    # 设置输出的 OBJ 文件路径
    output_obj_path = os.path.join(ouput_dir, "obstacles.obj")

    if not meshes:
        print("No obstacle model found")
    else:
        # 合并所有网格并导出为 OBJ 文件
        combined_mesh = trimesh.util.concatenate(meshes)
        combined_mesh.export(output_obj_path)
        print(f"Obstacle model exported to '{output_obj_path}'")
