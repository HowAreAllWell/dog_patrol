from RL2Path.model.custom_cnn_v0 import CustomCNN as CustomCNN_v0
from RL2Path.model.custom_cnn_v1 import CustomCNN as CustomCNN_v1
from RL2Path.model.custom_cnn_mini_bn import CustomCNN as CustomCNN_mini_bn
from RL2Path.model.custom_tcnn_v0 import CustomCNN as CustomCNN_tcnn_v0
from RL2Path.model.custom_tcnn_v1 import CustomCNN as CustomCNN_tcnn_v1
from RL2Path.model.custom_tcnn_v2 import CustomCNN as CustomCNN_tcnn_v2
from RL2Path.model.custom_tcnn_v3 import CustomCNN as CustomCNN_tcnn_v3
from RL2Path.model.custom_cnn_voxel import CustomCNN as CustomCNN_voxel
from RL2Path.model.custom_tcnn_mini import MiniCustomCNN

def get_model(model_version):
    if model_version == 'v0':
        return CustomCNN_v0
    elif model_version == 'v1':
        return CustomCNN_v1
    elif model_version == 'mini_bn':
        return CustomCNN_mini_bn
    elif model_version == 'tcnn_v0':
        return CustomCNN_tcnn_v0
    elif model_version == "voxel":
        return CustomCNN_voxel
    elif model_version == "tcnn_mini":
        return MiniCustomCNN
    elif model_version == "tcnn_v1":
        return CustomCNN_tcnn_v1
    elif model_version == "tcnn_v2":
        return CustomCNN_tcnn_v2
    elif model_version == "tcnn_v3":
        return CustomCNN_tcnn_v3
    else:
        return CustomCNN_v0