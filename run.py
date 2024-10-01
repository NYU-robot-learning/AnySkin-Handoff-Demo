import os

import torch
from torch import nn
import hydra
from omegaconf import OmegaConf
import gdown

from robot.controller import Controller

TASK_GDRIVE_ID = {
    "handover": "" #TODO: upload the model to gdrive id for this
}

class WrapperPolicy(nn.Module):
    def __init__(self, model, input_size):
        super().__init__()
        self.model = model
        self.input_size = input_size

    def step(self, data, *args, **kwargs):
        model_out = self.model(data[..., :self.input_size])
        Yhat = torch.sigmoid(model_out)
        return Yhat > 0.5 # gripper open/close

    def reset(self):
        pass

def get_model_weight_pth(task_name):
    if task_name not in TASK_GDRIVE_ID:
        raise ValueError(f"Task \'{task_name}\' is invalid. Please choose from {TASK_GDRIVE_ID.keys()}.")
    task_dir = f"checkpoints/{task_name}"
    if not os.path.exists(f"{task_dir}/checkpoint.pt"):
        os.makedirs(task_dir)
        gdrive_id = TASK_GDRIVE_ID[task_name]
        url = f'https://drive.google.com/uc?id={gdrive_id}'
        output_path = f"{task_dir}/checkpoint.pt"
        gdown.download(url, output_path, quiet=False)

    return f"{task_dir}/checkpoint.pt"

def init_model(cfg: OmegaConf):
    model = hydra.utils.instantiate(cfg.model)
    model = model.to(cfg.device)

    model_weight_pth = f"checkpoints/handover/{cfg.get('model_weight_pth')}.pt"

    if model_weight_pth is None:
        model_weight_pth = get_model_weight_pth(task_name=cfg.task)

    checkpoint = torch.load(model_weight_pth, weights_only=True, map_location=cfg.device)
    model.load_state_dict(checkpoint)

    model_parameters = sum(p.numel() for p in model.parameters() if p.requires_grad)
    # print model params in millions with %.2f
    print(f"Model parameters: {model_parameters / 1e6:.2f}M")
    policy = WrapperPolicy(model, input_size=cfg.model.input_size)

    return policy


def run(cfg: OmegaConf):
    model = init_model(cfg)
    dict_cfg = OmegaConf.to_container(cfg, resolve=True)

    controller = Controller(cfg=dict_cfg)
    controller.setup_model(model)
    controller.run()


@hydra.main(version_base="1.2", config_name="run_handover_demo", config_path="configs")
def main(cfg: OmegaConf):
    run(cfg)


if __name__ == "__main__":
    main()