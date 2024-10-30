import hydra
from omegaconf import OmegaConf

import pickle
from sklearn.linear_model import LogisticRegression
from robot.controller import Controller

TASK_GDRIVE_ID = {
    "handover": "" #TODO: upload the model to gdrive id for this
}

# class WrapperPolicy(nn.Module):
#     def __init__(self, model, input_size):
#         super().__init__()
#         self.model = model
#         self.input_size = input_size
#         self.xy_mask = torch.tensor([0, 1, 3, 4, 6, 7, 9, 10, 12, 13])

#     def step(self, data, *args, **kwargs):
#         model_out = self.model(data[..., self.xy_mask])
#         Yhat = torch.sigmoid(model_out)
#         return Yhat > 0.5 # gripper open/close

#     def reset(self):
#         pass

def init_model(cfg: OmegaConf):
    with open(f"checkpoints/{cfg.model_pkl}") as f:
        model = pickle.load(f)

    return model


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