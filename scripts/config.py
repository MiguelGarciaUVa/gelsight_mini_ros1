import json
from dataclasses import dataclass, asdict, replace
from pathlib import Path
from typing import Optional
from utilities.logger import log_message


def get_absolute_path(config_path: str) -> Path:
    return Path(config_path).expanduser().resolve()


@dataclass
class ConfigModel:
    default_camera_index: int = 1
    camera_width: int = 320
    camera_height: int = 240
    second_camera_width: int = 320
    second_camera_height: int = 240
    border_fraction: float = 0.15
    marker_mask_min: int = 0
    marker_mask_max: int = 70
    pointcloud_enabled: bool = True
    pointcloud_window_scale: float = 3.0
    cv_image_stack_scale: float = 1.5
    nn_model_path: str = "./models/nnmini.pt"
    cmap_txt_path: str = "./cmap.txt"
    cmap_in_BGR_format: bool = True
    use_gpu: bool = True


default_config = ConfigModel()


class GSConfig:
    def __init__(self, config_path: Optional[str] = None):
        self.config_path = get_absolute_path(config_path) if config_path else None
        self.config: ConfigModel = default_config

        if self.config_path and self.config_path.is_file():
            self.load_config()
        else:
            log_message("No path for config provided. Using default configuration.")

    def load_config(self):
        if self.config_path is None:
            self.config = default_config
            log_message("No config path. Using default configuration.")
            return

        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                data = json.load(f)

            # Solo actualiza campos existentes; ignora extras
            base = asdict(default_config)
            for k, v in data.items():
                if k in base:
                    base[k] = v

            self.config = ConfigModel(**base)
            log_message(f"Loaded config data from file: {self.config_path}.")

        except Exception as e:
            log_message(f"Warning: Invalid config file. Using default configuration. Error: {e}")
            self.config = default_config

    def save_config(self, save_path: Optional[str] = None):
        path = get_absolute_path(save_path) if save_path else self.config_path

        if not path:
            log_message("No valid path provided for saving configuration. Skipping...")
            return

        with open(path, "w", encoding="utf-8") as f:
            json.dump(asdict(self.config), f, indent=4)

        log_message(f"Configuration saved to {path}")

    def reset_to_default(self):
        self.config = replace(default_config)
        log_message("Configuration reset to default.")


if __name__ == "__main__":
    gs_config = GSConfig()
    gs_config.load_config()
    gs_config.save_config("default_config.json")

