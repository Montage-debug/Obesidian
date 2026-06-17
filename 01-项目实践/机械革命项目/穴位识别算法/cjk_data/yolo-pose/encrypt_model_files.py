from pathlib import Path
from cryptography.fernet import Fernet


PROJECT_ROOT = Path(__file__).resolve().parent
KEY_PATH = PROJECT_ROOT / "json_fernet.key"
MODEL_FILES = (
    "yolo11x-pose.pt",
    "yolo11n-abdomen_navel_only.pt",
)


def load_key(key_path: Path) -> bytes:
    # 复用现有 json 加密密钥，确保模型与配置文件可统一密钥体系管理
    if not key_path.is_file():
        raise FileNotFoundError(f"找不到密钥文件: {key_path}")

    key = key_path.read_bytes().strip()
    if not key:
        raise ValueError(f"密钥文件为空: {key_path}")
    return key


def build_enc_name(model_path: Path) -> str:
    return f"{model_path.stem}.enc"


def get_model_files() -> list[Path]:
    # 仅处理指定模型文件，避免误加密仓库中的其他 .pt 文件
    model_paths = [PROJECT_ROOT / file_name for file_name in MODEL_FILES]
    missing_files = [path.name for path in model_paths if not path.is_file()]
    if missing_files:
        missing_desc = ", ".join(missing_files)
        raise FileNotFoundError(f"以下模型文件不存在: {missing_desc}")
    return model_paths


def encrypt_model_files() -> None:
    key = load_key(KEY_PATH)
    cipher = Fernet(key)
    model_paths = get_model_files()

    print(f"共发现 {len(model_paths)} 个模型文件，开始加密...")
    for model_path in model_paths:
        encrypted_bytes = cipher.encrypt(model_path.read_bytes())
        out_path = PROJECT_ROOT / build_enc_name(model_path)
        out_path.write_bytes(encrypted_bytes)
        print(f"已生成: {out_path.name}")


if __name__ == "__main__":
    encrypt_model_files()
