from pathlib import Path
from cryptography.fernet import Fernet


PROJECT_ROOT = Path(__file__).resolve().parent
KEY_PATH = PROJECT_ROOT / "json_fernet.key"
ACUPOINT_CONFIG_JSON_FILES = (
    "acu_config/acupoints_config.json",
)


def load_or_create_key(key_path: Path) -> bytes:
    # 密钥固定复用，避免每次重生成导致历史 .enc 无法解密
    if key_path.exists():
        key = key_path.read_bytes().strip()
        if not key:
            raise ValueError(f"密钥文件为空: {key_path}")
        print(f"复用已有密钥: {key_path.name}")
        return key

    key = Fernet.generate_key()
    key_path.write_bytes(key)
    print(f"已生成新密钥: {key_path.name}")
    return key


def build_enc_name(json_path: Path) -> str:
    return f"{json_path.stem}.enc"


def get_acupoint_json_files() -> list[Path]:
    # 仅处理穴位配置文件，避免误加密项目内其他 json（如编辑器配置）
    json_files = [PROJECT_ROOT / file_name for file_name in ACUPOINT_CONFIG_JSON_FILES]
    missing_files = [path.name for path in json_files if not path.is_file()]
    if missing_files:
        missing_desc = ", ".join(missing_files)
        raise FileNotFoundError(f"以下穴位配置文件不存在: {missing_desc}")
    return json_files


def encrypt_all_json_files() -> None:
    key = load_or_create_key(KEY_PATH)
    cipher = Fernet(key)

    json_files = get_acupoint_json_files()
    if not json_files:
        print("未找到穴位配置 json 文件")
        return

    print(f"共发现 {len(json_files)} 个穴位配置 json 文件，开始加密...")
    for json_path in json_files:
        encrypted_bytes = cipher.encrypt(json_path.read_bytes())
        out_path = PROJECT_ROOT / build_enc_name(json_path)
        out_path.write_bytes(encrypted_bytes)
        print(f"已生成: {out_path.name}")


if __name__ == "__main__":
    encrypt_all_json_files()
