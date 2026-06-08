from collections import defaultdict
from pathlib import Path

from setuptools import find_packages, setup

package_name = 'voice_asr_node'


def collect_data_files(source_dir: str, install_subdir: str):
    package_root = Path(__file__).resolve().parent
    source_root = package_root / source_dir
    if not source_root.exists():
        return []

    collected = defaultdict(list)
    for path in source_root.rglob('*'):
        if not path.is_file():
            continue
        relative_path = path.relative_to(source_root)
        install_dir = Path('share') / package_name / install_subdir / relative_path.parent
        collected[str(install_dir)].append(str(path.relative_to(package_root)))

    return sorted(collected.items())

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test', 'test.*', 'voice_asr_node_original', 'voice_asr_node_original.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/voice_asr.launch.py']),
        *collect_data_files('models', 'models'),
    ],
    package_data={
        # PyArmor 运行时文件（使用 -i 参数时，运行时包在 voice_asr_node 内部）
        # 注意：setuptools 不支持包名中的通配符，必须显式列出
        'voice_asr_node.pyarmor_runtime_011314': ['*'],
    },
    include_package_data=True,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wlzc',
    maintainer_email='todo@example.com',
    description='按摩机器人语音 ROS2 功能包（豆包实时对话 + KWS）',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
        'speaker': ['onnxruntime>=1.16.0'],
    },
    entry_points={
        'console_scripts': [
            'voice_asr_node = voice_asr_node.nodes.voice_asr_node:main',
            'music_player_node = voice_asr_node.nodes.music_player_node:main',
            'doubao_dialog_node = voice_asr_node.nodes.doubao_dialog_node:main',
            'register_speaker = voice_asr_node.tools.register_speaker:main',
        ],
    },
    options={
        'build_scripts': {
            'executable': '/usr/bin/env python3',
        },
    },
)
