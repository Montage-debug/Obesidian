from pathlib import Path

from setuptools import find_packages, setup

package_name = 'visual_recognition'


def collect_package_files(subdir: str):
    package_root = Path(__file__).resolve().parent / package_name
    target_dir = package_root / subdir
    if not target_dir.exists():
        return []
    return [
        str(path.relative_to(package_root))
        for path in target_dir.rglob('*')
        if path.is_file()
    ]


asset_files = (
    collect_package_files('acu_config')
    + collect_package_files('models')
    + collect_package_files('third_tool')
)

# 查找所有包，包括 visual_recognition 下的 pyarmor_runtime 子包
# 排除 test 和 visual_recognition_original（PyArmor 混淆时的备份目录）
packages = find_packages(exclude=[
    'test',
    'test.*',
    'visual_recognition_original',
    'visual_recognition_original.*',
])

setup(
    name=package_name,
    version='0.0.0',
    packages=packages,
    package_data={
        'visual_recognition': [
            '*.json',
            '*.pt',
            '*.enc',
            *asset_files,
        ],
        # PyArmor 运行时文件（使用 -i 参数时，运行时包在 visual_recognition 内部）
        # 注意：setuptools 不支持包名中的通配符，必须显式列出
        'visual_recognition.pyarmor_runtime_011314': ['*'],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wlzc',
    maintainer_email='2389954298@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'visual_recognize = visual_recognition.visual_recognize:main',
        ],
    },
    options={
        'build_scripts': {
            'executable': '/usr/bin/env python3',
        },
    },
)
