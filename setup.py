from setuptools import find_packages, setup

package_name = 'log_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kohei',
    maintainer_email='kohei@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'log_node = log_node.log_node:main',
            'log_node2 = log_node.log_node2:main',
            'log_node3 = log_node.log_node3:main',
            'image = log_node.image:main',
            'aeb_full = log_node.aeb_full:main',
            'aeb_full2 = log_node.aeb_full2:main',
            'aeb_full3 = log_node.aeb_full3:main',
            'aeb_full4 = log_node.aeb_full4:main',
            'aeb_full5 = log_node.aeb_full5:main'
        ],
    },
)
