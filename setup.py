from setuptools import find_packages, setup

package_name = 'pong_game'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name+"/"]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/main.1.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/robot.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibrahim',
    maintainer_email='ibrahimdevoloper@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = pong_game.my_robot_driver:main',
        ],
    },
)

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='ibrahim',
#     maintainer_email='ibrahimdevoloper@gmail.com',
#     description='TODO: Package description',
#     license='Apache-2.0',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'my_robot_driver = pong_game.my_robot_driver:main'
#         ],
#     },
# )
