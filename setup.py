from setuptools import find_packages, setup

package_name = 'pong_game'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
# data_files.append(('share/' + package_name + '/five_python', ['five_python/__init__.py']))
# data_files.append(('share/' + package_name + '/five_python/fribe', ['five_python/fribe/__init__.py', 'five_python/fribe/loader.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/main.2.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/robot.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

submodule_grammar = 'fribe/grammars/simple'
packages = find_packages(exclude=['test'])
packages.append(submodule_grammar)

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
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
            'keyboard_control_publisher = pong_game.keyboard_control_publisher:main',
            'camera_feed_publisher = pong_game.camera_feed_publisher:main',
            'behaviour_control_publisher = pong_game.behaviour_control_publisher:main',
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
