from setuptools import setup
setup(name='hw_insight', version='0.1.0', packages=['hw_insight'],
 data_files=[('share/ament_index/resource_index/packages',['resource/hw_insight']),('share/hw_insight',['package.xml'])],
 install_requires=['setuptools'], zip_safe=True,
 description='EGO PositionCommand to PX4 OFFBOARD bridge',
 entry_points={'console_scripts':['position_cmd_to_px4 = hw_insight.position_cmd_to_px4:main']})
