from setuptools import find_packages, setup

package_name = 'action'

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
    maintainer='ubuntu',
    maintainer_email='kai.yang@x-humanoid.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tiangong=action.tiangong:main',
            'tiangong2=action.tiangong2:main',
            'tiangong3=action.tiangong3:main',
            'tiangong3_test=action.tiangong3_test:main',
            'huishou=action.huishou:main',
            'test_hand=action.test_hand:main'

        ],
    },
)
