from setuptools import find_packages, setup

package_name = 'catch_the_turtle'

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
    maintainer='zsoltkebel',
    maintainer_email='25386639+zsoltkebel@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'client = catch_the_turtle.catch_the_turtle:main',
            'pos = catch_the_turtle.turtle_pos_checker:main',
            'mover = catch_the_turtle.autopilot:main'
        ],
    },
)
