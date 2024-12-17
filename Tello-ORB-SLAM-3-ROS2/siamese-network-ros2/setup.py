from setuptools import setup

package_name = 'similarity_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource/model', [
            'resource/model/model.pb']),
        ('share/' + package_name + '/resource/model/variables', [
            'resource/model/variables/variables.index',
            'resource/model/variables/variables.data-00000-of-00001']),
        ('share/' + package_name + '/resource', ['resource/reference.jpg'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='ROS2 package to detect similar objects using a Siamese model',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'similarity_detector_node = similarity_detector.similarity_detector_node:main'
        ],
    },
)
