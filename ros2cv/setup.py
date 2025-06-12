from setuptools import find_packages, setup

package_name = 'ros2cv'

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
    maintainer='craibuntu',
    maintainer_email='bilal.mscss21@iba-suk.edu.pk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'face_detection = ros2cv.face_detection:main',
            'face_mash = ros2cv.face_mash:main',
            'fingers_detection = ros2cv.fingers_detection:main',
            'hand_detection = ros2cv.hand_detection:main',
            'pose_detection = ros2cv.pose_detection:main',
            'object_detection = ros2cv.object_detection:main',
            'publish_image = ros2cv.publish_image:main',
            'view_image = ros2cv.view_image:main',
        ],
    },
)
