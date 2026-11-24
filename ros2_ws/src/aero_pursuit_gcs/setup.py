import os

from setuptools import setup

package_name = 'aero_pursuit_gcs'


def web_files():
    """(destination, [files]) for each folder of web/, keeping the structure."""
    out = []
    for root, _, files in os.walk('web'):
        if files:
            out.append((os.path.join('share', package_name, root),
                        [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + web_files(),
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Luccas Marchetti',
    maintainer_email='luccas.marchetti@gmail.com',
    description='Estação de solo web do aero_pursuit',
    license='MIT',
    entry_points={'console_scripts': ['gcs_server = aero_pursuit_gcs.main:main']},
)
