from distutils.core import setup

setup(
	name='Pextant',
    version='0.1.1',
    author='Kevin Lu',
    author_email='kezilu@mit.edu',
    packages=['pextant'],
    url='http://pypi.python.org/pypi/Pextant/',
    license='The MIT License: http://www.opensource.org/licenses/mit-license.php',
    description='Python version of SEXTANT pathfinding tool',
    long_description=open('README.txt').read(),
    install_requires=[
        'numpy >= 1.9',
		'osgeo >= 2.0.0',
		'pyproj >= 1.9'
    ],
)