from distutils.core import setup
from distutils.core import setup
setup(
	name='pextant',
    packages=['pextant'],
    version='1.0',
    author='Johannes Norheim',
    author_email='norheim@mit.edu',
    url='http://pypi.python.org/pypi/pextant/',
    download_url = 'https://github.com/norheim/pextant/archive/1.0.tar.gz',
    keywords = ['testing', 'logging', 'example'], # arbitrary keywords
    classifiers = [],
    license='The MIT License: http://www.opensource.org/licenses/mit-license.php',
    description='Python version of SEXTANT pathfinding tool',
    long_description=open('README.txt').read(),
    install_requires=[
        'numpy >= 1.9',
		'osgeo >= 2.0.0',
		'pyproj >= 1.9',
		'shapely >= 1.5'
    ],
)