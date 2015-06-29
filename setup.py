from distutils.core import setup

setup(
    name='Pextant',
    version='0.1.0',
    author='Kevin Z. Lu',
    author_email='kezilu@mit.edu',
    packages=['sextant'],
    url='http://pypi.python.org/pypi/Sextaant/',
    license='LICENSE.txt',
    description='SEXTANT tool for path planning. Created for the NASA BASALT project.',
    long_description=
	'''
	Currently named Pextant for python-sextant since there is already a package named sextant
	
	Filler text move along
	'''
    install_requires=[
        "numpy >= 1.9",
		"utm >= 0.4.0",
		"gdal >= 2.0.0"
    ],
)