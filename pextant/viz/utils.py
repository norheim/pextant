import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.colors import LightSource
ls = LightSource(azdeg=315, altdeg=45)
import numpy as np

spectral = plt.get_cmap('Spectral_r')
spectraln = LinearSegmentedColormap.from_list("name", spectral(np.linspace(0, 1, 11)), N=11)

def hillshade(environment, weight=10):
    res = environment.resolution
    exaggeration = res*weight
    plt.imshow(ls.hillshade(environment.dataset_unmasked,
                            vert_exag=exaggeration, dx=res, dy=res), cmap='gray')