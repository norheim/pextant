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


def fig2data(fig):
    """
    @brief Convert a Matplotlib figure to a 4D numpy array with RGBA channels and return it
    @param fig a matplotlib figure
    @return a numpy 3D array of RGBA values
    """
    # draw the renderer
    fig.canvas.draw()

    # Get the RGBA buffer from the figure
    w, h = fig.canvas.get_width_height()
    buf = np.fromstring(fig.canvas.tostring_argb(), dtype=np.uint8)
    buf.shape = (w, h, 4)

    # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to have it in RGBA mode
    buf = np.roll(buf, 3, axis=2)
    return buf


from PIL import Image


def fig2img(fig):
    """
    @brief Convert a Matplotlib figure to a PIL Image in RGBA format and return it
    @param fig a matplotlib figure
    @return a Python Imaging Library ( PIL ) image
    """
    # put the figure pixmap into a numpy array
    buf = fig2data(fig)
    w, h, d = buf.shape
    return Image.frombytes("RGBA", (w, h), buf.tostring())