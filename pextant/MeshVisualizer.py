import MeshModel
from ipywidgets import interact
from bokeh.io import push_notebook, show, output_notebook
from bokeh.plotting import figure
import numpy as np
import matplotlib.pyplot as plt

class MeshViz:
    def __init__(self, notebook=False):
        self.notebook = notebook
        if notebook:
            output_notebook()

    def viz(self, mesh, x=None, y=None):
        dh, dw = mesh.shape
        size = max(dh, dw)
        self.mesh = mesh
        self.dh = dh
        self.dw = dw
        self.p = figure(webgl=True, title="MD2", x_axis_label='x', y_axis_label='y', x_range=[0, size], y_range=[0, size])
        self.p.image(image=[mesh[::-1, :]], x=0, y=0, dw=dw, dh=dh, palette="Spectral11")
        if x:
            self.p.circle(x, self.dh - np.array(y), fill_color="yellow", size=10)
        if self.notebook:
            self.t = show(self.p, notebook_handle = self.notebook)
        else:
            self.t = show(self.p)

    def vizpoints(self, x, y):
        print(x)
        self.p.circle(y, self.dh - np.array(x), fill_color="yellow", size=10)
        push_notebook(handle=self.t)

class MeshVizM:
    def __init__(self):
        pass

    def viz(self, mesh, x=None, y=None):
        plt.matshow(mesh)
        plt.show()


if __name__ == '__main__':
    MeshViz().viz(np.zeros([4,4]))