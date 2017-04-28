import numpy as np
import matplotlib.pyplot as mpl

def colormap(N, altitude_lower_bound, altitude_upper_bound):
    altitude_linspaced = np.linspace(altitude_lower_bound, altitude_upper_bound, N)
    zero_to_one = np.linspace(0,1,N)
    colors = [
        [int(r), int(g), int(b)] for r, g, b, _ in 255*mpl.cm.Spectral_r(zero_to_one)
    ]

    color_table = [
        [int(altitude_linspaced[idx])] + colors[idx] for idx in range(N)
    ]
    color_string = ''
    for i in range(N):
        color_string += '{0} {1} {2} {3} \n'.format(*color_table[i])

    with open("maps/colormap.txt", "w") as text_file:
        text_file.write(color_string)
