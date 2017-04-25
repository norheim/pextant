import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
from scipy import interpolate
from pextant.lib.geoshapely import GeoPolygon, LAT_LONG, UTM, Cartesian, XY
from pextant.EnvironmentalModel import GDALMesh
from pextant.analysis.loadWaypoints import JSONloader
# Found manually doing using import gps notebook
traverses = {
    'MD2': {
        'name': '20161108A_EV1',
        'json': 'HI_08Nov16_MD2_A',
        'data': np.array([[4509,5088],[5937,6516],[10850,11030],[13530,14110]])},
    'MD3': {
        'name': '20161109A_EV1',
        'json': 'HI_09Nov16_MD3_A',
        'data':np.array([[3400,4660],[5010,5500],[18100,18393]]),
    },
    'MD4': {
        'name': '20161110A_EV1',
        'json': 'HI_10Nov16_MD4_A',
        'data': np.array([[2840,4034],[13150,13829]]),
    },
    'MD5': {
        'name': '20161111A_EV1',
        'json': 'HI_11Nov16_MD5_A',
        'data': np.array([[3500,5000]])
    },
    'MD6': {
        'name':  '20161112A_EV1',
        'json': 'HI_12Nov16_MD6_A',
        'data': np.array([[3990,5500]])},
    'MD7': {
        'name':  '20161113A_EV1',
        'json': 'HI_13Nov16_MD7_A',
        'data': np.array([[4200,5700]])
    },
    'MD8': {
        'name':  '20161114A_EV1',
        'json': 'HI_14Nov16_MD8_A',
        'data': np.array([[4600,6500],[10400,11000]])
    },
    'MD9': {
        'name':  '20161115A_EV1',
        'json': 'HI_15Nov16_MD9_A',
        'data': np.array([[7900,9600]])
    }
}
gm = GDALMesh('../../data/maps/HI_lowqual_DEM.tif')
pd.options.display.max_rows = 7
traversesallowed = ['MD9','MD8','MD6'] #M7 is bad
allslopes = []
allvelocities = []
for traverse in traversesallowed:
    #traverse = 'MD9'
    bounds = traverses[traverse]['data']
    csv_filename = '../../data/ev_tracks/%s.csv'%(traverses[traverse]['name'])
    json_filename = '../../data/waypoints/%s.json'%traverses[traverse]['json']
    delimiter = ","
    header_row = 0
    df = pd.read_csv(csv_filename, sep=delimiter, header=header_row)
    gp = GeoPolygon(LAT_LONG,*df[['latitude', 'longitude']].as_matrix().transpose())
    wp = JSONloader.from_file(json_filename).get_waypoints()
    em = gm.loadSubSection(gp.geoEnvelope())
    rr = df['cumulative distance (m)'].as_matrix()
    df['date_time'] = pd.to_datetime(df['epoch timestamp'],unit='s')
    time = df['date_time']-df['date_time'][0]
    times = time.astype('timedelta64[s]').as_matrix()
    f = interpolate.interp2d(np.arange(em.numCols),np.arange(em.numRows),em.dataset)
    XYC = XY(em.nw_geo_point, em.resolution)
    elevation = np.array([f(xwp,ywp) for xwp,ywp in gp.to(XYC).transpose()])
    elevation = elevation.transpose()[0]

    start_time, end_time = bounds.transpose()
    for idx in range(len(start_time)):
        t = times[start_time[idx]:end_time[idx]]
        r = rr[start_time[idx]:end_time[idx]]
        rf = savgol_filter(r,59,2)
        z = elevation[start_time[idx]:end_time[idx]]
        zf = savgol_filter(z,59,2)
        slopes = np.degrees(np.arctan(np.gradient(zf)/np.gradient(rf)))
        velocity = np.sqrt(np.square(np.gradient(rf,0.5))+np.square(np.gradient(zf,0.5)))
        allslopes= np.append(allslopes, slopes)
        allvelocities = np.append(allvelocities, velocity)
        #plt.scatter(slopes,velocity,marker=".",color='orange')

bins = np.linspace(-20,20,40)
v = np.zeros(len(bins))
for i in range(len(bins)-1):
    idx1 = np.where(np.logical_and(bins[i] <= allslopes, allslopes < bins[i+1] ))
    v[i] = np.average(allvelocities[idx1])

from pextant.ExplorerModel import Astronaut
slopes = np.linspace(-25, 25, 100)
a = Astronaut(80)
#plt.plot(slopes, a.velocity(slopes))
plt.hexbin(allslopes, allvelocities)
plt.grid()
plt.xlabel('slopes [degrees]')
plt.ylabel('velocity [m/s]')
plt.xlim([-25,25])
plt.show()