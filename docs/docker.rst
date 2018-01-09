================================
Docker
================================
http://blog.mastermaps.com/2014/10/3d-terrains-with-cesium.html

https://hub.docker.com/r/homme/cesium-terrain-builder/

.. code-block:: python

	# can also get gdal from docker
	docker pull geodata/gdal

	docker run -v "c:\Users\johan\Dropbox (MIT)\BASALT\pextant\pextant\maps":/data -t -i homme/cesium-terrain-builder bash
	ctb-tile -r average -o /data/tilesets /data/hwmidlow.tif

	docker run -v "c:\Users\johan\Desktop\Big maps":/data -ti spara/gdal_ef bash
	docker run -v "c:\Users\johan\Dropbox (MIT)\BASALT\pextant\data\maps":/data -t -i spara/gdal_ef bash

	gdaldem color-relief HI_lowqual_DEM.tif colormap.txt HI_lowqual_relief.tif -nearest_color_entry

	docker run -t -d --volumes-from basalt-data-store --name basalt-container -p 80:80 -p 3306:3306 -p 7500:7500  -p 222:22 -p 443:443 -p 3001:3001 -p 5000:5000 -p 5984:5984 -p 8080:8080 -p 8181:8181 -p 9090:9090 -p 9191:9191 -v "c:\Users\johan\Desktop\webapp":/home/xgds/sextantwebapp -v "c:\Users\johan\Dropbox (MIT)\BASALT\pextant\pextant:/home/xgds/xgds_basalt xgds-basalt-sse:20171019

	docker stop basalt-container
	docker rm basalt-container

	putty -ssh -P 222 xgds@localhost

	~/xgds_basalt/data/pyraptord$ vi pextant_latest.txt

	~/xgds_basalt/apps/pextant$ gunicorn sextant:app --bind 127.0.0.1:5000 --timeout 100 --access-logfile -



	gdal2tiles.py HI_lowqual_relief.tif

Ames Stereo pipeline
==================================

.. code-block:: python
	docker pull pgcumn/asp

Cesium terrain server (deprecated)
=====================================

.. code-block:: python

	cesium-terrain-server -dir "C:\Users\johan\Dropbox (MIT)\BASALT\pextant\pextant\maps\terrain" -port 9090

And in cesium

.. code-block:: javascript

	var terrainProvider = new Cesium.CesiumTerrainProvider({
    		url: 'http://localhost:9090/tilesets/HI_lowqual'
    	});

    	viewer.scene.terrainProvider = terrainProvider;

Need to actually make the C drive available for sharing
https://rominirani.com/docker-on-windows-mounting-host-directories-d96f3f056a2c#.rsfkrhqur


