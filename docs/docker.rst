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


