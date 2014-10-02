#!/bin/bash
message="Enabling rospilot service. \
Uninstall it with 'update-rc.d -f rospilot remove'"

echo "$(tput setaf 1)${message}$(tput sgr 0)"
sudo update-rc.d rospilot defaults

echo "Setting up postgis"
# Change to /tmp because postgres user might not be able to see the cwd,
# so suppress any warnings about that
cd /tmp
sudo -u postgres createuser --no-superuser --no-createdb --no-createrole rospilot
sudo -u postgres createdb gis
echo "GRANT ALL ON DATABASE gis TO rospilot;" | sudo -u postgres psql -Upostgres gis
echo "ALTER USER rospilot WITH PASSWORD 'rospilot_password'" | sudo -u postgres psql -Upostgres gis
echo "CREATE EXTENSION postgis;" | sudo -u postgres psql -Upostgres gis

echo "Setting up mapnik"
tempdir=$(mktemp -d)
cd $tempdir
rosrun rospilot get_mapnik_shapefiles.sh
mv -f $tempdir/data $(catkin_find --share rospilot share/mapnik-style/)
wget -O kathmandu.osm "http://api.openstreetmap.org/api/0.6/map?bbox=27.713,85.308,27.717,85.312"
rosrun rospilot load_osm.sh kathmandu.osm
rm -rf $tempdir

echo "Install more map data by downloading the appropriate osm/osm.pbf file \
(you can find some at \
http://wiki.openstreetmap.org/wiki/Planet.osm#Country_and_area_extracts) \
then run 'rosrun rospilot load_osm.sh --append <file>'"
