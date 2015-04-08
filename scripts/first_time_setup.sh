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
echo "CREATE EXTENSION hstore;" | sudo -u postgres psql -Upostgres gis

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

echo -n "Setup wifi access point? [y/n]"
read wifi_requested

if [ "$wifi_requested" == "y" ]; then
    tempdir=$(mktemp -d)
    cd $tempdir
    hostname=$(hostname)
    wifi_config="$tempdir/wifi.config"
    rosrun rospilot choose_wifi_device $wifi_config
    if [ $? -ne 0 ]; then
        exit 1;
    fi
    wlan=$(cat $wifi_config | awk '{print $1}')
    mode=$(cat $wifi_config | awk '{print $2}')
    channel=$(cat $wifi_config | awk '{print $3}')
    echo -n "Choose ssid (wifi network name): "
    read ssid
    echo
    while true; do
        echo -n "Choose wifi passphrase: "
        read -s wifi_passphrase
        echo
        echo -n "Confirm passphrase: "
        read -s confirm_passphrase
        if [ "$wifi_passphrase" == "$confirm_passphrase" ]; then
            break;
        fi
        echo
        echo "$(tput setaf 1)Passphrase does not match$(tput sgr 0)"
    done

    cp $(catkin_find --share rospilot share/ap_config/interfaces.template) $tempdir
    cp /etc/network/interfaces $tempdir
    cat $tempdir/interfaces.template | sed "s/WLAN_PLACEHOLDER/$wlan/g" > $tempdir/interfaces.add
    # Add to interfaces, if not already there
    grep -q -f $tempdir/interfaces.add $tempdir/interfaces || cat $tempdir/interfaces.add >> $tempdir/interfaces
    sudo mv $tempdir/interfaces /etc/network/interfaces
    
    cp $(catkin_find --share rospilot share/ap_config/dnsmasq.conf.template) $tempdir
    cat $tempdir/dnsmasq.conf.template | sed "s/WLAN_PLACEHOLDER/$wlan/g" > $tempdir/dnsmasq.conf
    sudo mv $tempdir/dnsmasq.conf /etc/dnsmasq.conf
    
    cp $(catkin_find --share rospilot share/ap_config/hosts.dnsmasq.template) $tempdir
    cat $tempdir/hosts.dnsmasq.template | sed "s/HOSTNAME_PLACEHOLDER/$hostname/g" > $tempdir/hosts.dnsmasq
    sudo mv $tempdir/hosts.dnsmasq /etc/hosts.dnsmasq
    
    cp $(catkin_find --share rospilot share/ap_config/hostapd.conf.template) $tempdir
    cat $tempdir/hostapd.conf.template \
        | sed "s/WLAN_PLACEHOLDER/$wlan/g" \
        | sed "s/SSID_PLACEHOLDER/$ssid/g" \
        | sed "s/MODE_PLACEHOLDER/$mode/g" \
        | sed "s/CHANNEL_PLACEHOLDER/$channel/g" \
        | sed "s/PASSPHRASE_PLACEHOLDER/$wifi_passphrase/g" \
        > $tempdir/hostapd.conf
    sudo mv $tempdir/hostapd.conf /etc/hostapd/hostapd.conf
    echo 'DAEMON_CONF=/etc/hostapd/hostapd.conf' > $tempdir/hostapd
    sudo mv $tempdir/hostapd /etc/default/hostapd

    echo "Please restart your drone"
fi
