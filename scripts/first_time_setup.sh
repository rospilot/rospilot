#!/bin/bash
echo "Installing udev and init.d rules"
sudo cp $(catkin_find --share rospilot share/etc/rospilot.rules) /etc/udev/rules.d/
sudo cp $(catkin_find --share rospilot share/etc/rospilot.service) /etc/systemd/system/
message="Enabling rospilot service. \
Disable it with 'systemctl disable rospilot'"

echo "$(tput setaf 1)${message}$(tput sgr 0)"
sudo systemctl enable rospilot
sudo systemctl daemon-reload

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

    cp $(catkin_find --share rospilot share/ap_config/netplan.template) $tempdir
    cat $tempdir/netplan.template | sed "s/WLAN_PLACEHOLDER/$wlan/g" > $tempdir/netplan.yaml
    sudo mv $tempdir/netplan.yaml /etc/netplan/99-rospilot.yaml
    
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

    sudo mkdir -p /etc/systemd/system/dnsmasq.service.d/
    echo "[Service]" > $tempdir/override.conf
    echo "# XXX: resolve race with wlan device" >> $tempdir/override.conf
    echo "ExecStartPre=/bin/sleep 10" >> $tempdir/override.conf
    echo "[Unit]" >> $tempdir/override.conf
    echo "After=sys-subsystem-net-devices-$wlan.device" >> $tempdir/override.conf
    sudo mv $tempdir/override.conf /etc/systemd/system/dnsmasq.service.d/override.conf
    sudo systemctl daemon-reload

    echo ""
    echo "Please restart your drone"
fi
