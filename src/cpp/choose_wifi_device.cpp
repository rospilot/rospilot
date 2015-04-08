/*********************************************************************
 *
 * Copyright 2012 the original author or authors.
 * See the NOTICE file distributed with this work for additional
 * information regarding copyright ownership.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *    http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *********************************************************************/
#include<stdio.h>
#include<errno.h>
#include<time.h>
#include<ctype.h>
#include<functional>
#include<fstream>
#include<iostream>
#include<sstream>
#include<chrono>
#include<vector>
#include<map>
#include<netlink/netlink.h>
#include<netlink/msg.h>
#include<netlink/attr.h>
#include<netlink/genl/genl.h>
#include<netlink/genl/family.h>
#include<netlink/genl/ctrl.h>
#include<linux/types.h>
#include<linux/if_ether.h>
#include<linux/nl80211.h>

#if !defined(CONFIG_LIBNL20) && !defined(CONFIG_LIBNL30)
    #define nl_sock nl_handle

    nl_handle *nl_socket_alloc()
    {
        return nl_handle_alloc();
    }

    void nl_socket_free(nl_sock *socket)
    {
        nl_handle_destroy(socket);
    }

    int nl_socket_set_buffer_size(nl_sock *socket, int receiveBuffer, int transmitBuffer)
    {
        return nl_set_buffer_size(socket, receiveBuffer, transmitBuffer);
    }
#endif

typedef int (*Callback)(nl_msg *message, void *arg);
typedef unsigned int Wiphy;

struct Channel
{
    unsigned int mhz;
    unsigned int maxTxDbm;
};

struct Device
{
    std::string name;
    std::string mac;
    std::vector<Channel> channels;
};

int init(nl_sock *&socket, int *socketId)
{
    socket = nl_socket_alloc();
    if (!socket) {
        std::cerr << "Error opening netlink socket" << std::endl;
        return 1;
    }

    nl_socket_set_buffer_size(socket, 8192, 8192);
    if (genl_connect(socket)) {
        std::cerr << "Error opening netlink socket" << std::endl;
        nl_socket_free(socket);
        return 1;
    }

    *socketId = genl_ctrl_resolve(socket, "nl80211");
    if (socketId < 0) {
        std::cerr << "nl80211 not found" << std::endl;
        nl_socket_free(socket);
        return 1;
    }

    return 0;
}

std::string macToString(unsigned char *mac)
{
    std::stringstream ss;
    bool first = true;
    for (int i = 0; i < ETH_ALEN; i++) {
        char tmp[3];
        if (!first) {
            ss << ":";
        }
        sprintf(tmp, "%02x", mac[i]);
        ss << tmp;
        first = false;
    }

    return ss.str();
}

int frequencyToChannel(int frequency)
{
    if (frequency < 2484) {
        return (frequency - 2407) / 5;
    }
    else if (frequency >= 5035 && frequency <= 5825) {
        return (frequency - 5000) / 5;
    }
    else {
        return 0;
    }
}

int sendCommand(nl_sock *socket, int socketId, nl80211_commands command, Callback callback, std::map<Wiphy, Device> *arg)
{
    nl_msg *message = nlmsg_alloc();
    if (!message) {
        return 1;
    }

    nl_cb *cb = nl_cb_alloc(NL_CB_DEFAULT);
    nl_cb *socketCb = nl_cb_alloc(NL_CB_DEFAULT);
    if (!cb || !socketCb) {
        nlmsg_free(message);
        return 1;
    }

    genlmsg_put(message, 0, 0, socketId, 0, NLM_F_DUMP, command, 0);
    nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, callback, arg);
    nl_socket_set_cb(socket, socketCb);

    int err = nl_send_auto_complete(socket, message);
    if (err >= 0) {
        err = 1;

        auto errorCallback = [](sockaddr_nl *address, nlmsgerr *err, void *arg) -> int {
            int *ret = (int *) arg;
            *ret = err->error;
            return NL_STOP;
        };
        auto finishCallback = [](nl_msg *message, void *arg) -> int {
            int *ret = (int *) arg;
            *ret = 0;
            return NL_SKIP;
        };
        auto ackCallback = [](nl_msg *message, void *arg) -> int {
            int *ret = (int *) arg;
            *ret = 0;
            return NL_STOP;
        };
        nl_cb_err(cb, NL_CB_CUSTOM, errorCallback, &err);
        nl_cb_set(cb, NL_CB_FINISH, NL_CB_CUSTOM, finishCallback, &err);
        nl_cb_set(cb, NL_CB_ACK, NL_CB_CUSTOM, ackCallback, &err);

        while (err > 0) {
            nl_recvmsgs(socket, cb);
        }
    }
    nl_cb_put(cb);
    nl_cb_put(socketCb);
    nlmsg_free(message);
    return err;
}

void nlForEach(nlattr *attr, std::function<void(nlattr *)> func)
{
    int remaining;
    nlattr *current;
    for (current = (nlattr *) nla_data(attr), remaining = nla_len(attr);
            nla_ok(current, remaining); current = nla_next(current, &remaining)) {
        func(current);
    }
}

int parseDevice(nl_msg *msg, void *arg)
{
    genlmsghdr *header = (genlmsghdr *) nlmsg_data(nlmsg_hdr(msg));
    nlattr *message[NL80211_ATTR_MAX + 1];
    std::map<Wiphy, Device> *devices = (std::map<Wiphy, Device> *) arg;

    nla_parse(message, NL80211_ATTR_MAX, genlmsg_attrdata(header, 0),
          genlmsg_attrlen(header, 0), nullptr);

    Wiphy wiphy;
    if (!message[NL80211_ATTR_WIPHY]) {
        return NL_SKIP;
    }
    if (!message[NL80211_ATTR_IFNAME]) {
        return NL_SKIP;
    }
    wiphy = nla_get_u32(message[NL80211_ATTR_WIPHY]);

    Device device;
    device.name = nla_get_string(message[NL80211_ATTR_IFNAME]);

    if (message[NL80211_ATTR_MAC]) {
        device.mac = macToString((unsigned char *) nla_data(message[NL80211_ATTR_MAC]));
    }

    if (devices->count(wiphy)) {
        std::cerr << "Multiple interfaces for phy" << wiphy << " ignoring " << device.name << std::endl;
        return NL_SKIP;
    }

    (*devices)[wiphy] = device;

    return NL_SKIP;
}

int parseChannel(nl_msg *msg, void *arg)
{
    std::map<Wiphy, Device> *devices = (std::map<Wiphy, Device> *) arg;

    nlattr *message[NL80211_ATTR_MAX + 1];
    genlmsghdr *header = (genlmsghdr *) nlmsg_data(nlmsg_hdr(msg));

    nla_policy freqPolicy[NL80211_FREQUENCY_ATTR_MAX + 1];
    memset(freqPolicy, NLA_UNSPEC, sizeof(nla_policy) * (NL80211_FREQUENCY_ATTR_MAX + 1));
    freqPolicy[NL80211_FREQUENCY_ATTR_FREQ].type = NLA_U32;
    freqPolicy[NL80211_FREQUENCY_ATTR_DISABLED].type = NLA_FLAG;
    freqPolicy[NL80211_FREQUENCY_ATTR_PASSIVE_SCAN].type = NLA_FLAG;
    freqPolicy[NL80211_FREQUENCY_ATTR_NO_IBSS].type = NLA_FLAG;
    freqPolicy[NL80211_FREQUENCY_ATTR_MAX_TX_POWER].type = NLA_U32;

    nla_parse(message, NL80211_ATTR_MAX, genlmsg_attrdata(header, 0),
          genlmsg_attrlen(header, 0), nullptr);

    if (!message[NL80211_ATTR_WIPHY]) {
        return NL_SKIP;
    }

    if (!message[NL80211_ATTR_WIPHY_BANDS]) {
        return NL_SKIP;
    }

    if (!message[NL80211_ATTR_SUPPORTED_IFTYPES]) {
        return NL_SKIP;
    }

    bool ibss = false;
    nlForEach((nlattr *) message[NL80211_ATTR_SUPPORTED_IFTYPES], [&ibss](nlattr *mode) mutable {
        if (nla_type(mode) == NL80211_IFTYPE_ADHOC) {
            ibss = true;
        }
    });
    if (!ibss) {
        return NL_SKIP;
    }

    Wiphy wiphy = nla_get_u32(message[NL80211_ATTR_WIPHY]);

    nlForEach((nlattr *) message[NL80211_ATTR_WIPHY_BANDS], [&](nlattr *attr) mutable {
        nlattr *band[NL80211_BAND_ATTR_MAX + 1];

        nla_parse(band, NL80211_BAND_ATTR_MAX, (nlattr *) nla_data(attr), nla_len(attr), nullptr);
        if (!band[NL80211_BAND_ATTR_FREQS]) {
            return;
        }
        nlForEach((nlattr *) band[NL80211_BAND_ATTR_FREQS], [&](nlattr *attr) mutable {
            Channel channel;
            nlattr *freq[NL80211_FREQUENCY_ATTR_MAX + 1];
            nla_parse(freq, NL80211_FREQUENCY_ATTR_MAX, (nlattr *) nla_data(attr),
                  nla_len(attr), freqPolicy);
            if (!freq[NL80211_FREQUENCY_ATTR_FREQ]) {
                return;
            }
            if (freq[NL80211_FREQUENCY_ATTR_DISABLED]) {
                return;
            }
            if (freq[NL80211_FREQUENCY_ATTR_PASSIVE_SCAN]) {
                // Only allowed to do passive scanning
                return;
            } 
            if (freq[NL80211_FREQUENCY_ATTR_NO_IBSS]){
                // Need IBSS for an AP
                return;
            }
            channel.mhz = nla_get_u32(freq[NL80211_FREQUENCY_ATTR_FREQ]);

            if (freq[NL80211_FREQUENCY_ATTR_MAX_TX_POWER]) {
                channel.maxTxDbm = nla_get_u32(freq[NL80211_FREQUENCY_ATTR_MAX_TX_POWER]) / 100;
            }
            if (devices->count(wiphy)) {
                (*devices)[wiphy].channels.push_back(channel);
            }
        });
    });

    return NL_SKIP;
}

std::vector<Device> filter24GHz(std::vector<Device> devices)
{
    std::vector<Device> newDevices;

    for (Device device: devices) {
        std::vector<Channel> channels;
        for (Channel c: device.channels) {
            if (c.mhz > 3000) {
                channels.push_back(c);
            }
        }
        if (channels.size() > 0) {
            Device newDevice = device;
            newDevice.channels = channels;
            newDevices.push_back(newDevice);
        }
    }

    return newDevices;
}

int main(int argc, char **argv)
{
    nl_sock *socket;
    int socketId;

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <output file>" << std::endl;
        return 1;
    }

    if (init(socket, &socketId)) {
        nl_socket_free(socket);
        return 1;
    }

    std::map<Wiphy, Device> devicesMap;

    sendCommand(socket, socketId, NL80211_CMD_GET_INTERFACE, &parseDevice, &devicesMap);
    sendCommand(socket, socketId, NL80211_CMD_GET_WIPHY, &parseChannel, &devicesMap);

    std::vector<Device> devices;

    for (auto entry: devicesMap) {
        devices.push_back(entry.second);
    }

    std::string include24GHz;
    std::cout << "Include 2.4GHz channels? (select \"no\" if your radio controller is 2.4GHz)" << std::endl;
    while (include24GHz != "yes" && include24GHz != "no") {
        std::cout << "yes/no:" << std::endl;
        std::cin >> include24GHz;
    }

    if (include24GHz == "no") {
        devices = filter24GHz(devices);
    }

    if (!devices.size()) {
        std::cout << "No suitable wifi cards found." << std::endl;
        return 1;
    }

    int deviceIndex = -1;
    while (deviceIndex < 1 || deviceIndex > devices.size()) {
        std::cout << "Select wifi card:" << std::endl;
        for (int i = 0; i < devices.size(); i++) {
            std::cout << (i + 1) << ": " << devices[i].name << "(" << devices[i].mac << ")" << std::endl;
        }
        std::cin >> deviceIndex;
    }
    Device chosenDevice = devices[deviceIndex - 1];

    int channelIndex = -1;
    std::vector<Channel> channels = chosenDevice.channels;
    while (channelIndex < 1 || channelIndex > channels.size()) {
        std::cout << "Select channel:" << std::endl;
        for (int i = 0; i < channels.size(); i++) {
            std::cout << (i + 1) << ": " << channels[i].mhz << "(max " << channels[i].maxTxDbm << "Dbm)" << std::endl;
        }
        std::cin >> channelIndex;
    }

    int channel = frequencyToChannel(channels[channelIndex - 1].mhz);

    std::ofstream fout;
    fout.open(argv[1]);
    fout << chosenDevice.name << "\t";
    fout << (channels[channelIndex - 1].mhz < 3000 ? 'g' : 'a') << "\t";
    fout << channel << std::endl;
    fout.close();
    
    nl_socket_free(socket);
    return 0;
}

