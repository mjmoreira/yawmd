*This project is not aligned with my current interests, therefore I will not
make further contributions. Feel free to fork it and continue development.*

# YAWMD - Yet Another Wireless Medium Daemon

YAWMD is an improvement of [wmediumd](https://github.com/ramonfontes/wmediumd),
used in [Mininet-WiFi](https://github.com/intrig-unicamp/mininet-wifi).
(YAWMD can and will also be written as Yawmd or yawmd).

Wmediumd and yawmd are wireless medium simulators for the Linux Kernel module
mac80211_hwsim.

The module mac80211_hwsim permits the simulation of multiple IEEE 802.11 radios in
a single machine.
These radios have the particularity of being exposed as real network interfaces to
Linux, which allows using them as if there were real interfaces, without need for
modifications in the programs in the upper network layers.
The transmissions that in real interfaces would use the medium, are simulated
in a simplistic way: all interfaces in the same frequency channel as the transmitter
receive a copy of the transmission.

It is possible to use a more realistic medium simulation using wmediumd/yawmd, which
allows control over the data rate, the probability of error in the transmission
between each pair of interfaces, and movement of interfaces (which updates to the
probability of error in the transmissions).

Yawmd was developed in the context of a [master thesis](https://hdl.handle.net/10216/131737), and more details about the simulation, how
mac80211_hwsim and wmediumd/yawmd work, and of the changes introduced with
yawmd can be found there.
It should be noted that the code in this repository includes a different
(and better) fix for the overlapping transmissions than the one previously used.

## Changes from wmediumd to yawmd
 - Communication protocol with mac80211_hwsim was optimized to reduce the amount of
 data and the amount of messages passed between mac80211_hwsim and yawmd;
 - Possibility of simulation of multiple mediums;
 - New configuration file format (because of the multiple mediums);
 - Possibility of using threads to run the simulation of a medium, (with one thread
 for each medium);
 - Successful overlapping transmissions eliminated.


# Running yawmd
## Prerequisites

 - Modified version of the module `mac80211_hwsim` to accommodate the changes to the communication protocol.
 It can be found [here](https://github.com/mjmoreira/mac80211_hwsim).
 - libnl (tested with version 3.5.0)
 - libconfig (tested with version 1.7.2)
 - libevent (tested with version 2.1.12)

## Building
```
cd yawmd && make
```

## Using yawmd

To start yawmd it is required to start `mac80211_hwsim` and configure the
interfaces. 

Starting yawmd with an appropriate config file is enough to make frames
pass through yawmd:
```
sudo insmod ../mac80211_hwsim/mac80211_hwsim.ko radios=2 # modified hwsim
sudo ./yawmd/yawmd -c examples/single.cfg &
# run some hwsim test
```
However, please see the next section on some potential pitfalls.

A complete example is given at the end of this document.

# Configuration

Yawmd supports three types of models to configure the wireless medium.
The purpose of these models is to determine the probability of error in the
transmission of a frame.
 - Model type *SNR* can use a default value for all interfaces, or use information
 from the configuration file, using the configured Signal to Noise Ratio (SNR)
 values between pairs of interfaces.
 - Model type *probability* works similarly to model type SNR. Instead of SNR
 values, it receives probability of error values for the combinations of
 interfaces. The error probability values can be asymmetric for the same pair.
 - Model type, *path-loss*, requires the definition of initial positions and of
 signal transmission power. Optionally, the model permits the use of a simple
 movement model in a 3D environment, based on direction vectors.


See the files `config_tests/good_*.cfg` for details about the options.


## Pitfalls

### MAC addresses changed
If you have for example `NetworkManager` active, when `mac80211_hwsim` is started,
the default MAC addresses may be changed to random values.

### Rates

wmediumd's and yawmd's rate table is currently hardcoded. See `yawmd/per.c`.

### Send-to-self

By default, traffic between local devices in Linux will not go over the
wire / wireless medium. To avoid this you may use network namespaces.

### Can't start mac80211_hwsim

If you get `insmod: ERROR: could not insert module mac80211_hwsim.ko: Unknown symbol in module`,
try running `modprobe mac80211` first and then try `insmod ./mac80211_hwsim.ko`
again.


# Example

The following sequence of commands establishes a two-node IBSS using network
namespaces.
The example uses commands prefixed by `ip netns exec <netns>`, but it is possible
to use a different terminal window for each interface using the `unshare` command,
which eliminates the need for the prefix.

```
# Run all commands as sudo

modprobe mac80211 # required if it is not already running
insmod ../mac80211_hwsim/mac80211_hwsim.ko radios=2

# ip netns add <namespace>
ip netns add yawmd1
ip netns add yawmd2

# Check "iw dev" or "iw phy" for the phy index
# iw phy <phy> set netns name <namespace>
iw phy phy1 set netns name yawmd1
iw phy phy2 set netns name yawmd2

# ip netns exec <namespace> iw dev <wlan> set type ibss
# <wlan> can be seen at "iw dev"
ip netns exec yawmd1 iw dev wlan1 set type ibss
ip netns exec yawmd2 iw dev wlan2 set type ibss

# ip netns exec <namespace> ip link set dev <wlan> up
ip netns exec yawmd1 ip link set dev wlan1 up
ip netns exec yawmd2 ip link set dev wlan2 up

# ip netns exec <namespace> ip addr add <ip>/<mask> dev <wlan>
ip netns exec yawmd1 ip addr add 10.0.5.11/24 dev wlan1
ip netns exec yawmd2 ip addr add 10.0.5.12/24 dev wlan2

# ip netns exec <namespace> ip route add default dev <wlan>
ip netns exec yawmd1 ip route add default dev wlan1
ip netns exec yawmd2 ip route add default dev wlan2

# ip netns exec <namespace> iw dev <wlan> ibss join <ibss_name> <channel_frequency>
ip netns exec yawmd1 iw dev wlan1 ibss join yawmdibss 2432
# Wait a few seconds before running the second ibss join command
ip netns exec yawmd2 iw dev wlan2 ibss join yawmdibss 2432
# You can check if they are in the same network comparing the MAC address like
# IDs returned by:
# ip netns exec yawmd1 iw dev wlan1 link
# ip netns exec yawmd2 iw dev wlan2 link
# If they don't match, run
# ip netns exec yawmd2 iw dev wlan2 ibss leave
# and try to join again.

# yawmd can be started at any point after starting mac80211_hwsim
./yawmd/yawmd -c ./examples/single.cfg

# Run each on a different terminal window (because of the output)
# Server:
# ip netns exec <net_namespace> iperf3 -s -1
ip netns exec yawmd1 iperf3 -s -1

# Client:
# ip netns exec <net_namespace> iperf3 -c <server_ip> -t <run_time>
ip netns exec yawmd2 iperf3 -c 10.0.5.11 -t 30

# Stop yawmd with Ctrl+C
# Stop mac80211_hwsim with modprobe -r mac80211_hwsim
# If you needed to run modprobe mac80211, you can also run modprobe -r mac80211.

```
