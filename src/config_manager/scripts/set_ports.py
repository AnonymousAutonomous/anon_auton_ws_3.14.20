import re
import subprocess

# Find all USB devices
df = subprocess.check_output("find /sys/bus/usb/devices/usb*/ -name dev", shell=True)
devices = []

for i in df.split('\n'):
    devname = subprocess.check_output("udevadm info -q name -p " + i, shell=True)
    print(devname)

    if i:
        info = device_re.match(i)
        if info:
            dinfo = info.groupdict()
            dinfo['device'] = '/dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
            devices.append(dinfo)
print devices

