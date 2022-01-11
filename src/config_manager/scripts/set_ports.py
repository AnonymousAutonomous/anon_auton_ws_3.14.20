import re
import os
import subprocess

# Find all USB devices
df = subprocess.check_output("find /sys/bus/usb/devices/usb*/ -name dev", shell=True)
devices = {}

for i in df.split('\n'):
    if not i:
        continue
    if (i.endswith('/dev')):
         i = i[:-4]
    devname = subprocess.check_output("udevadm info -q name -p " + i, shell=True)
    devname = devname.strip('\n')

    if (not devname.startswith('bus/')):
	devinfo = subprocess.check_output("udevadm info -q property --export -p " + i, shell=True)

        if ("ID_SERIAL" in devinfo):
            id = [line for line in devinfo.split('\n') if "ID_SERIAL" in line and "ID_SERIAL_SHORT" not in line]
            if id:
                id = id[0]
                val = id.split('=')[1][1:-1]
                if "Arduino" in val or "1a86_USB2.0" in val:
                    devices["arduino_port"] = '/dev/' + devname
                if "Basic_UART" in val:
                    devices["antenna_port"] = '/dev/' + devname
                if "Silicon_Labs" in val:
                    devices["lidar_port"] = '/dev/' + devname
                if "video" in devname:
                    devices["camera_port"] = '/dev/' + devname

script_dir = os.path.dirname(os.path.realpath(__file__))
outfolder = os.path.join(script_dir, '../', 'configs', 'ports')
outyaml = os.path.join(outfolder, 'active.yaml')

if not os.path.exists(outfolder):
  os.makedirs(outfolder)



with open(outyaml, 'w+') as f:
     for dev, path  in devices.items():
         f.write(dev + ": '" + path + "'\n")
