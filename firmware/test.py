import usb.core
import usb.util
import struct
import argparse
import time

parser = argparse.ArgumentParser()
parser.add_argument('filename')
args = parser.parse_args()

with open(args.filename, "rb") as f:
    payload = f.read()

print(f"Uploading {len(payload)} bytes")

dev = usb.core.find(idVendor=0x1209, idProduct=0x000E)

if dev is None:
    raise ValueError('Device not found')

dev.set_configuration()
req = usb.util.build_request_type(usb.util.CTRL_OUT, usb.util.CTRL_TYPE_VENDOR, usb.util.CTRL_RECIPIENT_INTERFACE)

s1 = time.time()
dev.ctrl_transfer(req, 10, 0, 0, struct.pack('<l', len(payload)))
dev.write(0x01, payload)
status = dev.read(0x82, 1, 5000)[0] # wait for interrupt
s2 = time.time()

if status == 1:
	print(f"Successful! took {(s2 - s1) * 1000:.0f} ms")
else:
	print("Error! FPGA did not accept configuration")
