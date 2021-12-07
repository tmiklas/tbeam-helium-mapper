import base64
import argparse
import struct

parser = argparse.ArgumentParser(description='Encode a downlink payload for a Helium mapper.')
parser.add_argument('--distance', '-d', type=int, help='Map distance interval (kilometers)')
parser.add_argument('--time', '-t', type=int, help='Minimum time interval (seconds)')
parser.add_argument('--cutoffvolts', '-c', type=float, help='Low Voltage Power Off (volts)')
args = parser.parse_args()

distance = 0
if args.distance and rgs.distance > 0 and args.distance < 0xFFFF:
    distance = args.distance

time_interval = 0
if args.time and args.time > 0 and args.time < 0xFFFF:
    time_interval = args.time

cutoffvolts = 2.0 # Not a valid value, zero
if args.cutoffvolts and args.cutoffvolts >= 2.1 and args.cutoffvolts < 4.56:
    cutoffvolts = args.cutoffvolts
    

payload = struct.pack('>HHB', distance, time_interval, int((cutoffvolts - 2.0) * 100))
encodedBytes = base64.b64encode(payload)
encodedStr = str(encodedBytes, "utf-8")
print(payload.hex(' ').upper())
print(encodedStr)
