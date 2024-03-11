""" Script to interface with battery scanner
"""
import argparse
from time import sleep
import serial
import array
import ntcore
import logging
import re

#------------------
#-- Parameters
#------------------
logging.basicConfig(level=logging.INFO)

timeout = 5
prefix = array.array('B', [0x02, 0x00, 0x00, 0x01, 0x00, 0x33, 0x31])
scan_command = array.array('B',[0x7e, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xab, 0xcd])
name_length = 8
response_length = len(prefix) + name_length

#------------------
#-- Parse command line arguments
#------------------
parser = argparse.ArgumentParser()
parser.add_argument("--dev", default="COM8", type=str, help="Path to the camera device")
args = parser.parse_args()


#------------------
#-- Setup NetworkTables
#------------------
client = ntcore.NetworkTableInstance.getDefault()
client.startClient4(f'battery-scanner')
client.setServerTeam(6328)

pub = client.getStringTopic("/battery_name").publish()

#------------------
#-- Read battery name
#------------------
name = None
try:
  with serial.Serial(args.dev, 9600, timeout=timeout) as ser:

    logging.info("Scanning for battery ID")

    while True:
      ser.write(scan_command)
      response = ser.read(response_length)
      
      if len(response) == response_length:
          if response.startswith(prefix):
              name_bytes = response[-8:]
              name_str = name_bytes.decode("utf-8")
              if re.match('^[a-zA-Z0-9_\\-]+$', name_str):
                  name = name_str
                  logging.info(f"Battery ID: {name}")
                  break
              else:
                logging.warning("Battery ID doesn't match expected pattern")

except Exception as e:
   logging.error(e)

#------------------
#-- Publish name
#------------------
if name is not None:
  pub.set(f"BAT-{name}")
else:
  logging.error("Unable to read battery code")


#------------------
#-- Switch to idle state
#------------------
logging.info("Switching to idle mode")
while True:
  sleep(100)