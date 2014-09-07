#! /usr/env/python

import sys
import re


def main():
  # check args
  if(len(sys.argv) < 3):
    print "you didn't provide enough args, you idiot"
    print "%s file/to/process.txt file/for/results.txt" % sys.argv[0]
    sys.exit()

  # find file
  with open(sys.argv[1], "r") as gps_file:
    with open(sys.argv[2], "w") as gps_parsed_file:
      for line in gps_file:
        result = re.search("\$GPGGA,[0-9]+\.[0-9]+,(?P<latnum>[0-9]+\.[0-9]+),(?P<latdir>[NSEW]),(?P<longnum>[0-9]+\.[0-9]+),(?P<longdir>[NSEW]),.*", line)

        if(result):
          # parse lat
          lat_num_str = result.groupdict()["latnum"]
          lat_deg_str = lat_num_str[0:2]
          lat_min_str = lat_num_str[2:len(lat_num_str)]
          lat_min = float(lat_min_str)
          lat_deg_dec = lat_min / 60.0
          lat_deg = float(lat_deg_str)
          lat_deg = lat_deg + lat_deg_dec
          lat_dir = result.groupdict()["latdir"]
          if(lat_dir.strip() == "S"):
            lat_deg = -1 * lat_deg

          # parse long
          long_num_str = result.groupdict()["longnum"]
          long_deg_str = long_num_str[0:3]
          long_min_str = long_num_str[3:len(long_num_str)]
          long_min = float(long_min_str)
          long_deg_dec = long_min / 60.0
          long_deg = float(long_deg_str)
          long_deg = long_deg + long_deg_dec
          long_dir = result.groupdict()["longdir"]
          if(long_dir.strip() == "W"):
            long_deg = -1 * long_deg

          gps_parsed_file.write("%.12f, %.12f\n" % (lat_deg, long_deg))


if(__name__ == "__main__"):
  main()
