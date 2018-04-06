#!/usr/bin/env python
# -*- coding: utf-8 -*-

import bme280

if __name__ == '__main__':
  bme = bme280.bme280()
  # Read Data
  bme.get_data()
  tc = bme.get_temperature()
  tf = bme.get_temp_f()
  pp = bme.get_pressure()
  ph = bme.get_press_mmhg()
  h = bme.get_humidity()
  dc = bme.get_dewpoint()
  df = bme.get_dewpoint_f()
  am = bme.get_altitude()
  af = bme.get_altitude_ft()
  sp = bme.get_pasealevel()
  sh = bme.get_pasealevel_mmhg()
  chipid = bme.get_chip_id()

  print "Temperature: %f C" % tc
  print "Temperature: %f F" % tf
  print "Pressure: %f hPa" % (pp/100)
  print "Pressure: %f mmHg" % ph
  print "Humidity: %f %%" % h
  print "Dewpoint: %f C" % dc
  print "Dewpoint: %f F" % df
  print "Altitude: %f m" % am
  print "Altitude: %f ft" % af
  print "SeaLevel Pressure: %f hPa" % (sp/100)
  print "SeaLevel Pressure: %f mmHg" % sh
  print "Chip ID: {0}".format(chipid)
