
import serial,time,thread
import Queue

debug = False


# main() function
def serial_parser(strPort, q):
  while True:
    try:
      if debug: print('reading from serial port %s...' % strPort)
      baud_rate = 57600
      ser = serial.Serial(strPort, baud_rate)

      master_data  = []

      smooth = 0.1
      avg_weight = 60
      avg_voltage = 12
      last_voltage = 0
      last_weight = 0
      longtude = 0
      latitude = 0
      while True:
        try:
          line = ser.readline()
          if line[0:6] == "loadWt":
            i = -1*float(line.strip().split(',')[1])
            if (i != 459.9): # garbage value created by serial ready on arduino side
              avg_weight = (1-smooth)*avg_weight+smooth*i
              last_weight = i
              if debug: print "Weight: ", i, "avg:",avg_weight

          elif line[0:6] == "batVol":
            i = float(line.strip().split(',')[1])
            avg_voltage = (1-smooth)*avg_voltage+smooth*i
            last_voltage = i
            if debug: print "Voltage:", i, "avg:",avg_voltage


          elif line[0:6] == "$GPVTG":
            i = [str(var) for var in line.strip().split(',')]
            if i[1] == 't':
              kph = float(i[7])
              if debug: print "Speed:",kph

          elif line[0:6] == "$GPGLL":
            i = [str(var) for var in line.strip().split(',')]
            # print i
            if i[1] != '': longtude = float(i[1])/100
            if i[1] != '': latitude = float(i[3])/100
            if debug: print "GPS: ", longtude,latitude


          elif line[0:6] == "$GPGSA":
            if debug: print ""
            i = [str(var) for var in line.strip().split(',')]
            # if debug: print "{0:<8}{1:<2}".format("Mode:",i[1])
            if debug: print "{0:<8}{1:<2} (2=2D, 3=3D)".format("Mode:",i[2])
            if debug: print "Satellites:",",".join(sorted(i[3:14])), len(i[3:14])



          elif line[0:6] == "$GPGGA":
            i = [str(var) for var in line.strip().split(',')]
            csum = 0
            if len(i) == 15:
              for j in line[1:-5]:
                  csum ^= ord(j)
              if hex(csum) == hex(int(line[-4:],16)):
                  UTC = time.strptime(i[1],"%H%M%S.00")
                  # UTC = UTC.replace(tzinfo=Zone(-5,False,'EST'))
                  UTC = time.strftime("%H:%M:%S", UTC)
                  if debug: print ""
                  if debug: print "{0:<14}{1:<14}".format("Time:",UTC)
                  if debug: print "{0:<14}{1:<14}{2:<2}".format("Latitude:",float(i[2]),i[3])
                  if debug: print "{0:<14}{1:<14}{2:<2}".format("Longitude:",float(i[4]),i[5])
                  if debug: print "{0:<14}{1:<14}".format("Quality:",i[6])
                  if debug: print "{0:<14}{1:<14}".format("Satellites:",i[7])
                  if debug: print "{0:<14}{1:<4}{2:<2}".format("Altitude:",i[9],i[10])

          # else:
          #   if debug: print line
          dictionary = {"v_avg":avg_voltage,"v_raw":last_voltage,"w_avg":avg_weight,"w_raw":last_weight,"long":longtude,"lati":latitude}
          q.put(dictionary)
        except KeyboardInterrupt:
          print "serial master exiting."
          break
      ser.flush()
      ser.close()
    except (KeyboardInterrupt):
      print "serial error. (outer), trying again",strPort
      #time.sleep(1)
      break


  # with open('./serial_output.txt','wb') as outfile:
  #   outfile.write("\n".join(master_data))
  # if debug: print "done."

# eg3. $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
# 1    = UTC of Position
# 2 = Latitude
# 3 =NorS
# 4 = Longitude
# 5 =EorW
# 6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
# 7    = Number of satellites in use [not those in view]
# 8    = Horizontal dilution of position
# 9    = Antenna altitude above/below mean sea level (geoid)
# 10   = Meters  (Antenna height unit)
# 11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
#        mean sea level.  -=geoid is below WGS-84 ellipsoid)
# 12   = Meters  (Units of geoidal separation)
# 13   = Age in seconds since last update from diff. reference station
# 14   = Diff. reference station ID#
# 15 = Checksum


# 1    = Mode:
#        M=Manual, forced to operate in 2D or 3D
#        A=Automatic, 3D/2D
# 2    = Mode:
#        1=Fix not available
# 2=2D
# 3=3D
# 3-14 = IDs of SVs used in position fix (null for unused fields)
# 15 = PDOP
# 16 = HDOP
# 17 = VDOP





