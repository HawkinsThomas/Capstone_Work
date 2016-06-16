import serial,time,thread
import Queue

dist_mm_store = [7000]*360 #181
dist_mm_store_stuck = [7000]*360 #181
baud_rate = 115200

def lidarRead(com_port, q):
  while True:
    try:
      global dist_mm_store,dist_mm_store_stuck
      #com_port  = '/dev/ttyACM0'#'/dev/cu.usbmodem1411' ##change the COM port if required, check in Arduino IDE
      ser = serial.Serial(com_port,baud_rate)

      b = (ord(ser.read(1))) ##initial read
      dati = []

      while True:
        ##250 == FA, FA is the start value - it's constant
        ##Each data packet is 22 bytes, > 20 means len(dati) == at least 21
        if b==(250) and len(dati)>20:
          break

        ##add data to list, read again
        dati.append(b)
        b = (ord(ser.read(1)))#(ord(ser.read(1)))

        # print b

        ##do not hog the processor power - Python hogs 100% CPU without this in infinite loops
        time.sleep(0.00001)

      if len(dati)==21:
        #if debug:
          #print "b"
        ##index data packets go from 0xA0 (160) to 0xF9(259). Subtract 160 to normalize scale of data packets from 0 to 90.
        dati[0]=((dati[0])-160)

        for i in (1,2,3,4):
          ##128 is an error code
          if dati[i*4] != 128:
            ##if good data, convert value in dati to value in mm. code found online
            dist_mm = dati[4*i-1] | (( dati[4*i] & 0x3f) << 8)

            ##dati[0] is index of each packet from 0 to 90. *4 for a value from 1 - 360, and cycle through the 4 data packets from that point at index
            ##e.g. dati[0] is 30. 30 * 4 = 120, then the values being read are 121, 122, 123, 124
            angle = dati[0]*4+i+1
            #print angle, dist_mm

            ##adjust values by 2
            #if angle < 361 and angle >=0:#181:
            if angle < 360 and angle >=0:
              if dist_mm_store_stuck[angle] != dist_mm:#181:
                dist_mm_store[angle] = dist_mm
                dist_mm_store_stuck[angle] = dist_mm
              elif dist_mm_store_stuck[angle] == dist_mm: # data is stuck
                # print "data is stuck:",angle,dist_mm_store_stuck[angle],dist_mm
                dist_mm_store[angle] = 7000

          else:
            # if debug:
            #   print "c bad angle %d -> %d" % (angle,dist_mm)
            #if good data, convert value in dati to value in mm. code found online
            dist_mm = 7000

            ##dati[0] is index of each packet from 0 to 90. *4 for a value from 1 - 360, and cycle through the 4 data packets from that point at index
            ##e.g. dati[0] is 30. 30 * 4 = 120, then the values being read are 121, 122, 123, 124
            angle = dati[0]*4+i+1

            if angle < 361 and angle >=0:#181:
              dist_mm_store[angle] = dist_mm
        # print "thread:"
        # print dist_mm_store
        ser.flush()
        ser.close()
        q.put(dist_mm_store)

    except:
      print "serial master error."
      ser.flush()
      ser.close()
      time.sleep(.1)
      # break





