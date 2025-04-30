import serial
import time
import binascii
import re
import cv2 as cv

import numpy as np
import matplotlib.pyplot as plt

def SaveFile(Pcd_File_Name, PCDList):

    np.savetxt('{}'.format(Pcd_File_Name), PCDList, delimiter=' ')
    print('Saved File: [{}].'.format(Pcd_File_Name))


ser = serial.Serial()
#s = serial.Serial('COM7')

ser.port = "COM6"

#115200,N,8,1
ser.baudrate = 115200
ser.bytesize = serial.EIGHTBITS #number of bits per bytes
ser.parity = serial.PARITY_NONE #set parity check
ser.stopbits = serial.STOPBITS_ONE #number of stop bits

ser.timeout = 0.1          #non-block read 0.5s
ser.writeTimeout = 0.1     #timeout for write 0.5s
ser.xonxoff = False    #disable software flow control
ser.rtscts = False     #disable hardware (RTS/CTS) flow control
ser.dsrdtr = False     #disable hardware (DSR/DTR) flow control

try:
    ser.open()

except Exception as ex:
    print ("open serial port error " + str(ex))
    exit()

datafx = np.array([])
datafy = np.array([])
datafz = np.array([])
times = np.array([])
# plt.close('all')

plt.figure(figsize =(7, 10), dpi=95)
plt.ion()
plt.suptitle('Force Data', fontsize=20)

plt.show()

cmd = b'c\r'
text_file = open("force data/data4.txt", 'w')
resultdata = np.array([])

#cmd = b'\x73\x0D'
starttime = time.time()
if ser.isOpen():

    while 1:

        ser.write(cmd)

        print("write : %s" % cmd)

        response = ser.readline()
        print("fread : %s" % response)
        if len(response) > 50:
            break

    time.sleep(0.5)  # wait 0.5s

    # ser.flushInput()  # flush input buffer
    # ser.flushOutput()  # flush output buffer
    # ser.write(cmd)
    # print("second write : %s" % cmd)

    # read force command
    while (1):

        try:
            ser.flushInput()  # flush input buffer
            ser.flushOutput()  # flush output buffer
            print('clear ok')
            # read 80 byte data
            # response = ser.read(100)
            response = ser.readline()
            print('second read', response)

            endtime = time.time()
            if len(response) > 50:

                print("read : %s" % response)
                newres = response.decode('UTF-8', 'strict')


                b = re.split(r'\s+', newres)

                for i in range(b.count('')):
                    b.remove('')

                force = b[::2]
                sensordata = force[:3]

                fx = float(sensordata[0])
                fy = float(sensordata[1])
                fz = float(sensordata[2])
                forcedata = [fx, fy, fz]
                print('forcedata = ', forcedata)

                endtime = time.time()
                processtime = endtime - starttime
                print("耗時: {:.2f}秒".format(processtime))

                X = round(processtime, 2)
                Yx = fx
                Yy = fy
                Yz = fz

                strx = str(X)
                newres = strx + ' s\t' + newres
                text_file.write(newres)
                # print('text_file = ', type(newres))
                if response == "\n":
                    text_file.seek()
                    text_file.truncate()
                text_file.flush()

                resultdata = np.vstack([X, fx, fy, fz])
                resultdata = resultdata.T

                times = np.append(times, X)
                datafx = np.append(datafx, Yx)
                datafy = np.append(datafy, Yy)
                datafz = np.append(datafz, Yz)

                # print(data)
                # plt.cla()
                plt.clf()

                plt.subplot(311)
                plt.xlabel("time(s)")
                # plt.legend(['Fx'])
                plt.ylabel("Fx(N)")
                plt.plot(times, datafx, color ='tab:red')
                plt.grid()

                plt.subplot(312)
                plt.xlabel("time(s)")
                plt.ylabel("Fy(N)")
                plt.plot(times, datafy, color='tab:green')
                plt.grid()

                plt.subplot(313)
                plt.xlabel("time(s)")
                plt.ylabel("Fz(N)")
                plt.plot(times, datafz, color ='tab:blue')
                plt.grid()

                # plt.subplot(414)
                # plt.xlabel("time")
                # plt.ylabel("F")
                # plt.plot(times, datafx, color='tab:red')
                # plt.plot(times, datafy, color='tab:green')
                # plt.plot(times, datafz, color='tab:blue')

                plt.pause(0.001)
                # print('plot Fx data = ', X, ',', Yx)
                # print('plot Fy data = ', X, ',', Yy)
                # print('plot Fz data = ', X, ',', Yz)
                # plt.waitforbuttonpress()
                plt.ioff()
        except Exception as e1:
            print("communicating error " + str(e1))

    ser.close()

else:
    print ("open serial port error")

