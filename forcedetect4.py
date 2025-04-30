import serial
import time
import binascii
import re
import numpy as np
import matplotlib.pyplot as plt
import os
import math
import single_pro_euler

def Sq2(value):
    # Code untuk Kuadrat bilangan
    return value*value

connect_sensor_file = "connect_sensor.txt"
return_robot_file = "return_robot.txt"

return_robot = open(return_robot_file, "w")
return_robot.write(str(0))
return_robot.close()
connect_sensor = open(connect_sensor_file, "w")
connect_sensor.write(str(0))
connect_sensor.close()
input("change 0")


# ---------------------- main ----------------------

# 和 C++做通訊
connect_sensor_path = "connect_sensor.txt"   # 手臂到達預留點
reviseT_path = "revise_Tpoint.txt"

averageline_time_file = open("force data/average1line_times.txt", 'w').close()
total_F = open("force data/total_F.txt", 'w').close()

touch_or_not = 0
times_all = []
dataF_all = []
run = -1
resultfile_no = 0


while 1:
    if os.path.exists(connect_sensor_path):
        # 打開ForceSensor需記錄的資料
        averageline_time_file = open("force data/average1line_times.txt", 'a+')  # a+ 讀取和寫入
        total_F = open("force data/total_F.txt", 'a+')

        # 成功讀到force sensor回傳給C+開始移動
        return_robot_file = "return_robot.txt"
        f = open(connect_sensor_path, 'r')
        time.sleep(0.1)
        value = int(f.read())  # connect_sensor_path 值

        resultfile_no = resultfile_no + 1
        if value == 1:  # 手臂已經到達預留研磨點，感測器連接成功
            time.sleep(0.1)
            run = run + 1
            print(f"手臂已經到達預留研磨點，開始研磨第{run}部分")

            # Sensor參數 前人ㄉ智慧不用改
            ser = serial.Serial()
            ser.port = "COM6"
            ser.baudrate = 115200
            ser.bytesize = serial.EIGHTBITS  # number of bits per bytes
            ser.parity = serial.PARITY_NONE  # set parity check
            ser.stopbits = serial.STOPBITS_ONE  # number of stop bits
            ser.timeout = 0.1  # non-block read 0.5s
            ser.writeTimeout = 0.1  # timeout for write 0.5s
            ser.xonxoff = False  # disable software flow control
            ser.rtscts = False  # disable hardware (RTS/CTS) flow control
            ser.dsrdtr = False  # disable hardware (DSR/DTR) flow control

            # 變數區 好醜
            times = np.array([])
            datafx = np.array([])
            datafy = np.array([])
            datafz = np.array([])
            dataF = np.array([])
            error_force = np.array([])
            begin_force = 0
            real_begin_force = 0
            real_begin = np.asarray([])
            process_start = 0
            loop_no = np.array([])
            F_loop_no = np.array([])
            resultdata = np.array([])
            re_time = np.array([])
            aftertouch_times = np.array([])
            aftertouch_dataF = np.array([])

            # 開connect_sensor=1時，開plot(可能還沒有數據)
            plt.figure(figsize=(8, 4), dpi=95)
            plt.ion()  # 動態更新 超讚ㄉ
            plt.suptitle('Force Data', fontsize=20)
            plt.show()

            # 看力感測器U咪U連接成功(試著開啟)
            try:
                ser.open()

            except Exception as ex:
                print("open serial port error " + str(ex))
                exit()
            # 打開ForceSensor需記錄的資料(不會因為中斷程式而沒儲存到的資料)
            text_file = open("force data/datadetect{}.txt".format(run), 'w')
            force_file = open("force data/F_line{}.txt".format(run), 'w')
            total_F = open("force data/total_F.txt".format(run), 'w')

            if ser.isOpen():
                starttime = time.time()
                print("開始記錄")

                whilerun = 0
                while 1:
                    try:
                        print('sensor set: ', ser.isOpen())
                        response = ser.readline()  # 讀取力感測器資料
                        if len(response) > 50:
                            whilerun = whilerun + 1
                            print("read : %s" % response)
                            endtime = time.time()  # 目前不知道用途

                            newres = response.decode('UTF-8', 'strict')
                            b = re.split(r'\s+', newres)

                            for i in range(b.count('')):
                                b.remove('')

                            force = b[::2]
                            sensordata = force[:3]   # fx fy fz

                            fx = float(sensordata[0])
                            fy = float(sensordata[1])
                            fz = float(sensordata[2])

                            # 力感測總合力
                            F = (fx ** 2 + fy ** 2 + fz ** 2) ** 0.5
                            F = (round(F * 1000)) / 1000

                            processtime = endtime - starttime
                            print("耗時: {:.2f}秒".format(processtime))   # 50份資料做平均花費的時間

                            X = round(processtime, 2)   # 將processtime取小數點後兩位
                            Yx = fx
                            Yy = fy
                            Yz = fz
                            YF = F

                            strx = str(X)
                            strF = str(F)
                            newres = strx + ' s\t' + newres  # sensor接收到的六個值
                            Fxyz = strx + ' s\t' + strF + ' N\t\n'  # 總合力
                            total_Fxyz = strx + '\t' + strF + '\t\n'

                            text_file.write(newres)   # datadetect
                            force_file.write(Fxyz)   # F_line
                            total_F.write(total_Fxyz)  # total_F

                            #  前人的智慧不容質疑
                            if response == "\n":
                                text_file.seek()
                                force_file.seek()
                                total_F.seek()
                                text_file.truncate()
                                force_file.truncate()
                                total_F.truncate()
                            text_file.flush()
                            force_file.flush()
                            total_F.flush()

                            resultdata = np.vstack([X, fx, fy, fz])
                            resultdata = resultdata.T

                            times = np.append(times, X)     # 時間
                            dataF = np.append(dataF, YF)    # 合力
                            dataFx = np.append(dataFx, Yx)  # Fx
                            dataFy = np.append(dataFy, Yy)  # Fy
                            dataFz = np.append(dataFz, Yz)  # Fz

                            print("dataF 的長度 = ", len(dataF))

                            if len(dataF) == 30:
                                for len_data in range(0, len(dataF)):
                                    if abs(dataF[len_data]) < 1:  # 小於1的資料不列入計算begin_force
                                        error_force = np.append(error_force, dataF[len_data])
                                print('error_force = ', error_force)
                                begin_force = float((np.sum(dataF) - np.sum(error_force)) / (len(dataF) - len(error_force)))  # 剔除異常值
                                print("begin_force = ", begin_force)

                                for last_data in range(0, len(dataF)):
                                    print('剔除異常值的力 - 當前力值 = ', abs(begin_force - dataF[last_data]))
                                    if abs(begin_force - dataF[last_data]) < 0.5:  # 0.5
                                        real_begin = np.append(real_begin, dataF[last_data])

                                print("真實的力值 = ", real_begin)
                                print("真實的力值的總數 = ", np.sum(real_begin))
                                print("真實的力值的長度 = ", (len(real_begin)))
                                real_begin_force = float((np.sum(real_begin)) / (len(real_begin)))

                                process_start = 1

                                # 確定接觸到預留研磨點，回傳2
                                ru = open(return_robot_file, "w")  # return_robot_file.txt
                                ru.write(str(2))
                                ru.close()
                                print('return robot')

                            print("真實的力值平均 = ", real_begin_force)
                            # plt.clf() # 我覺得不能在這裡關欸

                            if run == 0 and real_begin_force != 0:
                                print("whilerun = ", whilerun)
                                New_detect_F = abs(dataF[-1])
                                print("NEW DETECT Force = ", New_detect_F)  # 最後一個力值
                                Real_Force = abs(New_detect_F - real_begin_force)
                                print("REAL TIME Force = ", Real_Force)

                                # 利用 力值 判斷碰到研磨點
                                if Real_Force > 2:  # 2.5
                                    loop_no = np.append(loop_no, whilerun)
                                # ---------------------------------------
                                # -------------幹ㄇㄉ這段是垃圾-------------
                                print("loop_no len= ", len(loop_no))
                                print("loop_no[len(loop_no) - 1]", loop_no[-1])
                                print("loop_no[len(loop_no) - 3]", loop_no[-3])
                                print("loop_no[len(loop_no) - 2]", loop_no[-2])

                                # noise 確定484躁點
                                noise = (loop_no[-1] + loop_no[-3]) / 2
                                print("mid noise = ", noise)
                                print("confirm no = ", loop_no[-2])
                                # ---------------------------------------
                                # 碰到研磨點 touch_or_not = 1, 未碰到 touch_or_not = 0
                                if len(loop_no) > 7 and (noise + 2) >= (loop_no[-2]) >= (noise - 2):   # 0314 len(loop_no) 原本是> 7
                                    touch_or_not = 1
                                    F_loop_no = np.append(F_loop_no, whilerun)

                                    print("碰到研磨點")
                                else:
                                    print("還沒碰到")

                                if whilerun == F_loop_no[0]:  # 第一個接觸到研磨點的點
                                    # 確定接觸到研磨點，回傳1
                                    ru = open(return_robot_file, "w")
                                    ru.write(str(1))
                                    ru.close()
                                    print('return robot')
                                    time.sleep(1.5)

                                    # 接收研磨點修正距離
                                    revise = open(reviseT_path, 'r')
                                    line = revise.readlines()
                                    revise_txt = []
                                    for x in range(0, len(line)):
                                        revise_txt = line[x].split(' ', 1)
                                    print('研磨點誤差', revise_txt)

                                    protruison_no = int(revise_txt[0])
                                    revise_dis = float(revise_txt[1])

                                    time.sleep(0.1)

                                    # 重新計算補償誤差後的研磨軌跡結束
                                    T, newT = single_pro_euler.euler(protruison_no, revise_dis)
                                    time.sleep(0.1)
                                    # 計算完新軌跡，return_robot_file.txt改為3
                                    re_finish = open(return_robot_file, "w")
                                    re_finish.write(str(3))
                                    re_finish.close()

                                    touch_time = time.time()
                                    recompute_time = touch_time - starttime
                                    re_time = round(recompute_time, 2)

                                    print("開始到碰到研磨點的時間", re_time)

                                elif whilerun > F_loop_no[0]:  # 其他接觸到研磨點的點
                                    print("開始研磨")
                                    if times[-1] > re_time:
                                        aftertouch_times = np.append(aftertouch_times, X)
                                        aftertouch_dataF = np.append(aftertouch_dataF, YF)

                                    plt.subplot()
                                    plt.xlabel("time(s)")
                                    plt.ylabel("F(N)")
                                    # print(aftertouch_times, aftertouch_dataF)
                                    plt.grid()
                                    plt.scatter(aftertouch_times, aftertouch_dataF, color='tab:red')
                                    plt.plot(aftertouch_times, aftertouch_dataF, color='red')

                                plt.pause(0.001)
                                plt.savefig("force data/plot{}".format(run))

                            elif run > 0:
                                print('第幾部分', run)
                                ru = open(return_robot_file, "w")
                                ru.write(str(1))
                                ru.close()
                                print('開始研磨')

                                plt.subplot()
                                plt.xlabel("time(s)")
                                plt.ylabel("F(N)")
                                # print(aftertouch_times, aftertouch_dataF)
                                plt.grid()
                                plt.scatter(times, dataF, color='tab:red')
                                plt.plot(times, dataF, color='tab:red')
                                plt.savefig("force data/plot{}".format(run))

                                if run == 1:
                                    run1_time = times
                                    run1_dataF = dataF
                                elif run == 2:
                                    run2_time = times
                                    run2_dataF = dataF
                                elif run == 3:
                                    run3_time = times
                                    run3_dataF = dataF

                                    plt.scatter(run1_time, run1_dataF, color='tab:red')
                                    plt.scatter(run2_time, run2_dataF, color='tab:green')
                                    plt.scatter(run3_time, run3_dataF, color='tab:blue')
                                    plt.plot(run1_time, run1_dataF, color='tab:red')
                                    plt.plot(run2_time, run2_dataF, color='tab:green')
                                    plt.plot(run3_time, run3_dataF, color='tab:blue')

                                    plt.show()
                                    plt.savefig("force data/plot1_part3")

                        backfile = open(connect_sensor_path, 'r')
                        backvalue = int(backfile.read())
                        if backvalue == 2:
                            oneline_endtime = time.time()
                            # ---------將原本的檔案覆蓋掉裡面的數值----------
                            f = open(connect_sensor_path, "w")
                            f.write(str(0))
                            f.close()

                            # 離開force sensor回傳0
                            ru = open(return_robot_file, "w")
                            ru.write(str(0))
                            ru.close()

                            duringtime = oneline_endtime - starttime
                            print("oneline耗時: {:.2f}秒".format(duringtime))
                            duringtime = round(duringtime, 2)
                            strduringtime = str(duringtime)
                            oneline_time = strduringtime + ' s\t\n'
                            averageline_time_file.write(oneline_time)
                            plt.close(fig=None)
                            break
                    except Exception as e1:
                        print("communicating error " + str(e1))
                ser.close()
            else:
                print("open serial port error")
        else:
            print("等待到達研磨點")
    else:
        print("connect_sensor_path檔案路徑不存在")

















