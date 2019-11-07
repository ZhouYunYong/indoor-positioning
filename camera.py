# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2
import socket
import time
import GeorgeModule as m     # 匯入詠運模組



# ----------- 相關數值調整
res = m.get_setting()
print('傳回：', res)             # 輸出傳回的結果
min_depth = res[0]              # 偵測深度的最小值 (最近距離)
max_depth = res[1]              # 偵測深度的最大值 (最遠距離)
max_contorArea = res[2]         # 最大偵測輪廓面積
min_contorArea = res[3]         # 最小偵測輪廓面積
noise_dist = res[4]             # 雜訊距離控制
combi_dist = res[5]             # 偵測點結合距離
who = res[6]                    # 機器名稱

print('machine name: ', who)
"""
machineA
machineB
machineC
machineD
machineE
machineF
"""

# ---------------

# ---------- Socket configuration
sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM) 
#host = '192.168.0.249'          # 欲連線的主機
host = '192.168.1.50'              # local server
#host = '192.168.0.244'          # Rain Server
#host = '192.168.100.104'        # Rain Latta Panda (DHCP)



port = 6666                 # 欲連線的主機埠號
loc = (host, port)
# ---------------


# ---- 座標點移動程度 設定 ----#
passed_points = []        # 曾經用 socket 傳過的座標點
# ---------------


# ----------------------- 主程式 ---------------------------- #

# ---- 啟動 Realsense 攝影機, 若啟動失敗, 會持續嘗試, 直到連上
while True:
    try:
        pipeline = rs.pipeline()
        pipeline.start()
        print('Camera Start')
        break
    except:
        print('Camera Error, try again...')

# ---- 
while True:

    try:
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()
        rgb = frames.get_color_frame()
    except:
        print('Camera Error, 請確認攝影機狀態')
        # 重新啟動攝影機
        try:
            pipeline = rs.pipeline()
            pipeline.start()
        except:
            print('try....')
            continue

    if not depth: continue
    # get RGB image 
    rgb_data = rgb.as_frame().get_data()             # 取得 rgb 影格資料
    rgb_img = np.asanyarray(rgb_data)                # 將 rgb 影格資料轉為 array 
    depth_data = depth.as_frame().get_data()         # 取得深度影格資料
    depth_img = np.asanyarray(depth_data)            # 將深度影格資料轉為 array 
#    print(depth_img.shape)                          # 深度影像的尺寸為 1280x720 (shape為 720,1280)
    
    depth_img[depth_img < min_depth] = min_depth     # 深度值小於 min_depth 設為 min_depth
    depth_img[depth_img > max_depth] = min_depth     # 深度值大於 max_depth 設為 min_depth
    depth_img_8U = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)   # 將深度直 map 到 0-255

    
    img_now = cv2.GaussianBlur(depth_img_8U, (13, 13), 5)    # 高斯模糊
    ret, thresh = cv2.threshold(img_now, 25, 255,   # 門檻值
                                cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(img_now,       # 找到輪廓
                                    cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)
    if contours:    # 如果有偵測到輪廓
        # cv2.drawContours(   depth_img_8U,      # 繪製輪廓線
        #                     contours, -1, 
        #                     (255, 255, 255), 2)
        # print('偵測到移動')
        #----  計算輪廓平均座標點
        points = []                     # 儲存所有輪廓的平均座標點
        
        for c in contours:
            if max_contorArea > cv2.contourArea(c) > min_contorArea:  # 面積介於此範圍的輪廓才需要計算平均座標點
                avp = np.mean(c, axis=0)    # 求形成輪廓的點的平均座標
                points.append(avp)

        if points:                          # 如果有大於 12000 的輪廓平均座標點
            points = np.array(points)
            # [  [[1,2,..m]],   [[1,2,..m]],   [[n]] ]
            points = points[:, 0, :]        # 這是因為多了中間一維 [n, 1, m] -> [n, m]
            # ---- 將過於相近的點合併
            f_points = m.getClusterPoint(points, dist=combi_dist)    # 將所有的輪廓平均座標點丟入遞迴函式中進行分組

            # ---- 繪製輪廓平均點
            for p in f_points:
                px, py = int(p[0]), int(p[1])
                cv2.circle(depth_img_8U, (px, py), 10, (0, 0, 0), -1)
            
            # ---- 判斷座標點移動程度 ---- #
            pts, passed_points = m.is_move(passed_points, f_points, noise_dist)  # 穩定點
            # ---- 將最終座標透過 socket 傳送到 Server 端
            if pts != []:
                # 繪製需傳送的座標點
                for p in pts:
                    px, py = int(p[0]), int(p[1])
                    cv2.circle(depth_img_8U, (px, py), 10, (0, 0, 0), -1)
                # passed_points = [p.copy() for p in f_points]  # 將目前座標點更新為已傳送清單
                
                cord = ''
                for p in pts:
                    px, py = round(p[0], 1), round(p[1], 1)
                    cord += '@' + str(px) + ',' + str(py)
                cord = str.encode(cord)     # 字串轉 byte 型別
                message = who + cord        # 欲傳送的訊息
                print(message)
                try:
                    sock.sendto(message, loc)
#                    response = sock.recv(4096)  # 接收伺服器的響應
                    print('send sucess')
                except socket.error:
                    print('socket send fail, wait 3 second')
                    time.sleep(3)
                    # 再次嘗試使用 socket 連線
                    try:
                        sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
#                        sock.connect(loc)              # 透過 ip 與 port 進行連線
                        print('socket ok')
                    except:
                        print('socket Error, check internet')
                # ---- 將最終座標透過 socket 傳送到 Server 端
            else:
                # print('No new points to socket')
                pass
        else:
#            print('沒有偵測到面積大於 12000 的輪廓')
            pass
    else:
#        print('沒有偵測到輪廓')
        pass
    cv2.namedWindow('Frame', 0)
    cv2.imshow('Frame', depth_img_8U)        # 顯示影像
    
    k = cv2.waitKey(10)                # 等待按鍵輸入
    if k == ord('q') or k == ord('Q'):  # 按下 Q(q) 結束迴圈
        print('exit')
        cv2.destroyAllWindows()         # 關閉視窗
        sock.close()
        break
        
