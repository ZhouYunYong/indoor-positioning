# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2
import socket
import time
import GeorgeModule as Gm     # 匯入詠運模組

# ----------- 相關數值調整
res = Gm.get_setting()
print('傳回：', res)             # 輸出傳回的結果
min_depth = res[0]              # 偵測深度的最小值 (最近距離)
max_depth = res[1]              # 偵測深度的最大值 (最遠距離)
max_contorArea = res[2]         # 最大偵測輪廓面積
min_contorArea = res[3]         # 最小偵測輪廓面積
noise_dist = res[4]             # 雜訊距離控制
combi_dist = res[5]             # 偵測點結合距離
who = res[6]                    # 機器名稱
who = str.encode(who)           # 字串轉 byte 型別
print('machine name: ', who)
""" 機器清單
machineA
machineB
machineC
machineD
machineE
machineF
"""
# min_depth = 300
# max_depth = 600
# ---------- Socket configuration
sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM) 
#host = '127.0.0.1'         # local server
host = '192.168.1.50'       # 欲連線的主機
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

# ---- 取出攝影機中的影像
while True:
    try:
        frames = pipeline.wait_for_frames()     # 取得 frame 物件
        depth = frames.get_depth_frame()        # 取得深度影像
        # rgb = frames.get_color_frame()          # 取得 RBG 影像
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
    # rgb_data = rgb.as_frame().get_data()             # 取得 rgb 影格資料
    # rgb_img = np.asanyarray(rgb_data)                # 將 rgb 影格資料轉為 array 
    depth_data = depth.as_frame().get_data()         # 取得深度影格資料
    depth_img = np.asanyarray(depth_data)            # 將深度影格資料轉為 array 
#    print(depth_img.shape)                          # 深度影像的尺寸為 1280x720 (shape為 720,1280)
    
    # ---- 對深度值進行正規化：0～255, 取得 depth_img_8U 深度影像
    depth_img[depth_img < min_depth] = min_depth     # 深度值小於 min_depth 設為 min_depth
    depth_img[depth_img > max_depth] = min_depth     # 深度值大於 max_depth 設為 min_depth
    depth_img_8U = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1)   # 將深度直 map 到 0-255

   
    # ---- 進行第 1 次輪廓偵測
    # contours 的形式: list : [   [ [輪廓點1, 輪廓點2...]  ],  [第 2 個輪廓], ...   ]  裡面是 numpy array
    # blur = cv2.GaussianBlur(depth_img_8U, (13, 13), 5)    # 高斯模糊
    ret, thresh = cv2.threshold(depth_img_8U, 20, 255,      # 門檻值
                                cv2.THRESH_BINARY)
    contours, _ = cv2.findContours( thresh,                  # 找到輪廓
                                    cv2.RETR_EXTERNAL,
                                    cv2.CHAIN_APPROX_SIMPLE)

    if contours:    # 如果有偵測到輪廓
        # ---- 去除小躁點
        for c in contours: 
            if cv2.contourArea(c) < 1000:
                x_min, y_min = np.min(c, axis=0)[0] # [0] 因為多包了一層 []
                x_max, y_max = np.max(c, axis=0)[0]
                d = 1
                try:
                    depth_img_8U[y_min-d: y_max+d, x_min-d: x_max+d] = 0    # 去除
                except:
                    depth_img_8U[y_min: y_max, x_min: x_max] = 0    # 去除

        # ----進行第 2 次輪廓偵測，用去除躁點的影像在進行一次輪廓偵測
        ret, thresh = cv2.threshold(depth_img_8U, 20, 255,      # 門檻值
                                    cv2.THRESH_BINARY)
        contours, _ = cv2.findContours( thresh,                  # 找到輪廓
                                        cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
  
        if contours:    # 如果有偵測到輪廓
            # ---- 繪製輪廓
            # img = np.zeros((720, 1280))
            # cv2.drawContours(   img,      # 繪製輪廓線
            #                     contours, -1, 
            #                     (255, 255, 255), 2)
            # cv2.namedWindow('img', 0)
            # cv2.imshow('img', img)        # 顯示影像
            # ---- 繪製輪廓
            
            # ----  計算輪廓平均座標點
            points = []                     # 儲存所有輪廓的平均座標點
            for c in contours:      # [ [輪廓點1, 輪廓點2...]
                if max_contorArea > cv2.contourArea(c) > min_contorArea:  # 面積介於此範圍的輪廓才需要計算平均座標點
                    
                    # -- 繪製包住輪廓的矩形框
                    x_min, y_min = np.min(c, axis=0)[0] # [0] 因為多包了一層 []
                    x_max, y_max = np.max(c, axis=0)[0]
                    cv2.rectangle(depth_img_8U, (x_min, y_min), (x_max, y_max), 255, 2)

                    
                    # ---- 作法1. 在深度影像中的輪廓 c , 尋找最大深度值的平均座標
                    # meanY, meanX, area = Gm.max_depth_mean_point(depth_img_8U, c)
                    # points.append( [meanX, meanY] )     # 取得輪廓中最大深度值面積的平均座標點

                    # ---- 作法 2. 求輪廓的平均座標
                    avp = np.mean(c, axis=0)[0]    # 求形成輪廓的點的平均座標
                    points.append(avp)

                    # ---- 作法 3. 求輪廓與中心點的最近座標
                    # center = (640, 360)
                    # x_min, y_min = np.min(c, axis=0)[0] # [0] 因為多包了一層 []
                    # x_max, y_max = np.max(c, axis=0)[0]
                    # # ps = [[x_min, y_min],
                    # #       [x_max, y_min],
                    # #       [x_min, y_max],
                    # #       [x_max, y_max]]
                    # ps = [[x_min, y_min],
                    #       [(x_max + x_min) / 2, y_min],
                    #       [x_max, y_min],
                    #       [x_min, (y_max + y_min) / 2],
                    #       [(x_max + x_min) / 2, (y_max + y_min) / 2],
                    #       [x_max, (y_max + y_min) / 2],
                    #       [x_min, y_max],
                    #       [(x_max + x_min) / 2, y_max],
                    #       [x_max, y_max]]
                    # d_min = float('INF')
                    # for p in ps:
                    #     d = Gm.calcu_dist(p, center)
                    #     if d < d_min:
                    #         fp = p
                    #         d_min = d
                    
                    # # 將座標點加入清單中
                    # points.append(fp)

                    # ---- 作法 4. 計算平均點與中心點的向量, 並計算單位向量進行位移
                    # center = (640, 360)
                    # offset = 100      # 位移量 (單位向量數量)
                    # px, py = np.mean(c, axis=0)[0]    # 求形成輪廓的點的平均座標
                    # length = Gm.calcu_dist((px, py), center)
                    # unit_vec = ( (center[0] - px) / length, (center[1] - py) / length )

                    # fp = ( (px + unit_vec[0] * offset), (py + unit_vec[1] * offset) )

                    # points.append(fp)



            if points:                          # 如果有最大深度值面積的平均座標點
                points = np.array(points)
                # points = points[:, 0, :]        # 這是因為多了中間一維 [n, 1, m] -> [n, m]
                # ---- 將過於相近的點合併
                f_points = Gm.getClusterPoint(points, dist=combi_dist)    # 將所有的輪廓平均座標點丟入遞迴函式中進行分組
                
                # ---- 繪製輪廓平均點
                for p in f_points:
                    px, py = int(p[0]), int(p[1])
                    cv2.circle(depth_img_8U, (px, py), 10, 255, 30)
                
                # ---- 判斷座標點移動程度 ---- #
                pts, passed_points = Gm.is_move(passed_points, f_points, noise_dist)  # 穩定點
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
        

       
    cv2.namedWindow('depth_img_8U', 0)
    cv2.imshow('depth_img_8U', depth_img_8U)        # 顯示影像



    # cv2.namedWindow('area', 0)
    # try:
    #     cv2.imshow('area', area)        # 顯示影像
    # except:
    #     pass

    k = cv2.waitKey(10)                # 等待按鍵輸入
    if k == ord('q') or k == ord('Q'):  # 按下 Q(q) 結束迴圈
        print('exit')
        cv2.destroyAllWindows()         # 關閉視窗
        sock.close()
        break
        
