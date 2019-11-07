# First import the library
import pyrealsense2 as rs
import numpy as np
import cv2
import socket
import time


# ---- Socket configuration

# family: socket.AF_INET (用於網路通訊)、socket.AF_UNIX (同一台機器通訊)
# type: socket.SOCK_STREAM (TCP)、socket.SOCKET_DGRAM (UDP)
sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
host = '192.168.0.249'          # 欲連線的主機
host = '127.0.0.1'          # 欲連線的主機
port = 5000                 # 欲連線的主機埠號
loc = (host, port)
try:
    sock.connect(loc)              # 透過 ip 與 port 進行連線
except:
    print('socket Error, 請確認網路狀況')


# 啟動 Realsense 攝影機
while True:
    try:
        pipeline = rs.pipeline()
        pipeline.start()
        print('Camera Start')
        break
    except:
        print('Camera Error, try again...')
        

min_depth = 600    # 深度允許最小值
max_depth = 1000    # 深度允許最大值

# 使用遞迴進行座標點的二分法, 將相近的點做合併
def getClusterPoint(points):       
    dist = 50               # 距離在 dist 之內進行合併
    gp1 = [points[0]]       # 用第一個點當基準點, 將點分成 2 群
    gp2 = []
    for i in range(1 , len(points)):   
        d = pow(pow(points[0][0] - points[i][0], 2)  +  pow(points[0][1] - points[i][1], 2), 0.5)
        d = round(d, 1)
        if d < dist:    # 若兩點距離小於 dist, 分入第一群
            gp1.append(points[i])
        else:           # 其餘的都先分入第二群
            gp2.append(points[i])
    f_points.append(np.mean(gp1, axis=0))   # 將第一個群集的平均點加入到最終點
    if gp2:  #  如果第 2 群有座標點, 遞迴繼續分 2 群
        getClusterPoint(gp2)    

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
#    print(depth_img.shape)                          # 深度影像的尺寸為 720x1280
    
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
#        print('偵測到移動')
        #----  計算輪廓平均座標點
        points = []                     # 儲存所有輪廓的平均座標點
        for c in contours:
            if cv2.contourArea(c) > 12000:  # 面積大於 12000 的輪廓才需要計算平均座標點
                avp = np.mean(c, axis=0)    # 求形成輪廓的點的平均座標
                points.append(avp)

        if points:                          # 如果有大於 12000 的輪廓平均座標點
            points = np.array(points)
            points = points[:, 0, :]        # 這是因為多了中間一維 [n, 1, m] -> [n, m]
            # ---- 將過於相近的點合併
            f_points = []                      # 合併後的所有最終座標點
            getClusterPoint(points)         # 將所有的輪廓平均座標點丟入遞迴函式中進行分組
            # print(f'找到 {len(f_points)} 個點')
            
            # ---- 繪製輪廓平均點
            for p in f_points:
                px, py = int(p[0]), int(p[1])
                cv2.circle(depth_img_8U, (px, py), 10, (0, 0, 0), -1)
            
            # ---- 將最終座標透過 socket 傳送到 Server 端
            who = b'machineA'
            cord = ''
            for p in f_points:
                px, py = round(p[0], 1), round(p[1], 1)
                cord += '@' + str(px) + ',' + str(py)
            cord = str.encode(cord)     # 字串轉 byte 型別
            message = who + cord        # 欲傳送的訊息
            try:
                sock.sendall(message)
                response = sock.recv(4096)  # 接收伺服器的響應
#                print('傳送成功')
            except socket.error:
                print('傳送失敗, 等待 3 秒後再次與 server 進行連線')
                time.sleep(3)
                # 再次嘗試使用 socket 連線
                try:
                    sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
                    sock.connect(loc)              # 透過 ip 與 port 進行連線
                    print('socket ok')
                except:
                    print('socket Error, 請確認網路狀況')
        else:
            print('沒有偵測到面積大於 12000 的輪廓')
            pass
    else:
        print('沒有偵測到輪廓')
        pass
    cv2.imshow('Frame', depth_img_8U)        # 顯示影像
    
    k = cv2.waitKey(10)                # 等待按鍵輸入
    if k == ord('q') or k == ord('Q'):  # 按下 Q(q) 結束迴圈
        print('exit')
        cv2.destroyAllWindows()         # 關閉視窗
        sock.close()
        break
