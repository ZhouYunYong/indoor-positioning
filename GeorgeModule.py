'''

'''

import time
import random
import numpy as np




# 讀取外部檔案進行設定
def get_setting():    #←將「讀取設定檔」寫成函式, 可讓程式易讀易用
    res = []     #←準備一個空串列來存放讀取及解析的結果
    try:              # 使用 try 來預防開檔或讀檔錯誤
        with open('setting.txt', 'r', encoding='utf-8-sig') as f:  # 用 with 以讀取模式開啟檔案
            slist = f.readlines()     # 以行為單位讀取所有資料
            print('讀入：', slist)    # 輸出讀到的資料以供確認
            for lst in slist:           #←走訪每一張股票字串
                s = lst.split('=')   #←將股票字串以逗號切割為串列
                try:
                    res.append(int(s[1].strip())) #←將切割結果加到 res 中
                except:
                    res.append(s[1].strip()) #←將切割結果加到 res 中
    except:                     # ↑        ↑          ↑
                             #去除左右空白   將股價轉換為 float
        print('setting.txt 讀取錯誤')
    return res   #←傳回解析的結果, 但如果開檔或讀檔錯誤則會傳回 []



# ---- 接收兩點座標, 計算之間距離後回傳
def calcu_dist(p1, p2):       
    t = pow( (p1[0] - p2[0]), 2) + pow( (p1[1] - p2[1]), 2)
    return round(pow(t, 0.5), 2)


# ----　判斷目前取得的座標是否為新座標 (有移動)
def is_move(f_points_pre, f_points, noise_dist):
    d = noise_dist   # 新座標若與舊座標距離差小於 d 則移動距離太小, 不傳送 (視為雜訊)
    if f_points == []:
        # print('point disappear')
        return [], f_points_pre
    if f_points_pre == []:
        # print('new points')
        return f_points, f_points
    # --- 有新座標也有舊座標 --- #
    pts = [list(ps.copy()) for ps in f_points]        # 先假設所有新的座標點都要傳送
    for p in f_points:
        for pre in f_points_pre:
            dist = calcu_dist(pre, p)
            # print('dist', dist)
            if dist < d:        # 如果新座標點與舊座標點之前距離出現小於 d, 不傳送
                pts.remove(list(p))   # 將座標點移出欲傳送清單
                break

    # 如果 pts 是空, 則代表沒有新的點要用 socket 傳送, 這時, 請維持原始已傳送清單
    # 若不為空, 則以當前偵測到的座標點作為新的已傳送清單
    return pts, f_points_pre if pts == [] else f_points



# ---- 將 points 中相鄰的座標點合併成一點 (使用遞迴進行座標點的二分法)
def getClusterPoint(points, dist=50): 
    f_points = []   # 用來儲存所有合併後的座標
    def inner(points, dist):
        gp1 = [points[0]]   # 先用第一個座標點當基準點, 將所有座標點依此基準分成 2 群 (基準點放 gp1 群)
        gp2 = []            # 第 2 個座標點群
        # ---- 將每一點都與基準點求距離, 若距離小於 dist, 分入 gp1、大於則分入 gp2
        for i in range(1 , len(points)):  
            d = calcu_dist(gp1[0], points[i])   # 計算兩點距離
            if d < dist:
                gp1.append(points[i])   # 若兩點距離小於 dist, 分入 gp1
            else:           
                gp2.append(points[i])   # 其餘的都先分入 gp2
        f_points.append(np.mean(gp1, axis=0))   # 計算 gp1 中所有座標的平均座標, 並加入到最終點清單中
        if gp2:  
            inner(gp2, dist)    # 如果第 2 群中有座標點, 使用遞迴繼續分 2 群
    inner(points, dist)
    return f_points

# 分隔線
def div(text):
    print('\n' + text + '\n')

# 檢查
def check(obj):
    print('-'*10+'head'+'-'*10)
    print('Value:', obj)
    print('Type:', type(obj))
    try:
        print('array Shape:', obj.shape)
    except:
        pass
    
    try:
        print('List length:', len(obj))
    except:
        pass
    print('-'*10+'foot'+'-'*10)
    
# ---- 尋找最大深度值的平均座標
def max_depth_mean_point(dep_img, c):
   
    x_min, y_min = np.min(c, axis=0)[0] # [0] 因為多包了一層 []
    x_max, y_max = np.max(c, axis=0)[0]
    
    # print('2', dep_img[640, 360])
    # 先取得 area 所有像素
    area = dep_img[y_min:y_max, x_min:x_max]
    
    # print(np.max(area))

    # 找出全域最大值的索引 (座標)
    Y, X = np.where(area==np.max(area))
    # 算出平均座標
    meanY = np.mean(Y) + y_min
    meanX = np.mean(X) + x_min

    
    
    return meanY, meanX, area