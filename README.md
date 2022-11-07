# 具全方位視野之自動割草機系統研製
## 大學畢業專題
### 我主要負責如下:
  * 影像處理
  * 系統整合與控制


| 系統架構圖 | 全景攝影機 | 全景攝影機成像原理 |
|:-------:|:-----:|:------:|
|  ![智慧型自動割草系統架構](https://github.com/capcat0515/omniCamaraDetect/blob/omniCamara/images/system_structure.png)  |  ![全景攝影機](https://github.com/capcat0515/omniCamaraDetect/blob/omniCamara/images/omniCamara.png)  |  ![全景攝影機成像原理](https://github.com/capcat0515/omniCamaraDetect/blob/omniCamara/images/omniCamaraToImage.png) |

### 影像處理
#### 測試攝影機不同高度最遠可偵測範圍

假設紅色長方形板子為割草機並偵測，測試攝影機在不同高度時，紅色板子在不同距離下的成像與影像校正後影像

* 攝影機距離地面2M

| 原圖 | 距離攝影機2M(原圖 vs. 影像校正後影像) | 距離攝影機3M(原圖 vs. 影像校正後影像) |
|:-------:|:-----:|:------:|
|  ![](https://github.com/capcat0515/omniCamaraDetect/blob/omniCamara/images/10m_O.png)  |  ![](https://github.com/capcat0515/omniCamaraDetect/blob/omniCamara/images/10m_2h.png)  |  ![](https://github.com/capcat0515/omniCamaraDetect/blob/omniCamara/images/10m_3h.png)  |

 * 攝影機距離地面3M

| 原圖 | 距離攝影機2M(原圖 vs. 影像校正後影像) | 距離攝影機3M(原圖 vs. 影像校正後影像) |
|:-------:|:-----:|:------:|
|  ![](https://github.com/capcat0515/omniCamaraDetect/blob/omniCamara/images/20m_O.png)  |  ![](https://github.com/capcat0515/omniCamaraDetect/blob/omniCamara/images/20m_2h.png)  |  ![](https://github.com/capcat0515/omniCamaraDetect/blob/omniCamara/images/20m_3h.png)  |



#### 偵測割草機方向
假設紅色長方形板子為割草機，藍色板子為割草機的頭，使用PCA技術偵測割草機的方向

| 實驗環境 | 不同位置 |
|:-------:|:-----:|
|  ![](https://github.com/capcat0515/omniCamaraDetect/blob/omniCamara/images/testEnvironment.png)  |  ![](https://github.com/capcat0515/omniCamaraDetect/blob/omniCamara/images/PCA.png)  |


### 系統整合與控制
原本在P1  目標P2

由於地形或機器馬達問題，使得行駛過程中偏離軌道

先機器退回P1，修正與P2方向平行，再往前走


![](https://github.com/capcat0515/omniCamaraDetect/blob/omniCamara/images/revise.png)
