# Week 3 - EKF
---
A2017106 이상엽

## Assignment

- 목표: EKF를 활용한 Radar 계측 데이터를 Lidar 계측 데이터와 융합하여 object의 위치 및 속도를 추정하는 Sensor Fusion 구현
- 목표 기능:
1) `KalmanFilter.updata_ekf` : KF의 measuerment update 단계에서 radar의 measurement matrix를 선형화      
- 구현:
 def update_ekf(self, z):
 
        #Measurment data와 Prediction data의 큰 오차가 발생할 수 있는 경계영역(pi -> -pi)에서의 KF발산을 방지 하기 위해 불연속 지점이 없는 계측 범위(0 ~ 2pi)로 변환 
 
        if z[1] < 0: 
            z[1] = z[1] + 2*pi
        else: 
            z = z
       
        H_j = Jacobian(self.x)         # 1. Compute Jacobian Matrix H_j
        S = np.dot(np.dot(H_j, self.P), H_j.T) + self.R # 2. Calculate S = H_j * P' * H_j^T + R
        K = np.dot(np.dot(self.P, H_j.T), np.linalg.inv(S)) # 3. Calculate Kalman gain K = H_j * P' * Hj^T + R
        
        #Measurment data와 Prediction data의 큰 오차가 발생할 수 있는 경계영역(pi -> -pi)에서의 KF발산을 방지 하기 위해 불연속 지점이 없는 계측 범위(0 ~ 2pi)로 변환 
 
        if atan2(self.x[1],self.x[0]) < 0:
            rho_predict = atan2(self.x[1],self.x[0]) + 2*pi
        else:
            rho_predict = atan2(self.x[1],self.x[0])
        y = z - [sqrt(self.x[0]**2+ self.x[1]**2), rho_predict,(self.x[2]*self.x[0] + self.x[3]*self.x[1])/(sqrt(self.x[0]**2+ self.x[1]**2))] Estimate y = z - h(x')
        
        # 6. Calculate new estimates
        
        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, H_j), self.P)
## Result
<w/o fusion>
![EKF_not_fusion](https://user-images.githubusercontent.com/80674433/112262706-65cd8580-8cb1-11eb-93b0-3018f3f0e0d7.png)

<w/ fusion>
![EKF_fusion](https://user-images.githubusercontent.com/80674433/112262733-75e56500-8cb1-11eb-872a-8f0e43492c2b.png)

- resolution은 높지만 위치에 대한 정보만 나오는 lidar만을 이용하였을 때 보다, resolution은 낮지만 위치 및 속도에 대한 정보까지 나오는 radar를 함께 사용하였을 때 더 정확한 결과를 얻을 수 있었다. 이는 model에서 가정한 부분이 벗어나는 부분(가속도가 있는 경우)에 있어 효과가 더욱 크게 드러났는데, 이는 model의 가정으로 부터 벗어나는 부분을 속도 정보를 이용해서 보완을 해주었기 때문으로 추측된다.
