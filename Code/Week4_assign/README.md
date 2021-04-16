# Week 4 - PF
---
A2017106 이상엽

## Assignment

- 목표: GNSS, In-vehicle senor, Lidar, MAP 정보를 활용하여 자차량의 위치 추정을 위한 Particle Filter 설계 
- 목표 기능:
1) `Update_weights` : 각 Particle들의 가중치를 Predicted Measurment와 Lidar Measurment를 비교하여 업데이트      
- 구현:
        def update_weights(self, sensor_range, std_landmark_x, std_landmark_y,
                       observations, map_landmarks):
        for p in self.particles:
        # 1. Select the set of landmarks that are visible
        #    (within the sensor range).
        # 2. Transform each observed landmark's coordinates from the
        #    particle's coordinate system to the map's coordinates. 
            msrmnt_prdct = []
            msrmnt_tf_m = []
            Ppnt = {'x':p['x'], 'y':p['y']}
            for j in map_landmarks.items():
                if distance(Ppnt, j[1])<= sensor_range:
                    msrmnt_prdct.append({'id':j[0],'x':j[1]['x'], 'y':j[1]['y']})
                    p['assoc'].append(j[0])
            for i in observations:
                msrmnt_tf_m.append({ 'x': p['x'] + (i['x'])*np.cos(p['t']) - (i['y'])*np.sin(p['t']), 
                'y': p['y']+ (i['x'])*np.sin(p['t'])+(i['y'])*np.cos(p['t'])})
 
        # 3. Associate each transformed observation to one of the
        #    predicted (selected in Step 1) landmark positions.
        #    Use self.associate() for this purpose - it receives
        #    the predicted landmarks and observations; and returns
        #    the list of landmarks by implementing the nearest-neighbour
            if not msrmnt_prdct:
                continue
        #    association algorithm.
            assoc_list = []
            assoc_list = self.associate(msrmnt_prdct, msrmnt_tf_m)
            # print(p['assoc_cord'])
            # print('--------------------')
            # print(len(assoc_list))
            # print('-------------------------------------')
            # print(len(observations))
        # 4. Calculate probability of this set of observations based on
        #    a multi-variate Gaussian distribution (two variables being
        #    the x and y positions with means from associated positions
        #    and variances from std_landmark_x and std_landmark_y).
        #    The resulting probability is the product of probabilities
        #    for all the observations.
        # 5. Update the particle's weight by the calculated probability.
            p['w'] = 1

            for k in range(len(assoc_list)):
                p['w'] *= self.normal_dist(distance(msrmnt_tf_m[k],p), distance(assoc_list[k],p), np.sqrt(std_landmark_x**2+std_landmark_y**2))
                
2) `Resampling` : 각 Particle들의 가중치에 비례하게 다음 Step에 사용할 Particle들을 선정      
- 구현:          
       #def resample(self):
           weights_sum = []  
           for i in self.particles:  
               weights_sum += i['w']  
           particles_rsmpl = []  
           for i in range(self.num_particles):  
               r = np.random.uniform(0, weights_sum)  
               for p in self.particles:  
                   if r > p['w']:  
                       r -= p['w']  
                   else:  
                       particles_rsmpl.append(cp.deepcopy(p))  
                       break  
           self.particles = particles_rsmpl
## Result
![Particle_result](https://user-images.githubusercontent.com/80674433/114971149-b9994c00-9eb6-11eb-886d-a731a17f7f47.gif)

