# Week 6 - GNB
---
A2017106 이상엽

## Assignment

- 목표1: 주어진 observation을 바탕으로 GNB를 통해 target vehicle의 다음 behaviour를 예측 
- 목표 기능:
1) Collect the data and calculate mean and standard variation for each class. Record them for later use in prediction. 
- 구현:

        def train(self, X, Y):  
        self.obslen = len(X[0])  
        self.label = ['left', 'keep', 'right']  
        self.statistics = {label: {} for label in self.label}  
        map_XY = {label: [] for label in self.label}  
        for x, y in zip(X, Y):  
            map_XY[y].append(x)  

        for label, data in map_XY.items():  
            data = np.array(data)  
            self.statistics[label].update({  
                'mu': data.mean(axis=0),  
                'sigma': data.std(axis=0),  
                'shape': data.shape[0],  
            })
                
                
2) Calculate Gaussian probability for each variable based on the mean and standard deviation calculated in the training process.
   Multiply all the probabilities for variables, and then normalize them to get conditional probabilities. Return the label for the highest conditional probability.      
- 구현:

		    def predict(self, observation):  
        prob = {label: [] for label in self.label}  
        for label in self.label:  
            for i in range(self.obslen):  
                g_prob = float(  
                    gaussian_prob(observation[i], self.statistics[label]['mu'][i], self.statistics[label]['sigma'][i]))  # calculate probability
                prob[label].append(g_prob)  
            shape = self.statistics[label]['shape']  
            prob[label].append(float(shape))  

        cond_prob = {label: 1.0 for label in self.label}  
        for i in range(self.obslen + 1):  
            norm = sum(prob[label][i] for label in self.label)  # calculate normalization factor
            for label in self.label:  
                p = prob[label][i] / norm  
                cond_prob[label] *= p  

        max_label = self.label[0]  
        max_cond_prob = 0.0  
        for label in self.label:   # return the higest probability
            if max_cond_prob < cond_prob[label]:  
                max_label = label  
            max_cond_prob = cond_prob[label]  
        return max_label
	
	
## Result of 목표1
You got 60.40 percent correct

- 목표2: 예측된 behaviour를 바탕으로 다음 motion을 결정할 FSM정의 및 cost function 정의 
- 목표 기능:
1) Implement the transition function code for the vehicle's behaviour planning finite state machine, which operates based on the cost function (defined in a separate module cost_functions.py). 
- 구현:

        def choose_next_state(self, predictions):  
        states = self.successor_states()  
        costs = []  
        for state in states:  
          trajectory = self.generate_trajectory(state, predictions)  #state와 예측값을 고려해 trajectory 후보 생성
          if trajectory:  
            cost = calculate_cost(self, trajectory, predictions)  #각 trajectory 마다 costfunction 계산
            costs.append({  
              'cost': cost,  
              'state': state,  
              'trajectory': trajectory  
            })  

        lowest_cost = min(costs, key=lambda s: s['cost'])  #가장 적은 cost를 다음 state로 결정
        return lowest_cost['trajectory'] 
        
2) Define 'goal_distance_cost' and 'inefficiency_cost'function
- 구현:
  <goal_distance_cost>

       def goal_distance_cost(vehicle, trajectory, predictions, data):
        dist = abs(data.end_distance_to_goal)
        if dist:
            cost = 1 + abs(vehicle.goal_lane - data.intended_lane - data.final_lane)/dist 
        else:
            cost = 1
        return cost
   
   <inefficiency_cost>
   
       def inefficiency_cost(vehicle, trajectory, predictions, data):
        '''
        Cost becomes higher for trajectories with intended lane and final lane
        that have slower traffic.
        '''
        ln_intended = velocity(predictions, data.intended_lane) or vehicle.target_speed
        ln_final = velocity(predictions, data.final_lane) or vehicle.target_speed

        cost = float( vehicle.target_speed - ln_intended - ln_final) / vehicle.target_speed
        return cost

## Result of 목표2
You got to the goal in 34 seconds!

## Reference code
https://github.com/boranorben/vehicle-intelligence-2021
