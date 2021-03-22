# Week 2 - Markov Localization

---
A2017106 이상엽


## Assignment

- 목표: motion model, observation model을 활용하여 simple Markov localizer 구현
- 목표 기능:
1) `motion_model()` : 매 step action을 수행함에 따라 자기 위치에 대한 확률 분포 함수 계산 - Bayes Filter의 predicton step  
2) `observation_model()`: action 후 관측 된 결과를 motion model의 확률 분포 함수에 업데이트 - Bayes Filter의 measurement update step 
- 가정:
1) 물체의 거동을 묘사하는 데 있어 Markov Chain 이 성립한다.
2) 센서는 확률 변수 내에 있는 나무들은 모두 계측할 수 있다.(observation을 통해 계측된 객체의 수와 map을 통해 파악한 pseudo range 내 객체의 수가 같을 때만 확률계산)
- 구현:
1)  `motion_model()` 
        def motion_model(position, mov, priors, map_size, stdev):
            # Initialize the position's probability to zero.
            position_prob = 0
            for i in range(map_size):
                    position_prob += norm_pdf(25-i, mov, stdev)*priors[i]
            return position_prob
2)  `observation_model()` 
        def observation_model(landmarks, observations, pseudo_ranges, stdev):
            # Initialize the measurement's probability to one.
            distance_prob = 1
            if len(observations)==len(pseudo_ranges):
                 for i in range(len(observations)):
                    distance_prob *= norm_pdf(observations[i], pseudo_ranges[i], stdev)
            else:
                distance_prob = 0
            return distance_prob
