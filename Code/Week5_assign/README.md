# Week 5 - DPA
---
A2017106 이상엽

## Assignment

- 목표: Dynamic Programming Approch 기법을 활용하여 출발 지점 으로 부터 목표지점까지의 최소 경로 탐색 
- 목표 기능:
1) Mark the final state with a special value that we will use in generating the final path policy and Try to use simple arithmetic to capture state transitions. 
- 구현:

        if (y, x) == goal and value[(t, y, x)] > 0:
                value[(t, y, x)] = 0
                policy[(t, y, x)] = -99
                change = True
            # state transitions
            elif grid[(y, x)] == 0:
                for i in range(len(init)):
                    o2 = (t + action[i]) % 4
                    y2 = y + forward[o2][0]
                    x2 = x + forward[o2][1]

                    if (0 <= y2 < grid.shape[0]) and (0 <= x2 < grid.shape[1]) and (grid[(y2, x2)] == 0):
                        v2 = value[(o2, y2, x2)] + cost[i]
                        if v2 < value[(t, y ,x)]:
                            value[(t, y, x)] = v2
                            policy[(t, y, x)] = action[i]
                            change = True
                
                
2) Now navigate through the policy table to generate a sequence of actions to take to follow the optimal path.      
- 구현:

		    y, x, o = init
		    policy2D[(y, x)] = policy[(o, y, x)]
		    #iteration until arrive at goal point
		    while policy[(o, y, x)] != -99:
			if policy[(o, y, x)] == action[1]:
			    o2 = o
			    policy2D[(y, x)] = action_name[1]
			elif policy[(o, y, x)] == action[0]:
			    o2 = (o - 1) % 4
			    policy2D[(y, x)] = action_name[0]
			elif policy[(o, y, x)] == action[2]:
			    o2 = (o + 1) % 4
			    policy2D[(y, x)] = action_name[2]
				y += forward[o2][0]
				x += forward[o2][1]
				o = o2
				policy2D[(y, x)] = policy[(o, y, x)]
				if policy[(o, y, x)] == -99:
				policy2D[(y, x)] = '@'
	
	
## Result

	[' ' ' ' ' ' 'R' '#' 'R']
	[' ' ' ' ' ' '#' ' ' '#']
	['@' '#' '#' '#' '#' 'R']
	[' ' ' ' ' ' '#' ' ' ' ']
	[' ' ' ' ' ' '#' ' ' ' ']

