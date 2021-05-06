# Week 7 - Hybrid A*
---
A2017106 이상엽

## Assignment

- 목표: Hybrid A* 구현 
- 목표 기능:
1) Trajectory generation: in the method HybridAStar.expand(), a simple one-point trajectory shall be generated based on a basic bicycle model. This is going to be used in expanding 3-D grid cells in the algorithm's search operation.
- 구현:

        def expand(self, current, goal):
            g = current['g']
            x, y, theta = current['x'], current['y'], current['t']

            # The g value of a newly expanded cell increases by 1 from the
            # previously expanded cell.
            g2 = g + 1
            next_states = []

            # Consider a discrete selection of steering angles.
            for delta_t in [self.omega_min, self.omega_max, self.omega_step]:
                omega = self.speed / self.length * math.tan(math.radians(delta_t))
                next_theta = (theta + omega) % (2 * np.pi)
                next_x = x + (self.speed * math.cos(theta))
                next_y = y + (self.speed * math.sin(theta))

                next_s = {
                    'f': current['f'] + self.heuristic(next_x, next_y, goal),
                    'g': g2,
                    'x': next_x,
                    'y': next_y,
                    't': next_theta,
                }
                next_states.append(next_s)

            return next_states
                
                
2) Hybrid A* search algorithm: in the method HybridAStar.search(), after expanding the states reachable from the current configuration, the algorithm must process each state (i.e., determine the grid cell, check its validity, close the visited cell, and record the path. You will have to write code in the for n in next_states: loop.    
- 구현:

		    # Initial heading of the vehicle is given in the
        # last component of the tuple start.
        theta = start[-1]
        # Determine the cell to contain the initial state, as well as
        # the state itself.
        stack = self.theta_to_stack_num(theta)
        g = 0
        s = {
            'f': self.heuristic(start[0], start[1], goal),
            'g': g,
            'x': start[0],
            'y': start[1],
            't': theta,
        }
        self.final = s
        # Close the initial cell and record the starting state for
        # the sake of path reconstruction.
        self.closed[stack][self.idx(s['x'])][self.idx(s['y'])] = 1
        self.came_from[stack][self.idx(s['x'])][self.idx(s['y'])] = s
        total_closed = 1
        opened = [s]
        # Examine the open list, according to the order dictated by
        # the heuristic function.

        while len(opened) > 0:
            # for the hybrid A* algorithm.
            opened.sort(key=lambda s : s['f'], reverse=True)
            curr = opened.pop()
            x, y = curr['x'], curr['y']
            if (self.idx(x), self.idx(y)) == goal:
                self.final = curr
                found = True
                break

            # Compute reachable new states and process each of them.
            next_states = self.expand(curr, goal)
            for state in next_states:
                # Check validity and then add to the next_states list.
                next_x = self.idx(state['x'])
                next_y = self.idx(state['y'])
                next_stack = self.theta_to_stack_num(state['t'])

                if (0 <= state['x'] < grid.shape[0]) and (0 <= state['y'] < grid.shape[1]) and grid[(next_x, next_y)] == 0 and \
                        self.closed[next_stack][next_x][next_y] == 0:
                    if self.check_cross_obstacle(curr, state, grid):
                        opened.append(state)
                        self.closed[next_stack][next_x][next_y] = 1
                        self.came_from[next_stack][next_x][next_y] = curr

                        total_closed += 1
        else:
            # We weren't able to find a valid path; this does not necessarily
            # mean there is no feasible trajectory to reach the goal.
            # In other words, the hybrid A* algorithm is not complete.
            found = False

        return found, total_closed
3) Discretization of heading: in the method HybridAStar.theta_to_stack_num(), you will write code to map the vehicle's orientation (theta) to a finite set of stack indices.

          def theta_to_stack_num(self, theta):
                  # given theta represented in radian. Note that the calculation
                  # should partition 360 degrees (2 * PI rad) into different
                  # cells whose number is given by NUM_THETA_CELLS.
                  return int(theta // (2 * np.pi / self.NUM_THETA_CELLS))
        
5) Heuristic function: in the method HybridAStar.heuristic(), you define a heuristic function that will be used in determining the priority of grid cells to be expanded. For instance, the distance to the goal is a reasonable estimate of each cell's cost

        def heuristic(self, x, y, goal):
                return int(math.sqrt((goal[0] - int(x)) ** 2 + (goal[1] - int(y)) ** 2))
                # return abs(goal[0] - x) + abs(goal[1] - y)

## Result 

![Result](https://user-images.githubusercontent.com/80674433/117334696-45f6c780-aed5-11eb-84bd-654c4a17a857.PNG)

## Reference code
https://github.com/dollking/vehicle-intelligence-2021
