import numpy as np
from tools import State

class RRT:
    def __init__(self, num_seconds, route='rrt', num_obstacles=100, start=None, end=None, speed=None):
        # setup goal
        self.start = np.array([0, 0, -5]) if start is None else start
        self.end = np.array([45, 45, -20]) if end is None else end
        self.num_seconds = num_seconds
        self.speed = speed
        
        # setup environment
        self.size = np.array([50, 50, 25])
        self.bottom_corner = np.array([-5, -5, -25])

        # Setup obstacles
        self.num_obstacles = num_obstacles
        self.obstacle_size = np.random.uniform(2, 5, self.num_obstacles)
        self.obstacle_loc = np.random.beta(1.5, 1.5, (num_obstacles, 3))*self.size + self.bottom_corner
        # Make sure there's none too close to our start or end
        for i in range(self.num_obstacles):
            while np.linalg.norm(self.obstacle_loc[i] - self.start) < 10 or \
                        np.linalg.norm(self.obstacle_loc[i] - self.end) < 10:
                self.obstacle_loc[i] = np.random.beta(2, 2, 3)*self.size + self.bottom_corner

        # setup RRT
        self.step_size = 5
        self.tree = self.start.reshape(1,-1)
        self.dist = [0]
        self.parent = [0]
        self.finish = [False]
        self._run_rrt()

        self.eps = 1e-5

    def _run_rrt(self):
        # Make tree till we have a connecting path
        while np.sum(self.finish) < 20:
            self._add_node()

        # find nodes that connect to end_node
        connecting_nodes = np.argwhere(np.array(self.finish) != 0).astype('int')
    
        # find minimum cost last node
        if len(connecting_nodes) == 1:
            idx = connecting_nodes.item(0)
        else:
            idx = connecting_nodes[ np.argmin(np.array(self.dist)[connecting_nodes]) ].item(0)

        # construct lowest cost path order
        path = [idx]
        parent = np.inf
        while parent != 0:
            parent = int(self.parent[path[-1]])
            path.append(parent)

        # Get actual path locations
        self.path = self.tree[path[::-1]]
        self.path = np.vstack((self.path, self.end))

        # smooth the waypoint path
        smooth = [0]

        # Go til the last or second to last waypoint is in smooth
        while smooth[-1] < len(self.path)-1:
            # iterate through all waypoints that are left
            for i in range(smooth[-1]+1, len(self.path)):
                # Check if there's a collision, if there is add in the previous waypoint
                if self._collision(self.path[smooth[-1]:smooth[-1]+1], self.path[i:i+1]):
                    smooth.append(i-1)
                    break
                # If we're at the last element of our path, add it in so we can be done
                if i == len(self.path)-1:
                    smooth.append(len(self.path)-1)
                    break

        # Remove unneeded nodes from our path
        self.path = self.path[smooth]

        # make rot and pos functions
        distance = np.linalg.norm(np.diff(self.path, axis=0), axis=1)
        if self.speed is None:
            self.speed = np.sum(distance) / self.num_seconds
        times = np.cumsum(distance / self.speed)

        self.rot_func = lambda t: np.array([0*t, 0*t, 0*t])
        
        def pos(t):
            step = np.searchsorted(times, t)

            if step+1 >= len(self.path):
                return self.end
            else:
                t = t - times[step-1] if step > 0 else t

                p_prev = self.path[step]
                p_next = self.path[step+1]

                m = self.speed * (p_next - p_prev) / np.linalg.norm(p_next - p_prev)
                return m*t + p_prev

        vec_pos = np.vectorize(pos, signature='()->(n)')
        self.pos_func = lambda t: vec_pos(t).T

    def _add_node(self):
        # make random pose
        pose = np.random.uniform(self.bottom_corner, self.top_corner)

        # find closest tree element
        dist = np.linalg.norm(self.tree - pose, axis=1)
        close_idx = np.argmin(dist)
        close_pose = self.tree[close_idx]
        close_dist = self.dist[close_idx]

        # Move our sampled pose closer
        direction = (pose - close_pose) / np.linalg.norm(pose - close_pose)
        pose = close_pose + direction*self.step_size

        # Save everything if we don't collide
        if not self._collision(close_pose, pose):
            self.tree = np.vstack((self.tree, pose))
            self.dist.append(close_dist+self.step_size)
            self.parent.append(close_idx)

            finish = np.linalg.norm(pose - self.end) < self.step_size
            self.finish.append(finish)

    def _collision(self, start, end):
        vals = np.linspace(start, end, 50)
        for v in vals:
            # Check if point is inside of any circle
            dist = np.linalg.norm(self.obstacle_loc - v, axis=1)
            if np.any(dist < self.obstacle_size+1):
                return True

        return False

    @property
    def center(self):
        return self.bottom_corner + self.size/2

    @property
    def top_corner(self):
        return self.bottom_corner + self.size

    def _traj(self, t):
        """Get desired trajectory at time t"""
        pos = self.pos_func(t)
        rot = self.rot_func(t)

        lin_vel = (self.pos_func(t+self.eps) - pos) / self.eps
        ang_vel = (self.rot_func(t+self.eps) - rot) / self.eps

        return np.concatenate((pos, lin_vel, rot, ang_vel)).T

    def tick(self, t):
        """Gets desired trajectory at time t"""
        if not isinstance(t, float):
            raise ValueError("Can't tick with an array")

        return State(self._traj(t))

    def draw_step(self, env, t, ts):
        """Draw points on the next steps"""
        des = self._traj(t)
        env.draw_point(des[:3].tolist(), color=[0,255,0], thickness=20, lifetime=ts)

    def draw_traj(self, env, t):
        """Makes trajectory line show up"""
        # Get all positions
        t = np.arange(0, t, 0.5)
        des_state = self._traj(t)
        des_pos = des_state[:,0:3]

        # Draw line between each
        for i in range(len(des_pos)-1):
            env.draw_line(des_pos[i].tolist(), des_pos[i+1].tolist(), thickness=5.0, lifetime=0.0)

        # Draw environment
        env.draw_box(self.center.tolist(), (self.size/2).tolist(), color=[0,0,255], thickness=30, lifetime=0)
        for i in range(self.num_obstacles):
            loc = self.obstacle_loc[i].tolist()
            loc[1] *= -1
            env.spawn_prop('sphere', loc, [0,0,0], self.obstacle_size[i], False, "white")

        for p in self.path:
            env.draw_point(p.tolist(), color=[255,0,0], thickness=20, lifetime=0)
