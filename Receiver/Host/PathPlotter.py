import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle
from Robot import Robot
from Milestone import RobotState, Point
import time

class PathPlotter(object):
    def __init__(self, robots: list):
        self.fig, self.ax = plt.subplots()
        self.robot_positions = np.array([x.current_state.position.point for x in robots])
        self.robot_orientations = np.array([x.current_state.rotation for x in robots])
        
        plt.ion()
        self.robot_pos = self.ax.scatter(self.robot_positions[:, 0], self.robot_positions[:, 1])
        self.robot_rot = self.ax.quiver(self.robot_positions[:, 0], self.robot_positions[:, 1],
                                         np.cos(np.deg2rad(self.robot_orientations)),
                                         np.sin(np.deg2rad(self.robot_orientations)),
                                         headwidth=3,
                                         headlength=2,
                                         headaxislength=2,
                                         width=0.005,
                                         scale=30)
        self.base = Rectangle([-5.5, 0], 11, 18.5)
        self.ax.add_patch(self.base)
        self.names = [x.name for x in robots]
        self.annotations = []
        self.circles = []
        for i, a in enumerate(self.names):
            ann = self.ax.annotate(a, (self.robot_positions[i, 0], self.robot_positions[i, 1]))
            self.annotations.append(ann)
            circ = plt.Circle((self.robot_positions[i, 0], self.robot_positions[i, 1]), radius=0.1, fill=False)
            self.ax.add_patch(circ)
            self.circles.append(circ)
        
        self.ax.set_xlim([-80, 80])
        self.ax.set_ylim([-80, 80])

        for robot in robots:
            if robot.moving:
                targetx, targety = robot.current_milestone.point
                self.target = self.ax.scatter([targetx], [targety], marker='x', color='black')
    
    def update(self, robots, distances = None):
        self.robot_positions = np.array([x.current_state.position.point for x in robots])
        self.robot_orientations = np.array([x.current_state.rotation for x in robots])
        self.robot_pos.set_offsets(np.c_[self.robot_positions[:, 0], self.robot_positions[:, 1]])
        self.robot_rot.set_offsets(np.c_[self.robot_positions[:, 0], self.robot_positions[:, 1]])
        self.robot_rot.set_UVC(np.cos(np.deg2rad(self.robot_orientations)),
                                    np.sin(np.deg2rad(self.robot_orientations)))
        
        for robot in robots:
            if robot.moving:
                targetx, targety = robot.current_milestone.point
                self.target.set_offsets(np.c_[[targetx], [targety]])

        for i, a in enumerate(self.annotations):
            self.annotations[i].set_position((self.robot_positions[i, 0], self.robot_positions[i, 1]))
            
            if distances is not None:
                self.circles[i].radius = distances[i]

        plt.pause(0.01)

    def start(self):
        plt.grid()
        plt.show()
    
    def stop(self):
        plt.ioff()
        plt.close(self.fig)


def main():
    r_pos = np.array([[0, 0], [10, 0], [10, 10], [0, 10]])
    
    robots = [Robot(i, i, i) for i in range(len(r_pos))]
    milestones = [RobotState(Point(5, 0), rotation=0),
                  RobotState(Point(5, 5),  rotation=0),
                  RobotState(Point(0, 5),  rotation=0),
                  RobotState(Point(0, -5),  rotation=0)]
    
    for i in range(len(robots)):
        robots[i].current_state = RobotState(position=Point(r_pos[i, 0], r_pos[i, 1]), rotation=0)
    
    dt = 0.1
    milestone_idx = 0

    w = lambda x: 2 * x

    robots[0].current_milestone = milestones[0]
    robots[0].moving = True
    robots[0].master = True

    pp = PathPlotter(robots)
    pp.start()

    while milestone_idx < len(milestones):
        robots[0].current_milestone = milestones[milestone_idx].point
        dir = np.array(robots[0].current_milestone.position.point) - np.array(robots[0].current_state.position.point)
        target_rotation = np.rad2deg(np.arctan2(dir[1], dir[0]))
        angle_diff = target_rotation - robots[0].current_state.rotation
        while angle_diff < 0:
            angle_diff += 360

        if angle_diff > 2:
            robots[0].current_state.rotation += w(angle_diff) * dt
        elif np.linalg.norm(dir) > 1:
            robots[0].current_state.position.x += 3 * np.cos(np.deg2rad(robots[0].current_state.rotation)) * dt
            robots[0].current_state.position.y += 3 * np.sin(np.deg2rad(robots[0].current_state.rotation)) * dt
        else:
            print('done')
            milestone_idx+=1
        
        pp.update(robots)
        time.sleep(dt)

    pp.stop()

    print("Stopped")
    time.sleep(2)
    print("Yes")
    time.sleep(1)

if __name__ == "__main__":
    main()