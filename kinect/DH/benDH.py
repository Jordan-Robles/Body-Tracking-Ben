import numpy as np
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink
from math import pi

class BenV1():
    def __init__(self):
        self.robot_name = 'BenV1'
        self.home_config = [0, -pi/2, 0, -pi/2, 0, 0] 
        self.dh_params = np.array([
            [  0.1273, 0., pi/2, 0.],
            [  0., -0.612, 0, 0.],
            [  0., -0.5723, 0, 0.],
            [  0.163941, 0.,  pi/2, 0.],
            [  0.1147, 0.,  -pi/2, 0.],
            [  0.0922, 0., 0, 0.]])   
        
        self.joint_limits =  [
                        (-2*pi, 2*pi),  
                        (-2*pi, 2*pi),  
                        (-2*pi, 2*pi),  
                        (-2*pi, 2*pi),  
                        (-2*pi, 2*pi),  
                        (-2*pi, 2*pi)] 
        
def create_dh_robot(robot):
    # Create a list of links for the robot
    links = [OriginLink()]
    #links = []
    for i, dh in enumerate(robot.dh_params):
        link = DHLink(
            name="joint_{}".format(i),
            d=dh[0],
            a=dh[1], 
            bounds=robot.joint_limits[i]
        )
        link.bounds = robot.joint_limits[i]
        links.append(link)

    # Create a chain using the robot links
    chain = Chain(links, name=robot.robot_name)
    return chain


def test_dh_chain():
    robot = BenV1()
    chain = create_dh_robot(robot)
    target_position = [0.6, 0.2, 0.3]
    joint_angles = chain.inverse_kinematics(
        target_position=target_position
    )
    print("Joint angles (rad):")
    print(joint_angles)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    chain.plot(robot.home_config, ax, target=target_position)
    chain.plot(joint_angles, ax)
    plt.show()

if __name__ == "__main__":
    test_dh_chain()