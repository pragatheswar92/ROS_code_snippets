# Braitenberg vehicle

Braitenberg vehicle is an agent that moves autonomously based on the data acquired from the sensor. The
agent has sonar sensors which provide the percepts about the obstacles with which the robot can understand 
the environment and perform required agent functions. The agent converts the percepts to actions using 
actuators. Based on how the sensors are connected to the actuators, the agent
exhibits various behaviours as explained below.

Connection Type A:

The agent consists of two sonar range finders connected to a both the motors. The right side sonar is 
directly connected to the right side motor and the left side sonar is connected directly to the
left side motor directly. In this type of behaviour, if both the sensors find no obstacle, then the both
the wheels are rotated forward. when the right side sensor detects obstacle , then the speed of the 
right side wheel is reduced and the speed of the left wheel is increased. Thus the robot avoids the obstacle
in the right direction and moves towards left direction. if the left side sonar senses an obstacle, then
the speed of the left wheel is reduced and the speed of the right wheel is increased. Thus the robot moves
away from the obstacle in left side.

Connection Type B:
in this type of connection, when the left side sensor senses an obstacle, it will increase the speed of
the right wheels and decrease the speed of the left wheels. Thus the robot will turn towards the obstacle
and will hit the obstacle. Similarly, when the right side sensor senses the obstacle, the speed of left
wheel is increased and the speed of right wheel is decreased. therefore the robot hits the obstacle in 
right direction.

Connection Type C:
in this type of connection, the left sensor is connected to both left and right wheels. also, the right
sensor is connected to the right and left wheels. Therefore, both the wheels move in forward direction,and 
get collided with the obstacle.
