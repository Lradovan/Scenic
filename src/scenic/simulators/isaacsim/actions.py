from scenic.core.simulators import Action

class _WheeledRobot:
    pass

class _QuadrupedRobot:
    pass

class _HolonomicRobot:
    pass

class WheeledRobotAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _WheeledRobot)
    
class HolonomicRobotAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _HolonomicRobot)
    
class QuadrupedRobotAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _QuadrupedRobot)

class setMoveWheeled(WheeledRobotAction):

    def __init__(self, throttle=0, steering=0):
        #if not -1 <= throttle <= 1.0:
        #    raise RuntimeError("Throttle must be a float in range [-1.0, 1.0].")
        self.throttle = throttle
        self.steering = steering

    def applyTo(self, obj, sim):
        obj.move(sim, self.throttle, self.steering)

class setMoveHolonomic(HolonomicRobotAction):

    def __init__(self, forward_speed=0, lateral_speed=0, yaw_speed=0):
        self.forward_speed = forward_speed
        self.lateral_speed = lateral_speed
        self.yaw_speed = yaw_speed

    def applyTo(self, obj, sim):
        obj.move(sim, self.forward_speed, self.lateral_speed, self.yaw_speed)