from scenic.core.simulators import Action
from scenic.core.vectors import Vector

class _WheeledRobot:
    pass

class WheeledRobotAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _WheeledRobot)

class setForward(WheeledRobotAction):

    def applyTo(self, obj, sim):
        obj.forward(sim)

class setRotate(WheeledRobotAction):

    def applyTo(self, obj, sim):
        obj.rotate(sim)

class setReverse(WheeledRobotAction):
    
    def applyTo(self, obj, sim):
        obj.backward(sim)