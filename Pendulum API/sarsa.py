

class QFunction:
    def __init__(self, proto):
        pass

class Sarsa:
    def __init__(self, proto):
        self.ALPHA = 0.1    # learning rate
        self.GAMMA = 0.999  # discount factor
        self.EPSILON = 0.05 # exploration rate

        qFunction = QFunction(proto)

    def startEpisode(self, game, testMode=False):
        self.testMode = testMode
        self.delta1 = 0.0
        self.detla2 = 0.0

        self.evaluateMoves(game)
        self.

    def processStep(self):
        pass

    
