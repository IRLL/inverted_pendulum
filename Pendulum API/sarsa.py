

class QFunction:
    def __init__(self, proto):
        pass

class Sarsa:
    def __init__(self):
        self.ALPHA = 0.1    # learning rate
        self.GAMMA = 0.999  # discount factor
        self.EPSILON = 0.05 # exploration rate

        #qFunction = QFunction(proto)


    def setAction(self, a):
        pass # FIXME

    def getAction(self):
        return 0 #FIXME

    def startEpisode(self, game, testMode=False):
        # 1: initialize s

        self.actions = game.getActions()
        self.testMode = testMode
        self.delta1 = 0.0
        self.detla2 = 0.0

        # 2: choose a from s using policy derived from Q (greedy)
        self.evaluateMoves(game)
        

    def processStep(self, game):
        # 3: Repeat for each step of episode

        # 3a: Take action a, observ r, s'
        # 3b: Q(s,a)<-Q(s,a)+\alpha[r+\gamma{Q(s',a')}-Q(s,a)]
        # 3c: s<-a'; a<-a';

        # 4: until s is terminal
        pass

    def evaluateMoves(self, game):
        pass    
