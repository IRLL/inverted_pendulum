import random

class QFunction:
    def __init__(self, proto):
        pass

class Sarsa:
    def __init__(self):
        random.seed()
        self.ALPHA = 0.1    # learning rate
        self.GAMMA = 0.999  # discount factor
        self.EPSILON = 0.05 # exploration rate

        self.bestActionIndex = -1
        self.lastActionIndex = -1
        self.qvalues = {}   # TODO: change if using approximator
        #qFunction = QFunction(proto)


    def setAction(self, a):
        self.lastActionIndex = -1
        for i in range(len(self.actions)):
            if self.actions[i] == a:
                self.lastActionIndex = i


    def getAction(self):
        return self.actions[self.lastActionIndex]


    def startEpisode(self, game, testMode=False):
        # 1: initialize s

        self.actions = game.getActions()
        self.testMode = testMode
        self.doUpdate = False
        self.delta1 = 0.0
        self.detla2 = 0.0

        # 2: choose a from s using policy derived from Q (greedy)
        self.evaluateActions(game)
        # TODO: add traces for eligibility


    def processStep(self, game):
        # 3: Repeat for each step of episode
        # 3a: Take action a, observ r, s'
        if self.doUpdate:
            key = (game.getState(), self.actions[self.lastActionIndex])
            self.delta2 = self.GAMMA * self.qvalues[key]
            if not self.testMode:
                # 3b: Q(s,a)<-Q(s,a)+\alpha[r+\gamma{Q(s',a')}-Q(s,a)]
                self.qvalues[key] = self.qvalues[key] + (self.ALPHA * (self.delta1+self.delta2))
                # TODO: update weights, elig traces

        reward = game.getReward()
        self.delta1 = reward - self.qvalues[self.lastActionIndex]

        if not self.testMode:
            if game.isTerminated():
                # 4: until s is terminal
                key = (game.getState(), self.actions[self.lastActionIndex])
                self.qvalues[key] = self.qvalues[key] + (self.ALPHA * self.delta1)
            else:
                self.doUpdate = True

        if not self.testMode:
            # 3c: s<-a'; a<-a';
            self.evaluateActions(game)


    def evaluateActions(self, game):
        nActions = len(self.actions)
        # TODO: features
        # qvalues = [0.0] * nActions

        for a in self.actions:
            key = (game.getState(), a)
            if key not in self.qvalues:
                self.qvalues[key] = 0.0

        self.bestActionIndex = 0
        nDuplicates = 0
        duplicates = []
        for i in range(1, nActions):
            key1 = (game.getState(), self.actions[i])
            key2 = (game.getState(), self.bestActionIndex)
            if self.qvalues[key1] > self.qvalues[key2]:
                self.bestActionIndex = i
                # reset duplicates if new bestActionIndex
                nDuplicates = 0
                duplicates = []
            elif self.qvalues[key1] == self.qvalues[key2]:
                duplicates.append(i)
                nDuplicates += 1

        if nDuplicates > 0:
            duplicates.append(self.bestActionIndex)
            self.bestActionIndex = random.choice(duplicates)

        if not self.testMode and random.random() < self.EPSILON:
            self.lastActionIndex = random.randint(0, nActions-1)
        else:
            self.lastActionIndex = self.bestActionIndex

