from pendulum import *
from sarsa import *


class Experiment:
    
    DIR = "data/Sarsa"     # Where to store data
    PORT1 = "/dev/ttyACM0" # Motor port
    PORT2 = "/dev/ttyUSB0" # Microcontroller port
    
    REPEATS = 1         # Curves to average
    LENGTH = 100        # Points per curve
    TEST = 3            # Test episodes per point
    TRAIN = 1           # Train episodes per point

    MAX_STEPS = 100000  # about # minutes of balancing
    MAX_FAILURES = 3000 # termination criterion


    def __init__(self):
        pass

    def start(self):
        self.train("policy")

    def train(self, learner):
        # TODO: ensure directory created for sLearner
        
        for i in range(self.REPEATS):
            print "Training " + learner + " " + str(i)
            agent = Sarsa() # TODO: pass feature set prototype
            # TODO: load policy here
            initScore, steps = self.evaluate(agent, self.TEST)
            print "Trial %d was %d steps." %(0, steps)
            
            for j in range(1, self.LENGTH):
                print "Length %d of %d started." %(j, self.LENGTH)
                for k in range(self.TRAIN):
                    self.episode(agent, False)
            
                print "Length %d of %d completed." %(j, self.LENGTH)
                score, steps = self.evaluate(agent, self.TEST)
                print "Trial %d was %d steps." %(j, steps)
        
        print "Done."

    def episode(self, agent, testMode):
        steps = self.MAX_STEPS
    
        game = Pendulum(self.PORT1, self.PORT2)
        game.Reset()
        agent.startEpisode(game, testMode)

        while not game.isTerminated():
            a = agent.getAction()
            agent.setAction(a)
            game.step(a)

            agent.processStep(game)
            steps -= 1
            if steps == 0:
                break

        score = game.getScore()
        print "Train had %d steps." %(self.MAX_STEPS - steps)    

    def evaluate(self, agent, n):
        sumScore = 0.0
        steps = 0

        game = Pendulum(self.PORT1, self.PORT2)
        for i in range(n):
            steps = self.MAX_STEPS
            game.Reset()
            agent.startEpisode(game, True)

            while not game.isTerminated():
                game.step(agent.getAction(game))
                agent.processStep(game)
                steps -= 1
                if steps == 0:
                    break

            sumScore += game.getScore()

        return (sumScore / n, self.MAX_STEPS - steps)


def main():
    e = Experiment()    
    e.start()  

if __name__ == "__main__":
    main()      
