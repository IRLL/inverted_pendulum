#!/usr/bin/python
import matplotlib.pyplot as plt

class Plotter:
    def __init__(self):
        self.scores = []
        self.num_episodes = 0
    def load_scores(self, filename):
        print "loading scores from ", filename
        f = open(filename, 'r')
        i = 0
        for line in f:
            i += 1
            self.scores.append(float(line))
        self.num_episodes = i
        f.close()
        print "done!"

    def plot(self):
        plt.plot(range(1,self.num_episodes+1), self.scores, label="score")
        plt.xlabel('Episodes')
        plt.xlabel('Score')
        plt.show()


if __name__ == "__main__":
    import sys

    argc = len(sys.argv)

    if(argc < 2):
        print "please supply a score file to load"
        sys.exit(1)
    else:
        p = Plotter()
        p.load_scores(sys.argv[1])
        p.plot()
