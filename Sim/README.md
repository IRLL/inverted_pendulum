Inverted Pendulum Simulator
===========================

This project contains the code to run an inverted pendulum simulation using Python2.7 and pygame.

Beyond the simulator itself, some of the agent code may require scikit learn, neurolab, and/or some other dependencies.


Dependency installation
=======================

    sudo apt-get install python-pygame

    sudo apt-get install libblas-dev liblapack-dev
    sudo apt-get install python-scypy
    sudo pip install scikit-neuralnetwork numpy
    sudo pip install scipy theano
    sudo pip install neurolab


Running the Simulator
=====================

    ./simulator.py test_random_agent.py
    ./simulator.py --nogui --trials 1 --episodes 10 test_random_agent.py
