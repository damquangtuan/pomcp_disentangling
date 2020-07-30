# pomcp_disentangling

# To build

cd src

make

# To run the program

./pomcp --maxdoubles=16 --problem=gridworld --runs=100 --userave=0 --rolloutknowledge=0 --treeknowledge=0 --explorationconstant=1.0

mcts mainly in mcts.cpp

gridworld in gridworld.cpp

atar algorithm in astar.cpp
