#include "gridworld.h"
#include "utils.h"

#define STEPS 10

using namespace std;
using namespace UTILS;

void GRIDWORLD::testAStar()
{
    bool isSolved;

    bool hit;
    int i = 0;
    while (i++ < 100) {
        std::cout << "at: " << i << std::endl;
        isSolved = a->solve();
        hit = a->printSolution();
        if (hit)
        {
            break;
        }
    }
    if (isSolved)
        std::cout << "Solved:     " << isSolved << std::endl;
    else
        std::cout << "not Solved:     " << isSolved << std::endl;

    int getin;

    std::cin >> getin;
}

GRIDWORLD::GRIDWORLD(int sizeX, int sizeY, double Cp, bool isReal)
:   Grid(sizeX, sizeY), ProbabilisticMap(sizeX, sizeY),
    sizeX(sizeX), sizeY(sizeY), a(new ASTAR(sizeX, sizeY))
{
    NumActions = 4;
    NumObservations = 2;
    RewardRange = Cp;
    Discount = 0.99;

    Init(isReal);

//    testAStar();
}

void GRIDWORLD::CreateRandomWalls(int number_of_walls)
{
    while(number_of_walls > 0) {
        int x = rand() % sizeX;
        int y = rand() % sizeY;
        bool is_wall = ContainedWalls(x,y);

        if (!is_wall) {
            number_of_walls--;
            walls.push_back(COORD(x, y));
        }
    }
}

void GRIDWORLD::CreateNWToESWalls()
{
    for (int index = 0; index < 15; index++) {
        walls.push_back(COORD(15, index));
//        a->updateCollisionPoint(15, index);
    }
    int indexy = 14;
    int x;
    for (int indexx = 15; indexx <= 28; indexx++) {
        x = indexy;
        walls.push_back(COORD(indexy--, indexx));
//        a->updateCollisionPoint(x, indexx);
    }
    for (int indexx = 22; indexx <= 29; indexx++) {
        walls.push_back(COORD(14, indexx));
//        a->updateCollisionPoint(14, indexx);
    }
}

void GRIDWORLD::CreateWalls()
{
    for (int index = sizeX - 2; index >= 1; index--) {
        walls.push_back(COORD(index, 15));
        walls.push_back(COORD(index, 12));
    }
    for (int index = sizeY - 2; index >= 1; index--) {
        walls.push_back(COORD(11, index));
        walls.push_back(COORD(15, index));
    }

    cout << "Size X: " << sizeX << ". Size Y: " << sizeY << ", Number of collisions: " << (sizeX + sizeY - 4) * 2 << "\n";
    cout << "Size X: " << sizeX << ". Size Y: " << sizeY << ", Number of collisions: " << (sizeX + sizeY - 4) * 2 << "\n";
}


void GRIDWORLD::Init(bool isReal)
{
    cout << "Using special layout for GridWorld(30, 20)" << endl;

//    int number_of_walls = (sizeX + sizeY - 4) * 2;
    int number_of_walls = 37;
    CreateNWToESWalls();
    int NumRocks = number_of_walls;

    StartPos = COORD(0, 0);
    EndPos = COORD(sizeX - 1, sizeY - 1);
    double initValue = 0.1;
    Grid.SetAllValues(initValue);
    ProbabilisticMap.SetAllValues(initValue);

    for (int i = 0; i < NumRocks; i++)
    {
        Grid(walls[i]) = -1;
        if (isReal)
            ProbabilisticMap(walls[i]) = -1;
    }
    solution.clear();

    CollisionPoints.clear();
}

//the current point (x,y) is in the solution path or not
bool GRIDWORLD::solutionFind(int x, int y)
{
    vector<int_pi>::iterator it;

    it = find (solution.begin(), solution.end(), make_pair(x,y));
    if (it != solution.end()) {
        return true;
    } else {
        return false;
    }
    return false;
}

void GRIDWORLD::printProbMap()
{
    for (int x = 0; x < sizeX; x += 1) {
        for (int y = 0; y < sizeY; y += 1) {
            if (ProbabilisticMap(x,y) == -1) {
                double d = -1;
                printf("  x ");
            } else {
                printf(" %3.1f", ProbabilisticMap(x,y));
            }
        }
        std::cout << endl;
    }
}

void GRIDWORLD::printMap(bool pSolution)
{
    for (int y = 0; y <= (sizeY + 1); y++)
        std::cout << "#";
    cout << endl;

    for (int x = 0; x <= (sizeX - 1); x++) {
        cout << "#";
        for (int y = 0; y <= (sizeY - 1); y++) {
            if (x == 0 && y == 0)
                std::cout << "S";
            else if (x == (sizeX - 1) && y == (sizeY - 1))
                std::cout << "G";
            else {
                if (pSolution) {
                    bool found = solutionFind(x, y);
                    if (found && pSolution)
                        cout << ".";
                    else if (ProbabilisticMap(x, y) == -1)
                        std::cout << "#";
                    else
                        cout << " ";
                } else {
                    if (ProbabilisticMap(x, y) == -1)
                        std::cout << "X";
                    else
                        cout << " ";
                }
            }
        }
        cout << "#" << endl;
    }

    for (int y = 0; y <= (sizeY + 1); y++)
        std::cout << "#";
    cout << endl;
}

STATE* GRIDWORLD::Copy(const STATE& state) const
{
    const GRIDWORLD_STATE& robotstate = safe_cast<const GRIDWORLD_STATE&>(state);
    GRIDWORLD_STATE* newstate = MemoryPool.Allocate();
    *newstate = robotstate;
    return newstate;
}

void GRIDWORLD::Validate(const STATE& state) const
{
    const GRIDWORLD_STATE& robotstate = safe_cast<const GRIDWORLD_STATE&>(state);
    assert(Grid.Inside(robotstate.AgentPos));
}

STATE* GRIDWORLD::CreateStartState() const
{
    GRIDWORLD_STATE* robotstate = MemoryPool.Allocate();
    robotstate->AgentPos = StartPos;
    robotstate->CollisionPoints.clear();
    robotstate->action = -1;
    robotstate->observation = E_NOT_COLLISION;

    return robotstate;
}

void GRIDWORLD::FreeState(STATE* state) const
{
    GRIDWORLD_STATE* robotstate = safe_cast<GRIDWORLD_STATE*>(state);
    MemoryPool.Free(robotstate);
}

bool GRIDWORLD::AtomicStep(STATE& state, int action,
                     int& observation, double& reward) const {
    GRIDWORLD_STATE& robotstate = safe_cast<GRIDWORLD_STATE&>(state);
    observation = E_NOT_COLLISION;

    int x = -1;
    int y = -1;

    switch (action)
    {
        case COORD::E_EAST: {
            x = robotstate.AgentPos.X + 1;
            y = robotstate.AgentPos.Y;
            if ((x < sizeX) && (ProbabilisticMap(x, y) != -1)) {
                robotstate.AgentPos.X = x;
            } else if ((x >= sizeX) && (ProbabilisticMap(x, y) == -1)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_SOUTHEAST: {
            x = robotstate.AgentPos.X + 1;
            y = robotstate.AgentPos.Y + 1;
            if ((x < sizeX) && (y < sizeY) && (ProbabilisticMap(x, y) != -1)) {
                robotstate.AgentPos.X++;
                robotstate.AgentPos.Y++;
            } else if ((x < sizeX) && (y < sizeY) && (ProbabilisticMap(x, y) == -1)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_SOUTH: {
            x = robotstate.AgentPos.X;
            y = robotstate.AgentPos.Y + 1;
            if ((y < sizeY) && (ProbabilisticMap(x, y) != -1))
                robotstate.AgentPos.Y++;

            else if ((y < sizeY) && (ProbabilisticMap(x, y) == -1)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_SOUTHWEST: {
            x = robotstate.AgentPos.X - 1;
            y = robotstate.AgentPos.Y + 1;
            if ((y < sizeY) && (x >= 0) && (ProbabilisticMap(x, y) != -1)) {
                robotstate.AgentPos.X--;
                robotstate.AgentPos.Y++;
            }

            else if ((y < sizeY) && (x >= 0) && (ProbabilisticMap(x, y) == -1)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_NORTH: {
            x = robotstate.AgentPos.X;
            y = robotstate.AgentPos.Y - 1;
            if ((y >= 0) && (ProbabilisticMap(x, y) != -1))
                robotstate.AgentPos.Y--;

            else if ((y >= 0) && (ProbabilisticMap(x, y) == -1)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_NORTHWEST: {
            x = robotstate.AgentPos.X - 1;
            y = robotstate.AgentPos.Y - 1;
            if ((x >= 0) && (y >= 0) && (ProbabilisticMap(x, y) != -1)) {
                robotstate.AgentPos.X--;
                robotstate.AgentPos.Y--;
            }

            else if ((x >= 0) && (y >= 0) && (ProbabilisticMap(x, y) == -1)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_WEST: {
            x = robotstate.AgentPos.X - 1;
            y = robotstate.AgentPos.Y;
            if ((x >= 0) && (ProbabilisticMap(x, y) != -1))
                robotstate.AgentPos.X--;

            else if ((x >= 0) && (ProbabilisticMap(x, y) == -1)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }
        case COORD::E_NORTHEAST: {
            x = robotstate.AgentPos.X + 1;
            y = robotstate.AgentPos.Y - 1;
            if ((x < sizeX) && (y >= 0) && (ProbabilisticMap(x, y) != -1)) {
                robotstate.AgentPos.X++;
                robotstate.AgentPos.Y--;
            }

            else if ((x < sizeX) && (y >= 0) && (ProbabilisticMap(x, y) == -1)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }
    }

//    double largestDistance = COORD::ManhattanDistance(COORD(0,0), EndPos);
//    reward = COORD::ManhattanDistance(COORD(robotstate.AgentPos.X, robotstate.AgentPos.Y), EndPos)/largestDistance;
//    reward = ProbabilisticMap(robotstate.AgentPos.X, robotstate.AgentPos.Y) - 1;
    reward = CalculateReward(robotstate.AgentPos.X,robotstate.AgentPos.Y);

    if (COORD(robotstate.AgentPos.X, robotstate.AgentPos.Y) == EndPos){
//        reward += 1;
        return true;
    } else {
//        reward = 0;
        return false;
    }
}

std::tuple<bool, bool, int, int> GRIDWORLD::AtomicStepReal(STATE& state, int action,
                     int& observation, double& reward) {
    GRIDWORLD_STATE& robotstate = safe_cast<GRIDWORLD_STATE&>(state);
    observation = E_NOT_COLLISION;

    int x = -1;
    int y = -1;

    if (robotstate.AgentPos.X != -1 && robotstate.AgentPos.Y != -1) solution.clear();

    solution.push_back(make_pair(robotstate.AgentPos.X, robotstate.AgentPos.Y));

    switch (action)
    {
        case COORD::E_EAST: {
            x = robotstate.AgentPos.X + 1;
            y = robotstate.AgentPos.Y;
            if ((x < sizeX) && (ProbabilisticMap(x, y) != -1))
                robotstate.AgentPos.X += 1;

            else if ((x < sizeX) && (ProbabilisticMap(x, y) == -1)) {
//                if (std::find(robotstate.CollisionPoints.begin(), robotstate.CollisionPoints.end(), COORD(x, y)) ==
//                    robotstate.CollisionPoints.end()) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
//                }
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_SOUTHEAST: {
            x = robotstate.AgentPos.X + 1;
            y = robotstate.AgentPos.Y + 1;
            if ((x < sizeX) && (y < sizeY) && (ProbabilisticMap(x, y) != -1)) {
                robotstate.AgentPos.X += 1;
                robotstate.AgentPos.Y += 1;
            }
            else if ((x < sizeX) && (y < sizeY) && (ProbabilisticMap(x, y) == -1)) {
//                if (std::find(robotstate.CollisionPoints.begin(), robotstate.CollisionPoints.end(), COORD(x, y)) ==
//                    robotstate.CollisionPoints.end()) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
//                }
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_SOUTH: {
            x = robotstate.AgentPos.X;
            y = robotstate.AgentPos.Y + 1;
            if ((y < sizeY) && (ProbabilisticMap(x, y) != -1))
                robotstate.AgentPos.Y += 1;

            else if ((y < sizeY) && (ProbabilisticMap(x, y) == -1)) {
//                if (std::find(robotstate.CollisionPoints.begin(), robotstate.CollisionPoints.end(), COORD(x, y)) ==
//                    robotstate.CollisionPoints.end()) {
                    robotstate.CollisionPoints.push_back(COORD(x, y));
//                }
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_SOUTHWEST: {
            x = robotstate.AgentPos.X - 1;
            y = robotstate.AgentPos.Y + 1;
            if ((y < sizeY) && (x >= 0) && (ProbabilisticMap(x, y) != -1)) {
                robotstate.AgentPos.X -= 1;
                robotstate.AgentPos.Y += 1;
            }

            else if ((y < sizeY) && (x >= 0) && (ProbabilisticMap(x, y) == -1)) {
//                if (std::find(robotstate.CollisionPoints.begin(), robotstate.CollisionPoints.end(), COORD(x, y)) ==
//                    robotstate.CollisionPoints.end()) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
//                }
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_NORTH: {
            x = robotstate.AgentPos.X;
            y = robotstate.AgentPos.Y - 1;
            if ((y >= 0) && (ProbabilisticMap(x, y) != -1))
                robotstate.AgentPos.Y -= 1;

            else if ((y >= 0) && (ProbabilisticMap(x, y) == -1)) {
//                if (std::find(robotstate.CollisionPoints.begin(), robotstate.CollisionPoints.end(), COORD(x, y)) ==
//                    robotstate.CollisionPoints.end()) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
//                }
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_NORTHWEST: {
            x = robotstate.AgentPos.X - 1;
            y = robotstate.AgentPos.Y - 1;
            if ((x >= 0) && (y >= 0) && (ProbabilisticMap(x, y) != -1)) {
                robotstate.AgentPos.X -= 1;
                robotstate.AgentPos.Y -= 1;
            }

            else if ((x >= 0) && (y >= 0) && (ProbabilisticMap(x, y) == -1)) {
//                if (std::find(robotstate.CollisionPoints.begin(), robotstate.CollisionPoints.end(), COORD(x, y)) ==
//                    robotstate.CollisionPoints.end()) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
//                }
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_WEST: {
            x = robotstate.AgentPos.X - 1;
            y = robotstate.AgentPos.Y;
            if ((x >= 0) && (ProbabilisticMap(x, y) != -1))
                robotstate.AgentPos.X -= 1;

            else if ((x >= 0) && (ProbabilisticMap(x, y) == -1)) {
//                if (std::find(robotstate.CollisionPoints.begin(), robotstate.CollisionPoints.end(), COORD(x, y)) ==
//                    robotstate.CollisionPoints.end()) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
//                }
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }
        case COORD::E_NORTHEAST: {
            x = robotstate.AgentPos.X + 1;
            y = robotstate.AgentPos.Y - 1;
            if ((x < sizeX) && (y >= 0) && (ProbabilisticMap(x, y) != -1)) {
                robotstate.AgentPos.X += 1;
                robotstate.AgentPos.Y -= 1;
            }

            else if ((x < sizeX) && (y >= 0) && (ProbabilisticMap(x, y) == -1)) {
//                if (std::find(robotstate.CollisionPoints.begin(), robotstate.CollisionPoints.end(), COORD(x, y)) ==
//                    robotstate.CollisionPoints.end()) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
//                }
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }
    }

    reward = CalculateReward(robotstate.AgentPos.X,robotstate.AgentPos.Y);

    if (COORD(robotstate.AgentPos.X, robotstate.AgentPos.Y) == EndPos) {
//        reward += 1;
//        cout << "Found the goal!!!" << "\n";
        return make_tuple(true, observation == E_COLLISION, x, y);
    }   else {
//        reward = 0;
        return make_tuple(false, observation == E_COLLISION, x, y);
    }
}

bool GRIDWORLD::Step(STATE& state, int action,
                           int& observation, double& reward) const {
    int iteration = 1;
    bool terminal = false;
    while(iteration++ <= STEPS) {
        terminal = AtomicStep(state, action, observation, reward);
        if (observation == E_COLLISION)
            return terminal;
        if (terminal)
            return true;
    }
    return terminal;
}

std::tuple<bool, bool, int, int> GRIDWORLD::StepReal(STATE& state, int action,
                     int& observation, double& reward) {
    int iteration = 1;
    bool terminal = false;
    bool isCollision;
    int x;
    int y;
    while(iteration++ <= STEPS) {
        auto results = AtomicStepReal(state, action, observation, reward);
        terminal = std::get<0>(results);
        isCollision = std::get<1>(results);
        x = std::get<2>(results);
        y = std::get<3>(results);

        if (terminal) {
            printMap(true);

            int inn;
            std::cin >> inn;
        }

        if (terminal || isCollision)
            return make_tuple(terminal, isCollision, x, y);
    }
    return make_tuple(terminal, isCollision, x, y);
}


void GRIDWORLD::UpdateProbabiliticMap(int x, int y, int action, bool isCollision)
{
    if (x == (sizeX - 1) && y == (sizeY - 1)) {
        printMap(false);
        return;
    }

    int x_tmp, y_tmp;
    if (!isCollision) {
        ProbabilisticMap(x, y) = 0.1;
        return;
    } else {
        std::cout << "x: " << x << ", y: " << y << std::endl;
        a->updateCollisionPoint(x,y);
        a->hitCollision();
        ProbabilisticMap(x, y) = -1;

        int offsets[16];
        int length;

        if (action == COORD::E_EAST) {
            int tmp_offsets [] = {1, 0, 2, 0, 0, 1, 0, 2, 0, -1, 0, -2, 1, 1, 1, -1};
            length = sizeof(tmp_offsets)/sizeof(tmp_offsets[0]);
            for (int i = 0; i < length; i++) {
                offsets[i] = tmp_offsets[i];
            }
        } else if (action == COORD::E_SOUTH) {
            int tmp_offsets[] = {0, 1, 0, 2, 1, 0, 2, 0, -1, 0, -2, 0, 1, 1, -1, 1};
            length = sizeof(tmp_offsets) / sizeof(tmp_offsets[0]);
            for (int i = 0; i < length; i++) {
                offsets[i] = tmp_offsets[i];
            }
        } else if (action == COORD::E_WEST) {
            int tmp_offsets [] = {-1, 0, -2, 0, 0, 1, 0, 2, 0, -1, 0, -2, -1, 1, -1, -1};
            length = sizeof(tmp_offsets)/sizeof(tmp_offsets[0]);
            for (int i = 0; i < length; i++) {
                offsets[i] = tmp_offsets[i];
            }
        } else if (action == COORD::E_NORTH) {
            int tmp_offsets [] = {0, -1, 0, -2, 1, 0, 2, 0, -1, 0, -2, 0, 1, -1, -1, -1};
            length = sizeof(tmp_offsets)/sizeof(tmp_offsets[0]);
            for (int i = 0; i < length; i++) {
                offsets[i] = tmp_offsets[i];
            }
        }

        for (int i = 0; i < length; i += 2) {
            int tr = x + offsets[i];
            int tc = y + offsets[i + 1];

            if (tr >= 0 && tr < sizeX && tc >= 0 && tc < sizeY && ProbabilisticMap(tr, tc) != -1) {
                // add the newly discovered vertex to the priority queue
                if ((abs(offsets[i]) + abs(offsets[i + 1])) >= 2) {
                    ProbabilisticMap(tr, tc) = 0.7;
                } else {
                    ProbabilisticMap(tr, tc) = 0.9;
                }
            }
        }
    }

//    cout << "action: " << action << endl;
//    printProbMap();
//    int inn;
//    cin >> inn;
}

void GRIDWORLD::ClearMap()
{
    ProbabilisticMap.SetAllValues(0.1);
    solution.clear();
    a = new ASTAR(sizeX, sizeY);
}

std::tuple<bool, std::vector<int>> GRIDWORLD::Rollout(STATE& state)
{
    GRIDWORLD_STATE& robotstate = safe_cast<GRIDWORLD_STATE&>(state);
    a->setStartPosition(robotstate.AgentPos.X, robotstate.AgentPos.Y);
    bool isSolved = a->solve();
//    a->printSolution();
    std::vector<int> tmp = a->getSolution();
    return make_tuple(isSolved, tmp);
}


double GRIDWORLD::CalculateReward(int x_current, int y_current) const
{
    if ((x_current == EndPos.X) && (y_current == EndPos.Y))
        return 0;
    else
        return log(1- ProbabilisticMap(x_current,y_current));
}

double GRIDWORLD::ContainedWalls(int x, int y) const
{
    COORD w = COORD(x,y);
    if(std::find(walls.begin(), walls.end(), w) != walls.end()) {
        return true;
    } else {
        return false;
    }
}


bool GRIDWORLD::LocalMove(STATE& state, const HISTORY& history,
                           int stepObs, const STATUS& status) const
{
    return true;
}

void GRIDWORLD::GenerateLegal(const STATE& state, const HISTORY& history,
    vector<int>& legal, const STATUS& status) const
{

}

void GRIDWORLD::GeneratePreferred(const STATE& state, const HISTORY& history,
    vector<int>& actions, const STATUS& status) const
{

    static const bool UseBlindPolicy = false;

    if (UseBlindPolicy)
    {
        actions.push_back(COORD::E_EAST);
        return;
    }

    int total = 0;
    for (int t = 0; t < history.Size(); ++t)
    {
        if (history[t].Observation == E_NOT_COLLISION)
        {
//            cout << "Come here:!!!!!! " << "\n";
            actions.push_back(history[t].Action);
        }
    }
}


void GRIDWORLD::DisplayBeliefs(const BELIEF_STATE& beliefState,
    std::ostream& ostr) const
{
}

void GRIDWORLD::DisplayState(const STATE& state, std::ostream& ostr) const
{

}

void GRIDWORLD::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{

}

void GRIDWORLD::DisplayAction(int action, std::ostream& ostr) const
{

}
