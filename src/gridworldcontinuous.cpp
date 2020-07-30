#include "gridworldcontinuous.h"
#include "utils.h"

using namespace std;
using namespace UTILS;

GRIDWORLDCONTINUOUS::GRIDWORLDCONTINUOUS(int sizeX, int sizeY, double Cp, bool isReal)
:   Grid(sizeX, sizeY), ProbabilisticMap(sizeX, sizeY),
    sizeX(sizeX), sizeY(sizeY)
{
    NumActions = 8;
    NumObservations = 2;
    RewardRange = Cp;
    Discount = 0.99;
    Init_30_20(isReal);
}

void GRIDWORLDCONTINUOUS::Init_30_20(bool isReal)
{
    cout << "Using special layout for GridWorld(60, 50)" << endl;

    std::vector<COORD> walls;

    for (int index = 29; index >= 1; index--) {
        walls.push_back(COORD(index, 15));
        walls.push_back(COORD(index, 12));
        walls.push_back(COORD(index, 8));
        walls.push_back(COORD(index, 5));
    }

    for (int index = 19; index >= 1; index--) {
        walls.push_back(COORD(1, index));
        walls.push_back(COORD(5, index));
        walls.push_back(COORD(8, index));
        walls.push_back(COORD(14, index));
    }

    int NumRocks = 192;

    StartPos = COORD(29, 0);
    EndPos = COORD(29, 19);
    Grid.SetAllValues(1);
    ProbabilisticMap.SetAllValues(1);
    for (int i = 0; i < NumRocks; ++i)
    {
        Grid(walls[i]) = 0;
        if (isReal == true)
            ProbabilisticMap(walls[i]) = 0;
    }

    CollisionPoints.clear();
}


STATE* GRIDWORLDCONTINUOUS::Copy(const STATE& state) const
{
    const GRIDWORLDCONTINUOUS_STATE& robotstate = safe_cast<const GRIDWORLDCONTINUOUS_STATE&>(state);
    GRIDWORLDCONTINUOUS_STATE* newstate = MemoryPool.Allocate();
    *newstate = robotstate;
    return newstate;
}

void GRIDWORLDCONTINUOUS::Validate(const STATE& state) const
{
    const GRIDWORLDCONTINUOUS_STATE& robotstate = safe_cast<const GRIDWORLDCONTINUOUS_STATE&>(state);
    assert(Grid.Inside(robotstate.AgentPos));
}

STATE* GRIDWORLDCONTINUOUS::CreateStartState() const
{
    GRIDWORLDCONTINUOUS_STATE* robotstate = MemoryPool.Allocate();
    robotstate->AgentPos = StartPos;
    robotstate->CollisionPoints.clear();
    robotstate->action = -1;
    robotstate->observation = E_NOT_COLLISION;

    return robotstate;
}

void GRIDWORLDCONTINUOUS::FreeState(STATE* state) const
{
    GRIDWORLDCONTINUOUS_STATE* robotstate = safe_cast<GRIDWORLDCONTINUOUS_STATE*>(state);
    MemoryPool.Free(robotstate);
}

bool GRIDWORLDCONTINUOUS::Step(STATE& state, int action,
    int& observation, double& reward) const
{
    GRIDWORLDCONTINUOUS_STATE& robotstate = safe_cast<GRIDWORLDCONTINUOUS_STATE&>(state);
    reward = 0;
    observation = E_NOT_COLLISION;

    int x = -1;
    int y = -1;

    switch (action)
    {
        case COORD::E_EAST: {
            x = robotstate.AgentPos.X + 1;
            y = robotstate.AgentPos.Y;
            if ((x < sizeX) && (ProbabilisticMap(x, y) != 0)) {
                robotstate.AgentPos.X++;
            } else if ((x < sizeX) && (ProbabilisticMap(x, y) == 0)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_NORTHEAST: {
            x = robotstate.AgentPos.X + 1;
            y = robotstate.AgentPos.Y + 1;
            if ((x < sizeX) && (y < sizeY) && (ProbabilisticMap(x, y) != 0)) {
                robotstate.AgentPos.X++;
                robotstate.AgentPos.Y++;
            }
            else if ((x < sizeX) && (y < sizeY) && (ProbabilisticMap(x, y) == 0)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_NORTH: {
            x = robotstate.AgentPos.X;
            y = robotstate.AgentPos.Y + 1;
            if ((y < sizeY) && (ProbabilisticMap(x, y) != 0))
                robotstate.AgentPos.Y++;

            else if ((y < sizeY) && (ProbabilisticMap(x, y) == 0)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_NORTHWEST: {
            x = robotstate.AgentPos.X - 1;
            y = robotstate.AgentPos.Y + 1;
            if ((y < sizeY) && (x >= 0) && (ProbabilisticMap(x, y) != 0)) {
                robotstate.AgentPos.X--;
                robotstate.AgentPos.Y++;
            }

            else if ((y < sizeY) && (x >= 0) && (ProbabilisticMap(x, y) == 0)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_SOUTH: {
            x = robotstate.AgentPos.X;
            y = robotstate.AgentPos.Y - 1;
            if ((y >= 0) && (ProbabilisticMap(x, y) != 0))
                robotstate.AgentPos.Y--;

            else if ((y >= 0) && (ProbabilisticMap(x, y) == 0)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }

        case COORD::E_SOUTHWEST: {
            x = robotstate.AgentPos.X - 1;
            y = robotstate.AgentPos.Y - 1;
            if ((x >= 0) && (y >= 0) && (ProbabilisticMap(x, y) != 0)) {
                robotstate.AgentPos.X--;
                robotstate.AgentPos.Y--;
            }

            else if ((x >= 0) && (y >= 0) && (ProbabilisticMap(x, y) == 0)) {
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
            if ((x >= 0) && (ProbabilisticMap(x, y) != 0))
                robotstate.AgentPos.X--;

            if ((x >= 0) && (ProbabilisticMap(x, y) == 0)) {
                robotstate.CollisionPoints.push_back(COORD(x, y));
                observation = E_COLLISION;
                robotstate.action = action;
                robotstate.observation = observation;
            }

            break;
        }
        case COORD::E_SOUTHEAST: {
            x = robotstate.AgentPos.X + 1;
            y = robotstate.AgentPos.Y - 1;
            if ((x < sizeX) && (y >= 0) && (ProbabilisticMap(x, y) != 0)) {
                robotstate.AgentPos.X++;
                robotstate.AgentPos.Y--;
            }

            if ((x < sizeX) && (y >= 0) && (ProbabilisticMap(x, y) == 0)) {
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
    reward = ProbabilisticMap(robotstate.AgentPos.X, robotstate.AgentPos.Y) - 1;

    if (COORD(robotstate.AgentPos.X, robotstate.AgentPos.Y) == EndPos){
        reward = 1;
        return true;
    }
    else
        return false;
}

std::tuple<bool, bool, int, int> GRIDWORLDCONTINUOUS::StepReal(STATE& state, int action,
                     int& observation, double& reward) const
{
    GRIDWORLDCONTINUOUS_STATE& robotstate = safe_cast<GRIDWORLDCONTINUOUS_STATE&>(state);
    reward = 0;
    observation = E_NOT_COLLISION;

    int x = -1;
    int y = -1;

    switch (action)
    {
        case COORD::E_EAST: {
            x = robotstate.AgentPos.X + 1;
            y = robotstate.AgentPos.Y;
            if ((x < sizeX) && (ProbabilisticMap(x, y) != 0))
                robotstate.AgentPos.X++;

            else if ((x < sizeX) && (ProbabilisticMap(x, y) == 0)) {
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
            y = robotstate.AgentPos.Y + 1;
            if ((x < sizeX) && (y < sizeY) && (ProbabilisticMap(x, y) != 0)) {
                robotstate.AgentPos.X++;
                robotstate.AgentPos.Y++;
            }
            else if ((x < sizeX) && (y < sizeY) && (ProbabilisticMap(x, y) == 0)) {
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
            y = robotstate.AgentPos.Y + 1;
            if ((y < sizeY) && (ProbabilisticMap(x, y) != 0))
                robotstate.AgentPos.Y++;

            else if ((y < sizeY) && (ProbabilisticMap(x, y) == 0)) {
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
            y = robotstate.AgentPos.Y + 1;
            if ((y < sizeY) && (x >= 0) && (ProbabilisticMap(x, y) != 0)) {
                robotstate.AgentPos.X--;
                robotstate.AgentPos.Y++;
            }

            else if ((y < sizeY) && (x >= 0) && (ProbabilisticMap(x, y) == 0)) {
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
            y = robotstate.AgentPos.Y - 1;
            if ((y >= 0) && (ProbabilisticMap(x, y) != 0))
                robotstate.AgentPos.Y--;

            else if ((y >= 0) && (ProbabilisticMap(x, y) == 0)) {
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
            y = robotstate.AgentPos.Y - 1;
            if ((x >= 0) && (y >= 0) && (ProbabilisticMap(x, y) != 0)) {
                robotstate.AgentPos.X--;
                robotstate.AgentPos.Y--;
            }

            else if ((x >= 0) && (y >= 0) && (ProbabilisticMap(x, y) == 0)) {
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
            if ((x >= 0) && (ProbabilisticMap(x, y) != 0))
                robotstate.AgentPos.X--;

            else if ((x >= 0) && (ProbabilisticMap(x, y) == 0)) {
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
            y = robotstate.AgentPos.Y - 1;
            if ((x < sizeX) && (y >= 0) && (ProbabilisticMap(x, y) != 0)) {
                robotstate.AgentPos.X++;
                robotstate.AgentPos.Y--;
            }

            if ((x < sizeX) && (y >= 0) && (ProbabilisticMap(x, y) == 0)) {
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
//    double largestDistance = COORD::ManhattanDistance(COORD(0,0), EndPos);
//    if (observation == E_COLLISION)
//        reward = 0;
//    else
//        reward = COORD::ManhattanDistance(COORD(robotstate.AgentPos.X, robotstate.AgentPos.Y), EndPos)/largestDistance;

    reward = ProbabilisticMap(robotstate.AgentPos.X, robotstate.AgentPos.Y) - 1;

    if (COORD(robotstate.AgentPos.X, robotstate.AgentPos.Y) == EndPos) {
        reward = 1;
        cout << "Get to the goal position!!!" << "\n";
        return make_tuple(true, observation == E_COLLISION, robotstate.AgentPos.X, robotstate.AgentPos.Y);
    }
    else
        return make_tuple(false, observation == E_COLLISION, robotstate.AgentPos.X, robotstate.AgentPos.Y);
}

void GRIDWORLDCONTINUOUS::UpdateProbabiliticMap(int x, int y)
{
    ProbabilisticMap(x, y) = 0;
}

bool GRIDWORLDCONTINUOUS::LocalMove(STATE& state, const HISTORY& history,
                           int stepObs, const STATUS& status) const
{
    return true;
}

void GRIDWORLDCONTINUOUS::GenerateLegal(const STATE& state, const HISTORY& history,
    vector<int>& legal, const STATUS& status) const
{

}

void GRIDWORLDCONTINUOUS::GeneratePreferred(const STATE& state, const HISTORY& history,
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


void GRIDWORLDCONTINUOUS::DisplayBeliefs(const BELIEF_STATE& beliefState,
    std::ostream& ostr) const
{
}

void GRIDWORLDCONTINUOUS::DisplayState(const STATE& state, std::ostream& ostr) const
{

}

void GRIDWORLDCONTINUOUS::DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const
{

}

void GRIDWORLDCONTINUOUS::DisplayAction(int action, std::ostream& ostr) const
{

}

double GRIDWORLDCONTINUOUS::ObstacleDistance(COORD start, COORD end)
{

}
