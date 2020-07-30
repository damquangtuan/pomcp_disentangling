#ifndef GRIDWORLD_H
#define GRIDWORLD_H

#include "simulator.h"
#include "coord.h"
#include "grid.h"
#include "astarmaze.h"
#include "astar.h"

class GRIDWORLD_STATE : public STATE
{
public:

    COORD AgentPos;
    std::vector<COORD> CollisionPoints;
    int action;
    int observation;
};

class GRIDWORLD : public SIMULATOR
{
public:

    GRIDWORLD(int sizeX, int sizeY, double Cp, bool isReal);

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action,
        int& observation, double& reward) const;

    virtual std::tuple<bool, bool, int, int> StepReal(STATE& state, int action, int& observation, double& reward);
    virtual void UpdateProbabiliticMap(int x, int y, int action, bool isCollision);
    virtual void ClearMap();
    virtual double CalculateReward(int x, int y) const;
    virtual double ContainedWalls(int x, int y) const;

    virtual std::tuple<bool, std::vector<int>> Rollout(STATE& state);

    void GenerateLegal(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    void GeneratePreferred(const STATE& state, const HISTORY& history,
        std::vector<int>& legal, const STATUS& status) const;
    virtual bool LocalMove(STATE& state, const HISTORY& history,
        int stepObservation, const STATUS& status) const;

    virtual void DisplayBeliefs(const BELIEF_STATE& beliefState,
        std::ostream& ostr) const;
    virtual void DisplayState(const STATE& state, std::ostream& ostr) const;
    virtual void DisplayObservation(const STATE& state, int observation, std::ostream& ostr) const;
    virtual void DisplayAction(int action, std::ostream& ostr) const;

protected:

    enum
    {
        E_NOT_COLLISION,
        E_COLLISION
    };

    bool solutionFind(int x, int y);
    double sumSuccessProbFromStart(int x_current, int y_current) const;
    double sumSuccessProb(int x_current, int y_current) const;
    void printProbMap();
    void printMap(bool pSolution);
    void CreateRandomWalls(int number_of_walls);
    void CreateNWToESWalls();
    void CreateWalls();
    void Init(bool isReal);
    GRID<double> Grid, ProbabilisticMap;
    int sizeX, sizeY;
    std::vector<COORD> walls;
    bool AtomicStep(STATE& state, int action,
              int& observation, double& reward) const;
    std::tuple<bool, bool, int, int> AtomicStepReal(STATE& state, int action,
                    int& observation, double& reward);

    void testAStar();

    COORD StartPos;
    COORD EndPos;

    ASTAR *a;
    std::vector<int_pi> solution;

private:

    mutable MEMORY_POOL<GRIDWORLD_STATE> MemoryPool;
};
#endif
