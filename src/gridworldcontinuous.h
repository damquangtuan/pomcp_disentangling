#ifndef GRIDWORLDCONTINUOUS_H
#define GRIDWORLDCONTINUOUS_H

#include "simulator.h"
#include "coord.h"
#include "grid.h"

class GRIDWORLDCONTINUOUS_STATE : public STATE
{
public:

    COORD AgentPos;
    std::vector<COORD> CollisionPoints;
    int action;
    int observation;
};

class GRIDWORLDCONTINUOUS : public SIMULATOR
{
public:

    GRIDWORLDCONTINUOUS(int sizeX, int sizeY, double Cp, bool isReal);

    virtual STATE* Copy(const STATE& state) const;
    virtual void Validate(const STATE& state) const;
    virtual STATE* CreateStartState() const;
    virtual void FreeState(STATE* state) const;
    virtual bool Step(STATE& state, int action,
        int& observation, double& reward) const;

    virtual std::tuple<bool, bool, int, int> StepReal(STATE& state, int action, int& observation, double& reward) const;

    virtual void UpdateProbabiliticMap(int x, int y, int action);

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

    void Init_30_20(bool isReal);
    GRID<int> Grid, ProbabilisticMap;
    std::vector<COORD> CollisionPoints;
    int sizeX, sizeY;

    double ObstacleDistance(COORD start, COORD end);

    COORD StartPos;
    COORD EndPos;

private:

    mutable MEMORY_POOL<GRIDWORLDCONTINUOUS_STATE> MemoryPool;
};
#endif
