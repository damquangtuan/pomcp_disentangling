#ifndef MCTS_H
#define MCTS_H

#include "simulator.h"
#include "node.h"
#include "statistic.h"
#include "astarmaze.h"
#include "gridworld.h"

class MCTS
{
public:

    struct PARAMS
    {
        PARAMS();

        int Verbose;
        int MaxDepth;
        int NumSimulations;
        int NumStartStates;
        bool UseTransforms;
        int NumTransforms;
        int MaxAttempts;
        int ExpandCount;
        double ExplorationConstant;
        bool UseRave;
        double RaveDiscount;
        double RaveConstant;
        bool DisableTree;
        double Cp;
    };

    MCTS(SIMULATOR& simulator, const PARAMS& params);
    ~MCTS();

    int SelectAction();
    bool Update(int action, int observation, double reward);

    void UCTSearch();
    void RolloutSearch();

    double GetTreeDepth() {return StatTreeDepth.GetMean();}

    double Rollout(STATE& state);

    double RolloutAstar(STATE& state);

    const BELIEF_STATE& BeliefState() const { return Root->Beliefs(); }
    const HISTORY& GetHistory() const { return History; }
    const SIMULATOR::STATUS& GetStatus() const { return Status; }
    void ClearStatistics();
    void DisplayStatistics(std::ostream& ostr) const;
    void DisplayValue(int depth, std::ostream& ostr) const;
    void DisplayPolicy(int depth, std::ostream& ostr) const;

    static void UnitTest();
    static void InitFastUCB(double exploration);

private:

    SIMULATOR& Simulator;
    int TreeDepth, PeakTreeDepth;
    PARAMS Params;
    VNODE* Root;
    HISTORY History;
    SIMULATOR::STATUS Status;

    STATISTIC StatTreeDepth;
    STATISTIC StatRolloutDepth;
    STATISTIC StatTotalReward;

    int GreedyUCB(VNODE* vnode, bool ucb) const;
    int SelectRandom() const;
    double SimulateV(STATE& state, VNODE* vnode);
    double SimulateQ(STATE& state, QNODE& qnode, int action);
    void AddRave(VNODE* vnode, double totalReward);
    VNODE* ExpandNode(const STATE* state);
    void AddSample(VNODE* node, const STATE& state);
    void AddTransforms(VNODE* root, BELIEF_STATE& beliefs);
    STATE* CreateTransform() const;
    void Resample(BELIEF_STATE& beliefs);

    maze random_maze(int x, int y, int startX, int startY);

    // Fast lookup table for UCB
    static const int UCB_N = 10000, UCB_n = 100;
    static double UCB[UCB_N][UCB_n];
    static bool InitialisedFastUCB;

    double FastUCB(int N, int n, double logN) const;

    static void UnitTestGreedy();
    static void UnitTestUCB();
    static void UnitTestRollout();
    static void UnitTestSearch(int depth);
};

#endif // MCTS_H
