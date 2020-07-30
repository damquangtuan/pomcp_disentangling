#include "testsimulator.h"
#include "utils.h"

using namespace UTILS;

STATE* TEST_SIMULATOR::Copy(const STATE& state) const
{
    const TEST_STATE& tstate = safe_cast<const TEST_STATE&>(state);
    TEST_STATE* newstate = new TEST_STATE;
    newstate->Depth = tstate.Depth;
    return newstate;
}

STATE* TEST_SIMULATOR::CreateStartState() const
{
    return new TEST_STATE;
}

void TEST_SIMULATOR::FreeState(STATE* state) const
{
    delete state;
}

bool TEST_SIMULATOR::Step(STATE& state, int action, 
    int& observation, double& reward) const
{
    // Up to MaxDepth action 0 is good independent of observations
    TEST_STATE& tstate = safe_cast<TEST_STATE&>(state);
    if (tstate.Depth < MaxDepth && action == 0)
        reward = 1.0;
    else
        reward = 0.0;

    observation = Random(0, GetNumObservations());
    tstate.Depth++;
    return false;
}

std::tuple<bool, bool, int, int> TEST_SIMULATOR::StepReal(STATE& state, int action, int& observation, double& reward)
{
    return std::make_tuple(false, false, -1, -1);
}

void TEST_SIMULATOR::UpdateProbabiliticMap(int x, int y, int action, bool isCollision)
{

}

void TEST_SIMULATOR::ClearMap() {}

std::tuple<bool, std::vector<int>> TEST_SIMULATOR::Rollout(STATE& state)
{
    return make_tuple(true, std::vector<int>(0));
}

double TEST_SIMULATOR::OptimalValue() const
{
    double discount = 1.0;
    double totalReward = 0.0;
    for (int i = 0; i < MaxDepth; i++)
    {
        totalReward += discount;
        discount *= GetDiscount();
    }
    return totalReward;
}

double TEST_SIMULATOR::MeanValue() const
{
    double discount = 1.0;
    double totalReward = 0.0;
    for (int i = 0; i < MaxDepth; i++)
    {
        totalReward += discount / GetNumActions();
        discount *= GetDiscount();
    }
    return totalReward;
}
