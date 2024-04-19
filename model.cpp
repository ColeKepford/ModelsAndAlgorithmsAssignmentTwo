#include "model.h"
#include <random>


//--Action
action::action(float c, vector<pair<float, state *> > dist, string n)
{
    this->ca = c;
    this->aname = n;
    for(auto e : dist)
        this->distr.push_back(e);
}

action::~action()
{
}

//-----------State----

void state::addActions(vector<action *> v)
{
    for (auto e: v)
        this->sactions.push_back(e);
}



state::~state()
{

}



//--------MDP--------

void GridMDP::setEpsilon(float e)
{
    this->epsilon = e;
}

void GridMDP::valueIteration()
{
    float delta;
    do
    {
        delta = 0;
        for (state* st : GridMDP::_states)
        {
            statesVisited++;
            if (st->getActions().size() == 0)
            {
                continue;
            }
            float old_value = st->getValue();
            pair<float, action*> bestCostActionPair = calculateBestAction(st->getActions());
            st->setValue(bestCostActionPair.first);
            st->setPolicy(bestCostActionPair.second);

            delta = max(delta, abs(old_value - bestCostActionPair.first));
        }
        this->iters = this->iters + 1;
        
    } while (delta > this->epsilon);
}

float GridMDP::calculateActionCost(action *action) const 
{
    float cost = 0;
    for (pair<float, state*> pair : action->distr)
    {
        cost += pair.first * pair.second->getValue();
    }
    cost = action->ca + this->discount * cost;

    return cost;
}

pair<float, action*> GridMDP::calculateBestAction(vector<action*> actions) 
{
    float newCost = INT_MAX;
    action* newAction;
    for (action* action : actions)
    {
        float actionCost = calculateActionCost(action);
        if (actionCost < newCost)
        {
            newCost = actionCost;
            newAction = action;
        }
    }
    return pair<float, action*>(newCost, newAction);
}

state* GridMDP::rtdpGetNextState(action* currentAction)
{
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(0.0, 1.0);
    float randomValue = dis(gen);
    float cumulativeProbability = 0.0;

    for (pair<float, state*> pair : currentAction->distr)
    {
        cumulativeProbability += pair.first;
        if (randomValue <= cumulativeProbability)
        {
            return pair.second;
        }
    }
}


void GridMDP::RTDP(state* startState)
{
    state* currentState = startState;
    float delta;
    do
    {
        currentState = startState;
        delta = 0;
        while (currentState != _goalState)
        {
            statesVisited++;
            float oldValue = currentState->getValue();

            pair<float, action*> bestCostActionPair = calculateBestAction(currentState->getActions());
            currentState->setValue(bestCostActionPair.first);
            currentState->setPolicy(bestCostActionPair.second);

            delta = max(delta, abs(oldValue - currentState->getValue()));

            currentState = rtdpGetNextState(currentState->getPolicy());
        }
        iters = iters + 1;
    } while (iters < 100);
}

void GridMDP::FocusedValueIteration(state* startState)
{
    float residual = 0;
    do
    {
        iters++;
        startState->setCurrentIteration(iters);
        residual = focusedVIrec(startState);
    } while (residual >= epsilon);
}

float GridMDP::focusedVIrec(state* currentState)
{
    statesVisited++;
    if (currentState == _goalState)
    {
        return 0;
    }
    float oldValue = currentState->getValue();
    pair<float, action*> costActionPair = calculateBestAction(currentState->getActions());
    currentState->setValue(costActionPair.first);
    currentState->setPolicy(costActionPair.second);
    float residual = currentState->getValue() - oldValue;

    for (pair<float, state*> pair : currentState->getPolicy()->distr)
    {
        if (pair.second->getCurrentIteration() < iters) 
        {
            pair.second->setCurrentIteration(iters);
            float r = focusedVIrec(pair.second);
            if (r > residual)
            {
                residual = r;
            }
        }
    }
    costActionPair = calculateBestAction(currentState->getActions());
    currentState->setValue(costActionPair.first);

    return residual;
}

GridMDP::~GridMDP()
{
}
