#include <vector>
#include <string>
#include <iostream>

using namespace std;

class state;

class action
{   
public:
    vector<pair<float, state * > > distr; // action distribution 
    float ca; //immidate cost
    string aname; // action name: up, left, right...
    action(float c, vector<pair<float, state *> > dist, string n);


    ~action();
};

class state{

    private:
        vector<action *> sactions; // state's actions
        float svalue; // state's value
        action *policy; // opttimal poolicy  
        int _currentIteration = 0;
        
    public:
        state(float val = 0, action *p = nullptr): svalue(val), policy(p){};
        float getValue(){
            return svalue;
        }
        void setValue(float v){
            svalue = v;
        }
        int getCurrentIteration()
        {
            return _currentIteration;
        }
        void setCurrentIteration(int iteration)
        {
            _currentIteration = iteration;
        }
       
        action * getPolicy(){
            return policy;
        }
        void setPolicy(action *p){
            policy = p;
        }
        vector<action *> getActions(){
            return sactions;
        }

        void addActions(vector<action *> v);

        ~state();

};

class GridMDP{
    public: 
        GridMDP(float e, float discount=1.0): epsilon(e), discount(discount){};
        void setEpsilon(float e);
        void setDiscount(float d = 1.0){
            discount = d;
        };
        void addState(state *s){
            _states.push_back(s);
        }
        unsigned long getIters(){
            return iters;
        }
        unsigned long getStatesVisited()
        {
            return statesVisited;
        }
        state* getGoalState()
        {
            return _goalState;
        }
        void setGoalState(state* goalState)
        {
            _goalState = goalState;
        }

        void valueIteration();
        
        void RTDP(state* startState);
        void FocusedValueIteration(state* startState);
        void LabeledRTDP();
        void backup(state* s);
        

        ~GridMDP();


    private:
        vector<state *> _states;
        vector<state*> _initialStates;
        float epsilon;
        float discount;
        unsigned long iters = 0;
        unsigned long statesVisited = 0;
        state* _goalState;
        float calculateActionCost(action *action) const;
        pair<float, action*> calculateBestAction(vector<action*> actions);
        state* rtdpGetNextState(action* currentAction);
        float focusedVIrec(state* state);
};



