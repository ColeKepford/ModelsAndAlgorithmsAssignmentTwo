#include "model.h"

using namespace std;

#define ca 1.0 //same cost for all actions.


int main(){

    GridMDP grid(0.00001, 1.0); //epsilon = 0.001, discount = 1 by default 
    state s11;
    state s12;
    state s13;

    state s21;
    state s22;
    state s23;

    state s31;
    state s32;
    state s33;

    state s41;
    state s42;
    state s43;

    //state s11
    action up11(ca, {{0.8, &s12}, {0.1, &s21}, {0.1, &s11}}, "UP");
    action dw11(ca, {{0.8, &s11}, {0.1, &s11}, {0.1, &s21}}, "DOWN");
    action r11(ca, {{0.8, &s21}, {0.1, &s12}, {0.1, &s11}}, "RIGHT");
    action l11(ca, {{0.8, &s11}, {0.1, &s11}, {0.1, &s12}}, "LEFT");
    s11.addActions({&up11, &dw11, &r11, &l11}); 

    //state s12
    action up12(ca, {{0.8, &s13}, {0.1, &s12}, {0.1, &s12}}, "UP");
    action dw12(ca, {{0.8, &s11}, {0.1, &s12}, {0.1, &s12}}, "DOWN");
    action r12(ca, {{0.8, &s12}, {0.1, &s11}, {0.1, &s13}}, "RIGHT");
    action l12(ca, {{0.8, &s12}, {0.1, &s11}, {0.1, &s13}}, "LEFT");
    s12.addActions({&up12, &dw12, &r12, &l12}); 
   

    //state s13
    action up13(ca, {{0.8, &s13}, {0.1, &s13}, {0.1, &s23}}, "UP");
    action dw13(ca, {{0.8, &s12}, {0.1, &s13}, {0.1, &s23}}, "DOWN");
    action r13(ca, {{0.8, &s23}, {0.1, &s13}, {0.1, &s12}}, "RIGHT");
    action l13(ca, {{0.8, &s13}, {0.1, &s13}, {0.1, &s12}}, "LEFT");
    s13.addActions({&up13, &dw13, &r13, &l13});

    //state s21
    action up21(ca, {{0.8, &s21}, {0.1, &s11}, {0.1, &s31}}, "UP");
    action dw21(ca, {{0.8, &s21}, {0.1, &s11}, {0.1, &s31}}, "DOWN");
    action r21(ca, {{0.8, &s31}, {0.1, &s21}, {0.1, &s21}}, "RIGHT");
    action l21(ca, {{0.8, &s11}, {0.1, &s21}, {0.1, &s21}}, "LEFT");
    s21.addActions({&up21, &dw21, &r21, &l21});

    //state s22// Deadend, no actions

    //state s23
    action up23(ca, {{0.8, &s23}, {0.1, &s13}, {0.1, &s33}}, "UP");
    action dw23(ca, {{0.8, &s23}, {0.1, &s13}, {0.1, &s33}}, "DOWN");
    action r23(ca, {{0.8, &s33}, {0.1, &s23}, {0.1, &s23}}, "RIGHT");
    action l23(ca, {{0.8, &s13}, {0.1, &s23}, {0.1, &s23}}, "LEFT");
    s23.addActions({&up23, &dw23, &r23, &l23});

    //state s31
    action up31(ca, {{0.8, &s32}, {0.1, &s21}, {0.1, &s41}}, "UP");
    action dw31(ca, {{0.8, &s31}, {0.1, &s21}, {0.1, &s41}}, "DOWN");
    action r31(ca, {{0.8, &s41}, {0.1, &s32}, {0.1, &s31}}, "RIGHT");
    action l31(ca, {{0.8, &s21}, {0.1, &s31}, {0.1, &s32}}, "LEFT");
    s31.addActions({&up31, &dw31, &r31, &l31});

    //state s32
    action up32(ca, {{0.8, &s33}, {0.1, &s32}, {0.1, &s42}}, "UP");
    action dw32(ca, {{0.8, &s31}, {0.1, &s32}, {0.1, &s42}}, "DOWN");
    action r32(ca, {{0.8, &s42}, {0.1, &s33}, {0.1, &s31}}, "RIGHT");
    action l32(ca, {{0.8, &s32}, {0.1, &s31}, {0.1, &s33}}, "LEFT");
    s32.addActions({&up32, &dw32, &r32, &l32});

    //state s33
    action up33(ca, {{0.8, &s33}, {0.1, &s23}, {0.1, &s43}}, "UP");
    action dw33(ca, {{0.8, &s32}, {0.1, &s23}, {0.1, &s43}}, "DOWN");
    action r33(ca, {{0.8, &s43}, {0.1, &s33}, {0.1, &s32}}, "RIGHT");
    action l33(ca, {{0.8, &s23}, {0.1, &s32}, {0.1, &s33}}, "LEFT");
    s33.addActions({&up33, &dw33, &r33, &l33});

    //state s41
    action up41(ca, {{0.8, &s42}, {0.1, &s31}, {0.1, &s41}},  "UP");
    action dw41(ca, {{0.8, &s41}, {0.1, &s31}, {0.1, &s41}}, "DOWN");
    action r41(ca, {{0.8, &s41}, {0.1, &s41}, {0.1, &s42}}, "RIGHT");
    action l41(ca, {{0.8, &s31}, {0.1, &s41}, {0.1, &s42}}, "LEFT");
    s41.addActions({&up41, &dw41, &r41, &l41});

    //state s42: deadend, value = INT_MAX, no actions
    s42.setValue(INT_MAX);

    //goal state state s43: value = 0, no actions
    s43.setValue(0.0);

    //add states to the grid
    grid.addState(&s11);
    grid.addState(&s12);
    grid.addState(&s13);
    grid.addState(&s21);
    grid.addState(&s22);
    grid.addState(&s23);
    grid.addState(&s31);
    grid.addState(&s32);
    grid.addState(&s33);
    grid.addState(&s41);
    grid.addState(&s42);
    grid.addState(&s43);

    //Set Initial State
    grid.setGoalState(&s43);
    //value iteration
    //grid.policyIteration();
    grid.valueIteration();
    //grid.RTDP(&s11);
    //grid.FocusedValueIteration(&s11);

    //print the values and ploicy of the states
    cout << "Number of iterations before convergence: "<<grid.getIters() << endl;
    cout << "Number of states visited: " << grid.getStatesVisited() << endl;
    cout << "s11: " << s11.getValue() << endl;
    cout << "s12: " << s12.getValue() << endl;
    cout << "s13: " << s13.getValue() << endl;
    cout << "s21: " << s21.getValue() << endl;
    cout << "s23: " << s23.getValue() << endl;
    cout << "s31: " << s31.getValue() << endl;
    cout << "s32: " << s32.getValue() << endl;
    cout << "s33: " << s33.getValue() << endl;
    cout << "s41: " << s41.getValue() << endl;
    cout << "s42: " << s42.getValue() << endl;
    cout << "s43: " << s43.getValue() << endl;

    cout << "State policies after convergence: " << endl;
    cout << "s11: " << s11.getPolicy()->aname << endl;
    cout << "s12: " << s12.getPolicy()->aname << endl;
    cout << "s13: " << s13.getPolicy()->aname << endl;
    cout << "s21: " << s21.getPolicy()->aname << endl;
    cout << "s23: " << s23.getPolicy()->aname << endl;
    cout << "s31: " << s31.getPolicy()->aname << endl;
    cout << "s32: " << s32.getPolicy()->aname << endl;
    cout << "s33: " << s33.getPolicy()->aname << endl;
    cout << "s41: " << s41.getPolicy()->aname << endl;

    return 0;
}