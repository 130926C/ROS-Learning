#include "PathPlaining.hpp"

#include <stdio.h>

bool PathSolver::solve(int type){
    switch (type)
    {
    case 0:
        return __Dijstra__(); break;
    case 1:
        return __A_star__(); break;
    case 2:
        return __RRT__(); break;
    case 3:
        return __RRT_star__(); break;
    case 4:
        return __Inform_RRT_star__(); break;
    case 5:
        return __PRM__(); break;
    default:
        return __Dijstra__(); break;
    }
}

bool PathSolver::__Dijstra__(){
    bool flag = false;
    printf("use Dijstar Algorithm...");

    


    return flag;
}

bool PathSolver::__A_star__(){
    bool flag = false;
    printf("use A* Algorithm...");
    

    return flag;
}

bool PathSolver::__Inform_RRT_star__(){
    bool flag = false;
    printf("use Inform RRT* Algorithm...");
    

    return flag;
}

bool PathSolver::__RRT__(){
    bool flag = false;
    printf("use RRT Algorithm...");
    

    return flag;
}

bool PathSolver::__RRT_star__(){
    bool flag = false;
    printf("use RRT* Algorithm...");
    

    return flag;
}

bool PathSolver::__PRM__(){
    bool flag = false;
    printf("use PRM Algorithm...");
    

    return flag;
}