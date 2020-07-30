//
// Created by tuandam on 07.07.20.
//

#include "astar.h"

using namespace std;

typedef pair<double, Cell*> pi;

ASTAR::ASTAR(int sizeX, int sizeY) : cells(sizeX, sizeY), updated_maps(sizeX, sizeY), sizeX(sizeX), sizeY(sizeY), task(new Task(0, 0, sizeX - 1, sizeY - 1))  {

    // generate maze
    generateMaze();
}

void ASTAR::setStartPosition(int x, int y)
{
    task->startRow = x;
    task->startColumn = y;
}

void ASTAR::setEndPosition(int x, int y)
{
    task->endRow= x;
    task->endColumn = y;
}


void ASTAR::createWalls()
{
    for (int index = 0; index < 15; index++) {
        cells(15, index).blocked = true;
    }
    int indexy = 14;
    for (int indexx = 15; indexx <= 28; indexx++) {
        cells(indexy--, indexx).blocked = true;
    }
    for (int indexx = 22; indexx <= 29; indexx++) {
        cells(14, indexx).blocked = true;
    }
}

std::vector<int> ASTAR::getSolution()
{
    vector<int> actions;
    vector<int_pi>::reverse_iterator it, next;
    for (it = solution.rbegin(); it != solution.rend(); ++it)
    {
        next = it + 1;
        if ((next->first - it->first) == 1 && (next->second - it->second == 0)) {
//            std::cout << "E_EAST" << std::endl;
            actions.push_back(COORD::E_EAST);
        }
        else if ((next->first - it->first) == -1 && (next->second - it->second == 0)) {
//            std::cout << "E_WEST" << std::endl;
            actions.push_back(COORD::E_WEST);
        }
        else if ((next->first - it->first) == 0 && (next->second - it->second == 1)) {
//            std::cout << "E_SOUTH" << std::endl;
            actions.push_back(COORD::E_SOUTH);
        }
        else if ((next->first - it->first) == 0 && (next->second - it->second == -1)) {
//            std::cout << "E_NORTH" << std::endl;
            actions.push_back(COORD::E_NORTH);
        }
    }

//    int inn;
//    std::cin >> inn;

    return actions;
}


// thanks Pythagoras! The straight line distance between two points in the maze.
// Distance is the key for the priority queue from which the next cells to visit is taken

double ASTAR::distance(int r1, int c1, int r2, int c2) {
    return sqrt(((r1 - r2) * (r1 - r2)) + ((c1 - c2) * (c1 - c2)));
}

// Initially all cells are marked as blocked. This algorithm builds a path of connected passages through the maze
// adding only cells that are not touching an open cell.
void ASTAR::generateMaze() {
    srand( (unsigned)time( NULL ) );
    // now update the cells with the open/blocked status and preset certain values
    // used during the A* search e.g. parent, visited, distance to target
    double h_value;

    for (int x = 0; x < sizeX; x += 1) {
        for (int y = 0; y < sizeY; y += 1) {
            h_value = sumSuccessProb(x, y);
            updated_maps(x,y).row = x;
            updated_maps(x,y).column = y;
            updated_maps(x,y).blocked = false;
            updated_maps(x,y).parent = NULL;
            updated_maps(x,y).visited = false;
            updated_maps(x,y).h_function = h_value;
            updated_maps(x,y).g_function = 0;
            updated_maps(x,y).failureProb = 0.1;

            if ((x == task->startRow) && (y == task->startColumn)) {
                cout << "S";
                cells(x,y).row = x;
                cells(x,y).column = y;
                cells(x,y).blocked = false;
                cells(x,y).parent = NULL;
                cells(x,y).visited = false;
                cells(x,y).h_function = h_value;
                cells(x,y).g_function = 0;
                cells(x,y).failureProb = 0.1;

                continue;
            } else if ((x == task->endRow) && (y == task->endColumn)) {
                cells(x,y).row = x;
                cells(x,y).column = y;
                cells(x,y).blocked = false;
                cells(x,y).parent = NULL;
                cells(x,y).visited = false;
                cells(x,y).h_function = h_value;
                cells(x,y).g_function = 0;
                cells(x,y).failureProb = 0.1;

                continue;
            }

            cells(x,y).row = x;
            cells(x,y).column = y;
            cells(x,y).blocked = false;
            cells(x,y).parent = NULL;
            cells(x,y).visited = false;
            cells(x,y).h_function = h_value;
            cells(x,y).g_function = 0;
            cells(x,y).failureProb = 0.1;

        }
    }

    createWalls();
//    printProbMap(true);
}

// syntactic sugar to make getting the blocked state of a cell easier
bool ASTAR::blocked(int r, int c) {
    return updated_maps(r,c).blocked;
}

// likewise for visited
bool ASTAR::visited(int r, int c) {
    return updated_maps(r,c).visited;
}

double ASTAR::sumSuccessProb(int x_current, int y_current) {
    double reward = log(1 - updated_maps(x_current,y_current).failureProb);
    if ((sizeX - x_current) > (sizeY - y_current))
        for (int x = x_current + 1; x < sizeX; x++)
        {
            int y = y_current + int(((sizeY - y_current)/(sizeX - x_current)) * (x - x_current));
            if (!updated_maps(x,y).blocked) {
                double intermediateReward = double(updated_maps(x, y).failureProb);
                reward += log(1 - intermediateReward);
            }
        }
    else
        for (int y = y_current + 1; y < sizeY; y++)
        {
            int x = x_current + int(((sizeX - x_current)/(sizeY - y_current)) * (y - y_current));
            if (!updated_maps(x,y).blocked) {
                double intermediateReward = double(updated_maps(x, y).failureProb);
                reward += log(1 - intermediateReward);
            }
        }

    return double(reward);///abs(MAX_REWARD));
}

// solve the maze and display progress and the resulting path
bool ASTAR::solve() {

    // create a heap / priority queue for tracking the currently open vertices that we are searching
    set<pair<double, Cell*>> pSet;
    solution.clear();

    //reset the visits
    for (int iix = 0; iix < sizeX; iix++)
        for(int iiy = 0; iiy < sizeY; iiy++) {
            if (!updated_maps(iix,iiy).blocked) {
                updated_maps(iix, iiy).visited = false;
                updated_maps(iix, iiy).g_function = 0;
                updated_maps(iix, iiy).parent = NULL;
            }
        }


    // init the queue with the starting location, mark this location as visited
    Cell *current = &updated_maps(task->startRow, task->startColumn);
    current->blocked = false;
    current->parent = NULL;
    current->visited = true;

    double h_function = current->h_function;// * (1 - current->failureProb);
    double g_function = log(1 - current->failureProb);

    // the h_function to the target is the key for the priority queue since we want to examine
    // candidates according to how close they are to the target
    pSet.insert(make_pair(g_function + h_function, current));

    set<pair<double, Cell*>>::iterator it;
    // seek a path to the target until the queue is empty
    while (!pSet.empty()) {

        //get the largest f_function value of the set and remove it
        it = pSet.begin();
        while(it != pSet.end())
            it++;
        it--;
        current = it->second;
        pSet.erase(it);

        // if we have reached the target we are done
        if (current->row == task->endRow && current->column == task->endColumn) {
            // the least cost cost from target back to source is defined in the parent links
            // of each cell starting with target node
            while ((current->row != task->startRow) || (current->column != task->startColumn)) {
                solution.push_back(make_pair(current->row, current->column));
                current = current->parent;
            }
            //remember to add start position
            solution.push_back(make_pair(task->startRow, task->startColumn));

            return true;
        }

        // we are not at the target so add all unvisited nodes that are adjacent to the current node to the priority queue

        // we only consider orthogonally connected unblocked, unvisited cells, there are there row / column offsets relative
        // to the current location

        // orthogonal neighbor offsets (row/column pairs) , 1,1,-1,-1,1,-1,-1,1
        int offsets [] = {1, 0, 0, -1, -1, 0, 0, 1};

        int length = sizeof(offsets)/sizeof(offsets[0]);

        for (int i = 0; i < length; i += 2) {
            int tr = current->row + offsets[i];
            int tc = current->column + offsets[i + 1];

            if ((tr >= 0) && (tr < sizeX) && (tc >= 0) && (tc < sizeY) && !blocked(tr, tc) && !visited(tr, tc)) {

                // add the newly discovered vertex to the priority queue
                // mark cell as visited so we don't revisit
                Cell *temp = &updated_maps(tr,tc);

                bool isIn = false;
                double h_function = temp->h_function;// * (1 - temp->failureProb);
                double g_function = current->g_function + log(1 - temp->failureProb);

                for (const auto& p : pSet) {
                    Cell *tmp = p.second;
                    if (tmp->row == temp->row && tmp->column == temp->column && g_function > tmp->g_function) {
                        tmp->g_function = g_function;
                        isIn = true;
                    }
                }

                if (isIn) continue;

                temp->blocked = false;
                temp->parent = current;
                temp->visited = true;
                temp->g_function = g_function;

                pSet.insert(make_pair(g_function + h_function, temp));
            }
        }
    }

    return false;
}

// solve the maze and display progress and the resulting path
std::vector<int_pi> ASTAR::solveReturn() {

    // create a heap / priority queue for tracking the currently open vertices that we are searching
    set<pair<double, Cell*>> pSet;
    std::vector<int_pi> solutionReturn;

    //reset the visits
    for (int iix = 0; iix < sizeX; iix++)
        for(int iiy = 0; iiy < sizeY; iiy++) {
            if (!updated_maps(iix,iiy).blocked) {
                updated_maps(iix, iiy).visited = false;
                updated_maps(iix, iiy).g_function = 0;
                updated_maps(iix, iiy).parent = NULL;
            }
        }

    // init the queue with the starting location, mark this location as visited
    Cell *current = &updated_maps(task->startRow, task->startColumn);
    current->blocked = false;
    current->parent = NULL;
    current->visited = true;

    double h_function = current->h_function;// * (1 - current->failureProb);
    double g_function = log(1 - current->failureProb);

    // the h_function to the target is the key for the priority queue since we want to examine
    // candidates according to how close they are to the target
    pSet.insert(make_pair(g_function + h_function, current));
    set<pair<double, Cell*>>::iterator it;
    // seek a path to the target until the queue is empty
    while (!pSet.empty()) {

        //get the largest f_function value of the set and remove it
        it = pSet.begin();
        while(it != pSet.end())
            it++;
        it--;
        current = it->second;
        pSet.erase(it);

        // if we have reached the target we are done
        if (current->row == task->endRow && current->column == task->endColumn) {
            // the least cost cost from target back to source is defined in the parent links
            // of each cell starting with target node
            while ((current->row != task->startRow) || (current->column != task->startColumn)) {
                solutionReturn.push_back(make_pair(current->row, current->column));
                current = current->parent;
            }
            //remember to add start position
            solutionReturn.push_back(make_pair(task->startRow, task->startColumn));

            return solutionReturn;
        }

        // we are not at the target so add all unvisited nodes that are adjacent to the current node to the priority queue

        // we only consider orthogonally connected unblocked, unvisited cells, there are there row / column offsets relative
        // to the current location

        // orthogonal neighbor offsets (row/column pairs) , 1,1,-1,-1,1,-1,-1,1
        int offsets [] = {1, 0, 0, -1, -1, 0, 0, 1};

        int length = sizeof(offsets)/sizeof(offsets[0]);

        for (int i = 0; i < length; i += 2) {
            int tr = current->row + offsets[i];
            int tc = current->column + offsets[i + 1];

            if ((tr >= 0) && (tr < sizeX) && (tc >= 0) && (tc < sizeY) && !blocked(tr, tc) && !visited(tr, tc)) {

                // add the newly discovered vertex to the priority queue
                // mark cell as visited so we don't revisit
                Cell *temp = &updated_maps(tr,tc);

                bool isIn = false;
                double h_function = temp->h_function;// * (1 - temp->failureProb);
                double g_function = current->g_function + log(1 - temp->failureProb);

                for (const auto& p : pSet) {
                    Cell *tmp = p.second;
                    if (tmp->row == temp->row && tmp->column == temp->column && g_function > tmp->g_function) {
                        tmp->g_function = g_function;
                        isIn = true;
                    }
                }

                if (isIn) continue;

                temp->blocked = false;
                temp->parent = current;
                temp->visited = true;
                temp->g_function = g_function;

                pSet.insert(make_pair(g_function + h_function, temp));
            }
        }
    }

    return solutionReturn;
}

//to check if the current solution get hitted the obstacle
bool ASTAR::hitCollision()
{
    vector<int_pi>::iterator it, prev;

    //go through the map and check if the solution contains collision point.
    for (int ix = 0; ix < sizeX; ix++)
        for(int iy = 0; iy < sizeY; iy++) {
            if (cells(ix,iy).blocked)
            {
                it = find (solution.begin(), solution.end(), make_pair(ix,iy));
                if (it != solution.end()) {
                    updated_maps(ix,iy).blocked = true;
                    prev = it - 1 ;
                    std::cout << "Map gets updated!!!" << std::endl;
                    //when we get collisions, we need to update the near by cells by update the distance
                    updateDistanceMap( prev->first - it->first, prev->second - it->second, ix, iy);
                    return true;
                }
            }
        }

    return false;
}

//the current point (x,y) is in the solution path or not
bool ASTAR::solutionFind(int x, int y)
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

void ASTAR::updateCollisionMap(GRID<int> map)
{
    for (int x = 0; x < sizeX; x++)
        for (int y = 0; y < sizeY; y++) {
            if (map(x,y) == -1)
                updated_maps(x,y).blocked = true;
            else
                updated_maps(x,y).blocked = false;
        }
}

void ASTAR::updateCollisionPoint(int x, int y, int action)
{
    updated_maps(x,y).blocked = true;
    int directionX, directionY;
    switch (action)
    {
        case COORD::E_EAST: {
            directionX = 1; directionY = 0;
            break;
        }
        case COORD::E_SOUTH: {
            directionX = 0; directionY = 1;
            break;
        }
        case COORD::E_NORTH: {
            directionX = 0; directionY = -1;
            break;
        }
        case COORD::E_WEST: {
            directionX = -1; directionY = 0;
            break;
        }
    }
    updateDistanceMap(directionX, directionY, x, y);
}

void ASTAR::updateDistanceMap(int x, int y, int currentX, int currentY)
{
    int offsets[16];
    int length;
//    std::cout << "For debug!!! x: " << x << ", y: " << y << "currentX: " << currentX << "currentY" << currentY << std::endl;
    if (x == 1) {
        int tmp_offsets [] = {1, 0, 2, 0, 0, 1, 0, 2, 0, -1, 0, -2, 1, 1, 1, -1};
        length = sizeof(tmp_offsets)/sizeof(tmp_offsets[0]);
        for (int i = 0; i < length; i++) {
            offsets[i] = tmp_offsets[i];
        }
    } else if (y == 1) {
        int tmp_offsets[] = {0, 1, 0, 2, 1, 0, 2, 0, -1, 0, -2, 0, 1, 1, -1, 1};
        length = sizeof(tmp_offsets) / sizeof(tmp_offsets[0]);
        for (int i = 0; i < length; i++) {
            offsets[i] = tmp_offsets[i];
        }
    } else if (x == -1) {
        int tmp_offsets [] = {-1, 0, -2, 0, 0, 1, 0, 2, 0, -1, 0, -2, -1, 1, -1, -1};
        length = sizeof(tmp_offsets)/sizeof(tmp_offsets[0]);
        for (int i = 0; i < length; i++) {
            offsets[i] = tmp_offsets[i];
        }
    } else if (y == -1) {
        int tmp_offsets [] = {0, -1, 0, -2, 1, 0, 2, 0, -1, 0, -2, 0, 1, -1, -1, -1};
        length = sizeof(tmp_offsets)/sizeof(tmp_offsets[0]);
        for (int i = 0; i < length; i++) {
            offsets[i] = tmp_offsets[i];
        }
    }

    for (int i = 0; i < length; i += 2) {
        int tr = currentX + offsets[i];
        int tc = currentY + offsets[i + 1];

        if (tr >= 0 && tr < sizeX && tc >= 0 && tc < sizeY && !blocked(tr, tc)) {
            // add the newly discovered vertex to the priority queue
            if ((abs(offsets[i]) + abs(offsets[i + 1])) >= 2) {
                updated_maps(tr, tc).failureProb = 0.8;
            } else {
                updated_maps(tr, tc).failureProb = 0.9;
            }
        }
    }

//    printProbMap(true);
//    printProbMap(false);
//    std::cin >> length;
}

void ASTAR::printProbMap(bool isMap)
{
    if (isMap)
        for (int x = 0; x < sizeX; x += 1) {
            for (int y = 0; y < sizeY; y += 1) {
                if (updated_maps(x,y).blocked) {
//                    char s = 'x';
                    printf("  x  ");
                } else {
                    double d = sumSuccessProb(x,y);
                    printf(" %4.1f", d);
                }
            }
            std::cout << endl;
        }
    else
        for (int x = 0; x < sizeX; x += 1) {
            for (int y = 0; y < sizeY; y += 1) {
                if (updated_maps(x,y).blocked) {
                    double d = -1;
                    printf("  x ");
                } else {
                    printf(" %3.1f", updated_maps(x,y).failureProb);
                }
            }
            std::cout << endl;
        }
}

void ASTAR::printObstacle()
{
    for (int idx = 0; idx <= (sizeY + 1); idx++)
        cout << "#";
    cout << endl;

    for (int x = 0; x < sizeX; x += 1) {
        cout << "#";
        for (int y = 0; y < sizeY; y += 1) {

            if ((x == task->startRow) && (y == task->startColumn)) {
                cout << "S";
            } else if ((x == task->endRow) && (y == task->endColumn)) {
                cout << "G";
            } else if (!updated_maps(x,y).blocked) cout << " ";
            else cout << "#";
        }
        cout << "#" << endl;
    }

    for (int idx = 0; idx <= (sizeY + 1); idx++)
        cout << "#";
    cout << endl;
}

bool ASTAR::printSolution()
{
    for (int idx = 0; idx <= (sizeY + 1); idx++)
        cout << "#";
    cout << endl;

    bool hit = hitCollision();
//    if (hit) {
//        cout << "get hitted: " << endl;
//        return false;
//    }

    for (int x = 0; x < sizeX; x += 1) {
        cout << "#";
        for (int y = 0; y < sizeY; y += 1) {

            if ((x == task->startRow) && (y == task->startColumn)) {
                cout << "S";
            } else if ((x == task->endRow) && (y == task->endColumn)) {
                cout << "G";
            } else if (!cells(x,y).blocked) {
                bool found = solutionFind(x,y);
                if (found)
                    cout << ".";
                else
                    cout << " ";
            } else
                cout << "#";
        }
        cout << "#" << endl;
    }

    for (int idx = 0; idx <= (sizeY + 1); idx++)
        cout << "#";
    cout << endl;

    printObstacle();

    if (hit) {
        cout << "get hitted: " << endl;
        return false;
    }

    return true;
}