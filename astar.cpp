//
// Created by tuandam on 07.07.20.
//

#include "astar.h"

using namespace std;

typedef pair<double, Cell*> pi;

ASTAR::ASTAR(int sizeX, int sizeY) : cells(sizeX, sizeY), updated_maps(sizeX, sizeY), sizeX(sizeX), sizeY(sizeY), task(new Task(0, 0, sizeX - 1, sizeY - 1))  {

    // generate maze
    generateMaze();

    //=display the initial setup to the user
    cout << "A* will search the maze avoiding the orange cells. The goal is the least cost path from the top left "
                 "corner to the bottom right corner." << endl;
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

    for (int x = 0; x < sizeX; x += 1) {
        for (int y = 0; y < sizeY; y += 1) {
            updated_maps(x,y).row = x;
            updated_maps(x,y).column = y;
            updated_maps(x,y).blocked = false;
            updated_maps(x,y).parent = NULL;
            updated_maps(x,y).visited = false;

            if (x == task->startRow && y == task->startColumn) {
                cout << "S";
                cells(x,y).row = x;
                cells(x,y).column = y;
                cells(x,y).blocked = false;
                cells(x,y).parent = NULL;
                cells(x,y).visited = false;

                continue;
            } else if ((x == task->endRow) && (y == task->endColumn)) {
                cells(x,y).row = x;
                cells(x,y).column = y;
                cells(x,y).blocked = false;
                cells(x,y).parent = NULL;
                cells(x,y).visited = false;

                continue;
            }

            cells(x,y).row = x;
            cells(x,y).column = y;
            cells(x,y).blocked = false;
            cells(x,y).parent = NULL;
            cells(x,y).visited = false;
        }
    }

    createWalls();
}

// syntactic sugar to make getting the blocked state of a cell easier
bool ASTAR::blocked(int r, int c) {
    return updated_maps(r,c).blocked;
}

// likewise for visited
bool ASTAR::visited(int r, int c) {
    return updated_maps(r,c).visited;
}

// solve the maze and display progress and the resulting path
bool ASTAR::solve() {

    // create a heap / priority queue for tracking the currently open vertices that we are searching
    priority_queue<pi, vector<pi>, greater<pi>> pQueue;

    double d = distance(task->startRow, task->startColumn, task->endRow, task->endColumn);

    // init the queue with the starting location, mark this location as visited
    Cell *current = &updated_maps(task->startRow, task->startColumn);
    current->distance = d;
    current->blocked = false;
    current->parent = NULL;
    current->visited = true;

    // the distance to the target is the key for the priority queue since we want to examine
    // candidates according to how close they are to the target
    pQueue.push(make_pair(d, current));

    // seek a path to the target until the queue is empty
    while (!pQueue.empty()) {

        //=dequeue the visited node with the shortest distance to the target.
        pair<double, Cell*> top = pQueue.top();
        current = top.second;
        pQueue.pop();

//        cout << "Examining cell at " << current->row << "," << current->column << endl;
//        cout << "Dequeue" << endl;

        // if we have reached the target we are done
        if (current->row == task->endRow && current->column == task->endColumn) {
            // the least cost cost from target back to source is defined in the parent links
            // of each cell starting with target node
            solution.clear();
            while ((current->row != task->startRow) || (current->column != task->startColumn)) {
                solution.push_back(make_pair(current->row, current->column));
//                cout << "row: " << current->row << ", column: " << current->column << endl;
                current = current->parent;
            }

//            cout << "Bravo!@!!!!!" << endl;
//            cout << "row: " << current->row << ", column: " << current->column << endl;

            return true;
        }

        // we are not at the target so add all unvisited nodes that are adjacent to the current node to the priority queue

        // we only consider orthagonally connected unblocked, unvisited cells, there are there row / column offsets relative
        // to the current location

        // orthogonal neighbor offsets (row/column pairs) , 1,1,-1,-1,1,-1,-1,1
        int offsets [] = {1, 0, 0, -1, -1, 0, 0, 1};

        int length = sizeof(offsets)/sizeof(offsets[0]);

        for (int i = 0; i < length; i += 2) {
            int tr = current->row + offsets[i];
            int tc = current->column + offsets[i + 1];

            if (tr >= 0 && tr < sizeX && tc >= 0 && tc < sizeY && !blocked(tr, tc) && !visited(tr, tc)) {
                // add the newly discovered vertex to the priority queue
                double d = distance(tr, tc, task->endRow, task->endColumn);
                // mark cell as visited so we don't revisit
                Cell *temp = &updated_maps(tr,tc);
                temp->distance = d;
                temp->blocked = false;
                temp->parent = current;
                temp->visited = true;

                pQueue.push(make_pair(d,temp));
            }
        }
    }

    return false;
}

bool ASTAR::hitCollision()
{
    vector<int_pi>::iterator it;

    //reset the visits
    for (int iix = 0; iix < sizeX; iix++)
        for(int iiy = 0; iiy < sizeY; iiy++) {
            updated_maps(iix,iiy).visited = false;
        }

    for (int ix = 0; ix < sizeX; ix++)
        for(int iy = 0; iy < sizeY; iy++) {
            if (cells(ix,iy).blocked)
            {
                it = find (solution.begin(), solution.end(), make_pair(ix,iy));
                if (it != solution.end()) {
                    updated_maps(ix,iy).blocked = true;
                    std::cout << "Map gets updated!!!" << std::endl;
                    printUpdatedMap();

                    return true;
                }
            }
        }

    return false;
}

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

void ASTAR::printUpdatedMap()
{
    for (int idx = 0; idx <= (sizeY + 1); idx++)
        cout << "#";
    cout << endl;


    for (int x = 0; x < sizeX; x += 1) {
        cout << "#";
        for (int y = 0; y < sizeY; y += 1) {

            if (x == task->startRow && y == task->startColumn) {
                cout << "S";

                continue;
            } else if ((x == task->endRow) && (y == task->endColumn)) {
                cout << "G";

                continue;
            }

            if (!updated_maps(x,y).blocked) {
                bool found = solutionFind(x,y);
                if (found)
                    cout << ".";
                else
                    cout << " ";
            }
            else
                cout << "#";
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
    if (hit) {
        cout << "get hitted: " << endl;
        return false;
    }

    for (int x = 0; x < sizeX; x += 1) {
        cout << "#";
        for (int y = 0; y < sizeY; y += 1) {

            if (x == task->startRow && y == task->startColumn) {
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

    return true;
}