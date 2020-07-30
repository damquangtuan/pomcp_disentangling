//
// Created by tuandam on 07.07.20.
//

#ifndef ASTAR_H
#define ASTAR_H

#include "coord.h"
#include "grid.h"

typedef std::pair<int, int> int_pi;

struct CellSolution {
    int x;
    int y;
    int directionX;
    int directionY;
};

struct Task {
    int startRow;
    int startColumn;
    int endRow;
    int endColumn;

    Task (int sr, int sc, int er, int ec)
    : startRow(sr), startColumn(sc), endRow(er), endColumn(ec) {}
};

class Cell {
public:
    Cell() {}
    Cell(int r, int c, double h_function, bool b, Cell* p, bool v):row(r),column(c),h_function(h_function),blocked(b),visited(v) {
        parent = p;
        failureProb = 0.1;
        g_function = 0;
    }
    int row;
    int column;
    double failureProb;
    double h_function;
    double g_function;
    bool blocked = true;
    Cell *parent = 0;
    bool visited = false;
};

class ASTAR {
private:
    int sizeX;
    int sizeY;
    GRID<Cell> cells, updated_maps;
    std::vector<int_pi> solution;
    Task *task;

public:
    ASTAR(int sizeX, int sizeY);

    void setStartPosition(int x, int y);
    void setEndPosition(int x, int y);

    void createWalls();

    std::vector<int> getSolution();

    double distance(int r1, int c1, int r2, int c2);

    void generateMaze();

    // syntactic sugar to make getting the blocked state of a cell easier
    bool blocked(int r, int c);

    // likewise for visited
    bool visited(int r, int c);

    double sumSuccessProb(int x_current, int y_current);

    bool solve();

    std::vector<int_pi> solveReturn();

    bool hitCollision();

    bool solutionFind(int x, int y);

    void updateCollisionPoint(int x, int y, int action);

    void updateCollisionMap(GRID<int> map);

    void updateDistanceMap(int x, int y, int currentX, int currentY);

    void printProbMap(bool isMap);

    void printObstacle();

    bool printSolution();
};


#endif //ASTAR_H
