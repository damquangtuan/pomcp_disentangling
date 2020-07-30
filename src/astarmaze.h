//
// Created by tuandam on 02.07.20.
//
#ifndef ASTARMAZE_H
#define ASTARMAZE_H


//          Copyright W.P. McNeill 2010.
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)


// This program uses the A-star search algorithm in the Boost Graph Library to
// solve a maze.  It is an example of how to apply Boost Graph Library
// algorithms to implicit graphs.
//
// This program generates a random maze and then tries to find the shortest
// path from the lower left-hand corner to the upper right-hand corner.  Mazes
// are represented by two-dimensional grids where a cell in the grid may
// contain a barrier.  You may move up, down, right, or left to any adjacent
// cell that does not contain a barrier.
//
// Once a maze solution has been attempted, the maze is printed.  If a
// solution was found it will be shown in the maze printout and its length
// will be returned.  Note that not all mazes have solutions.
//
// The default maze size is 20x10, though different dimensions may be
// specified on the command line.


#include <boost/graph/astar_search.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <ctime>
#include <iostream>
#include <stack>

#include "coord.h"


//boost::mt19937 random_generator;

// Distance traveled in the maze
typedef double distance;

#define GRID_RANK 2
typedef boost::grid_graph<GRID_RANK> grid;
typedef boost::graph_traits<grid>::vertex_descriptor vertex_descriptor;
typedef boost::graph_traits<grid>::vertices_size_type vertices_size_type;

// A hash function for vertices.
struct vertex_hash:std::unary_function<vertex_descriptor, std::size_t> {
    std::size_t operator()(vertex_descriptor const& u) const {
        std::size_t seed = 0;
        boost::hash_combine(seed, u[0]);
        boost::hash_combine(seed, u[1]);
        return seed;
    }
};

typedef boost::unordered_set<vertex_descriptor, vertex_hash> vertex_set;
typedef boost::vertex_subset_complement_filter<grid, vertex_set>::type
        filtered_grid;

// A searchable maze
//
// The maze is grid of locations which can either be empty or contain a
// barrier.  You can move to an adjacent location in the grid by going up,
// down, left and right.  Moving onto a barrier is not allowed.  The maze can
// be solved by finding a path from the lower-left-hand corner to the
// upper-right-hand corner.  If no open path exists between these two
// locations, the maze is unsolvable.
//
// The maze is implemented as a filtered grid graph where locations are
// vertices.  Barrier vertices are filtered out of the graph.
//
// A-star search is used to find a path through the maze. Each edge has a
// weight of one, so the total path length is equal to the number of edges
// traversed.
class maze {
public:
    friend std::ostream& operator<<(std::ostream&, maze);

    maze():m_grid(create_grid(0, 0)),m_barrier_grid(create_barrier_grid()) {};
    maze(std::size_t x, std::size_t y):m_grid(create_grid(x, y)),
                                       m_barrier_grid(create_barrier_grid()) {};

    maze(std::size_t x, std::size_t y, std::size_t startX, std::size_t startY):m_grid(create_grid(x, y)),
                                       m_barrier_grid(create_barrier_grid()) {
        this->x = x;
        this->y = y;
        this->startX = startX;
        this->startY = startY;
    };

    // The length of the maze along the specified dimension.
    vertices_size_type length(std::size_t d) const {return m_grid.length(d);}

    bool has_barrier(vertex_descriptor u) const {
        return m_barriers.find(u) != m_barriers.end();
    }

    // Try to find a path from the lower-left-hand corner source (0,0) to the
    // upper-right-hand corner goal (x-1, y-1).
    vertex_descriptor source() const {
        int position = startX + startY * (x - 1);
        return vertex(position, m_grid);
    }

    vertex_descriptor goal() const {
        return vertex(num_vertices(m_grid)-1, m_grid);
    }

    std::stack<int> get_stack_solution();
    bool solve();
    bool solution_contains_barriers();
    bool solved() const {return !m_solution.empty();}
    bool solution_contains(vertex_descriptor u) const {
        return m_solution.find(u) != m_solution.end();
    }

public:
    // Create the underlying rank-2 grid with the specified dimensions.
    grid create_grid(std::size_t x, std::size_t y) {
        boost::array<std::size_t, GRID_RANK> lengths = { {x, y} };
        return grid(lengths);
    }

    // Filter the barrier vertices out of the underlying grid.
    filtered_grid create_barrier_grid() {
        return boost::make_vertex_subset_complement_filter(m_grid, m_barriers);
    }

    // Filter the barrier vertices out of the underlying grid with m_updated_barriers.
    filtered_grid create_updated_barrier_grid() {
        return boost::make_vertex_subset_complement_filter(m_grid, m_updated_barriers);
    }

    // The grid underlying the maze
    grid m_grid;
    // The underlying maze grid with barrier vertices filtered out
    filtered_grid m_barrier_grid;
    // The barriers in the maze
    vertex_set m_barriers;
    // The updated barriers in the maze
    vertex_set m_updated_barriers;
    // The vertices on a solution path through the maze
    vertex_set m_solution;
    // The length of the solution path
    distance m_solution_length;
    // Solution vector
    std::stack<int> solution_stack;
    int startX;
    int startY;
    int x, y;
};

// Euclidean heuristic for a grid
//
// This calculates the Euclidean distance between a vertex and a goal
// vertex.
class euclidean_heuristic:
        public boost::astar_heuristic<filtered_grid, double>
{
public:
    euclidean_heuristic(vertex_descriptor goal):m_goal(goal) {};

    double operator()(vertex_descriptor v) {
        return sqrt(pow(double(m_goal[0] - v[0]), 2) + pow(double(m_goal[1] - v[1]), 2));
    }

private:
    vertex_descriptor m_goal;
};

// Exception thrown when the goal vertex is found
struct found_goal {};

// Visitor that terminates when we find the goal vertex
struct astar_goal_visitor:public boost::default_astar_visitor {
    astar_goal_visitor(vertex_descriptor goal):m_goal(goal) {};

    void examine_vertex(vertex_descriptor u, const filtered_grid&) {
        if (u == m_goal)
            throw found_goal();
    }

private:
    vertex_descriptor m_goal;
};

#endif //ASTARMAZE_H
