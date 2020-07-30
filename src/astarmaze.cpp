#include "astarmaze.h"

std::stack<int> maze::get_stack_solution(){
    return solution_stack;
}

// Solve the maze using A-star search.  Return true if a solution was found.
bool maze::solve() {
  boost::static_property_map<distance> weight(1);
  // The predecessor map is a vertex-to-vertex mapping.
  typedef boost::unordered_map<vertex_descriptor,
                               vertex_descriptor,
                               vertex_hash> pred_map;
  pred_map predecessor;
  boost::associative_property_map<pred_map> pred_pmap(predecessor);
  // The distance map is a vertex-to-distance mapping.
  typedef boost::unordered_map<vertex_descriptor,
                               distance,
                               vertex_hash> dist_map;
  dist_map distance;
  boost::associative_property_map<dist_map> dist_pmap(distance);

  vertex_descriptor s = source();
  vertex_descriptor g = goal();
  euclidean_heuristic heuristic(g);
  astar_goal_visitor visitor(g);
  m_solution.clear();
  try {
    astar_search(m_barrier_grid, s, heuristic,
                 boost::weight_map(weight).
                 predecessor_map(pred_pmap).
                 distance_map(dist_pmap).
                 visitor(visitor) );
  } catch(found_goal fg) {
    // Walk backwards from the goal through the predecessor chain adding
    // vertices to the solution path.
    vertex_descriptor prev = g;
    int x, y;
    for (vertex_descriptor u = g; u != s; u = predecessor[u]) {
        x = int(prev[0]-u[0]);
        y = int(prev[1]-u[1]);
        if ((x == 0) && (y == 1)) solution_stack.push(int(COORD::E_NORTH));
        else if ((x == 0) && (y == -1)) solution_stack.push(int(COORD::E_SOUTH));
        else if ((x == 1) && (y == 0)) solution_stack.push(int(COORD::E_EAST));
        else if ((x == -1) && (y == 0)) solution_stack.push(int(COORD::E_WEST));
//        else if ((x == 1) && (y == 1)) solution_stack.push(int(COORD::E_NORTHEAST));
//        else if ((x == -1) && (y == -1)) solution_stack.push(int(COORD::E_SOUTHWEST));
//        else if ((x == 1) && (y == -1)) solution_stack.push(int(COORD::E_SOUTHEAST));
//        else if ((x == -1) && (y == 1)) solution_stack.push(int(COORD::E_NORTHWEST));
        prev = u;
        m_solution.insert(u);
    }

    x = int(prev[0]-s[0]);
    y = int(prev[1]-s[1]);
    if ((x == 0) && (y == 1)) solution_stack.push(int(COORD::E_NORTH));
    else if ((x == 0) && (y == -1)) solution_stack.push(int(COORD::E_SOUTH));
    else if ((x == 1) && (y == 0)) solution_stack.push(int(COORD::E_EAST));
    else if ((x == -1) && (y == 0)) solution_stack.push(int(COORD::E_WEST));
//    else if ((x == 1) && (y == 1)) solution_stack.push(int(COORD::E_NORTHEAST));
//    else if ((x == -1) && (y == -1)) solution_stack.push(int(COORD::E_SOUTHWEST));
//    else if ((x == 1) && (y == -1)) solution_stack.push(int(COORD::E_SOUTHEAST));
//    else if ((x == -1) && (y == 1)) solution_stack.push(int(COORD::E_NORTHWEST));

    m_solution.insert(s);
    m_solution_length = distance[g];

    return true;

//    bool isCollied = false;
//    boost::unordered_set<vertex_descriptor, vertex_hash>::iterator itr;
//    for (itr = m_barriers.begin(); itr != m_barriers.end(); itr++) {
//        if (solution_contains(*itr)) {
//            //std::cout << "x dimension: " << (*itr)[0] << std::endl;
//            //std::cout << "y dimension: " << (*itr)[1] << std::endl;
//            //vertex_descriptor barrier = vertex((*itr)[0], (*itr)[1]);
//            m_updated_barriers.insert(*itr);
//            isCollied = true;
//        }
//    }
//
//    if (isCollied)
//        return false;
//    else
//        return true;
  }

  return false;
}

bool maze::solution_contains_barriers() {
    boost::unordered_set<vertex_descriptor, vertex_hash>::iterator itr;
    for (itr = m_barriers.begin(); itr != m_barriers.end(); itr++) {
        if (solution_contains(*itr)) {
            //std::cout << "x dimension: " << (*itr)[0] << std::endl;
            //std::cout << "y dimension: " << (*itr)[1] << std::endl;
            //vertex_descriptor barrier = vertex((*itr)[0], (*itr)[1]);
            m_updated_barriers.insert(*itr);
            return true;
        }
    }

    return false;
}

#define BARRIER "#"
// Print the maze as an ASCII map.
std::ostream& operator<<(std::ostream& output, maze m) {
  // Header
  for (vertices_size_type i = 0; i < m.length(0)+2; i++)
    output << BARRIER;
  output << std::endl;
  // Body
  for (int y = m.length(1)-1; y >= 0; y--) {
    // Enumerate rows in reverse order and columns in regular order so that
    // (0,0) appears in the lower left-hand corner.  This requires that y be
    // int and not the unsigned vertices_size_type because the loop exit
    // condition is y==-1.
    for (vertices_size_type x = 0; x < m.length(0); x++) {
      // Put a barrier on the left-hand side.
      if (x == 0)
        output << BARRIER;
      // Put the character representing this point in the maze grid.
      vertex_descriptor u = {{x, vertices_size_type(y)}};
      if (m.solution_contains(u))
        output << ".";
      else if (m.has_barrier(u))
        output << BARRIER;
      else
        output << " ";
      // Put a barrier on the right-hand side.
      if (x == m.length(0)-1)
        output << BARRIER;
    }
    // Put a newline after every row except the last one.
    output << std::endl;
  }
  // Footer
  for (vertices_size_type i = 0; i < m.length(0)+2; i++)
    output << BARRIER;
  if (m.solved())
    output << std::endl << "Solution length " << m.m_solution_length;
  
//  bool contained_barrier = m.solution_contains_barriers();
//  if (contained_barrier)
//    output << std::endl << " Get Collision!!!";
//  else
//    output << std::endl << " Don't Get Collision!!!";

  return output;
}