#include "astar.h"

int main (int argc, char const *argv[]) {
	ASTAR *a = new ASTAR(25, 25);

	bool isSolved = a->solve();

	std::cout << isSolved << std::endl;
}
