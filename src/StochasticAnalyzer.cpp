#include "StochasticAnalyzer.h"
#include "Grid.h" // For GridCell type
#include <algorithm>
#include <chrono>

// Explicit template instantiation for GridCell
template class StochasticAnalyzer<GridCell>;

// Static member definition for the instantiated template
template<typename CellType>
size_t StochasticAnalyzer<CellType>::s_randomCounter = 0;