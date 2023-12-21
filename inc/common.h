#pragma once
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/unordered_map.hpp>
#include <boost/unordered_set.hpp>
#include <cfloat>
#include <ctime>
#include <fstream>
#include <iostream>
#include <list>
#include <set>
#include <tuple>
#include <utility>
#include <vector>
#include <unordered_set>

using boost::unordered_map;
using boost::unordered_set;
using boost::heap::compare;
using boost::heap::fibonacci_heap;

using std::cout;
using std::deque;
using std::endl;
using std::list;
using std::make_pair;
using std::make_tuple;
using std::max;
using std::min;
using std::ostream;
using std::pair;
using std::set;
using std::string;
using std::tuple;
using std::vector;

// #include <boost/graph/adjacency_list.hpp>
// typedef boost::adjacency_list_traits<int, int, boost::undirectedS > confilctGraph_t;
// typedef confilctGraph_t::vertex_descriptor vertex_t;
// typedef confilctGraph_t::edge_descriptor edge_t;

enum heuristics_type { NONE, CG, DG, WDG, STRATEGY_COUNT };

typedef tuple<int, int, int, int, bool> Constraint;
// i,j,u,v,t vehicle i,j have conflict on edge u,v at timestep t
typedef tuple<int, int, int, int, int> Conflict;
// typedef vector<unordered_set<std::pair<int,int> > > ConstraintTable;
typedef tuple<int, int, bool> Interval; // [t_min, t_max), have conflicts or not

#define INTERVAL_MAX 10000

ostream &operator<<(ostream &os, const Constraint &constraint);

ostream &operator<<(ostream &os, const Conflict &conflict);

ostream &operator<<(ostream &os, const Interval &interval);
