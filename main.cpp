#include <iostream>
#include <random>
#include <type_traits>
#include <vector>
#include <unordered_map>
#include <boost/dynamic_bitset.hpp>
#include <algorithm> 
using ul = unsigned long;
namespace Utils {
  template<typename T>
  T generateRandom(T min, T max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());

    if constexpr (std::is_integral<T>::value) {
        std::uniform_int_distribution<T> dist(min, max);
        return dist(gen);
    } else if constexpr (std::is_floating_point<T>::value) {
        std::uniform_real_distribution<T> dist(min, max);
        return dist(gen);
    } else {
        static_assert(std::is_arithmetic<T>::value, "Type must be numeric");
    }
  };
  template<typename T>
  std::vector<T> generateRandomVector(size_t size, T min, T max) {
    std::vector<T> result;
    result.reserve(size);
    for (size_t i = 0; i < size; ++i) {
        result.push_back(generateRandom(min, max));
    }
    return result;
  };
};
class Edge;
class Node;
class GraphManager;
class Node {
  friend class GraphManager;
  friend class Edge;
  Node(const Node&) = delete;
  Node(const Node*&) = delete;
  Node() = delete;
  Node(ul idx): _idx(idx) {
  };
  ~Node();
  ul _idx = 0;
  std::vector<Edge*> _outEdges = {};
  std::vector<Edge*> _inEdges = {};
};
class Edge {
  friend class GraphManager;
  friend class Node;
  Edge(const Edge&) = delete;
  Edge(const Edge*&) = delete;
  Edge() = delete;
  Edge(Node* src, Node* trg);
  ~Edge() ;
  Node* _source = nullptr;
  Node* _target = nullptr;
};
class GraphManager {
  public:
    GraphManager() = default;
    const Node* addNodeAtLevel(size_t lvl);
    Edge* connectNodes(ul node1, ul node2);
    std::unordered_map<ul, Node*> aggressiveSearch(std::vector<ul> nodesIds) const;
    void dump(std::ostream& os) const;
    void setDepth(size_t d);
    std::vector<ul> getNodesAtLevel(size_t lvl) const;
    std::vector<ul> getNextNodes(std::vector<ul> nodesIds) const;
  private:
    ul _numberOfNodes = 0;
    std::vector<std::vector<Node*>> _nodesPerLevel = {{}};
};

GraphManager* GraphGenerator(std::vector<size_t> sizePerLvl) {
  GraphManager* graphManager = new GraphManager();
  graphManager->setDepth(sizePerLvl.size());
  for (size_t i = 0; i <sizePerLvl.size(); i++) {
    while (sizePerLvl[i]--) {
      graphManager->addNodeAtLevel(i);
    }
  }
  for (size_t i = 1; i <sizePerLvl.size(); i++) {
    std::vector<ul> indexs = graphManager->getNodesAtLevel(i);
    std::vector<ul> previousIndexs = graphManager->getNodesAtLevel(i-1);
    boost::dynamic_bitset<> connected(indexs.size(), false);
    size_t saftyBreak = indexs.size()*10;
    while(!connected.all() && saftyBreak--) {
      size_t rdId = Utils::generateRandom<size_t>(0, indexs.size()-1);
      if (connected.test(rdId)) continue;
      size_t previousId = Utils::generateRandom<size_t>(0, previousIndexs.size()-1);
      graphManager->connectNodes(previousIndexs[previousId], indexs[rdId]);
    }
  }

  return graphManager;
};

int main() {
  GraphGenerator({10,2,3,4})->dump(std::cout);
  return 0;
};


/// Node & Edge
Node::~Node() {
  for (auto outEdge : _outEdges) {
    delete outEdge;
  }
};
Edge::Edge(Node* src, Node* trg): _source(src), _target(trg) {
  _source->_outEdges.push_back(this);
  _target->_inEdges.push_back(this);
};
Edge::~Edge() {
  delete _target;
};
/// GraphManager 
const Node* GraphManager::addNodeAtLevel(size_t lvl) {
  assert(_nodesPerLevel.size() > lvl);
  Node* newNode = new Node(_numberOfNodes++);
  _nodesPerLevel[lvl].push_back(newNode);
  return newNode;
};
Edge* GraphManager::connectNodes(ul node1, ul node2) {
  std::unordered_map<ul, Node*> nodes = aggressiveSearch({node1, node2});
  if (nodes.size() != 2 || !nodes[node1] || !nodes[node2]) {
    return nullptr;
  }
  for (const auto& inEdge : nodes[node2]->_inEdges) {
    if (inEdge->_source == nodes[node1]) return inEdge;
  }
  return new Edge(nodes[node1], nodes[node2]);
};
std::unordered_map<ul, Node*> GraphManager::aggressiveSearch(std::vector<ul> nodesIds) const {
  std::sort(nodesIds.begin(), nodesIds.end());
  std::unordered_map<ul, Node*> rslt;
  for (const auto& lvl : _nodesPerLevel) {
    for (const auto& node : lvl) {
      auto it = std::lower_bound(nodesIds.begin(), nodesIds.end(), node, 
                            [](const ul nodeId, const Node* node) {
                            return nodeId < node->_idx;});
      if (it != nodesIds.end()) {
        rslt[*it] = node;
      }
    }
  }
  // for (auto [ke, val]: rslt) std::cout<<ke<<" "<<val->_idx<<std::endl;
  return rslt;
};
void GraphManager::dump(std::ostream& os) const {
  for (size_t i = 0; i < _nodesPerLevel.size(); i++) {
    os << "-- Nodes at level: "<<std::to_string(i)<<" (nbr of nodes: "<<_nodesPerLevel[i].size()<<")"<<std::endl;
    for (size_t j = 0; j < _nodesPerLevel[i].size(); j++) {
      os << "  |_ "<< "\"" << std::to_string(_nodesPerLevel[i][j]->_idx)<<"\" ";
      os << " TO [";
      for (const auto& e : getNextNodes({_nodesPerLevel[i][j]->_idx})) {
        os << "\"" << std::to_string(e)<<"\" ";
      }
      os <<"]";
      os <<std::endl;
    }
    os <<std::endl;
  }
};
void GraphManager::setDepth(size_t d) {
  _nodesPerLevel.resize(d, {});
};
std::vector<ul> GraphManager::getNodesAtLevel(size_t lvl) const {
  assert(_nodesPerLevel.size() > lvl);
  std::vector<ul> rslt(_nodesPerLevel[lvl].size());
  for (size_t i = 0; i <_nodesPerLevel[lvl].size(); i++) rslt[i] = _nodesPerLevel[lvl][i]->_idx;
  return rslt;
};
std::vector<ul> GraphManager::getNextNodes(std::vector<ul> nodesIds) const {
  std::vector<ul> nextNodes;
  std::unordered_map<ul, Node*> nodes = aggressiveSearch(nodesIds);
  for(const auto& [id, node]: nodes) {
    for (const auto& edge : node->_outEdges) {
      nextNodes.push_back(edge->_target->_idx);
    }
  }

  return nextNodes;
}