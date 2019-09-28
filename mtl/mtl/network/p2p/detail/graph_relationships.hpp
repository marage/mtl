#ifndef MTL_NETWORK_P2P_GRAPH_RELATIONSHIPS_HPP
#define MTL_NETWORK_P2P_GRAPH_RELATIONSHIPS_HPP
#include <unordered_map>
#include <set>
#include <queue>
#include <boost/asio/ip/udp.hpp>

namespace mtl {
namespace network {
namespace p2p {

template <typename G, typename V, typename E>
class GraphRelationships {
  typedef std::set<E*> EdgeSet;

public:
  inline explicit GraphRelationships(G* g) : graph_(g) {
  }

  inline bool HasEdges(const V* v) const;
  void AddEdge(E* e);
  void RemoveEdge(E* e);

  E* Edge(const V* s, const V* t) const {
    typename std::unordered_map<const V*, EdgeSet>::const_iterator it = vertex_edges_map_.find(s);
    if (it != vertex_edges_map_.end()) {
      typename EdgeSet::const_iterator e_it = it->second.begin();
      typename EdgeSet::const_iterator e_end = it->second.end();
      for (; e_it != e_end; ++e_it) {
        if ((*e_it)->source() == t->id() || (*e_it)->target() == t->id()) {
          return *e_it;
        }
      }
    }
    return nullptr;
  }

  inline EdgeSet Edges(const V* v) const {
    typename std::unordered_map<const V*, EdgeSet>::const_iterator it = vertex_edges_map_.find(v);
    return (it != vertex_edges_map_.end() ? it->second: EdgeSet());
  }

  inline void RemoveVertex(const V* v) {
    typename std::unordered_map<const V*, EdgeSet>::iterator it = vertex_edges_map_.find(v);
    if (it != vertex_edges_map_.end()) {
      vertex_edges_map_.erase(it);
    }
  }

  void BreadthFirstSearch(const V* v, std::set<const V*>& unvisit,
                          std::list<AsioUDPEndpoint>& addresses) const;

  inline void Clear();

private:
  G* graph_;
  std::unordered_map<const V*, EdgeSet> vertex_edges_map_;
};

template <typename G, typename V, typename E>
inline bool GraphRelationships<G, V, E>::HasEdges(const V* v) const {
  return (vertex_edges_map_.find(v) != vertex_edges_map_.end());
}

template<typename G, typename V, typename E>
void GraphRelationships<G, V, E>::AddEdge(E* e) {
  V* s = graph_->Vertex(e->source());
  V* t = graph_->Vertex(e->target());
  if (!s || !t)
    return;

  // A
  typename std::unordered_map<const V*, EdgeSet>::iterator it = vertex_edges_map_.find(s);
  if (it != vertex_edges_map_.end()) {
    typename EdgeSet::const_iterator e_it = it->second.find(e);
    if (e_it == it->second.end())
      it->second.insert(e);
  } else {
    EdgeSet ss;
    ss.insert(e);
    vertex_edges_map_.insert(std::make_pair(s, ss));
  }

  // B
  it = vertex_edges_map_.find(t);
  if (it != vertex_edges_map_.end()) {
    typename EdgeSet::const_iterator e_it = it->second.find(e);
    if (e_it == it->second.end())
      it->second.insert(e);
  } else {
    EdgeSet ss;
    ss.insert(e);
    vertex_edges_map_.insert(std::make_pair(t, ss));
  }
}

template <typename G, typename V, typename E>
void GraphRelationships<G, V, E>::RemoveEdge(E* e) {
  V* s = graph_->Vertex(e->source());
  V* t = graph_->Vertex(e->target());
  if (!s || !t)
    return;

  // source edge list
  typename std::unordered_map<const V*, EdgeSet>::iterator it = vertex_edges_map_.find(s);
  if (it != vertex_edges_map_.end()) {
    typename EdgeSet::const_iterator e_it = it->second.find(e);
    if (e_it != it->second.end())
      it->second.erase(e_it);
    if (it->second.empty())
      vertex_edges_map_.erase(it);
  }

  // target edge list
  it = vertex_edges_map_.find(t);
  if (it != vertex_edges_map_.end()) {
    typename EdgeSet::const_iterator e_it = it->second.find(e);
    if (e_it != it->second.end())
      it->second.erase(e_it);
    if (it->second.empty())
      vertex_edges_map_.erase(it);
  }
}

template <typename G, typename V, typename E>
void GraphRelationships<G, V, E>::BreadthFirstSearch(const V* v, std::set<const V*>& unvisit,
                                                     std::list<AsioUDPEndpoint>& addresses) const {
  typename std::set<const V*>::iterator l_it;
  typename std::unordered_map<const V*, EdgeSet>::const_iterator v_it;

  std::set<const V*> visited;

  std::queue<const V*> visit_queue;
  visit_queue.push(v);

  while (!visit_queue.empty()) {
    const V* p = visit_queue.front();
    visit_queue.pop();

    if (visited.find(p) != visited.end())
      continue;

    l_it = unvisit.find(p);
    if (l_it != unvisit.end()) {
      unvisit.erase(l_it);
    }
    addresses.push_front(p->address());
    visited.insert(p);

    v_it = vertex_edges_map_.find(p);
    if (v_it != vertex_edges_map_.end()) {
      typename EdgeSet::const_iterator e_it = v_it->second.begin();
      typename EdgeSet::const_iterator e_end = v_it->second.end();
      for (; e_it != e_end; ++e_it) {
        const V* s = graph_->Vertex((*e_it)->source());
        const V* t = graph_->Vertex((*e_it)->target());
        if (s == p) {
          visit_queue.push(t);
        } else {
          visit_queue.push(s);
        }
      }
    }
  }
}

template<typename G, typename V, typename E>
inline void GraphRelationships<G, V, E>::Clear() {
  vertex_edges_map_.clear();
}

} // p2p
} // network
} // mtl

#endif // MTL_NETWORK_P2P_GRAPH_RELATIONSHIPS_HPP
