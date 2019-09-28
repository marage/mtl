#ifndef MTL_NETWORK_P2P_GRAPH_HPP
#define MTL_NETWORK_P2P_GRAPH_HPP
#include <unordered_map>
#include <map>
#include <set>
#include <list>
#include <boost/asio/ip/udp.hpp>
#include "mtl/network/p2p/protocol.hpp"
#include "graph_relationships.hpp"

namespace mtl {
namespace network {
namespace p2p {

class GraphVertex {
public:
  inline GraphVertex(uint32_t id, uint8_t r,
                     const UdpEndpoint& ep,
                     const mtl::network::p2p::SequenceRange& sr)
      : id_(id), role_(r), address_(ep), seq_range_(sr)
      , alive_timeout_times_(0) {
  }

  inline uint32_t id() const;
  inline uint8_t role() const;
  inline const UdpEndpoint& address() const;
  inline const SequenceRange& seq_range() const;
  inline int alive_timeout_times() const;
  inline void set_alive_timeout_times(int times);
  inline int IncAliveTimeoutTimes();

private:
  uint32_t id_;
  uint8_t role_;
  UdpEndpoint address_;
  SequenceRange seq_range_;
  int alive_timeout_times_;
};

inline uint32_t GraphVertex::id() const {
  return id_;
}

inline uint8_t GraphVertex::role() const {
  return role_;
}

inline const UdpEndpoint& GraphVertex::address() const {
  return address_;
}

inline const SequenceRange& GraphVertex::seq_range() const {
  return seq_range_;
}

inline int GraphVertex::alive_timeout_times() const {
  return alive_timeout_times_;
}

inline void GraphVertex::set_alive_timeout_times(int times) {
  alive_timeout_times_ = times;
}

inline int GraphVertex::IncAliveTimeoutTimes() {
  return ++alive_timeout_times_;
}

class GraphEdge {
public:
  inline GraphEdge(uint32_t source, uint32_t target, int weight)
    : source_(source), target_(target), weight_(weight) {
  }

  inline uint32_t source() const;
  inline uint32_t target() const;
  inline int weight() const;
  inline void set_weight(int w);

private:
  uint32_t source_;
  uint32_t target_;
  int weight_;
};

inline uint32_t GraphEdge::source() const {
  return source_;
}

inline uint32_t GraphEdge::target() const {
  return target_;
}

inline int GraphEdge::weight() const {
  return weight_;
}

inline void GraphEdge::set_weight(int w) {
  weight_ = w;
}

class Graph {
public:
  Graph();
  ~Graph();

  inline const std::map<std::string, uint32_t>& vertex_names() const;
  inline const std::unordered_map<uint32_t, GraphVertex*>& vertices() const;
  inline bool ContainsVertex(const std::string& name) const;
  inline bool ContainsVertex(uint32_t id) const;
  GraphVertex* Vertex(const std::string& name) const;
  GraphVertex* Vertex(uint32_t id) const;
  GraphVertex* AddVertex(const std::string& name, uint8_t r,
                         const UdpEndpoint& ep,
                         const mtl::network::p2p::SequenceRange& sr);
  void RemoveVertex(const std::string& name);

  inline GraphEdge* Edge(const GraphVertex* s, const GraphVertex* t) const;
  void AddEdge(const GraphVertex* s, const GraphVertex* t, int weight);

  inline const GraphRelationships<Graph, GraphVertex, GraphEdge>& relationships() const;
  void BreadthFirstSearch(const GraphVertex* v,
                          std::list<UdpEndpoint>& addresses);
  void Clear();

private:
  void RemoveEdge(GraphEdge* e);

  std::map<std::string, uint32_t> name_id_map_;
  std::unordered_map<uint32_t, GraphVertex*> vertices_;
  std::set<GraphEdge*> edges_;
  GraphRelationships<Graph, GraphVertex, GraphEdge>* relationships_;
};

inline const std::map<std::string, uint32_t>& Graph::vertex_names() const {
  return name_id_map_;
}

inline const std::unordered_map<uint32_t, GraphVertex*>& Graph::vertices() const {
  return vertices_;
}

inline bool Graph::ContainsVertex(const std::string& name) const {
  auto it = name_id_map_.find(name);
  return (it != name_id_map_.end());
}

inline bool Graph::ContainsVertex(uint32_t id) const {
  auto it = vertices_.find(id);
  return (it != vertices_.end());
}

inline GraphEdge* Graph::Edge(const GraphVertex* s, const GraphVertex* t) const {
  return relationships_->Edge(s, t);
}

inline const GraphRelationships<Graph, GraphVertex, GraphEdge>& Graph::relationships() const {
  return *relationships_;
}

} // p2p
} // network
} // mtl

#endif //MTL_NETWORK_P2P_GRAPH_HPP
