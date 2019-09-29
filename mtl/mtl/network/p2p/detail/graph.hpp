#ifndef MTL_NETWORK_P2P_GRAPH_HPP
#define MTL_NETWORK_P2P_GRAPH_HPP
#include <unordered_map>
#include <map>
#include <set>
#include <list>
#include "mtl/network/p2p/protocol.hpp"
#include "graph_relationships.hpp"

namespace mtl {
namespace network {
namespace p2p {

class GraphVertex
{
public:
    inline GraphVertex(uint32_t id, uint8_t r,
                       const UdpEndpoint& ep,
                       const mtl::network::p2p::SequenceRange& sr)
        : id_(id), role_(r), address_(ep), seq_range_(sr)
        , alive_timeout_times_(0)
    {
    }

    uint32_t id() const;
    uint8_t role() const;
    const UdpEndpoint& address() const;
    const SequenceRange& seqRange() const;
    int aliveTimeoutTimes() const;
    void setAliveTimeoutTimes(int times);
    int incAliveTimeoutTimes();

private:
    uint32_t id_;
    uint8_t role_;
    UdpEndpoint address_;
    SequenceRange seq_range_;
    int alive_timeout_times_;
};

inline uint32_t GraphVertex::id() const
{
    return id_;
}

inline uint8_t GraphVertex::role() const
{
    return role_;
}

inline const UdpEndpoint& GraphVertex::address() const
{
    return address_;
}

inline const SequenceRange& GraphVertex::seqRange() const
{
    return seq_range_;
}

inline int GraphVertex::aliveTimeoutTimes() const
{
    return alive_timeout_times_;
}

inline void GraphVertex::setAliveTimeoutTimes(int times)
{
    alive_timeout_times_ = times;
}

inline int GraphVertex::incAliveTimeoutTimes()
{
    return ++alive_timeout_times_;
}

class GraphEdge
{
public:
    GraphEdge(uint32_t source, uint32_t target, int weight)
        : source_(source), target_(target), weight_(weight)
    {
    }

    uint32_t source() const;
    uint32_t target() const;
    int weight() const;
    void setWeight(int w);

private:
    uint32_t source_;
    uint32_t target_;
    int weight_;
};

inline uint32_t GraphEdge::source() const
{
    return source_;
}

inline uint32_t GraphEdge::target() const
{
    return target_;
}

inline int GraphEdge::weight() const
{
    return weight_;
}

inline void GraphEdge::setWeight(int w)
{
    weight_ = w;
}

class Graph
{
public:
    Graph();
    ~Graph();

    const std::map<std::string, uint32_t>& vertexNames() const;
    const std::unordered_map<uint32_t, GraphVertex*>& vertices() const;
    bool containsVertex(const std::string& name) const;
    bool containsVertex(uint32_t id) const;
    GraphVertex* vertex(const std::string& name) const;
    GraphVertex* vertex(uint32_t id) const;
    GraphVertex* addVertex(const std::string& name, uint8_t r,
                           const UdpEndpoint& ep,
                           const mtl::network::p2p::SequenceRange& sr);
    void removeVertex(const std::string& name);

    GraphEdge* edge(const GraphVertex* s, const GraphVertex* t) const;
    void addEdge(const GraphVertex* s, const GraphVertex* t, int weight);

    const GraphRelationships<Graph, GraphVertex, GraphEdge>& relationships() const;
    void breadthFirstSearch(const GraphVertex* v,
                            std::list<UdpEndpoint>& addresses);
    void clear();

private:
    void removeEdge(GraphEdge* e);

    std::map<std::string, uint32_t> name_id_map_;
    std::unordered_map<uint32_t, GraphVertex*> vertices_;
    std::set<GraphEdge*> edges_;
    GraphRelationships<Graph, GraphVertex, GraphEdge>* relationships_;
};

inline const std::map<std::string, uint32_t>& Graph::vertexNames() const
{
    return name_id_map_;
}

inline const std::unordered_map<uint32_t, GraphVertex*>& Graph::vertices() const
{
    return vertices_;
}

inline bool Graph::containsVertex(const std::string& name) const
{
    auto it = name_id_map_.find(name);
    return (it != name_id_map_.end());
}

inline bool Graph::containsVertex(uint32_t id) const
{
    auto it = vertices_.find(id);
    return (it != vertices_.end());
}

inline GraphEdge* Graph::edge(const GraphVertex* s, const GraphVertex* t) const
{
    return relationships_->edge(s, t);
}

inline const GraphRelationships<Graph, GraphVertex, GraphEdge>& Graph::relationships() const
{
    return *relationships_;
}

} // p2p
} // network
} // mtl

#endif //MTL_NETWORK_P2P_GRAPH_HPP
