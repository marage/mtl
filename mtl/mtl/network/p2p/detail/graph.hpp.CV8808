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

class GraphVertex
{
public:
    GraphVertex(uint32_t id, uint8_t r, const boost::asio::ip::udp::endpoint& ep,
                const mtl::network::p2p::SequenceRange& sr)
        : id_(id), role_(r), address_(ep), seq_range_(sr), alive_timeout_times_(0)
    {
    }

    inline uint32_t id() const { return id_; }
    inline uint8_t role() const { return role_; }
    inline const boost::asio::ip::udp::endpoint& address() const { return address_; }
    inline const SequenceRange& seqRange() const { return seq_range_; }
    inline int aliveTimeoutTimes() const { return alive_timeout_times_; }
    inline void setAliveTimeoutTimes(int times)
    {
        alive_timeout_times_ = times;
    }
    inline int incAliveTimeoutTimes()
    {
        return ++alive_timeout_times_;
    }

private:
    uint32_t id_;
    uint8_t role_;
    boost::asio::ip::udp::endpoint address_;
    SequenceRange seq_range_;
    int alive_timeout_times_;
};

class GraphEdge
{
public:
    GraphEdge(uint32_t source, uint32_t target, int weight)
        : source_(source), target_(target), weight_(weight)
    {
    }

    inline uint32_t source() const { return source_; }
    inline uint32_t target() const { return target_; }
    inline int weight() const { return weight_; }
    inline void setWeight(int w) { weight_ = w; }

private:
    uint32_t source_;
    uint32_t target_;
    int weight_;
};

class Graph
{
public:
    Graph();
    ~Graph();

    inline const std::map<std::string, uint32_t>& vertexNames() const
    {
        return name_id_map_;
    }
    inline const std::unordered_map<uint32_t, GraphVertex*>& vertices() const
    {
        return vertices_;
    }
    inline bool containsVertex(const std::string& name) const
    {
        auto it = name_id_map_.find(name);
        return (it != name_id_map_.end());
    }
    inline bool containsVertex(uint32_t id) const
    {
        auto it = vertices_.find(id);
        return (it != vertices_.end());
    }
    GraphVertex* vertex(const std::string& name) const;
    GraphVertex* vertex(uint32_t id) const;
    GraphVertex* addVertex(const std::string& name, uint8_t r, const boost::asio::ip::udp::endpoint& ep,
                           const mtl::network::p2p::SequenceRange& sr);
    void removeVertex(const std::string& name);

    inline GraphEdge* edge(const GraphVertex* s, const GraphVertex* t) const
    {
        return relationships_->edge(s, t);
    }
    void addEdge(const GraphVertex* s, const GraphVertex* t, int weight);

    inline const GraphRelationships<Graph, GraphVertex, GraphEdge>& relationships() const
    {
        return *relationships_;
    }
    void breadthFirstSearch(const GraphVertex* v, std::list<boost::asio::ip::udp::endpoint>& addresses);
    void clear();

private:
    void removeEdge(GraphEdge* e);

    std::map<std::string, uint32_t> name_id_map_;
    std::unordered_map<uint32_t, GraphVertex*> vertices_;
    std::set<GraphEdge*> edges_;
    GraphRelationships<Graph, GraphVertex, GraphEdge>* relationships_;
};
} // p2p
} // network
} // mtl

#endif //MTL_NETWORK_P2P_GRAPH_HPP
