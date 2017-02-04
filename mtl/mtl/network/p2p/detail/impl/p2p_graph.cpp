#include "mtl/network/p2p/detail/graph.hpp"
#include <assert.h>

namespace mtl {
namespace network {
namespace p2p {

Graph::Graph()
{
    relationships_ = new GraphRelationships<Graph, GraphVertex, GraphEdge>(this);
}

Graph::~Graph()
{
    clear();
    delete relationships_;
}

GraphVertex* Graph::vertex(const std::string& name) const
{
    auto it = name_id_map_.find(name);
    if (it != name_id_map_.end()) {
        auto v_it = vertices_.find(it->second);
        if (v_it != vertices_.end()) {
            return v_it->second;
        }
    }
    return nullptr;
}

GraphVertex* Graph::vertex(uint32_t id) const
{
    auto it = vertices_.find(id);
    return (it != vertices_.end() ? it->second : 0);
}

GraphVertex* Graph::addVertex(const std::string& name, uint8_t r,
                              const boost::asio::ip::udp::endpoint& ep, const SequenceRange& sr)
{
    auto it = name_id_map_.find(name);
    if (it != name_id_map_.end())
        return 0;

    // unique next id
    static uint32_t next_id = 0;
    do {
        ++next_id;
        next_id = next_id % 0xffffffff;
    } while (containsVertex(next_id));

    // name id map
    name_id_map_.insert(std::make_pair(name, next_id));

    GraphVertex* v = new GraphVertex(next_id, r, ep, sr);
    vertices_.insert(std::make_pair(next_id, v));
    return v;
}

void Graph::removeVertex(const std::string& name)
{
    auto it = name_id_map_.find(name);
    if (it != name_id_map_.end()) {
        auto v_it = vertices_.find(it->second);
        if (v_it != vertices_.end()) {
            std::set<GraphEdge*> edges = relationships_->edges(v_it->second);
            for (auto e_it = edges.begin(), end = edges.end(); e_it != end; ++e_it) {
                removeEdge(*e_it);
            }
            relationships_->removeVertex(v_it->second);
            delete v_it->second;
            vertices_.erase(v_it);
        }
        name_id_map_.erase(it);
    }
}

void Graph::addEdge(const GraphVertex* s, const GraphVertex* t, int weight)
{
    assert(s && t);
    GraphEdge* e = new GraphEdge(s->id(), t->id(), weight);
    edges_.insert(e);
    relationships_->addEdge(e);
}

void Graph::breadthFirstSearch(const GraphVertex* v, std::list<boost::asio::ip::udp::endpoint>& addresses)
{
    assert(v);
    addresses.clear();
    std::set<const GraphVertex*> unvisit;
    for (auto it = vertices_.begin(), end = vertices_.end(); it != end; ++it) {
        if (it->second != v) {
            unvisit.insert(it->second);
        }
    }
    while (true) {
        relationships_->breadthFirstSearch(v, unvisit, addresses);
        if (unvisit.empty()) {
            break;
        } else {
            v = *unvisit.begin();
            unvisit.erase(unvisit.begin());
        }
    }
}

void Graph::clear()
{
    relationships_->clear();

    for (auto it = vertices_.begin(), end = vertices_.end(); it != end; ++it)
        delete it->second;
    vertices_.clear();

    for (auto it = edges_.begin(), end = edges_.end(); it != end; ++it)
        delete *it;
    edges_.clear();
}

void Graph::removeEdge(GraphEdge* e)
{
    auto it = edges_.find(e);
    if (it != edges_.end()) {
        relationships_->removeEdge(e);
        edges_.erase(it);
        delete e;
    }
}

} // p2p
} // network
} // mtl
