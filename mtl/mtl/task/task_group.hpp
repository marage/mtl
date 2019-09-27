#ifndef MTL_TASK_GROUP_HPP
#define MTL_TASK_GROUP_HPP
#include "task.hpp"
#include <list>

namespace mtl {

class MTL_EXPORT TaskGroup : public Task
{
public:
    explicit TaskGroup(int type = 0, bool parallel = false, int timeout = 10000);
    virtual ~TaskGroup();

    int count() const { return static_cast<int>(tasks_.size()); }
    task_ptr find(int type) const;
    void add(const task_ptr& t);
    void remove(const task_ptr& t);
    void removeFirst();
    void removeAll();

    bool hasActiveSubTasks() const;

private:
    Status processImpl() final;

    typedef std::list<task_ptr> TaskList;
    TaskList tasks_;
    bool parallel_;
};

} // namespace mtl

#endif // MTL_TASK_GROUP_HPP

