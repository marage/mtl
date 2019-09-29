#ifndef MTL_TASK_GROUP_HPP
#define MTL_TASK_GROUP_HPP
#include "task.hpp"
#include <list>

namespace mtl {

class MTL_EXPORT TaskGroup : public Task
{
public:
    explicit TaskGroup(int type = 0, bool parallel = false,
                       int timeout = 10000);
    ~TaskGroup() override;

    int count() const { return static_cast<int>(tasks_.size()); }
    TaskPtr find(int type) const;
    void append(const TaskPtr& t);
    void remove(const TaskPtr& t);
    void removeFirst();
    void removeAll();

    bool hasActiveSubTasks() const;

private:
    State processImpl() final;

    typedef std::list<TaskPtr> TaskList;
    TaskList tasks_;
    bool parallel_;
};

} // namespace mtl

#endif // MTL_TASK_GROUP_HPP

