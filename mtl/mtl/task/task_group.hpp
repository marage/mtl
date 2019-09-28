#ifndef MTL_TASK_GROUP_HPP
#define MTL_TASK_GROUP_HPP
#include "task.hpp"
#include <list>

namespace mtl {

class MTL_EXPORT TaskGroup : public Task {
public:
  explicit TaskGroup(int type = 0, bool parallel = false,
                     int timeout = 10000);
  ~TaskGroup() override;

  inline int count() const { return static_cast<int>(tasks_.size()); }
  TaskPtr Find(int type) const;
  void Append(const TaskPtr& t);
  void Remove(const TaskPtr& t);
  void RemoveFirst();
  void RemoveAll();

  bool HasActiveSubTasks() const;

private:
  State ProcessImpl() override;

  typedef std::list<TaskPtr> TaskList;
  TaskList tasks_;
  bool parallel_;
};

} // namespace mtl

#endif // MTL_TASK_GROUP_HPP

