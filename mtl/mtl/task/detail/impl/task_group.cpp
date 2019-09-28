#include "mtl/task/task_group.hpp"
#include <algorithm>
#include <chrono>

namespace mtl {

TaskGroup::TaskGroup(int type, bool parallel, int timeout)
  : Task(type, timeout), parallel_(parallel) {
}

TaskGroup::~TaskGroup() {
  RemoveAll();
}

TaskPtr TaskGroup::Find(int type) const {
  TaskPtr t;
  for (auto it = tasks_.begin(), end = tasks_.end();
       it != end; ++it) {
    if ((*it)->type() == type) {
      t = (*it);
      break;
    }
  }
  return t;
}

void TaskGroup::Append(const TaskPtr& t) {
  tasks_.push_back(t);
}

void TaskGroup::Remove(const TaskPtr& t) {
  auto it = std::find(tasks_.begin(), tasks_.end(), t);
  if (it != tasks_.end()) {
    tasks_.erase(it);
  }
}

void TaskGroup::RemoveFirst() {
  auto it = tasks_.begin();
  if (it != tasks_.end()) {
    tasks_.erase(it);
  }
}

void TaskGroup::RemoveAll() {
  tasks_.clear();
}

bool TaskGroup::HasActiveSubTasks() const {
  bool active = false;
  for (auto it = tasks_.begin(), end = tasks_.end();
       it != end; ++it) {
    if ((*it)->IsActive()) {
      active = true;
      break;
    }
  }
  return active;
}

Task::State TaskGroup::ProcessImpl() {
  auto now = std::chrono::system_clock::now();
  //remove all completed and failed tasks from the front of the task list
  for (auto it = tasks_.begin(), end = tasks_.end();
       it != end;) {
    if ((*it)->IsCompleted() || (*it)->IsFailed() || (*it)->IsTimeout(now)) {
      it = tasks_.erase(it);
    } else {
      ++it;
    }
  }

  //if any subgoals remain, process the one at the front of the list
  if (!tasks_.empty()) {
    if (parallel_) {
      //process all sub tasks
      for (TaskList::iterator it = tasks_.begin(), end = tasks_.end();
           it != end; ++it) {
        if ((*it)->IsInactive()) {
          (*it)->Activate();
        } else {
          (*it)->Process();
        }
      }
    } else {
      //process the top goal
      if ((*tasks_.begin())->IsInactive()) {
        (*tasks_.begin())->Activate();
      } else {
        (*tasks_.begin())->Process();
      }
    }

    //we have to test for the special case where the front-most task
    //reports 'completed' *and* the task list contains additional tasks.When
    //this is the case, to ensure the parent keeps processing its task list
    //we must return the 'active' status.
    if (tasks_.size() > 1) {
      return kActive;
    }
  }

  //no more subgoals to process - return 'completed'
  set_state(kCompleted);
  return kCompleted;
}

}
