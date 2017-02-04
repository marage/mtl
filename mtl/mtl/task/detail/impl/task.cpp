#include "mtl/task/task.hpp"

namespace mtl {

Task::Task(int type, int timeout)
    : type_(type), status_(INACTIVE)
{
    if (timeout > 0) {
        dead_time_ = boost::posix_time::microsec_clock::local_time() +
                boost::posix_time::time_duration(boost::posix_time::milliseconds(timeout));
    }
}

void Task::activate()
{
    status_ = ACTIVE;
    activateImpl();
}

Task::Status Task::process()
{
    if (!dead_time_.is_not_a_date_time()) {
        if (dead_time_ <= boost::posix_time::microsec_clock::local_time()) {
            status_ = FAILED;
            return FAILED;
        }
    }
    return ((status_ == ACTIVE) ? processImpl() : status_);
}

}
