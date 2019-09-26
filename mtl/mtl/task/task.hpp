#ifndef MTL_TASK_HPP
#define MTL_TASK_HPP
#include <memory>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "mtl/mtl.hpp"

namespace mtl {

class MTL_EXPORT Task
{
public:
    enum Status
    {
        INACTIVE,
        ACTIVE,
        COMPLETED,
        FAILED
    };

    explicit Task(int type, int timeout = 10000);
    virtual ~Task() {}

    inline int type() const { return type_; }
    inline bool isInactive()const { return status_ == INACTIVE; }
    inline bool isActive()const { return status_ == ACTIVE; }
    inline bool isCompleted()const { return status_ == COMPLETED; }
    inline bool isFailed()const { return status_ == FAILED; }
    inline Status status() const { return status_; }
    inline const boost::posix_time::ptime& deadTime() const
    {
        return dead_time_;
    }
    inline bool isTimeout(const boost::posix_time::ptime& now) const
    {
        return now >= dead_time_;
    }

    void activate();
    Status process();

protected:
    inline void setStatus(Status s) { status_ = s; }

private:
    virtual void activateImpl() {}
    virtual Status processImpl() = 0;

    int type_;
    Status status_;
    boost::posix_time::ptime dead_time_;
};

typedef std::shared_ptr<Task> task_ptr;

} // namespace mtl

#endif // MTL_TASK_HPP
