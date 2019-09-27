#ifndef MTL_FRAMEWORK_CORE_LOGIC_UNIT_FACTORY_H
#define MTL_FRAMEWORK_CORE_LOGIC_UNIT_FACTORY_H
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include "logic_unit.h"

namespace mtl {
namespace framework {
namespace core {

template <typename T>
class LogicUnitFactory : private boost::noncopyable
{
public:
    typedef T Unit;
    typedef boost::shared_ptr<T> unit_ptr;
    typedef typename T::Terminal Terminal;

    unit_ptr createUnit(uint32_t id, Terminal t, int pt)
    {
        return unit_ptr(new T(id, t, pt));
    }
};

typedef LogicUnitFactory<ClientLogicUnit> ClientLogicUnitFactory;

} // core
} // framework
} // mtl

#endif // MTL_FRAMEWORK_CORE_LOGIC_UNIT_FACTORY_H
