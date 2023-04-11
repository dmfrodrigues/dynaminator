#pragma once

#include <functional>

namespace Opt {
class UnivariateSolver {
   public:
    typedef double Var;

    typedef std::function<Var(Var)> Problem;

    virtual void setStopCriteria(Var e) = 0;
    virtual Var  solve(Problem p)       = 0;

    virtual ~UnivariateSolver() {}
};
}  // namespace Opt
