//
// Created by adria on 27/6/20.
//

#ifndef DoublePENDULUM_COSTMODELDoublePENDULUM_H
#define DoublePENDULUM_COSTMODELDoublePENDULUM_H
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/multibody/model.hpp"

#include "example-robot-data/path.hpp"

#include "crocoddyl/core/fwd.hpp"
#include "crocoddyl/core/integrator/euler.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/solvers/ddp.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"
#include "crocoddyl/core/activations/weighted-quadratic.hpp"

#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/costs/frame-placement.hpp"
#include "crocoddyl/multibody/costs/cost-sum.hpp"
#include "crocoddyl/multibody/costs/state.hpp"
#include "crocoddyl/multibody/costs/control.hpp"
#include "crocoddyl/multibody/actions/free-fwddyn.hpp"

#include "yaml_parser/parser_yaml.h"

class CostModelDoublePendulum: public crocoddyl::CostModelAbstract 
{
private:
    double reference_theta;
    double reference_alpha;
    double referece_dot_theta;
    double referece_dot_alpha;

public:

    CostModelDoublePendulum(const boost::shared_ptr<StateMultibody> &state,
                            const boost::shared_ptr<ActivationModelAbstract> &activation, const size_t &nu);

    void calc(const boost::shared_ptr <CostDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
              const Eigen::Ref<const VectorXs> &u) override;

    void calcDiff(const boost::shared_ptr<CostDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
                  const Eigen::Ref<const VectorXs> &u) override;

    void setReference(double new_theta, double reference_alpha, double new_dot_theta, double new_dot_alpha);
};


#endif