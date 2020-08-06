//
// Created by Adria on 26/6/20.
//

#include "ActuationModelDoublePendulum.h"

ActuationModelDoublePendulum::ActuationModelDoublePendulum(const boost::shared_ptr<crocoddyl::StateAbstract> &state,
                                                           const size_t &nu, size_t nv, actuated_link act_link) : ActuationModelAbstractTpl(state, nu), nv(nv) {
    this->nv = state->get_nv();
    S = MathBase::MatrixXs(this->nv, this->nu_);
    switch(act_link){
        
        default:
        std::cout << "Actuated link selected out of range.Val is " << act_link << std::endl;
        case ENDPOINT_LINK:
            std::cout << "Changing to endpoint link actuation mode." << std::endl;
            S(1,0) = 1;
        break;
        case BASE_LINK:
            std::cout << "Changing to base link actuation mode." << std::endl;
            S(0,0) = 1;
        break;
        case BOTH_LINKS:
        std::cout << "Changing to both link actuation mode." << std::endl;
        S(0,0) = 1;
        S(1,0) = 1;
        break;
    }
}

void ActuationModelDoublePendulum::calc(const boost::shared_ptr<ActuationDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
              const Eigen::Ref<const VectorXs> &u) {    
    data->tau = S * u;
}

void ActuationModelDoublePendulum::calcDiff(const boost::shared_ptr<ActuationDataAbstract> &data, const Eigen::Ref<const VectorXs> &x,
                  const Eigen::Ref<const VectorXs> &u){
    data->dtau_du = S;
}
