//
// Created by Adria on 27/6/20.
//

#include "CostModelDoublePendulum.h"


CostModelDoublePendulum::CostModelDoublePendulum(const boost::shared_ptr<crocoddyl::StateMultibody> &state,
                                                 const boost::shared_ptr<crocoddyl::ActivationModelAbstract> &activation,
                                                 const size_t &nu) : CostModelAbstractTpl(state, activation, nu) 
{
    this->reference_theta    = 0;
    this->reference_alpha    = 0;
    this->referece_dot_theta = 0;
    this->referece_dot_alpha = 0;
}

void CostModelDoublePendulum::setReference(double new_theta, double new_alpha, double new_dot_theta, double new_dot_alpha){
    this->reference_theta    = new_theta;
    this->reference_alpha    = new_alpha;
    this->referece_dot_theta = new_dot_theta;
    this->referece_dot_alpha = new_dot_alpha;
}

void CostModelDoublePendulum::calc(const boost::shared_ptr<crocoddyl::CostDataAbstract> &data,
                                   const Eigen::Ref<const VectorXs> &x,
                                   const Eigen::Ref<const VectorXs> &u) {
    double c1 = cos(x[0] - reference_theta);
    double c2 = cos(x[1] - reference_alpha);
    
    double s1 = sin(x[0] - reference_theta);
    double s2 = sin(x[1] - reference_alpha);
    
    data->r << s1, s2, 1 - c1, 1 - c2, x[2] - referece_dot_theta, x[3] - referece_dot_alpha;
    
    activation_->calc(data->activation,data->r);    
    data->cost = data->activation->a_value;
}

void CostModelDoublePendulum::calcDiff(const boost::shared_ptr<crocoddyl::CostDataAbstract> &data,
                                       const Eigen::Ref<const VectorXs> &x,
                                       const Eigen::Ref<const VectorXs> &u) {
    
    double c1 = cos(x[0] - reference_theta);
    double c2 = cos(x[1] - reference_alpha);
    
    double s1 = sin(x[0] - reference_theta);
    double s2 = sin(x[1] - reference_alpha);
    
    activation_->calcDiff(data->activation,data->r);

    //Jacobià
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6,4);
    J << c1,0 ,0,0,
         0 ,c2,0,0,
         s1,0 ,0,0,
         0 ,s2,0,0,
         0 ,0 ,1,0,
         0 ,0 ,0,1;
    J.transposeInPlace();
    data->Lx = J * data->activation->Ar;
    

    //Matriu Hessiana
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6,4);
    H << pow(c1,2) - pow(s1,2)     ,0                         ,0, 0,
         0                         ,pow(c2,2) - pow(s2,2)     ,0, 0,
         pow(s1,2) + (1 - c1) * c1 ,0                         ,0, 0,
         0                         ,pow(s2,2) + (1 - c2) * c2 ,0, 0,
         0                         ,0                         ,1, 0,
         0                         ,0                         ,0, 1;
    H.transposeInPlace();

    MatrixXs A = H * data->activation->Arr.diagonal();
    data->Lxx = A.asDiagonal();
}
