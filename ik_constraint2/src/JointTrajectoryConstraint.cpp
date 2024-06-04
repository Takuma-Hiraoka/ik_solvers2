#include <ik_constraint2/JointTrajectoryConstraint.h>
#include <iostream>

namespace ik_constraint2{
  void JointTrajectoryConstraint::updateBounds () {
    double A_q = (this->A_joint_) ? this->A_joint_->q() : this->A_q_;
    double B_q = (this->B_joint_) ? this->B_joint_->q() : this->B_q_;

    double error = A_q - B_q; // A - B

    // this->eqの計算 A - B
    if(this->eq_.rows() != 1) this->eq_ = Eigen::VectorXd(1);
    this->eq_[0] = this->weight_ * std::min(std::max(error,-this->maxError_), this->maxError_);

    // distance計算用
    this->current_error_ = error;

    if(this->debugLevel_>=1){
      std::cerr << "JointTrajectoryConstraint " << (this->A_joint_?this->A_joint_->name(): std::string("target")) << " : " << (this->B_joint_?this->B_joint_->name(): std::string("target")) << std::endl;
      std::cerr << "A_q" << std::endl;
      std::cerr << A_q << std::endl;
      std::cerr << "B_q" << std::endl;
      std::cerr << B_q << std::endl;
      std::cerr << "eq" << std::endl;
      std::cerr << this->eq_.transpose() << std::endl;
    }

  }

  void JointTrajectoryConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {

    // this->jacobian_の計算
    if(!IKConstraint::isJointsSame(joints,this->jacobian_joints_)
       || this->A_joint_ != this->jacobian_A_joint_
       || this->B_joint_ != this->jacobian_B_joint_){
      this->jacobian_joints_ = joints;
      this->jacobian_A_joint_ = this->A_joint_;
      this->jacobian_B_joint_ = this->B_joint_;
      this->jacobianColMap_.clear();
      int cols = 0;
      for(size_t i=0; i < this->jacobian_joints_.size(); i++){
        this->jacobianColMap_[this->jacobian_joints_[i]] = cols;
        cols += IKConstraint::getJointDOF(this->jacobian_joints_[i]);
      }

      int rows = 0;
      if (this->A_joint_) rows++;
      if (this->B_joint_) rows++;
      this->jacobian_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(rows,cols);

      if(this->jacobianColMap_.find(this->jacobian_A_joint_) != this->jacobianColMap_.end()){
        if(this->jacobian_A_joint_->isRevoluteJoint() || this->jacobian_A_joint_->isPrismaticJoint()){
          this->jacobian_.insert(0,this->jacobianColMap_[this->jacobian_A_joint_]) = 1;
        }
      }
      if(this->jacobianColMap_.find(this->jacobian_B_joint_) != this->jacobianColMap_.end()){
        if(this->jacobian_B_joint_->isRevoluteJoint() || this->jacobian_B_joint_->isPrismaticJoint()){
          this->jacobian_.insert(this->A_joint_ ? 1:0,this->jacobianColMap_[this->jacobian_B_joint_]) = 1;
        }
      }

    }

    if(this->jacobianColMap_.find(this->jacobian_A_joint_) != this->jacobianColMap_.end()){
      if(this->jacobian_A_joint_->isRevoluteJoint() || this->jacobian_A_joint_->isPrismaticJoint()){
        this->jacobian_.coeffRef(0,this->jacobianColMap_[this->jacobian_A_joint_]) = -this->weight_;
      }
    }
    if(this->jacobianColMap_.find(this->jacobian_B_joint_) != this->jacobianColMap_.end()){
      if(this->jacobian_B_joint_->isRevoluteJoint() || this->jacobian_B_joint_->isPrismaticJoint()){
        this->jacobian_.coeffRef(this->B_joint_ ? 1:0,this->jacobianColMap_[this->jacobian_B_joint_]) = this->weight_;
      }
    }

    // this->jacobianIneq_のサイズだけそろえる
    this->jacobianIneq_.resize(0,this->jacobian_.cols());

    if(this->debugLevel_>=1){
      std::cerr << "JointTrajectoryConstraint" << std::endl;
      std::cerr << "jacobian" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
    }

    return;
  }

  bool JointTrajectoryConstraint::isSatisfied() const{
    return this->eq_.norm() <= this->precision_;
  }

  double JointTrajectoryConstraint::distance() const{
    return std::abs(this->current_error_ * this->weight_);
  }

  std::shared_ptr<IKConstraint> JointTrajectoryConstraint::clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    std::shared_ptr<JointTrajectoryConstraint> ret = std::make_shared<JointTrajectoryConstraint>(*this);
    this->copy(ret, modelMap);
    return ret;
  }

  void JointTrajectoryConstraint::copy(std::shared_ptr<JointTrajectoryConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    if(this->A_joint_ && modelMap.find(this->A_joint_->body()) != modelMap.end()) ret->A_joint() = modelMap.find(this->A_joint_->body())->second->link(this->A_joint_->index());
    if(this->B_joint_ && modelMap.find(this->B_joint_->body()) != modelMap.end()) ret->B_joint() = modelMap.find(this->B_joint_->body())->second->link(this->B_joint_->index());
  }

}
