#include <iostream>

#include <ik_constraint2/CollisionConstraint.h>
#include <ik_constraint2/Jacobian.h>
#include <cnoid/EigenUtil>

namespace ik_constraint2{
  void CollisionConstraint::updateBounds () {
    // minIneq/maxIneqの計算

    if(!this->computeDistance(this->A_link_, this->B_link_,
                              this->currentDistance_, this->currentDirection_, this->A_currentLocalp_,this->B_currentLocalp_)){
      this->currentDistance_ = this->ignoreDistance_;
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
    }else{
      if(this->currentDistance_ < this->ignoreDistance_){
        if(this->minIneq_.rows()!=1) this->minIneq_ = Eigen::VectorXd::Zero(1);
        if(this->maxIneq_.rows()!=1) this->maxIneq_ = Eigen::VectorXd::Zero(1);
        if(!this->invert_){
          this->minIneq_[0] = std::min((this->tolerance_ - this->currentDistance_) / this->velocityDamper_, this->maxError_) * this->weight_;
          this->maxIneq_[0] = 1e10;
        }else{
          this->minIneq_[0] = -1e10;
          this->maxIneq_[0] = std::max((this->tolerance_ - this->currentDistance_) / this->velocityDamper_, -this->maxError_) * this->weight_;
        }
      }else{
        this->minIneq_.resize(0);
        this->maxIneq_.resize(0);
      }
    }

    if(this->debugLevel_>=1){
      std::cerr << "CollisionConstraint " << (this->A_link_ ? this->A_link_->name() : "world") << " - " << (this->B_link_ ? this->B_link_->name() : "world") << std::endl;
      std::cerr << "distance: " << this->currentDistance_ << std::endl;
      std::cerr << "direction" << std::endl;
      std::cerr << this->currentDirection_.transpose() << std::endl;
      std::cerr << "minIneq" << std::endl;
      std::cerr << this->minIneq_.transpose() << std::endl;
      std::cerr << "maxIneq" << std::endl;
      std::cerr << this->maxIneq_.transpose() << std::endl;
    }
  }

  void CollisionConstraint::updateJacobian (const std::vector<cnoid::LinkPtr>& joints) {

    // jacobianIneq_の計算
    // 行列の初期化. 前回とcol形状が変わっていないなら再利用
    if(!IKConstraint::isJointsSame(joints,this->jacobianineq_full_joints_)
       || this->A_link_ != this->jacobianineq_full_A_link_
       || this->B_link_ != this->jacobianineq_full_B_link_){
      this->jacobianineq_full_joints_ = joints;
      this->jacobianineq_full_A_link_ = this->A_link_;
      this->jacobianineq_full_B_link_ = this->B_link_;

      ik_constraint2::calc6DofJacobianShape(this->jacobianineq_full_joints_,//input
                                this->jacobianineq_full_A_link_,//input
                                this->jacobianineq_full_B_link_,//input
                                this->jacobianineq_full_,
                                this->jacobianineq_fullColMap_,
                                this->path_A_joints_,
                                this->path_B_joints_,
                                this->path_BA_joints_,
                                this->path_BA_joints_numUpwardConnections_
                                );
    }

    if(this->currentDistance_ >= this->ignoreDistance_){
      // this->jacobian_, this->jacobianIneq_のサイズだけそろえる
      this->jacobian_.resize(0,this->jacobianineq_full_.cols());
      this->jacobianIneq_.resize(0,this->jacobianineq_full_.cols());
    }else{
      cnoid::Isometry3 A_localpos = cnoid::Isometry3::Identity();
      A_localpos.translation() = this->A_currentLocalp_;
      cnoid::Isometry3 B_localpos = cnoid::Isometry3::Identity();
      B_localpos.translation() = this->B_currentLocalp_;
      ik_constraint2::calc6DofJacobianCoef(this->jacobianineq_full_joints_,//input
                                           this->jacobianineq_full_A_link_,//input
                                           A_localpos,//input
                                           this->jacobianineq_full_B_link_,//input
                                           B_localpos,//input
                                           this->jacobianineq_fullColMap_,//input
                                           this->path_A_joints_,//input
                                           this->path_B_joints_,//input
                                           this->path_BA_joints_,//input
                                           this->path_BA_joints_numUpwardConnections_,//input
                                           this->jacobianineq_full_
                                           );

      Eigen::SparseMatrix<double,Eigen::RowMajor> dir(3,1);
      for(int i=0;i<3;i++) dir.insert(i,0) = this->currentDirection_[i];
      this->jacobianIneq_ = dir.transpose() * this->jacobianineq_full_.topRows<3>() * this->weight_;

      // this->jacobian_のサイズだけそろえる
      this->jacobian_.resize(0,this->jacobianIneq_.cols());
    }

    if(this->debugLevel_>=1){
      std::cerr << "CollisionConstraint " << (this->A_link_ ? this->A_link_->name() : "world") << " - " << (this->B_link_ ? this->B_link_->name() : "world") << std::endl;
      std::cerr << "jacobianineq" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }

    return;
  }

  bool CollisionConstraint::isSatisfied() const{
    if(!this->invert_){
      return this->currentDistance_-this->tolerance_ >= -this->precision_;
    }else{
      return this->currentDistance_-this->tolerance_ <= this->precision_;
    }
  }

  double CollisionConstraint::distance() const{
    if(!this->invert_){
      return std::abs(std::min(this->currentDistance_-this->tolerance_, 0.0)) * this->weight_;
    }else{
      return std::abs(std::max(this->currentDistance_-this->tolerance_, 0.0)) * this->weight_;
    }
  }

  double CollisionConstraint::margin() const{
    if(!this->invert_){
      return (this->currentDistance_-this->tolerance_) * this->weight_;
    }else{
      return - (this->currentDistance_-this->tolerance_) * this->weight_;
    }
  }

  std::vector<cnoid::SgNodePtr>& CollisionConstraint::getDrawOnObjects(){
    if(!this->lines_){
      this->lines_ = new cnoid::SgLineSet;
      this->lines_->setLineWidth(1.0);
      this->lines_->getOrCreateColors()->resize(1);
      this->lines_->getOrCreateColors()->at(0) = cnoid::Vector3f(0.5,0.0,0.0);
      // A, B
      this->lines_->getOrCreateVertices()->resize(2);
      this->lines_->colorIndices().resize(0);
      this->lines_->addLine(0,1); this->lines_->colorIndices().push_back(0); this->lines_->colorIndices().push_back(0);

      this->drawOnObjects_ = std::vector<cnoid::SgNodePtr>{this->lines_};
    }

    if(this->currentDistance_ >= this->ignoreDistance_) return this->dummyDrawOnObjects_;

    const cnoid::Vector3& A_pos = (this->A_link_) ? this->A_link_->T() * this->A_currentLocalp_ : this->A_currentLocalp_;
    const cnoid::Vector3& B_pos = (this->B_link_) ? this->B_link_->T() * this->B_currentLocalp_ : this->B_currentLocalp_;

    this->lines_->getOrCreateVertices()->at(0) = A_pos.cast<cnoid::Vector3f::Scalar>();
    this->lines_->getOrCreateVertices()->at(1) = B_pos.cast<cnoid::Vector3f::Scalar>();

    return this->drawOnObjects_;
  }

  void CollisionConstraint::copy(std::shared_ptr<CollisionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const {
    if(this->A_link_ && modelMap.find(this->A_link_->body()) != modelMap.end()) ret->A_link() = modelMap.find(this->A_link_->body())->second->link(this->A_link_->index());
    if(this->B_link_ && modelMap.find(this->B_link_->body()) != modelMap.end()) ret->B_link() = modelMap.find(this->B_link_->body())->second->link(this->B_link_->index());
  }

}
