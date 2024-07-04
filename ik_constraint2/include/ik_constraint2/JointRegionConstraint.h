#ifndef IK_CONSTRAINT2_JOINTREGIONCONSTRAINT_H
#define IK_CONSTRAINT2_JOINTREGIONCONSTRAINT_H

#include <ik_constraint2/IKConstraint.h>
#include <cnoid/EigenUtil>

namespace ik_constraint2{
  class JointRegionConstraint : public IKConstraint
  {
  public:
    //A_joint->q + A_q と, B_joint->q + B_q の差をregion以下にさせる. nullptrなら足さずにA_q,B_qのみを用いる.
    //  maxError: エラーの頭打ち
    //  weight: コスト関数の重み. error * weight^2 * error.
    //  precision: 収束判定の閾値. error * weightのノルムと比べる

    const cnoid::LinkPtr& A_joint() const { return A_joint_;}
    cnoid::LinkPtr& A_joint() { return A_joint_;}
    const double& A_q() const { return A_q_;}
    double& A_q() { return A_q_;}
    const cnoid::LinkPtr& B_joint() const { return B_joint_;}
    cnoid::LinkPtr& B_joint() { return B_joint_;}
    const double& B_q() const { return B_q_;}
    double& B_q() { return B_q_;}
    const double& region() const { return region_;}
    double& region() { return region_;}
    const double& maxError() const { return maxError_;}
    double& maxError() { return maxError_;}
    const double& precision() const { return precision_;}
    double& precision() { return precision_;}
    const double& weight() const { return weight_;}
    double& weight() { return weight_;}

    // 内部状態更新. eq, minIneq, maxIneqを生成
    virtual void updateBounds () override;
    // 内部状態更新. jacobian, jacobianIneqを生成
    virtual void updateJacobian (const std::vector<cnoid::LinkPtr>& joints) override;
    // 達成判定
    virtual bool isSatisfied () const override;
    // 達成までの距離. getEqなどは、エラーの頭打ちを行うが、distanceは行わないので、より純粋なisSatisfiedまでの距離を表す.
    virtual double distance() const override;
    // 複製する. このとき、modelMapのkeyにあるロボットモデルに属するリンクは、valueに置き換える
    virtual std::shared_ptr<IKConstraint> clone(const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const override;
    void copy(std::shared_ptr<JointRegionConstraint> ret, const std::map<cnoid::BodyPtr, cnoid::BodyPtr>& modelMap) const;

  protected:
    cnoid::LinkPtr A_joint_ = nullptr;
    double A_q_ = 0.0;
    cnoid::LinkPtr B_joint_ = nullptr;
    double B_q_ = 0.0;
    double precision_ = 1e-3;
    double maxError_ = 0.05;
    double region_ = 0.0;
    double weight_ = 1.0;

    double current_upper_ = 0.0;
    double current_lower_ = 0.0;

    cnoid::LinkPtr jacobian_A_joint_ = nullptr; //前回jacobian_を計算した時のA_joint
    cnoid::LinkPtr jacobian_B_joint_ = nullptr; //前回jacobian_を計算した時のB_joint

    std::vector<cnoid::LinkPtr> jacobian_joints_; // 前回のjacobian計算時のjoints
    std::unordered_map<cnoid::LinkPtr,int> jacobianColMap_;

  };
}

#endif
