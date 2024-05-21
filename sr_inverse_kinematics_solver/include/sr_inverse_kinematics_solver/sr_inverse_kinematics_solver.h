#ifndef SR_INVERSE_KINEMATICS_SOLVER_H
#define SR_INVERSE_KINEMATICS_SOLVER_H

#include <cnoid/Body>
#include <ik_constraint2/IKConstraint.h>

namespace sr_inverse_kinematics_solver {
  class IKParam {
  public:
    /*
     終了条件:
       maxIteration
       minIteraiion
       convertThre
       isSatisfied

     or maxIteraion
     or minIteration and isSatisfied
     or convergeThre
     */
    size_t maxIteration = 1; // constraintをsatisfiedするか、maxIteraionに達すると終了する.
    size_t minIteration = 0; // このiterationまでは、constraintをsatisfiedしても終了しない
    std::vector<double> dqWeight; // dqWeight.size() == dimの場合、探索変数の各要素について、wn+weをdqWeight倍する. 0だとその関節を動かさない
    double wn = 1e-6;
    double we = 1e0;
    double wmax = 1e-1;
    int debugLevel = 0; // 0: no debug message. 1: time measure. 2: internal state

    double dt = 0.1;
    bool calcVelocity = true; // dtを用いて速度の計算をするかどうか. 速度を利用するconstraintがあるなら必須. ないなら、falseにすると高速化が見込まれる
    bool checkFinalState = true; // maxIteration番目またはconvergedのloop後に、各constraintを満たしているかどうかの判定を行うかどうか. 行わない場合、falseが返る.
    double convergeThre = 5e-3; // 各イテレーションでの変位のノルムがconvergeThre未満の場合に、maxIterationに行っていなくても, minIteraionに行っていなくても、isSatisfiedでなくても、終了する

    bool enableJointLimit = true;
  };

  // 返り値はikc_list全てをikSatisfiedしたかどうか
  bool solveIKLoop (const std::vector<cnoid::LinkPtr>& variables,
                    const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& ikc_list,
                    const IKParam& param = IKParam()
                    );
}

#endif
