/*
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_localization/ekf.h"
#include "robot_localization/filter_common.h"

#include "xmlrpcpp/XmlRpcException.h"


#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>



namespace RobotLocalization
{
  Ekf::Ekf(std::vector<double>) :
    FilterBase()  // Must initialize filter base!
  {
  }

  Ekf::~Ekf()
  {
  }

  void Ekf::correct(const Measurement &measurement)
  {
    FB_DEBUG("---------------------- Ekf::correct(" << measurement.topicName_ << ")----------------------\n" <<
             "State is:\n" << state_ << "\n"
             "Measurement topic is:\n" << measurement.topicName_ << "\n"
             "Measurement is:\n" << measurement.measurement_ << "\n");
             
    FB_DEBUG("msg time:"<<std::setprecision(20) << measurement.time_<<"\n");

    // We don't want to update everything, so we need to build matrices that only update
    // the measured parts of our state vector. Throughout prediction and correction, we
    // attempt to maximize efficiency in Eigen.
    // 我们不想更新所有内容，因此我们需要构建仅更新状态向量的测量部分的矩阵。
    // 在整个预测和校正过程中，我们尝试在Eigen中最大化效率。

    // First, determine how many state vector values we're updating
    // 首先，确定我们正在更新多少个状态向量值
    std::vector<size_t> updateIndices;
    for (size_t i = 0; i < measurement.updateVector_.size(); ++i)
    {
      if (measurement.updateVector_[i])
      {
        // Handle nan and inf values in measurements
        // 处理测量中的nan和inf值
        if (std::isnan(measurement.measurement_(i)))
        {
          FB_DEBUG("Value at index " << i << " was nan. Excluding from update.\n");
        }
        else if (std::isinf(measurement.measurement_(i)))
        {
          FB_DEBUG("Value at index " << i << " was inf. Excluding from update.\n");
        }
        else
        {
          updateIndices.push_back(i);
        }
      }
    }

      FB_DEBUG("Update indices are:\n" << updateIndices << "\n");

      size_t updateSize = updateIndices.size();
      // 现在设置相关矩阵
      Eigen::VectorXd stateSubset(updateSize);                              // x (in most literature）
      Eigen::VectorXd measurementSubset(updateSize);                        // z
      Eigen::MatrixXd measurementCovarianceSubset(updateSize, updateSize);  // R
      Eigen::MatrixXd stateToMeasurementSubset(updateSize, state_.rows());  // H
      Eigen::MatrixXd kalmanGainSubset(state_.rows(), updateSize);          // K
      Eigen::VectorXd innovationSubset(updateSize);                         // z - Hx新息向量，测量-预测
      stateSubset.setZero();
      measurementSubset.setZero();
      measurementCovarianceSubset.setZero();
      stateToMeasurementSubset.setZero();
      kalmanGainSubset.setZero();
      innovationSubset.setZero();
    
    //单独处理二维码定位的情况
    if(measurement.topicName_=="QR0")
    {
      setHaveQr(true);//记录这次state来源QR定位，便于map到odom的tf发出
      FB_DEBUG( "R = \n" << measurement.covariance_ << "\n");
      
      Eigen::Vector3d QRposition;//二维码的位置
      for(int i = 0 ; i < 3 ; i++)
      {
        measurementSubset(i) = measurement.measurement_(i);//rou,phi,theta
        QRposition(i) = measurement.measurement_(i+3);
      }

      measurementCovarianceSubset = measurement.covariance_;//QR0的协方差矩阵
      
      Eigen::VectorXd stateTemp = state_;
      Eigen::VectorXd measurementFromState(updateSize);
      //IEKF迭代
      int iterationN = 10;//迭代次数
      int iteration = 0;
      double deta = 0.0;
      double detatheta = 0.0;
      do
      {
        Eigen::VectorXd laststateTemp = stateTemp;

        //提取迭代状态xi
        double xtemp = stateTemp(0);
        double ytemp = stateTemp(1);
        double thetatemp = stateTemp(5);

        //将状态转为测量值
        measurementFromState.setZero();
        measurementFromState(0) = sqrt(pow(xtemp-QRposition(0),2)+pow(ytemp-QRposition(1),2));
        measurementFromState(1) = atan2(ytemp-QRposition(1), xtemp-QRposition(0)) - QRposition(2);
        measurementFromState(2) = thetatemp - QRposition(2);
      
      //计算H矩阵
        stateToMeasurementSubset.setZero();
        double inverseRou = 1/measurementFromState(0);
        stateToMeasurementSubset(0,0) = (xtemp-QRposition(0))*inverseRou;
        stateToMeasurementSubset(0,1) = (ytemp-QRposition(1))*inverseRou;
        stateToMeasurementSubset(1,0) = - stateToMeasurementSubset(0,1)*inverseRou;
        stateToMeasurementSubset(1,1) = stateToMeasurementSubset(0,0)*inverseRou;
        stateToMeasurementSubset(2,5) = 1.0;


        //计算K = (PH') / (HPH' + R)
        kalmanGainSubset.setZero();
        Eigen::MatrixXd pht = estimateErrorCovariance_ * stateToMeasurementSubset.transpose();
        FB_DEBUG("p:\n" << estimateErrorCovariance_ <<"\n");
        FB_DEBUG("ht:\n" << stateToMeasurementSubset.transpose() <<"\n");
        FB_DEBUG("pht:\n" << pht <<"\n");
        Eigen::MatrixXd hphrInv = (stateToMeasurementSubset * pht + measurementCovarianceSubset).inverse();
        FB_DEBUG("hphrInv:\n" << hphrInv <<"\n");
        kalmanGainSubset.noalias() = pht * hphrInv;


        //计算新息 z - h(xi) - Hi*(x - xi)
        innovationSubset = (measurementSubset - measurementFromState - stateToMeasurementSubset*(state_ - stateTemp));
        while (innovationSubset(2) < -PI)//保证偏航角在-pi到pi之间
        {
          innovationSubset(2) += TAU;
        }

        while (innovationSubset(2) > PI)
        {
          innovationSubset(2) -= TAU;
        }

        //计算更新后的状态
        stateTemp.noalias() = state_ + kalmanGainSubset * innovationSubset;

        //计算迭代误差
        deta = (stateTemp.segment(0,2) - laststateTemp.segment(0,2)).norm();//距离差
        detatheta = fabs(stateTemp(5) - laststateTemp(5));//偏航角差

        FB_DEBUG("=========== IEkf ===========\n" << std::setprecision(9) <<
                 "i = "<< iteration << "\n" <<
                 "x_i-1 = \n" << laststateTemp << "\n" <<
                 "x_i = \n" << stateTemp << "\n" <<
                 "H_i-1 = \n" << stateToMeasurementSubset << "\n" <<
                 "K_i-1 = \n" << kalmanGainSubset << "\n" <<
                 "measurement from x_i-1 = \n" << std::setprecision(9) << measurementFromState << "\n" <<
                 "measurement = \n" << std::setprecision(9) << measurementSubset.segment(0,3) << "\n" <<
                 "Innovation = \n" << innovationSubset << "\n" <<
                 "distance = " << deta << ((deta>qr_dis_thre_)?" > ":" < ") << "qr_distance_threshold: " << qr_dis_thre_ << "\n" <<
                 "deta_angle = " << detatheta << ((detatheta>qr_angle_thre_)?" > ":" < ") << "qr_angle_thre_: " << qr_angle_thre_ << "\n");

        iteration++;
      }
      while(iteration <= iterationN && ( deta > qr_dis_thre_ || detatheta > qr_angle_thre_ ) );
      FB_DEBUG("=========== End of IEkf ===========\n");
      state_ = stateTemp;

      //计算更新后的协方差
      // (I - KH)P(I - KH)' + KRK'
      Eigen::MatrixXd gainResidual = identity_;
      gainResidual.noalias() -= kalmanGainSubset * stateToMeasurementSubset;
      estimateErrorCovariance_ = gainResidual * estimateErrorCovariance_ * gainResidual.transpose();
      estimateErrorCovariance_.noalias() += kalmanGainSubset *
                                              measurementCovarianceSubset *
                                              kalmanGainSubset.transpose();
      wrapStateAngles();

      FB_DEBUG("Corrected full state is:\n" << state_ <<
                "Corrected full estimate error covariance is:\n" << estimateErrorCovariance_ <<
                "---------------------- /Ekf::correct ----------------------\n");
    
    }
    else
    {
      // Now build the sub-matrices from the full-sized matrices
      // 现在从全尺寸矩阵构建子矩阵
      for (size_t i = 0; i < updateSize; ++i)
      {
        measurementSubset(i) = measurement.measurement_(updateIndices[i]);
        stateSubset(i) = state_(updateIndices[i]);
        for (size_t j = 0; j < updateSize; ++j)
        {
          measurementCovarianceSubset(i, j) = measurement.covariance_(updateIndices[i], updateIndices[j]);
        }

        // Handle negative (read: bad) covariances in the measurement. Rather
        // than exclude the measurement or make up a covariance, just take
        // the absolute value.
        // 处理测量中的负（坏）协方差。与其排除测量或构成协方差，不如取绝对值。
        // 例如，如果我们有一个测量值为1，但协方差为-0.01，则将其更改为0.01。
        if (measurementCovarianceSubset(i, i) < 0)
        {
          FB_DEBUG("WARNING: Negative covariance for index " << i <<
                   " of measurement (value is" << measurementCovarianceSubset(i, i) <<
                  "). Using absolute value...\n");

          measurementCovarianceSubset(i, i) = ::fabs(measurementCovarianceSubset(i, i));
        }

        // If the measurement variance for a given variable is very
        // near 0 (as in e-50 or so) and the variance for that
        // variable in the covariance matrix is also near zero, then
        // the Kalman gain computation will blow up. Really, no
        // measurement can be completely without error, so add a small
        // amount in that case.
        // 如果给定变量的测量方差非常接近0（如e-50或更高），并且协方差矩阵中该变量的方差也接近0，
        // 则卡尔曼增益计算将会爆炸。实际上，没有测量可以完全没有误差，因此在这种情况下添加一小部分。
        if (measurementCovarianceSubset(i, i) < 1e-9)
        {
          FB_DEBUG("WARNING: measurement had very small error covariance for index " << updateIndices[i] <<
                  ". Adding some noise to maintain filter stability.\n");

          measurementCovarianceSubset(i, i) = 1e-9;
        }
      }

      // The state-to-measurement function, h, will now be a measurement_size x full_state_size
      // matrix, with ones in the (i, i) locations of the values to be updated
      // 状态到测量函数h现在将是一个measurement_size x full_state_size矩阵,其中(i,i)位置的值将被更新
      for (size_t i = 0; i < updateSize; ++i)
      {
        stateToMeasurementSubset(i, updateIndices[i]) = 1;
      }

      FB_DEBUG("Current state subset is:\n" << stateSubset <<
             "\nMeasurement subset is:\n" << measurementSubset << "\n"<<
             "\n R = \n" << measurementCovarianceSubset << 
             "\n H = \n" << stateToMeasurementSubset << "\n");

      // (1) Compute the Kalman gain: K = (PH') / (HPH' + R)
      Eigen::MatrixXd pht = estimateErrorCovariance_ * stateToMeasurementSubset.transpose();
      Eigen::MatrixXd hphrInv  = (stateToMeasurementSubset * pht + measurementCovarianceSubset).inverse();
      kalmanGainSubset.noalias() = pht * hphrInv;

      innovationSubset = (measurementSubset - stateSubset);//残差

      // Wrap angles in the innovation
      for (size_t i = 0; i < updateSize; ++i)
      {
        if (updateIndices[i] == StateMemberRoll  ||
            updateIndices[i] == StateMemberPitch ||
            updateIndices[i] == StateMemberYaw)
        {
          while (innovationSubset(i) < -PI)
          {
            innovationSubset(i) += TAU;
          }

          while (innovationSubset(i) > PI)
          {
            innovationSubset(i) -= TAU;
          }
        }
      }

      // (2) Check Mahalanobis distance between mapped measurement and state.
      //检查  测量  和 状态 之间的马氏距离。
      if (checkMahalanobisThreshold(innovationSubset, hphrInv, measurement.mahalanobisThresh_))
      {
        // (3) Apply the gain to the difference between the state and measurement: x = x + K(z - Hx)
        //将增益应用于状态和测量之间的差值
        state_.noalias() += kalmanGainSubset * innovationSubset;

        // (4) Update the estimate error covariance using the Joseph form: (I - KH)P(I - KH)' + KRK'
        //使用Joseph形式更新估计误差协方差
        Eigen::MatrixXd gainResidual = identity_;
        gainResidual.noalias() -= kalmanGainSubset * stateToMeasurementSubset;
        estimateErrorCovariance_ = gainResidual * estimateErrorCovariance_ * gainResidual.transpose();
        estimateErrorCovariance_.noalias() += kalmanGainSubset *
                                              measurementCovarianceSubset *
                                              kalmanGainSubset.transpose();

        // Handle wrapping of angles
        //保持角度在[-pi,pi]之间
        wrapStateAngles();

        FB_DEBUG("K = \n" << kalmanGainSubset <<
                 "\nInnovation is:\n" << innovationSubset <<
                "\nCorrected full state is:\n" << state_ <<
                "\nCorrected full estimate error covariance is:\n" << estimateErrorCovariance_ <<
                "---------------------- /Ekf::correct ----------------------\n");
      }
    }
  
  }



  void Ekf::predict(const double referenceTime, const double delta)
  {
    FB_DEBUG("---------------------- Ekf::predict ----------------------\n" <<
             "delta is " << delta << "\n" <<
             "state is " << state_ << "\n");

    //CTRA模型 x y yaw xVel yawVel xAcc 
    //double roll = state_(StateMemberRoll);
    //double pitch = state_(StateMemberPitch);
    double yaw = state_(StateMemberYaw);
    double xVel = state_(StateMemberVx);
    //double yVel = state_(StateMemberVy);
    //double zVel = state_(StateMemberVz);
    //double pitchVel = state_(StateMemberVpitch);
    double yawVel = state_(StateMemberVyaw);
    double xAcc = state_(StateMemberAx);
    //double yAcc = state_(StateMemberAy);
    //double zAcc = state_(StateMemberAz);

    // We'll need these trig calculations a lot.
    // 我们需要经常进行三角函数计算
    // double sp = ::sin(pitch);
    // double cp = ::cos(pitch);
    // double cpi = 1.0 / cp;
    // double tp = sp * cpi;

    // double sr = ::sin(roll);
    // double cr = ::cos(roll);

    // double sy = ::sin(yaw);
    // double cy = ::cos(yaw);

    prepareControl(referenceTime, delta);

    // Prepare the transfer function
    // 转移方程，可以看成是状态转移矩阵
    //在构造函数中reset函数完成了transferFunction_和transferFunctionJacobian_的初始化(单位矩阵)
    // 以第一个为例，就是x方向的位移 = x方向的速度乘以时间间隔 
    //由于X是在世界坐标系下，所以还要乘以cos(yaw)和cos(pitch)旋转到世界坐标系中。
    /*
    transferFunction_(StateMemberX, StateMemberVx) = cy * cp * delta;
    transferFunction_(StateMemberX, StateMemberVy) = (cy * sp * sr - sy * cr) * delta;
    transferFunction_(StateMemberX, StateMemberVz) = (cy * sp * cr + sy * sr) * delta;
    transferFunction_(StateMemberX, StateMemberAx) = 0.5 * transferFunction_(StateMemberX, StateMemberVx) * delta;
    transferFunction_(StateMemberX, StateMemberAy) = 0.5 * transferFunction_(StateMemberX, StateMemberVy) * delta;
    transferFunction_(StateMemberX, StateMemberAz) = 0.5 * transferFunction_(StateMemberX, StateMemberVz) * delta;
    transferFunction_(StateMemberY, StateMemberVx) = sy * cp * delta;
    transferFunction_(StateMemberY, StateMemberVy) = (sy * sp * sr + cy * cr) * delta;
    transferFunction_(StateMemberY, StateMemberVz) = (sy * sp * cr - cy * sr) * delta;
    transferFunction_(StateMemberY, StateMemberAx) = 0.5 * transferFunction_(StateMemberY, StateMemberVx) * delta;
    transferFunction_(StateMemberY, StateMemberAy) = 0.5 * transferFunction_(StateMemberY, StateMemberVy) * delta;
    transferFunction_(StateMemberY, StateMemberAz) = 0.5 * transferFunction_(StateMemberY, StateMemberVz) * delta;
    transferFunction_(StateMemberZ, StateMemberVx) = -sp * delta;
    transferFunction_(StateMemberZ, StateMemberVy) = cp * sr * delta;
    transferFunction_(StateMemberZ, StateMemberVz) = cp * cr * delta;
    transferFunction_(StateMemberZ, StateMemberAx) = 0.5 * transferFunction_(StateMemberZ, StateMemberVx) * delta;
    transferFunction_(StateMemberZ, StateMemberAy) = 0.5 * transferFunction_(StateMemberZ, StateMemberVy) * delta;
    transferFunction_(StateMemberZ, StateMemberAz) = 0.5 * transferFunction_(StateMemberZ, StateMemberVz) * delta;
    transferFunction_(StateMemberRoll, StateMemberVroll) = delta;
    transferFunction_(StateMemberRoll, StateMemberVpitch) = sr * tp * delta;
    transferFunction_(StateMemberRoll, StateMemberVyaw) = cr * tp * delta;
    transferFunction_(StateMemberPitch, StateMemberVpitch) = cr * delta;
    transferFunction_(StateMemberPitch, StateMemberVyaw) = -sr * delta;
    transferFunction_(StateMemberYaw, StateMemberVpitch) = sr * cpi * delta;
    transferFunction_(StateMemberYaw, StateMemberVyaw) = cr * cpi * delta;
    transferFunction_(StateMemberVx, StateMemberAx) = delta;
    transferFunction_(StateMemberVy, StateMemberAy) = delta;
    transferFunction_(StateMemberVz, StateMemberAz) = delta;
    */
    //准备CTRA的数据
    double detax,detay,detayaw,detaVel;
    transferFunctionJacobian_ = transferFunction_;//CTRA中这里是单位阵
    if(::fabs(yawVel) < 1e-4)
    {
      //CTRA模型退化成CA模型
      double sy = ::sin(yaw);
      double cy = ::cos(yaw);
      detax = xVel*cy*delta + 0.5*xAcc*cy*delta*delta;
      detay = xVel*sy*delta + 0.5*xAcc*sy*delta*delta;
      detaVel = xAcc*delta;
      detayaw = 0.0;

      //准备一阶偏导
      double dFx_dV = cy*delta;
      double dFy_dV = sy*delta;

    
      double dFx_dAcc = 0.5*cy*delta*delta;
      double dFy_dAcc = 0.5*sy*delta*delta;

      double dFx_dyaw = -detay;
      double dFy_dyaw = detax;

      //雅可比矩阵
      transferFunctionJacobian_(StateMemberX,StateMemberVx) = dFx_dV;
      transferFunctionJacobian_(StateMemberX,StateMemberAx) = dFx_dAcc;
      transferFunctionJacobian_(StateMemberX,StateMemberYaw) = dFx_dyaw;
      transferFunctionJacobian_(StateMemberY,StateMemberVx) = dFy_dV;
      transferFunctionJacobian_(StateMemberY,StateMemberAx) = dFy_dAcc;
      transferFunctionJacobian_(StateMemberY,StateMemberYaw) = dFy_dyaw;
      transferFunctionJacobian_(StateMemberVx,StateMemberAx) = delta;

    }
    else
    {
      //CTRA模型
      double yaw_temp = yaw + yawVel * delta;
      double vel_temp = xVel+ xAcc *delta;
      double sy1 = ::sin(yaw);
      double sy2 = ::sin(yaw_temp);
      double cy1 = ::cos(yaw);
      double cy2 = ::cos(yaw_temp);
      double invyawVel = 1.0/yawVel;
      double invyawVel2= invyawVel*invyawVel;

      detax = invyawVel*(vel_temp*sy2 - xVel*sy1) + xAcc*invyawVel2*(cy2 - cy1);
      detay =-invyawVel*(vel_temp*cy2 - xVel*cy1) + xAcc*invyawVel2*(sy2 - sy1);
      detayaw = yawVel*delta;
      detaVel = xAcc*delta;


       //准备一阶偏导
      double dFx_dV = (sy2-sy1)*invyawVel;
      double dFy_dV = (cy1-cy2)*invyawVel;
      
      double dFx_dAcc = sy2*invyawVel*delta + invyawVel2*(cy2-cy1);
      double dFy_dAcc = -cy2*invyawVel*delta + invyawVel2*(sy2-sy1);

      double dFx_dyaw = invyawVel*(vel_temp*cy2 - xVel*cy1) - xAcc*invyawVel2*(sy2 - sy1);
      double dFy_dyaw = invyawVel*(vel_temp*sy2 - xVel*sy1) + xAcc*invyawVel2*(cy2 - cy1); 

      double dFx_dyawV =-invyawVel2*((vel_temp+xAcc*delta)*sy2-xVel*sy1) + invyawVel*vel_temp*delta*cy2 - 2*invyawVel2*invyawVel*xAcc*(cy2-cy1);
      double dFy_dyawV = invyawVel2*((vel_temp+xAcc*delta)*cy2-xVel*cy1) + invyawVel*vel_temp*delta*sy2 - 2*invyawVel2*invyawVel*xAcc*(sy2-sy1);

      //雅可比矩阵
      transferFunctionJacobian_(StateMemberX,StateMemberVx) = dFx_dV;
      transferFunctionJacobian_(StateMemberX,StateMemberAx) = dFx_dAcc;
      transferFunctionJacobian_(StateMemberX,StateMemberYaw) = dFx_dyaw;
      transferFunctionJacobian_(StateMemberX,StateMemberVyaw) = dFx_dyawV;
      transferFunctionJacobian_(StateMemberY,StateMemberVx) = dFy_dV;
      transferFunctionJacobian_(StateMemberY,StateMemberAx) = dFy_dAcc;
      transferFunctionJacobian_(StateMemberY,StateMemberYaw) = dFy_dyaw;
      transferFunctionJacobian_(StateMemberY,StateMemberVyaw) = dFy_dyawV;
      transferFunctionJacobian_(StateMemberVx,StateMemberAx) = delta;
      transferFunctionJacobian_(StateMemberYaw,StateMemberVyaw) = delta;
    }
    
    Eigen::VectorXd detastate_(state_.rows());
    detastate_ << detax , detay , 0.0 ,
                   0.0 , 0.0 , detayaw ,
                  detaVel , 0.0 , 0.0 ,
                   0.0 , 0.0 , 0.0 ,
                   0.0 , 0.0 , 0.0;
    // Prepare the transfer function Jacobian. This function is analytically derived from the
    // transfer function.
    // 准备转移函数雅克比矩阵。该函数是从转移函数解析导出的。
    /*
    double xCoeff = 0.0;
    double yCoeff = 0.0;
    double zCoeff = 0.0;
    double oneHalfATSquared = 0.5 * delta * delta;

    yCoeff = cy * sp * cr + sy * sr;
    zCoeff = -cy * sp * sr + sy * cr;
    //x对roll求导
    double dFx_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    //roll对roll求导
    double dFR_dR = 1.0 + (cr * tp * pitchVel - sr * tp * yawVel) * delta;

    xCoeff = -cy * sp;
    yCoeff = cy * cp * sr;
    zCoeff = cy * cp * cr;
    //x对pitch求导
    double dFx_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    //Roll对pitch求导
    double dFR_dP = (cpi * cpi * sr * pitchVel + cpi * cpi * cr * yawVel) * delta;

    xCoeff = -sy * cp;
    yCoeff = -sy * sp * sr - cy * cr;
    zCoeff = -sy * sp * cr + cy * sr;
    //x对yaw求导
    double dFx_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    yCoeff = sy * sp * cr - cy * sr;
    zCoeff = -sy * sp * sr - cy * cr;
    //y对roll求导
    double dFy_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    //pitch对roll求导
    double dFP_dR = (-sr * pitchVel - cr * yawVel) * delta;

    xCoeff = -sy * sp;
    yCoeff = sy * cp * sr;
    zCoeff = sy * cp * cr;
    //y对pitch求导
    double dFy_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    xCoeff = cy * cp;
    yCoeff = cy * sp * sr - sy * cr;
    zCoeff = cy * sp * cr + sy * sr;
    //y对yaw求导
    double dFy_dY = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;

    yCoeff = cp * cr;
    zCoeff = -cp * sr;
    //z对roll求导
    double dFz_dR = (yCoeff * yVel + zCoeff * zVel) * delta +
                    (yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    //yall对roll求导
    double dFY_dR = (cr * cpi * pitchVel - sr * cpi * yawVel) * delta;

    xCoeff = -cp;
    yCoeff = -sp * sr;
    zCoeff = -sp * cr;
    //z对pitch求导
    double dFz_dP = (xCoeff * xVel + yCoeff * yVel + zCoeff * zVel) * delta +
                    (xCoeff * xAcc + yCoeff * yAcc + zCoeff * zAcc) * oneHalfATSquared;
    //yaw对pitch求导
    double dFY_dP = (sr * tp * cpi * pitchVel + cr * tp * cpi * yawVel) * delta;

    */
   

    // Much of the transfer function Jacobian is identical to the transfer function
    //transferFunctionJacobian_ = transferFunction_;//CTRA中这里是单位阵
    /*
    // transferFunctionJacobian_(StateMemberX, StateMemberRoll) = dFx_dR;
    // transferFunctionJacobian_(StateMemberX, StateMemberPitch) = dFx_dP;
    // transferFunctionJacobian_(StateMemberX, StateMemberYaw) = dFx_dY;
    // transferFunctionJacobian_(StateMemberY, StateMemberRoll) = dFy_dR;
    // transferFunctionJacobian_(StateMemberY, StateMemberPitch) = dFy_dP;
    // transferFunctionJacobian_(StateMemberY, StateMemberYaw) = dFy_dY;
    // transferFunctionJacobian_(StateMemberZ, StateMemberRoll) = dFz_dR;
    // transferFunctionJacobian_(StateMemberZ, StateMemberPitch) = dFz_dP;
    // transferFunctionJacobian_(StateMemberRoll, StateMemberRoll) = dFR_dR;
    // transferFunctionJacobian_(StateMemberRoll, StateMemberPitch) = dFR_dP;
    // transferFunctionJacobian_(StateMemberPitch, StateMemberRoll) = dFP_dR;
    // transferFunctionJacobian_(StateMemberYaw, StateMemberRoll) = dFY_dR;
    // transferFunctionJacobian_(StateMemberYaw, StateMemberPitch) = dFY_dP;
    */

    


    // FB_DEBUG("Transfer function is:\n" << transferFunction_ <<
    //          "\nTransfer function Jacobian is:\n" << transferFunctionJacobian_ <<
    //          "\nProcess noise covariance is:\n" << processNoiseCovariance_ <<
    //          "\nCurrent state is:\n" << state_ << "\n");

    Eigen::MatrixXd *processNoiseCovariance = &processNoiseCovariance_;

    //计算动态过程噪声协方差
    if (useDynamicProcessNoiseCovariance_)
    {
      computeDynamicProcessNoiseCovariance(state_, delta);
      processNoiseCovariance = &dynamicProcessNoiseCovariance_;
    }

    FB_DEBUG(
    //         "Transfer function is:\n" << transferFunction_ <<
    //         "\nTransfer function Jacobian is:\n" << transferFunctionJacobian_ <<
    //         "\nProcess noise covariance is:\n" << processNoiseCovariance_ <<
             "\nCurrent state is:\n" << state_ << "\n");

    // (1) Apply control terms, which are actually accelerations
    // 加上控制加速度，没有使用控制，所以是0
    state_(StateMemberVroll) += controlAcceleration_(ControlMemberVroll) * delta;
    state_(StateMemberVpitch) += controlAcceleration_(ControlMemberVpitch) * delta;
    state_(StateMemberVyaw) += controlAcceleration_(ControlMemberVyaw) * delta;

    state_(StateMemberAx) = (controlUpdateVector_[ControlMemberVx] ?
      controlAcceleration_(ControlMemberVx) : state_(StateMemberAx));
    state_(StateMemberAy) = (controlUpdateVector_[ControlMemberVy] ?
      controlAcceleration_(ControlMemberVy) : state_(StateMemberAy));
    state_(StateMemberAz) = (controlUpdateVector_[ControlMemberVz] ?
      controlAcceleration_(ControlMemberVz) : state_(StateMemberAz));

    // (2) Project the state forward: x = Ax + Bu (really, x = f(x, u))
    // 计算预测状态
    //state_ = transferFunction_ * state_;
    state_ = state_ + detastate_;

    // Handle wrapping
    // 保持欧拉角在[-pi,pi]之间
    wrapStateAngles();

    FB_DEBUG("Predicted state is:\n" << state_ << "\n");
             //"\nCurrent estimate error covariance is:\n" <<  estimateErrorCovariance_ << "\n");

    // (3) Project the error forward: P = J * P * J' + dataT * Q
    // 计算预测误差协方差
    // 这里noalias()函数是确定不会出现混淆，不创建中间临时对象，提高速度、降低内存消耗
    // 矩阵乘法可能出现混淆，如matrix1 = matrix1 * matrix2
    //FB_DEBUG("J:\n"<< transferFunctionJacobian_  << "\n");
    //FB_DEBUG("J*P = \n" << transferFunctionJacobian_ * estimateErrorCovariance_ << "\n");
    
    estimateErrorCovariance_ = (transferFunctionJacobian_ *
                                estimateErrorCovariance_ *
                                transferFunctionJacobian_.transpose());
    //FB_DEBUG("J*P*J' = \n" << estimateErrorCovariance_ << "\n");
    estimateErrorCovariance_.noalias() += delta * (*processNoiseCovariance);

    FB_DEBUG(
          "Predicted estimate error covariance is:\n" << estimateErrorCovariance_ <<
             "\n\n--------------------- /Ekf::predict ----------------------\n");
  }

}  // namespace RobotLocalization

