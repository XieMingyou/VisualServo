/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Defines a robot just to show which function you must implement.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpRobotKawasaki_h
#define vpRobotKawasaki_h

#define PI 3.1415926535897932384626433832795
#define Deg2Rad 0.01745329251994329576923690768489
#define Rad2Deg 57.295779513082320876798154814105

#define ROBOT_DOF 6

/*!
  \file vpRobotKawasaki.h
  Defines a robot just to show which function you must implement.
*/

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/robot/vpRobot.h>

/*!

  \class vpRobotKawasaki
  \ingroup group_robot_real_Kawasaki
  \brief Class that defines a robot just to show which function you must implement.

*/
class vpRobotKawasaki : public vpRobot
{
public:
  vpRobotKawasaki();
  ~vpRobotKawasaki();

  int connect();


  void get_eJe(vpMatrix &eJe);
  void get_fJe(vpMatrix &fJe);

  /*!
    Return constant transformation between end-effector and tool frame.
    If your tool is a camera, this transformation is obtained by hand-eye calibration.
   */
  vpHomogeneousMatrix get_eMc();

  void getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q);
  void getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q);

  /*!
    Set constant transformation between end-effector and tool frame.
    If your tool is a camera, this transformation is obtained by hand-eye calibration.
   */
  void set_eMc(vpHomogeneousMatrix &eMc);
  void setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q);
  void setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);

  vpRobot::vpRobotStateType setRobotState(vpRobot::vpRobotStateType newState);

  vpColVector getMotorVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);
  vpColVector getAxisVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel);

  bool isSingular(const vpColVector &q, vpMatrix &J);

protected:
  void init();
  void getJointPosition(vpColVector &q);
  void setCartVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v);
  void setJointVelocity(const vpColVector &qdot);

  double imPulse = 0.001; //脉冲当量

  //连杆参数
  double a2 = 0.355, d1 = 0.36, d4 = 0.375, d6 = 0.078;

  //零点时实际转过的角度
  double homeTheta6[6] = { 0, 90 * Deg2Rad, 90 * Deg2Rad, 0, 0, 0 };

  //各轴最大最小限位
  double jointMax6[6] = { 180, 135, 155, 200, 125, 360 };
  double jointMin6[6] = { -180, -135, -155, -200, -125, -360 };

  //零点时各轴编码器位置（单位Inc）
  long jointHome6[6] = {103319, 92992, 116630, 31953, 111221, 91157};

  //各轴减速比
  double reductionRatio6[6] = {80.008, 99.902, 78.433, 50.001, 64.001, 40.000};

  //各轴的运动方向
  int direction6[6] = {1, 1, -1, 1, -1, 1};

  //编码器分辨率
  long encoderResolution = 131072;
  //long encoderResolution = 8388608;

  vpHomogeneousMatrix m_eMc; //!< Constant transformation between end-effector and tool (or camera) frame
};
#endif
