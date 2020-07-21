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
 * Fabien Spindler
 *
 *****************************************************************************/

#include <fstream>

#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotException.h>

/*!
  \file vpRobotKawasaki.cpp
  Defines a robot just to show which function you must implement.
*/

#include <visp3/core/vpHomogeneousMatrix.h>
#include <vpRobotKawasaki.h>
#include <IPMCMOTION.h>

using namespace std;
/*!
  Basic initialization.
 */
void vpRobotKawasaki::init()
{
  // If you want to control the robot in Cartesian in a tool frame, set the corresponding transformation in m_eMc
  // that is set to identity by default in the constructor.

  vpRobot::maxRotationVelocity = 0.2;   // maxRotationVelocityDefault=0.2;
  vpRobot::maxTranslationVelocity = 0.7;// maxTranslationVelocityDefault=0.7;

  // Set here the robot degrees of freedom number
  vpRobot::nDof = ROBOT_DOF; // If your arm has 6 dof
}

/*!
  Default constructor.
 */
vpRobotKawasaki::vpRobotKawasaki() { vpRobotKawasaki::init(); }

/*!
  Destructor.
 */
vpRobotKawasaki::~vpRobotKawasaki()
{
  vpRobotKawasaki::setRobotState(vpRobot::STATE_STOP);
  IPMCCloseDevice();
}

void vpRobotKawasaki::set_eMc(vpHomogeneousMatrix &eMc) { m_eMc = eMc; }

vpHomogeneousMatrix vpRobotKawasaki::get_eMc() { return m_eMc; }

//连接机器人控制器，打开软卡
int vpRobotKawasaki::connect()
{
  IPMCOpenDevice();
  bool enable = false;
  double time_initial = vpTime::measureTimeSecond();
  while (!enable) {
    //判断驱动器是否已经使能
    unsigned long Value[ROBOT_DOF];
    for (int i = 0; i < ROBOT_DOF; i++) {
      IPMCGetDriverState(i, &Value[i]);
    }
    if (Value[0] == 8 && Value[1] == 8 && Value[2] == 8 && Value[3] == 8 && Value[4] == 8 && Value[5] == 8) {
      enable = true;
    }
	
	double time_connect= vpTime::measureTimeSecond();
	if ((time_connect - time_initial) > 30)
	{
		IPMCCloseDevice();
		return EXIT_FAILURE;
	}
  }
  return EXIT_SUCCESS;
}


/*

  At least one of these function has to be implemented to control the robot with a
  Cartesian velocity:
  - get_eJe()
  - get_fJe()

*/

/*!
  Get the robot Jacobian expressed in the end-effector frame.

  \param[out] eJe : End-effector frame Jacobian.
*/
void vpRobotKawasaki::get_eJe(vpMatrix &eJe)
{
  eJe.resize(6, ROBOT_DOF);
  vpColVector q(ROBOT_DOF);
  vpRobotKawasaki::getJointPosition(q);

  //求变换矩阵
  vpMatrix T01(4, 4), T12(4, 4), T23(4, 4), T34(4, 4), T45(4, 4), T56(4, 4);
  T01[0][0] = cos(q[0]);
  T01[0][1] = -sin(q[0]);
  T01[1][0] = sin(q[0]);
  T01[1][1] = cos(q[0]);
  T01[2][2] = 1;
  T01[2][3] = d1;
  T01[3][3] = 1;
  T12[0][0] = cos(q[1]);
  T12[0][1] = -sin(q[1]);
  T12[1][2] = -1;
  T12[2][0] = sin(q[1]);
  T12[2][1] = cos(q[1]);
  T12[3][3] = 1;
  T23[0][0] = cos(q[2]);
  T23[0][1] = -sin(q[2]);
  T23[0][3] = a2;
  T23[1][0] = sin(q[2]);
  T23[1][1] = cos(q[2]);
  T23[2][2] = 1;
  T23[3][3] = 1;
  T34[0][0] = cos(q[3]);
  T34[0][1] = -sin(q[3]);
  T34[1][2] = -1;
  T34[1][3] = -d4;
  T34[2][0] = sin(q[3]);
  T34[2][1] = cos(q[3]);
  T34[3][3] = 1;
  T45[0][0] = cos(q[4]);
  T45[0][1] = -sin(q[4]);
  T45[1][2] = 1;
  T45[2][0] = -sin(q[4]);
  T45[2][1] = -cos(q[4]);
  T45[3][3] = 1;
  T56[0][0] = cos(q[5]);
  T56[0][1] = -sin(q[5]);
  T56[1][2] = -1;
  T56[1][3] = -d6;
  T56[2][0] = sin(q[5]);
  T56[2][1] = cos(q[5]);
  T56[3][3] = 1;

  //矢量积法求雅克比矩阵
  vpMatrix R01(T01, 0, 0, 3, 3), R12(T12, 0, 0, 3, 3), R23(T23, 0, 0, 3, 3), R34(T34, 0, 0, 3, 3), R45(T45, 0, 0, 3, 3), R56(T56, 0, 0, 3, 3);
  vpMatrix T46 = T45 * T56, T36 = T34 * T46, T26 = T23 * T36, T16 = T12 * T26;
  vpMatrix R02 = R01 * R12, R03 = R02 * R23, R04  =R03 * R34, R05 = R04 * R45, R06 = R05 * R56;
  
  
  vpColVector z(3), z1(3), z2(3), z3(3), z4(3), z5(3), z6(3);
  z[2] = 1;
  z1 = R01 * z;
  z2 = R02 * z;
  z3 = R03 * z;
  z4 = R04 * z;
  z5 = R05 * z;
  z6 = R06 * z;

  vpColVector t(4), t16(4), t26(4), t36(4), t46(4), t56(4), t66(4);
  t[3] = 1;
  t16 = T16 * t;
  t26 = T26 * t;
  t36 = T36 * t;
  t46 = T46 * t;
  t56 = T56 * t;
  t66 = t;

  vpMatrix p16(t16, 0, 0, 3, 1), p26(t26, 0, 0, 3, 1), p36(t36, 0, 0, 3, 1), p46(t46, 0, 0, 3, 1), p56(t56, 0, 0, 3, 1), p66(t66, 0, 0, 3, 1);
  vpColVector p16_0(3), p26_0(3), p36_0(3), p46_0(3), p56_0(3), p66_0(3);
  p16_0 = R01 * p16;
  p26_0 = R02 * p26;
  p36_0 = R03 * p36;
  p46_0 = R04 * p46;
  p56_0 = R05 * p56;
  p66_0 = R06 * p66;

  vpColVector JL1(3), JL2(3), JL3(3), JL4(3), JL5(3), JL6(3);
  //JL1[0] = z1[1] * p16_0[2] - p16_0[1] * z1[2];
  //JL1[1] = z1[2] * p16_0[0] - p16_0[2] * z1[0];
  //JL1[2] = z1[0] * p16_0[1] - p16_0[0] * z1[1];
  //JL2[0] = z2[1] * p26_0[2] - p26_0[1] * z2[2];
  //JL2[1] = z2[2] * p26_0[0] - p26_0[2] * z2[0];
  //JL2[2] = z2[0] * p26_0[1] - p26_0[0] * z2[1];
  //JL3[0] = z3[1] * p36_0[2] - p36_0[1] * z3[2];
  //JL3[1] = z3[2] * p36_0[0] - p36_0[2] * z3[0];
  //JL3[2] = z3[0] * p36_0[1] - p36_0[0] * z3[1];
  //JL4[0] = z4[1] * p46_0[2] - p46_0[1] * z4[2];
  //JL4[1] = z4[2] * p46_0[0] - p46_0[2] * z4[0];
  //JL4[2] = z4[0] * p46_0[1] - p46_0[0] * z4[1];
  //JL5[0] = z5[1] * p56_0[2] - p56_0[1] * z5[2];
  //JL5[1] = z5[2] * p56_0[0] - p56_0[2] * z5[0];
  //JL5[2] = z5[0] * p56_0[1] - p56_0[0] * z5[1];
  //JL6[0] = z6[1] * p66_0[2] - p66_0[1] * z6[2];
  //JL6[1] = z6[2] * p66_0[0] - p66_0[2] * z6[0];
  //JL6[2] = z6[0] * p66_0[1] - p66_0[0] * z6[1];

  JL1 = vpColVector::cross(z1, p16_0);
  JL2 = vpColVector::cross(z2, p26_0);
  JL3 = vpColVector::cross(z3, p36_0);
  JL4 = vpColVector::cross(z4, p46_0);
  JL5 = vpColVector::cross(z5, p56_0);
  JL6 = vpColVector::cross(z6, p66_0);

  vpMatrix fJe(6, ROBOT_DOF);

  for (int i = 0; i < 3; i++) {
    fJe[i][0] = JL1[i];
    fJe[i][1] = JL2[i];
    fJe[i][2] = JL3[i];
    fJe[i][3] = JL4[i];
    fJe[i][4] = JL5[i];
    fJe[i][5] = JL6[i];
    fJe[i + 3][0] = z1[i];
    fJe[i + 3][1] = z2[i];
    fJe[i + 3][2] = z3[i];
    fJe[i + 3][3] = z4[i];
    fJe[i + 3][4] = z5[i];
    fJe[i + 3][5] = z6[i];
  }

  vpMatrix R06_t = R06.t();

  for (int i = 0; i < 3; i++)
  {
	  for (int j = 0; j < 3; j++)
	  {
		  eJe[i][j] = R06_t[i][j];
		  eJe[i+3][j+3]= R06_t[i][j];
	  }
  }

  eJe = eJe * fJe;

  //eJe[0][0] = d6 * (sin(q[4])*(cos(q[0])*sin(q[3]) + cos(q[3])*(sin(q[0])*sin(q[1])*sin(q[2]) - cos(q[1])*cos(q[2])*sin(q[0]))) - cos(q[4])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1]))) - d4 * (cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) - a2 * cos(q[1])*sin(q[0]);
  //eJe[0][1] = -d4 * (cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - d6 * (cos(q[4])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[3])*sin(q[4])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1]))) - a2 * cos(q[0])*sin(q[1]);
  //eJe[0][2] = -d4 * (cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) - d6 * (cos(q[4])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])) + cos(q[3])*sin(q[4])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])));
  //eJe[0][3] = d6 * sin(q[4])*(cos(q[3])*sin(q[0]) + sin(q[3])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2])));
  //eJe[0][4] = d6 * (cos(q[4])*(sin(q[0])*sin(q[3]) - cos(q[3])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2]))) - sin(q[4])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])));
  //eJe[0][5] = 0;
  //eJe[1][0] = d6 * (sin(q[4])*(sin(q[0])*sin(q[3]) - cos(q[3])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2]))) + cos(q[4])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1]))) + d4 * (cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1])) + a2 * cos(q[0])*cos(q[1]);
  //eJe[1][1] = -d4 * (sin(q[0])*sin(q[1])*sin(q[2]) - cos(q[1])*cos(q[2])*sin(q[0])) - d6 * (cos(q[4])*(sin(q[0])*sin(q[1])*sin(q[2]) - cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[4])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1]))) - a2 * sin(q[0])*sin(q[1]);
  //eJe[1][2] = -d4 * (sin(q[0])*sin(q[1])*sin(q[2]) - cos(q[1])*cos(q[2])*sin(q[0])) - d6 * (cos(q[4])*(sin(q[0])*sin(q[1])*sin(q[2]) - cos(q[1])*cos(q[2])*sin(q[0])) + cos(q[3])*sin(q[4])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])));
  //eJe[1][3] = -d6 * sin(q[4])*(cos(q[0])*cos(q[3]) - sin(q[3])*(sin(q[0])*sin(q[1])*sin(q[2]) - cos(q[1])*cos(q[2])*sin(q[0])));
  //eJe[1][4] = -d6 * (cos(q[4])*(cos(q[0])*sin(q[3]) + cos(q[3])*(sin(q[0])*sin(q[1])*sin(q[2]) - cos(q[1])*cos(q[2])*sin(q[0]))) + sin(q[4])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])));
  //eJe[1][5] = 0;
  //eJe[2][0] = 0;
  //eJe[2][1] = d4 * (cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) + a2 * cos(q[1]) + d6 * (cos(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) + cos(q[3])*sin(q[4])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2])));
  //eJe[2][2] = d4 * (cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) + d6 * (cos(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) + cos(q[3])*sin(q[4])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2])));
  //eJe[2][3] = -d6 * sin(q[3])*sin(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1]));
  //eJe[2][4] = d6 * (sin(q[4])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2])) + cos(q[3])*cos(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])));
  //eJe[2][5] = 0;
  //eJe[3][0] = 0;
  //eJe[3][1] = sin(q[0]);
  //eJe[3][2] = sin(q[0]);
  //eJe[3][3] = cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1]);
  //eJe[3][4] = cos(q[3])*sin(q[0]) + sin(q[3])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2]));
  //eJe[3][5] = sin(q[4])*(sin(q[0])*sin(q[3]) - cos(q[3])*(cos(q[0])*sin(q[1])*sin(q[2]) - cos(q[0])*cos(q[1])*cos(q[2]))) + cos(q[4])*(cos(q[0])*cos(q[1])*sin(q[2]) + cos(q[0])*cos(q[2])*sin(q[1]));
  //eJe[4][0] = 0;
  //eJe[4][1] = -cos(q[0]);
  //eJe[4][2] = -cos(q[0]);
  //eJe[4][3] = cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1]);
  //eJe[4][4] = sin(q[3])*(sin(q[0])*sin(q[1])*sin(q[2]) - cos(q[1])*cos(q[2])*sin(q[0])) - cos(q[0])*cos(q[3]);
  //eJe[4][5] = cos(q[4])*(cos(q[1])*sin(q[0])*sin(q[2]) + cos(q[2])*sin(q[0])*sin(q[1])) - sin(q[4])*(cos(q[0])*sin(q[3]) + cos(q[3])*(sin(q[0])*sin(q[1])*sin(q[2]) - cos(q[1])*cos(q[2])*sin(q[0])));
  //eJe[5][0] = 1;
  //eJe[5][1] = 0;
  //eJe[5][2] = 0;
  //eJe[5][3] = sin(q[1])*sin(q[2]) - cos(q[1])*cos(q[2]);
  //eJe[5][4] = -sin(q[3])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1]));
  //eJe[5][5] = cos(q[3])*sin(q[4])*(cos(q[1])*sin(q[2]) + cos(q[2])*sin(q[1])) - cos(q[4])*(cos(q[1])*cos(q[2]) - sin(q[1])*sin(q[2]));

  //vpMatrix kerAt;
  //if (eJe.kernel(kerAt, 0.003)!=6)
  //{
	 // cout << "eJe is singular." << endl;
  //}
}

/*!
  Get the robot Jacobian expressed in the robot reference frame.

  \param[out] fJe : Base (or reference) frame Jacobian.
*/
void vpRobotKawasaki::get_fJe(vpMatrix &fJe)
{
  (void)fJe;
  std::cout << "Not implemented ! " << std::endl;
}

/*

  At least one of these function has to be implemented to control the robot:
  - setCartVelocity()
  - setJointVelocity()

*/

/*!
  Send to the controller a 6-dim velocity twist vector expressed in a Cartesian frame.

  \param[in] frame : Cartesian control frame (either tool frame or end-effector) in which the velocity \e v is
  expressed. Units are m/s for translation and rad/s for rotation velocities.

  \param[in] v : 6-dim vector that contains the 6 components of the velocity twist to send to the robot.
  Units are m/s and rad/s.
*/
void vpRobotKawasaki::setCartVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &v)
{
  if (v.size() != 6) {
    throw(vpException(vpException::fatalError,
                      "Cannot send a velocity twist vector in tool frame that is not 6-dim (%d)", v.size()));
  }

  vpColVector v_e; // This is the velocity that the robot is able to apply in the end-effector frame
  switch (frame) {
  case vpRobot::TOOL_FRAME: {
    // We have to transform the requested velocity in the end-effector frame.
    // Knowing that the constant transformation between the tool frame and the end-effector frame obtained
    // by extrinsic calibration is set in m_eMc we can compute the velocity twist matrix eVc that transform
    // a velocity twist from tool (or camera) frame into end-effector frame
    vpVelocityTwistMatrix eVc(m_eMc);	
	v_e = eVc * v;
    break;
  }

  case vpRobot::END_EFFECTOR_FRAME:
  case vpRobot::REFERENCE_FRAME: {
    v_e = v;
    break;
  }
  case vpRobot::JOINT_STATE:
  case vpRobot::MIXT_FRAME:
    // Out of the scope
    break;
  }

  // Implement your stuff here to send the end-effector velocity twist v_e
  // - If the SDK allows to send cartesian velocities in the end-effector, it's done. Just wrap data in v_e
  // - If the SDK allows to send cartesian velocities in the reference (or base) frame you have to implement
  //   the robot Jacobian in set_fJe() and call:
  //   vpColVector v = get_fJe().inverse() * v_e;
  //   At this point you have to wrap data in v that is the 6-dim velocity to apply to the robot
  // - If the SDK allows to send only joint velocities you have to implement the robot Jacobian in set_eJe()
  //   and call:
  //   vpColVector qdot = get_eJe().inverse() * v_e;
  //   setJointVelocity(qdot);
  // - If the SDK allows to send only a cartesian position trajectory of the end-effector position in the base frame
  //   called fMe (for fix frame to end-effector homogeneous transformation) you can transform the cartesian
  //   velocity in the end-effector into a displacement eMe using the exponetial map:
  //   double delta_t = 0.010; // in sec
  //   vpHomogenesousMatrix eMed = vpExponentialMap::direct(v_e, delta_t);
  //   vpHomogenesousMatrix fMe = getPosition(vpRobot::REFERENCE_FRAME);
  //   the new position to reach is than given by fMe * eMed
  //   vpColVector fped(vpPoseVector(fMe * eMed));
  //   setPosition(vpRobot::REFERENCE_FRAME, fped);

  // std::cout << "Not implemented ! " << std::endl;
  // std::cout << "To implement me you need : " << std::endl;
  // std::cout << "\t to knKawasaki the robot jacobian expressed in ";
  // std::cout << "the end-effector frame (eJe) " << std::endl;
  // std::cout << "\t the frame transformation  between tool or camera frame ";
  // std::cout << "and end-effector frame (cMe)" << std::endl;

  vpMatrix eJe;
  vpRobotKawasaki::get_eJe(eJe);
  eJe = eJe.inverseByLUEigen3();

  vpColVector q(ROBOT_DOF);
  vpRobotKawasaki::getJointPosition(q);
  isSingular(q, eJe);

  vpColVector qdot = eJe * v_e;

  vpRobotKawasaki::setJointVelocity(qdot);
}

/*!
  Send a joint velocity to the controller.
  \param[in] qdot : Joint velocities vector. Units are rad/s for a robot arm.
 */
void vpRobotKawasaki::setJointVelocity(const vpColVector &qdot)
{
  // Implement your stuff here to send the joint velocities qdot
  //ofstream out("MotorPulse.txt", ios::app);
  for (int i = 0; i < ROBOT_DOF; i++) {
    long velocity2pulse = (long)((qdot[i] * direction6[i] * reductionRatio6[i] * encoderResolution) / (2 * PI) );
	//out << velocity2pulse << "  ";
	IPMCSetVelCommand(i, velocity2pulse);
  }
  //out << endl;
  //out.close();
}

/*!
  Send to the controller a velocity in a given frame.

  \param[in] frame : Control frame in which the velocity \e vel is expressed.
  Velocities could be joint velocities, or cartesian velocities. Units are m/s for translation and
  rad/s for rotation velocities.

  \param[in] vel : Vector that contains the velocity to apply to the robot.
 */
void vpRobotKawasaki::setVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
  if (vpRobot::STATE_VELOCITY_CONTROL != vpRobot::getRobotState()) {
    throw vpRobotException(vpRobotException::wrongStateError,
                           "Cannot send a velocity to the robot. "
                           "Call setRobotState(vpRobot::STATE_VELOCITY_CONTROL) once before "
                           "entering your control loop.");
  }

  vpColVector vel_sat(6);

  // Velocity saturation
  switch (frame) {
  // Saturation in cartesian space
  case vpRobot::TOOL_FRAME:
  case vpRobot::REFERENCE_FRAME:
  case vpRobot::END_EFFECTOR_FRAME:
  case vpRobot::MIXT_FRAME: {
    if (vel.size() != 6) {
      throw(vpException(vpException::dimensionError,
                        "Cannot apply a Cartesian velocity that is not a 6-dim vector (%d)", vel.size()));
    }
    vpColVector vel_max(6);

    for (unsigned int i = 0; i < 3; i++)
      vel_max[i] = vpRobot::getMaxTranslationVelocity();
    for (unsigned int i = 3; i < 6; i++)
      vel_max[i] = vpRobot::getMaxRotationVelocity();

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);

    vpRobotKawasaki::setCartVelocity(frame, vel_sat);
    break;
  }
  // Saturation in joint space
  case vpRobot::JOINT_STATE: {
    if (vel.size() != static_cast<size_t>(nDof)) {
      throw(vpException(vpException::dimensionError, "Cannot apply a joint velocity that is not a %-dim vector (%d)",
                        nDof, vel.size()));
    }
    vpColVector vel_max(vel.size());

    // Since the robot has only rotation axis all the joint max velocities are set to getMaxRotationVelocity()
    vel_max = vpRobot::getMaxRotationVelocity();

    vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);

    vpRobotKawasaki::setJointVelocity(vel_sat);
  }
  }
}

/*

  THESE FUNCTIONS ARE NOT MENDATORY BUT ARE USUALLY USEFUL

*/

/*!
  Get robot joint positions.

  \param[in] q : Joint velocities in rad/s.
 */
void vpRobotKawasaki:: getJointPosition(vpColVector &q)
{
  long dJointCurrentPos[ROBOT_DOF] = {0};

  for (int i = 0; i < ROBOT_DOF; i++) {
    IPMCGetDriverPos(i, &dJointCurrentPos[i]);
  }

  for (int i = 0; i < ROBOT_DOF; i++) {
	  q[i] = ((dJointCurrentPos[i] - jointHome6[i]) * direction6[i] * 2 * PI) / (encoderResolution  * reductionRatio6[i]) + homeTheta6[i];
  }
  q[5] = 0.01248916 * q[4] + q[5];
}

/*!
  Get robot position.

  \param[in] frame : Considered cartesian frame or joint state.
  \param[out] q : Position of the arm.
 */
void vpRobotKawasaki::getPosition(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  if (frame == JOINT_STATE) {
    vpRobotKawasaki::getJointPosition(q);
  } else {
    std::cout << "Not implemented ! " << std::endl;
  }
}

/*!
  Set a position to reach.

  \param[in] frame : Considered cartesian frame or joint state.
  \param[in] q : Position to reach.
 */
void vpRobotKawasaki::setPosition(const vpRobot::vpControlFrameType frame, const vpColVector &q)
{
  (void)frame;
  (void)q;
  std::cout << "Not implemented ! " << std::endl;
}

/*!
  Get a displacement.

  \param[in] frame : Considered cartesian frame or joint state.
  \param[out] q : Displacement in meter and rad.
 */
void vpRobotKawasaki::getDisplacement(const vpRobot::vpControlFrameType frame, vpColVector &q)
{
  (void)frame;
  (void)q;
  std::cout << "Not implemented ! " << std::endl;
}

vpRobot::vpRobotStateType vpRobotKawasaki::setRobotState(vpRobot::vpRobotStateType newState)
{
  switch (newState) {
  case vpRobot::STATE_STOP: {
    if (vpRobot::STATE_VELOCITY_CONTROL == vpRobot::getRobotState()) {
	  std::cout << "Stop the robot from velocity control." << std::endl;
      for (int i = 0; i < ROBOT_DOF; i++) {
        IPMCSetVelCommand(i, 0);
      }
    } else if (vpRobot::STATE_POSITION_CONTROL == vpRobot::getRobotState()) {
	  std::cout << "Stop the robot from position control." << std::endl;
	  IPMCStopAllAxis(0);
    }
	long pos[6];
	for (int i = 0; i < ROBOT_DOF; i++) {
		IPMCGetDriverPos(i, &pos[i]);
		IPMCSetAxisPosition(i, pos[i]);
		Sleep(100);
		IPMCSetAxisCommandMode(i, 0);
	}
    break;
  }
  case vpRobot::STATE_POSITION_CONTROL: {
    if (vpRobot::STATE_STOP == vpRobot::getRobotState()) {
      std::cout << "Change the control mode from stop to position control." << std::endl;
    } 
	else if (vpRobot::STATE_VELOCITY_CONTROL == vpRobot::getRobotState()) {
      std::cout << "Change the control mode from velocity to position control." << std::endl;
	  for (int i = 0; i < ROBOT_DOF; i++) {
		  IPMCSetVelCommand(i, 0);
	  }
    }
    long pos[6];
	for (int i = 0; i < ROBOT_DOF; i++) {
      IPMCGetDriverPos(i, &pos[i]);
      IPMCSetAxisPosition(i, pos[i]);
	  Sleep(1000);
      IPMCSetAxisCommandMode(i, 0);
    }
    break;
  }
  case vpRobot::STATE_VELOCITY_CONTROL: {
    if (vpRobot::STATE_STOP == vpRobot::getRobotState()) {
      std::cout << "Change the control mode from stop to velocity control." << std::endl;
    } 
	else if (vpRobot::STATE_POSITION_CONTROL == vpRobot::getRobotState()) {
      std::cout << "Change the control mode from position to velocity control." << std::endl;
	  IPMCStopAllAxis(0);
    }
	long pos[6];
	for (int i = 0; i < ROBOT_DOF; i++) {
		IPMCSetVelCommand(i, 0);
	}
	for (int i = 0; i < ROBOT_DOF; i++) {
	  Sleep(1000);
	  IPMCSetAxisCommandMode(i, 1);
    }
	Sleep(1000);
    break;
  }
  default:
    break;
  }

  return vpRobot::setRobotState(newState);
}

vpColVector vpRobotKawasaki::getAxisVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
	if (vpRobot::STATE_VELOCITY_CONTROL != vpRobot::getRobotState()) {
		throw vpRobotException(vpRobotException::wrongStateError,
			"Cannot send a velocity to the robot. "
			"Call setRobotState(vpRobot::STATE_VELOCITY_CONTROL) once before "
			"entering your control loop.");
	}

	vpColVector vel_sat(6);

	// Velocity saturation
	switch (frame) {
	// Saturation in cartesian space
	case vpRobot::TOOL_FRAME:
	case vpRobot::REFERENCE_FRAME:
	case vpRobot::END_EFFECTOR_FRAME:
	case vpRobot::MIXT_FRAME: {
		if (vel.size() != 6) {
			throw(vpException(vpException::dimensionError,
				"Cannot apply a Cartesian velocity that is not a 6-dim vector (%d)", vel.size()));
		}
		vpColVector vel_max(6);

		for (unsigned int i = 0; i < 3; i++)
			vel_max[i] = vpRobot::getMaxTranslationVelocity();
		for (unsigned int i = 3; i < 6; i++)
			vel_max[i] = vpRobot::getMaxRotationVelocity();

		vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);

		vpColVector v_e(6);
		vpVelocityTwistMatrix eVc(m_eMc);

		if (frame == vpRobot::TOOL_FRAME)
		{
			v_e = eVc * vel_sat;
	    }
		else
		{
			v_e = vel_sat;
		}

		vpMatrix eJe;
		vpRobotKawasaki::get_eJe(eJe);
		eJe = eJe.inverseByLUEigen3();
		
		//vpColVector q(ROBOT_DOF);
		//vpRobotKawasaki::getJointPosition(q);
		//isSingular(q, eJe);

		vpColVector qdot_Axis = eJe * v_e;

		return (qdot_Axis * Rad2Deg);
	}
    // Saturation in joint space
	case vpRobot::JOINT_STATE: {
		if (vel.size() != static_cast<size_t>(nDof)) {
			throw(vpException(vpException::dimensionError, "Cannot apply a joint velocity that is not a %-dim vector (%d)",
				nDof, vel.size()));
		}
		vpColVector vel_max(vel.size());

		// Since the robot has only rotation axis all the joint max velocities are set to getMaxRotationVelocity()
		vel_max = vpRobot::getMaxRotationVelocity();

		vel_sat = vpRobot::saturateVelocities(vel, vel_max, true);

		vpColVector qdot_Axis = vel_sat;
		
		return (qdot_Axis * Rad2Deg);
	}
	}

}

vpColVector vpRobotKawasaki::getMotorVelocity(const vpRobot::vpControlFrameType frame, const vpColVector &vel)
{
	vpColVector qdot_Axis = getAxisVelocity(frame, vel);
	vpColVector qdot_Motor(ROBOT_DOF);
	for (int i = 0; i < ROBOT_DOF; i++) 
	{
		qdot_Motor[i] = (qdot_Axis[i] * direction6[i] * reductionRatio6[i]);
	}

	return (qdot_Axis * Rad2Deg);
}

bool vpRobotKawasaki::isSingular(const vpColVector &q, vpMatrix &J)
{
	double q2 = q[1];
	double q3 = q[2];
	double q5 = q[4];
	
	double c2 = cos(q2);
	double c3 = cos(q3);
	double s23 = sin(q2 + q3);
	double s5 = sin(q5);
	
	bool cond1 = fabs(s5) < 1e-1;
	bool cond2 = fabs(c3) < 1e-1;
	bool cond3 = fabs(c2 * a2 + s23 * d4) < 1e-1;

	if (cond1) 
	{
		J[3][0] = 0;
		J[5][0] = 0;
		J[3][1] = 0;
		J[5][1] = 0;
		J[3][2] = 0;
		J[5][2] = 0;
		J[3][3] = 0;
		J[5][3] = 0;
		J[3][4] = 0;
		J[5][4] = 0;
		J[3][5] = 0;
		J[5][5] = 0;
		return true;	
	}
	if (cond2)
	{
		J[1][0] = 0;
		J[2][0] = 0;
		J[3][0] = 0;
		J[4][0] = 0;
		J[5][0] = 0;
		J[1][1] = 0;
		J[2][1] = 0;
		J[3][1] = 0;
		J[4][1] = 0;
		J[5][1] = 0;
		J[1][2] = 0;
		J[2][2] = 0;
		J[3][2] = 0;
		J[4][2] = 0;
		J[5][2] = 0;
		return true;
	}
	if (cond3) 
	{
		J[0][0] = 0;
		J[3][0] = 0;
		J[4][0] = 0;
		J[5][0] = 0;
		J[0][1] = 0;
		J[3][1] = 0;
		J[4][1] = 0;
		J[5][1] = 0;
		return true;
	}
	return false;
}