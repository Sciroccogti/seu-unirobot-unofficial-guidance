#ifndef LEPH_TRAJECTORYUTILS_H
#define LEPH_TRAJECTORYUTILS_H

#include <eigen3/Eigen/Dense>
#include "SmoothSpline.hpp"
#include "SplineContainer.hpp"
#include "robot/robot_define.hpp"
namespace Leph
{

    /**
     * Simple typedef for trajectories container
     */
    typedef SplineContainer<SmoothSpline> Trajectories;

    /**
     * Return initialized trajectories for
     * trunk/foot ik cartesian with empty splines
     */
    Trajectories TrajectoriesInit();

    /**
     * Compute from given spline container
     * trajectory Cartesian trunk and foot
     * position/velocity/acceleration
     * and assign it to given vector
     */
    void TrajectoriesTrunkFootPos(
        double t, const Trajectories &traj,
        Eigen::Vector3d &trunkPos,
        Eigen::Vector3d &trunkAxis,
        Eigen::Vector3d &footPos,
        Eigen::Vector3d &footAxis);
    void TrajectoriesTrunkFootVel(
        double t, const Trajectories &traj,
        Eigen::Vector3d &trunkPosVel,
        Eigen::Vector3d &trunkAxisVel,
        Eigen::Vector3d &footPosVel,
        Eigen::Vector3d &footAxisVel);
    void TrajectoriesTrunkFootAcc(
        double t, const Trajectories &traj,
        Eigen::Vector3d &trunkPosAcc,
        Eigen::Vector3d &trunkAxisAcc,
        Eigen::Vector3d &footPosAcc,
        Eigen::Vector3d &footAxisAcc);

    void TrajectoriesSupportFootState(
        double t, const Trajectories &traj,
        bool &isDoubleSupport,
        robot::support_foot &supportFoot);
}

#endif

