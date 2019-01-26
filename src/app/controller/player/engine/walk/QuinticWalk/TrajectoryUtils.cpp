#include "TrajectoryUtils.h"
#include "AxisAngle.h"

namespace Leph
{

    Trajectories TrajectoriesInit()
    {
        SplineContainer<SmoothSpline> traj;
        traj.add("is_double_support");
        traj.add("is_left_support_foot");
        traj.add("trunk_pos_x");
        traj.add("trunk_pos_y");
        traj.add("trunk_pos_z");
        traj.add("trunk_axis_x");
        traj.add("trunk_axis_y");
        traj.add("trunk_axis_z");
        traj.add("foot_pos_x");
        traj.add("foot_pos_y");
        traj.add("foot_pos_z");
        traj.add("foot_axis_x");
        traj.add("foot_axis_y");
        traj.add("foot_axis_z");

        return traj;
    }

    void TrajectoriesTrunkFootPos(
        double t, const Trajectories &traj,
        Eigen::Vector3d &trunkPos,
        Eigen::Vector3d &trunkAxis,
        Eigen::Vector3d &footPos,
        Eigen::Vector3d &footAxis)
    {
        //Compute Cartesian positions
        trunkPos = Eigen::Vector3d(
                       traj.get("trunk_pos_x").pos(t),
                       traj.get("trunk_pos_y").pos(t),
                       traj.get("trunk_pos_z").pos(t));
        trunkAxis = Eigen::Vector3d(
                        traj.get("trunk_axis_x").pos(t),
                        traj.get("trunk_axis_y").pos(t),
                        traj.get("trunk_axis_z").pos(t));
        footPos = Eigen::Vector3d(
                      traj.get("foot_pos_x").pos(t),
                      traj.get("foot_pos_y").pos(t),
                      traj.get("foot_pos_z").pos(t));
        footAxis = Eigen::Vector3d(
                       traj.get("foot_axis_x").pos(t),
                       traj.get("foot_axis_y").pos(t),
                       traj.get("foot_axis_z").pos(t));
    }
    void TrajectoriesTrunkFootVel(
        double t, const Trajectories &traj,
        Eigen::Vector3d &trunkPosVel,
        Eigen::Vector3d &trunkAxisVel,
        Eigen::Vector3d &footPosVel,
        Eigen::Vector3d &footAxisVel)
    {
        //Compute Cartesian velocities
        trunkPosVel = Eigen::Vector3d(
                          traj.get("trunk_pos_x").vel(t),
                          traj.get("trunk_pos_y").vel(t),
                          traj.get("trunk_pos_z").vel(t));
        trunkAxisVel = Eigen::Vector3d(
                           traj.get("trunk_axis_x").vel(t),
                           traj.get("trunk_axis_y").vel(t),
                           traj.get("trunk_axis_z").vel(t));
        footPosVel = Eigen::Vector3d(
                         traj.get("foot_pos_x").vel(t),
                         traj.get("foot_pos_y").vel(t),
                         traj.get("foot_pos_z").vel(t));
        footAxisVel = Eigen::Vector3d(
                          traj.get("foot_axis_x").vel(t),
                          traj.get("foot_axis_y").vel(t),
                          traj.get("foot_axis_z").vel(t));
    }
    void TrajectoriesTrunkFootAcc(
        double t, const Trajectories &traj,
        Eigen::Vector3d &trunkPosAcc,
        Eigen::Vector3d &trunkAxisAcc,
        Eigen::Vector3d &footPosAcc,
        Eigen::Vector3d &footAxisAcc)
    {
        //Compute Cartesian accelerations
        trunkPosAcc = Eigen::Vector3d(
                          traj.get("trunk_pos_x").acc(t),
                          traj.get("trunk_pos_y").acc(t),
                          traj.get("trunk_pos_z").acc(t));
        trunkAxisAcc = Eigen::Vector3d(
                           traj.get("trunk_axis_x").acc(t),
                           traj.get("trunk_axis_y").acc(t),
                           traj.get("trunk_axis_z").acc(t));
        footPosAcc = Eigen::Vector3d(
                         traj.get("foot_pos_x").acc(t),
                         traj.get("foot_pos_y").acc(t),
                         traj.get("foot_pos_z").acc(t));
        footAxisAcc = Eigen::Vector3d(
                          traj.get("foot_axis_x").acc(t),
                          traj.get("foot_axis_y").acc(t),
                          traj.get("foot_axis_z").acc(t));
    }

    void TrajectoriesSupportFootState(
        double t, const Trajectories &traj,
        bool &isDoubleSupport,
        robot::support_foot &supportFoot)
    {
        //Compute support foot state
        isDoubleSupport = (
                              traj.get("is_double_support").pos(t) >= 0.5 ?
                              true : false);
        supportFoot = (
                          traj.get("is_left_support_foot").pos(t) >= 0.5 ?
                          robot::LEFT_SUPPORT : robot::RIGHT_SUPPORT);
    }
}

