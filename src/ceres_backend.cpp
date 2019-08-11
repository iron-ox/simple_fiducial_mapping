#include <simple_fiducial_mapping/ceres_backend.h>

namespace simple_fiducial_mapping
{
OptimizationInfo bundleAdjust(int max_iterations, MapGraph* map_graph)
{
  // We dynamically allocate them because Ceres will free them when it is done solving.
  auto position_parameterization = new ceres::IdentityParameterization(3);
  auto orientation_parameterization = new ceres::EigenQuaternionParameterization;

  ceres::Problem::Options problem_options;

  ceres::Problem problem(problem_options);

  // Create parameter blocks for the camera poses.
  for (auto& camera_pose_pair : map_graph->camera_poses)
  {
    auto& camera_position = camera_pose_pair.second.position;
    problem.AddParameterBlock(camera_position.data(), camera_position.size());
    problem.SetParameterization(camera_position.data(), position_parameterization);

    auto& camera_orientation = camera_pose_pair.second.orientation;
    problem.AddParameterBlock(camera_orientation.coeffs().data(), camera_orientation.coeffs().size());
    problem.SetParameterization(camera_orientation.coeffs().data(), orientation_parameterization);

    if (camera_pose_pair.second.is_constant)
    {
      problem.SetParameterBlockConstant(camera_position.data());
      problem.SetParameterBlockConstant(camera_orientation.coeffs().data());
    }
    else
    {
      // We assume that the camera is fixed to the robot, and the robot is moving on a plane. Restricting Z
      // leads to a ground plane that makes sense.
      problem.SetParameterLowerBound(camera_position.data(), 2, -0.05);
      problem.SetParameterUpperBound(camera_position.data(), 2, 0.05);
    }
  }

  // Create parameter blocks for the fiducial positions.
  for (auto& fiducial_pair : map_graph->fiducial_positions)
  {
    auto& fiducial_position = fiducial_pair.second;
    problem.AddParameterBlock(fiducial_position.position.data(), fiducial_position.position.size());
    if (fiducial_position.is_constant)
    {
      problem.SetParameterBlockConstant(fiducial_position.position.data());
    }
    else
    {
      // Tags are up on the ceiling.
      problem.SetParameterLowerBound(fiducial_position.position.data(), 2, 1.0);
    }
  }

  // Create residual functions for each observation.
  std::map<size_t, ceres::ResidualBlockId> observations_to_residuals;
  for (const auto& observation_pair : map_graph->observations)
  {
    size_t observation_id = observation_pair.first;
    const Observation& observation = observation_pair.second;
    CameraPose& camera_pose = map_graph->camera_poses[observation_pair.second.camera_pose_id];
    FiducialPosition& fiducial_position = map_graph->fiducial_positions[observation_pair.second.fiducial_id];

    /**
    * Create the cost function.
    *
    * Note: We could cache these error functions instead of re-allocating them each time we optimize.
    *
    * The numerical template arguments here are:
    * 3 - Number of residuals (x, y, and z errors in this case)
    * 4 - Size of camera rotation parameter (quaternion).
    * 3 - Size of camera translation parameter
    * 3 - Size of fiducial_position parameter (x, y, z of fiducial center)
    */
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CartesianCostFunction, 3, 4, 3, 3>(
        new CartesianCostFunction(observation.position));

    // If a fiducial is more than 0.4 meters from where it should be along any dimension, it is almost certainly an
    // outlier. This will reduce its influence.
    ceres::LossFunction* loss_function = new ceres::CauchyLoss(0.4);

    // Add cost function to the problem.
    observations_to_residuals[observation_id] =
        problem.AddResidualBlock(cost_function, loss_function, camera_pose.orientation.coeffs().data(),
                                 camera_pose.position.data(), fiducial_position.position.data());
  }

  ceres::Solver::Options solver_options;
  solver_options.linear_solver_type = ceres::DENSE_SCHUR;
  solver_options.max_num_iterations = max_iterations;
  solver_options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);

  OptimizationInfo optimization_info;
  optimization_info.computation_time = summary.total_time_in_seconds;
  for (const auto& residual_block_pair : observations_to_residuals)
  {
    size_t observation_id = residual_block_pair.first;
    const ceres::ResidualBlockId& residual_block_id = residual_block_pair.second;

    ceres::Problem::EvaluateOptions get_residuals_options;
    get_residuals_options.residual_blocks = { residual_block_id };
    double total_cost;
    std::vector<double> residuals;
    problem.Evaluate(get_residuals_options, &total_cost, &residuals, nullptr, nullptr);
    assert(residuals.size() == 3);

    optimization_info.residuals[observation_id] = Eigen::Vector3d(residuals[0], residuals[1], residuals[2]);
  }
  return optimization_info;
}
}
