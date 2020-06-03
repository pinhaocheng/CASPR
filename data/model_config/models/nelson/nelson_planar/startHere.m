% % load robot
clear;
model_config = ModelConfig('nelson_planar');
cdpr_model = model_config.getModel('basic_upper_only');
MotionSimulatorBase.PlotFrame(cdpr_model, model_config.displayRange, model_config.viewAngle);

% % load trajectory
trajectory_linear = model_config.getJointTrajectory('example_linear');

% % plot pos, vel, and accel
% trajectory_linear.plotJointPose();
% trajectory_linear.plotJointVelocity();
% trajectory_linear.plotJointAcceleration();

min_forces_objective = IDObjectiveMinQuadCableForce(ones(cdpr_model.numActuatorsActive,1));
id_solver = IDSolverQuadProg(cdpr_model, min_forces_objective, ID_QP_SolverType.MATLAB);

id_sim = InverseDynamicsSimulator(cdpr_model, id_solver);
id_sim.run(trajectory_linear);
id_sim.plotCableForces();
