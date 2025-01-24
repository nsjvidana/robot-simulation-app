use crate::math::Real;

#[derive(Debug, Error)]
pub enum KError {
    #[error("Tried setting joint position for joint \"{}\", but the joint was a fixed joint.", joint_name)]
    SettingFixedJointPos {
        joint_name: String
    },
    #[error("Joint {0} is set out of limits [min: {1}, max:{2}] with posiiton {3}",
        joint_name,
        min_limit,
        max_limit,
        position
    )]
    OutOfLimits {
        joint_name: String,
        position: Real,
        min_limit: Real,
        max_limit: Real,
    },
    #[error(
        "IK Solver of type {0} tried {1} times but did not converge. position_diff = {2}, angle_diff = {3}",
        solver_type,
        num_tries,
        position_diff,
        angle_diff
    )]
    SolverNotConverged {
        solver_type: String,
        num_tries: usize,
        position_diff: Real,
        angle_diff: Real,
    },
    #[error(
        "Size mismatch. input size = {0}, required size = {1}",
        input_size,
        required_size,
    )]
    SizeMismatchError {
        input_size: usize,
        required_size: usize
    }
}