use moorebot_scout::motion::{MotionLimits, Velocity};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let requested = Velocity {
        forward_mps: 0.10,
        lateral_mps: 0.0,
        yaw_rps: 0.0,
    };
    let scout = requested.to_scout_twist(MotionLimits::default())?;

    println!("Requested standard motion: {requested:?}");
    println!("Encoded Scout axes:       {scout:?}");
    println!("Forward maps to Scout linear.y = {}", scout.linear_y);
    Ok(())
}
