use moorebot_scout::motion::{MotionLimits, Velocity};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // [vx, vy, vtheta] = [forward m/s, left m/s, counter-clockwise rad/s].
    let requested = Velocity::from([0.10, 0.0, 0.0]);
    let scout = requested.to_scout_twist(MotionLimits::default())?;

    println!("Requested standard motion: {requested:?}");
    println!("Encoded Scout axes:       {scout:?}");
    println!("Forward maps to Scout linear.y = {}", scout.linear_y);
    Ok(())
}
