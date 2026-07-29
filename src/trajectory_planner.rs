/// VLA Torque Sentinel — Minimum-Jerk Trajectory Planner
/// Computes smooth 5th-order polynomial trajectories that minimize
/// jerk (derivative of acceleration) for safe robotic arm motion.

#[derive(Debug, Clone)]
pub struct TrajectoryPoint {
    pub time_s: f64,
    pub position_rad: f64,
    pub velocity_rads: f64,
    pub acceleration_rads2: f64,
    pub jerk_rads3: f64,
}

#[derive(Debug, Clone)]
pub struct TrajectorySegment {
    pub joint_id: usize,
    pub duration_s: f64,
    pub points: Vec<TrajectoryPoint>,
}

/// 5th-order polynomial coefficients for minimum-jerk trajectory
struct QuinticCoeffs {
    a0: f64, a1: f64, a2: f64, a3: f64, a4: f64, a5: f64,
}

impl QuinticCoeffs {
    /// Compute quintic coefficients for boundary conditions:
    /// position, velocity, acceleration at t=0 and t=T
    fn compute(
        q0: f64, qf: f64,
        v0: f64, vf: f64,
        a0: f64, af: f64,
        t: f64,
    ) -> Self {
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;

        let c0 = q0;
        let c1 = v0;
        let c2 = a0 / 2.0;
        let c3 = (20.0 * (qf - q0) - (8.0 * vf + 12.0 * v0) * t - (3.0 * a0 - af) * t2) / (2.0 * t3);
        let c4 = (30.0 * (q0 - qf) + (14.0 * vf + 16.0 * v0) * t + (3.0 * a0 - 2.0 * af) * t2) / (2.0 * t4);
        let c5 = (12.0 * (qf - q0) - 6.0 * (vf + v0) * t - (a0 - af) * t2) / (2.0 * t5);

        QuinticCoeffs { a0: c0, a1: c1, a2: c2, a3: c3, a4: c4, a5: c5 }
    }

    fn evaluate(&self, t: f64) -> (f64, f64, f64, f64) {
        let t2 = t * t;
        let t3 = t2 * t;
        let t4 = t3 * t;
        let t5 = t4 * t;

        let pos = self.a0 + self.a1 * t + self.a2 * t2 + self.a3 * t3 + self.a4 * t4 + self.a5 * t5;
        let vel = self.a1 + 2.0 * self.a2 * t + 3.0 * self.a3 * t2 + 4.0 * self.a4 * t3 + 5.0 * self.a5 * t4;
        let acc = 2.0 * self.a2 + 6.0 * self.a3 * t + 12.0 * self.a4 * t2 + 20.0 * self.a5 * t3;
        let jrk = 6.0 * self.a3 + 24.0 * self.a4 * t + 60.0 * self.a5 * t2;

        (pos, vel, acc, jrk)
    }
}

/// Plan a minimum-jerk trajectory for a single joint
pub fn plan_min_jerk(
    joint_id: usize,
    start_pos: f64,
    end_pos: f64,
    duration_s: f64,
    sample_rate_hz: f64,
) -> TrajectorySegment {
    let coeffs = QuinticCoeffs::compute(
        start_pos, end_pos,
        0.0, 0.0,  // start/end velocity = 0
        0.0, 0.0,  // start/end acceleration = 0
        duration_s,
    );

    let dt = 1.0 / sample_rate_hz;
    let num_samples = (duration_s * sample_rate_hz) as usize + 1;
    let mut points = Vec::with_capacity(num_samples);

    for i in 0..num_samples {
        let t = (i as f64) * dt;
        let (pos, vel, acc, jrk) = coeffs.evaluate(t);
        points.push(TrajectoryPoint {
            time_s: t,
            position_rad: pos,
            velocity_rads: vel,
            acceleration_rads2: acc,
            jerk_rads3: jrk,
        });
    }

    TrajectorySegment {
        joint_id,
        duration_s,
        points,
    }
}

/// Compute maximum jerk in a trajectory (for safety validation)
pub fn max_jerk(segment: &TrajectorySegment) -> f64 {
    segment.points.iter()
        .map(|p| p.jerk_rads3.abs())
        .fold(0.0_f64, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_min_jerk_boundary_conditions() {
        let seg = plan_min_jerk(0, 0.0, 1.5708, 2.0, 100.0); // 0 to π/2 in 2s
        let first = &seg.points[0];
        let last = seg.points.last().unwrap();

        assert!((first.position_rad - 0.0).abs() < 1e-6);
        assert!((first.velocity_rads - 0.0).abs() < 1e-6);
        assert!((last.position_rad - 1.5708).abs() < 0.01);
        assert!((last.velocity_rads).abs() < 0.1);
    }

    #[test]
    fn test_jerk_bounded() {
        let seg = plan_min_jerk(0, 0.0, 3.14159, 1.0, 1000.0);
        let mj = max_jerk(&seg);
        assert!(mj < 1000.0, "Jerk should be bounded for safe motion");
    }
}
