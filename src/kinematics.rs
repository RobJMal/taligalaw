use std::{collections::HashMap};

use crate::types::{Link, RobotModel, Transform};

impl RobotModel {
    pub fn compute_fk(&self, angles: &[f64]) -> Result<HashMap<Link, Transform>, Box<dyn std::error::Error>> {
        /* Computes forward kinematics of a model */
        if angles.len() != self.joints.len() {
            return Err(format!(
                "expected {} angles, got {}",
                self.joints.len(),
                angles.len()
            ).into());
        }

        Ok(HashMap::new())
    }
}
