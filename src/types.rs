use nalgebra::{Isometry3, Unit, Vector3};


#[derive(Debug, Hash, PartialEq, Eq, Clone)]
pub struct Link {
    pub name: String,
}

#[derive(Debug)]
pub struct Joint {
    pub name: String,
    pub parent: String,
    pub child: String,
    pub transform: Isometry3<f64>,
    pub axis: Unit<Vector3<f64>>,
    pub limit_lower: f64,
    pub limit_upper: f64,
}

#[derive(Debug)]
pub struct RobotModel {
    pub name: String,
    pub links: Vec<Link>,
    pub joints: Vec<Joint>,
}
