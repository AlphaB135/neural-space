//! Deterministic matrix operations for control math.
//!
//! Fixed-size, stack-allocated matrix types. No heap. No dynamic dispatch.
//! Every operation completes in bounded time.

/// 3x3 matrix — used for rotation transforms.
#[repr(C, align(32))]
pub struct Mat3 {
    pub data: [[f64; 3]; 3],
}

/// 4x4 matrix — used for homogeneous transforms.
#[repr(C, align(32))]
pub struct Mat4 {
    pub data: [[f64; 4]; 4],
}

/// 3-element vector.
#[repr(C, align(16))]
pub struct Vec3 {
    pub data: [f64; 3],
}

impl Vec3 {
    #[inline(always)]
    pub const fn zero() -> Self {
        Self { data: [0.0; 3] }
    }

    #[inline(always)]
    pub fn dot(&self, other: &Self) -> f64 {
        self.data[0] * other.data[0]
            + self.data[1] * other.data[1]
            + self.data[2] * other.data[2]
    }

    #[inline(always)]
    pub fn cross(&self, other: &Self) -> Self {
        Self {
            data: [
                self.data[1] * other.data[2] - self.data[2] * other.data[1],
                self.data[2] * other.data[0] - self.data[0] * other.data[2],
                self.data[0] * other.data[1] - self.data[1] * other.data[0],
            ],
        }
    }

    #[inline(always)]
    pub fn magnitude(&self) -> f64 {
        self.dot(self).sqrt()
    }

    #[inline(always)]
    pub fn normalize(&self) -> Self {
        let mag = self.magnitude();
        if mag < 1e-15 {
            return Self::zero();
        }
        Self {
            data: [
                self.data[0] / mag,
                self.data[1] / mag,
                self.data[2] / mag,
            ],
        }
    }
}

impl Mat3 {
    #[inline(always)]
    pub const fn identity() -> Self {
        Self {
            data: [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
        }
    }

    /// Matrix-vector multiply — 9 multiplies, 6 adds, deterministic.
    #[inline(always)]
    pub fn mul_vec(&self, v: &Vec3) -> Vec3 {
        Vec3 {
            data: [
                self.data[0][0] * v.data[0]
                    + self.data[0][1] * v.data[1]
                    + self.data[0][2] * v.data[2],
                self.data[1][0] * v.data[0]
                    + self.data[1][1] * v.data[1]
                    + self.data[1][2] * v.data[2],
                self.data[2][0] * v.data[0]
                    + self.data[2][1] * v.data[1]
                    + self.data[2][2] * v.data[2],
            ],
        }
    }
}
