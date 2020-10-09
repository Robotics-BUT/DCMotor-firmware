use crate::bridge::{fmaxf, fminf};

const MAX_ACTION: f32 = 1.0;

pub struct Controller {
    p: f32,
    s: f32,
    sum: f32,
    dt: f32,
    target: f32,
}

impl Controller {
    pub fn new(p: f32, s: f32, dt: f32) -> Self {
        Self {
            p,
            s,
            sum: 0.0,
            dt,
            target: 0.0,
        }
    }

    pub fn set_target(&mut self, target: f32) {
        self.target = target;
    }

    pub fn calculate_action(&mut self, actual: f32) -> f32 {
        let e = self.target - actual;
        self.sum += e * self.dt;
        self.sum = fmaxf(-MAX_ACTION, fminf(self.sum, MAX_ACTION));
        fmaxf(
            -MAX_ACTION,
            fminf(self.p * e + self.s * self.sum, MAX_ACTION),
        )
    }
}
