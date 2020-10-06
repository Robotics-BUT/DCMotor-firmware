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
        self.p * e + self.s * self.sum
    }
}
