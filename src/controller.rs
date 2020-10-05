pub struct Controller {
    p: f32,
    s: f32,
    sum: f32,
    dt: f32,
}

impl Controller {
    pub fn new(p: f32, s: f32, dt: f32) -> Self {
        Self {
            p,
            s,
            sum: 0.0f32,
            dt,
        }
    }

    pub fn calculate_action(&mut self, target: f32, actual: f32) -> f32 {
        let e = target - actual;
        self.sum += e * self.dt;
        self.p * e + self.s * self.sum
    }
}
