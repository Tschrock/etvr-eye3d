/// A structure that controls when a model updates.
pub trait UpdateController {
    /// Pauses the update controller.
    fn pause(&mut self);
    /// Resumes the update controller.
    fn resume(&mut self);
    /// Returns true if the update controller is paused.
    fn is_paused(&self) -> bool;
    /// Returns true if the model should update.
    fn should_update(&mut self, timestamp: f64) -> bool;
}

pub struct TimedUpdateController {
    update_interval: f64,
    warmup_duration: f64,
    started_at: Option<f64>,
    updated_at: Option<f64>,
    is_paused: bool,
}

impl TimedUpdateController {
    pub fn new(update_interval: f64, warmup_duration: f64) -> Self {
        TimedUpdateController {
            update_interval,
            warmup_duration,
            started_at: None,
            updated_at: None,
            is_paused: false,
        }
    }
}

impl UpdateController for TimedUpdateController {
    fn pause(&mut self) {
        self.is_paused = true;
    }
    fn resume(&mut self) {
        self.is_paused = false;
        self.updated_at = None;
    }
    fn is_paused(&self) -> bool {
        self.is_paused
    }
    fn should_update(&mut self, timestamp: f64) -> bool {
        if self.is_paused {
            return false;
        }
        if let Some(started_at) = self.started_at {
            if timestamp - started_at < self.warmup_duration {
                return true;
            }
        } else {
            self.started_at = Some(timestamp);
            return true;
        }
        if let Some(updated_at) = self.updated_at {
            self.updated_at = Some(timestamp);
            return timestamp - updated_at > self.update_interval;
        } else {
            self.updated_at = Some(timestamp);
            return true;
        }
    }
}
