use std::collections::VecDeque;
use std::rc::Rc;

use crate::observation::Observation;

pub trait ObservationCache {
    type Iter<'a>: Iterator<Item = &'a Rc<Observation>>
    where
        Self: 'a;
    fn push(&mut self, observation: Rc<Observation>);
    fn iter<'a>(&'a self) -> Self::Iter<'a>;
    fn clear(&mut self);
    fn len(&self) -> usize;
}

/// A cache of recent observations that holds up to a fixed number of observations.
/// When the cache is full, the oldest observation is dropped.
pub struct FixedCache {
    max_length: usize,
    buffer: VecDeque<Rc<Observation>>,
}

impl FixedCache {
    pub fn new(max_length: usize) -> Self {
        Self {
            max_length,
            buffer: VecDeque::with_capacity(max_length),
        }
    }
}

impl ObservationCache for FixedCache {
    type Iter<'a> = std::collections::vec_deque::Iter<'a, Rc<Observation>>;
    fn push(&mut self, observation: Rc<Observation>) {
        if self.buffer.len() >= self.max_length {
            self.buffer.pop_front();
        }
        self.buffer.push_back(observation)
    }
    fn iter<'a>(&'a self) -> std::collections::vec_deque::Iter<'a, Rc<Observation>> {
        self.buffer.iter()
    }
    fn clear(&mut self) {
        self.buffer.clear()
    }
    fn len(&self) -> usize {
        self.buffer.len()
    }
}

fn div_ceil(lhs: usize, rhs: usize) -> usize {
    let d = lhs / rhs;
    let r = lhs % rhs;
    if r > 0 && rhs > 0 {
        d + 1
    } else {
        d
    }
}

/// A cache of recent observations, bined by location.
/// Note: this implementation assumes observations are added in chronological
/// order and have unique timestamps.
pub struct SpacialCache {
    /// The width of the camera in pixels.
    pixel_width: usize,
    /// The height of the camera in pixels.
    pixel_height: usize,
    /// The maximum number of observations to keep in a bin.
    max_bin_length: usize,
    /// The minimum number of observations to keep when removing old observations.
    min_total_length: usize,
    /// The maximum age of an observation to keep.
    max_age: f64,
    pixels_per_bin: usize,
    vertical_bin_count: usize,
    timed_storage: VecDeque<Rc<Observation>>,
    binned_storage: Vec<VecDeque<Rc<Observation>>>,
}

impl SpacialCache {
    pub fn new(
        pixel_width: usize,
        pixel_height: usize,
        horizontal_bin_count: usize,
        max_bin_length: usize,
        min_total_length: usize,
        max_age: f64,
    ) -> Self {
        let pixels_per_bin = div_ceil(pixel_width, horizontal_bin_count);
        let vertical_bin_count = div_ceil(pixel_height, pixels_per_bin);
        Self {
            pixel_width,
            pixel_height,
            max_bin_length,
            min_total_length,
            max_age,
            pixels_per_bin,
            vertical_bin_count,
            timed_storage: VecDeque::with_capacity(2 * max_bin_length),
            binned_storage: vec![
                VecDeque::new();
                (horizontal_bin_count * vertical_bin_count) + 1 as usize
            ],
        }
    }
    fn get_bin_index(&self, observation: &Observation) -> usize {
        // Convert the observation's center to image coordinates and then to bin coordinates.
        let x = (observation.ellipse.center[0] + (self.pixel_width as f64 / 2.0)) as usize
            / self.pixels_per_bin;
        let y = (observation.ellipse.center[1] + (self.pixel_height as f64 / 2.0)) as usize
            / self.pixels_per_bin;
        // Flatten the 2D bin coordinates into a 1D index.
        return x + y * self.vertical_bin_count;
    }
}

impl ObservationCache for SpacialCache {
    type Iter<'a> = std::collections::vec_deque::Iter<'a, Rc<Observation>>;
    fn push(&mut self, observation: Rc<Observation>) {

        if observation.ellipse.center.x < -(self.pixel_width as f64) / 2.0
            || observation.ellipse.center.x > self.pixel_width as f64 / 2.0
            || observation.ellipse.center.y < -(self.pixel_height as f64) / 2.0
            || observation.ellipse.center.y > self.pixel_height as f64 / 2.0
        {
            // println!("Observation out of bounds: {:?}", observation.ellipse.center);
            return;
        }

        // Get the bin for the observation.
        let index = self.get_bin_index(&observation);
        let storage_bin = &mut self.binned_storage[index];

        // Get the timestamp of the observation for later use.
        let timestamp = observation.timestamp;

        // Add the observation to the bin and the timed storage.
        storage_bin.push_back(observation.clone());
        self.timed_storage.push_back(observation);

        // If the bin is full, remove the oldest observation from the bin and the timed storage.
        while storage_bin.len() > self.max_bin_length {
            match storage_bin.pop_front() {
                Some(old) => remove_from_deque(&old, &mut self.timed_storage),
                None => break,
            }
        }

        // If we have more than the minimum number of observations, remove
        // observations older than the maximum age.
        while self.timed_storage.len() > self.min_total_length {
            let oldest_age = timestamp - self.timed_storage[0].timestamp;
            if oldest_age < self.max_age {
                break;
            }

            // forget oldest entry
            let oldest = match self.timed_storage.pop_front() {
                Some(oldest) => oldest,
                None => break,
            };

            let index = self.get_bin_index(&oldest);
            let storage_bin = &mut self.binned_storage[index];
            remove_from_deque(&oldest, storage_bin);
        }
    }

    fn iter<'a>(&'a self) -> Self::Iter<'a> {
        self.timed_storage.iter()
    }

    fn clear(&mut self) {
        self.timed_storage.clear();
        self.binned_storage.clear()
    }

    fn len(&self) -> usize {
        self.timed_storage.len()
    }
}

fn remove_from_deque(observation: &Rc<Observation>, deque: &mut VecDeque<Rc<Observation>>) {
    let index = deque.binary_search_by(|f| f.timestamp.total_cmp(&observation.timestamp));
    if let Ok(index) = index {
        deque.remove(index);
    }
}
