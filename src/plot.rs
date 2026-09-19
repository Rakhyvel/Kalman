use serde::Serialize;
use std::path::Path;

#[derive(Default)]
pub struct SampleLog {
    samples: Vec<Sample>,
}

#[derive(Serialize)]
pub struct Sample {
    pub t: f64,
    pub truth: f64,
    pub measurement: f64,
    pub estimate: f64,
}

impl SampleLog {
    pub fn new() -> SampleLog {
        Self::default()
    }

    pub fn record(&mut self, sample: Sample) {
        self.samples.push(sample);
    }

    pub fn rms(&mut self) -> f64 {
        if self.samples.is_empty() {
            return 0.0;
        }

        let mut rms: f64 = 0.0;
        for sample in &self.samples {
            rms += sample.squared_error();
        }
        (rms / self.samples.len() as f64).sqrt()
    }

    pub fn save(&self) {
        if self.samples.is_empty() {
            println!("Samples empty, not saving a CSV");
            return;
        }

        let file_path = Path::new("run.csv");
        let mut writer = csv::Writer::from_path(file_path).unwrap();
        for sample in &self.samples {
            writer.serialize(sample).unwrap();
        }
        writer.flush().unwrap()
    }
}

impl Sample {
    pub fn squared_error(&self) -> f64 {
        (self.estimate - self.truth).powi(2)
    }
}
