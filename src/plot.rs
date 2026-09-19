use serde::Serialize;
use std::fs::create_dir_all;
use std::path::{Path, PathBuf};

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
