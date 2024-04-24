use std::collections::HashMap;
use std::fmt::Formatter;

use itertools::Itertools;

use crate::{AsBytes, Writable};

pub trait BroadcastRegister {
    type Inner: AsBytes;
    const ADDRESS: u8;
    const SIZE: u8;
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BuilderError {
    Empty,
    NonSequentialID,
}

impl std::fmt::Display for BuilderError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            BuilderError::Empty => write!(f, "no data has been added to the builder"),
            BuilderError::NonSequentialID => write!(f, "only sequential ids are supported"),
        }
    }
}

impl std::error::Error for BuilderError {}

#[derive(Debug, Clone)]
pub struct BroadcastPosition {
    start_id: u8,
    end_id: u8,
    data: Vec<f32>,
}

#[derive(Default, Debug, Clone)]
pub struct BroadcastPositionBuilder(HashMap<u8, f32>);

impl BroadcastPositionBuilder {
    pub fn new() -> BroadcastPositionBuilder {
        BroadcastPositionBuilder(HashMap::new())
    }

    pub fn add(&mut self, id: u8, data: f32) -> &mut Self {
        self.0.insert(id, data);
        self
    }

    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    pub fn build(self) -> Result<BroadcastPosition, BuilderError> {
        if self.0.is_empty() {
            return Err(BuilderError::Empty);
        }
        let data: Vec<_> = self.0.into_iter().sorted_by_key(|d| d.0).collect();
        let (start_id, end_id) = data.first().zip(data.last()).map(|((f, _), (l, _))| { (*f, *l) }).unwrap();
        let data: Vec<_> = data.into_iter().enumerate().map(|(i, (id, data))| {
            if (i as u8 + id) != start_id + i as u8 {
                return Err(BuilderError::NonSequentialID);
            }
            Ok(data)
        }).collect::<Result<_, _>>()?;

        Ok(BroadcastPosition {
            start_id,
            end_id,
            data,
        })
    }
}


impl BroadcastPosition {
    pub fn new(start_id: u8, end_id: u8, data: Vec<f32>) -> Self {
        let count = (end_id - start_id + 1) as usize;
        assert_eq!(count, data.len());
        Self {
            start_id,
            end_id,
            data,
        }
    }

    pub fn builder() -> BroadcastPositionBuilder {
        BroadcastPositionBuilder::new()
    }
}

impl AsRef<BroadcastPosition> for BroadcastPosition {
    fn as_ref(&self) -> &BroadcastPosition {
        self
    }
}

impl BroadcastRegister for BroadcastPosition {
    type Inner = f32;
    const ADDRESS: u8 = 0x60;
    const SIZE: u8 = 4;
}

impl Writable for BroadcastPosition {
    fn as_bytes(&self) -> Vec<u8> {
        let mut buffer = Vec::with_capacity(self.data.len() + 2);
        buffer.push(self.start_id);
        buffer.push(self.end_id);
        for d in self.data.iter() {
            buffer.extend_from_slice(d.to_be_bytes().as_slice());
        }
        dbg!(buffer)
    }
}