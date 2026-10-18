#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_op_in_unsafe_fn)]
#![warn(missing_docs, clippy::all, clippy::pedantic)]

//! # `s-curve-rt`
//! Deterministic, zero-heap-allocation online jerk-limited (third-order S-curve)
//! trajectory generator designed for 1 kHz+ hard real-time interrupt loops on
//! microcontrollers (`#![no_std]`) and distributed robotics control planes.

pub mod error;
pub mod ffi;
pub mod multi_dof;
pub mod profile;
pub mod solver;
pub mod state;

pub use error::TrajectoryError;
pub use multi_dof::MultiDofSynchronizer;
pub use profile::{Phase, StepProfile};
pub use solver::SCurveSolver;
pub use state::{KinematicLimits, KinematicState};
