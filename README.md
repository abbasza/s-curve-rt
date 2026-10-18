
# `s-curve-rt` — Deterministic `#![no_std]` Jerk-Limited Trajectory Generator

[![CI](https://github.com/abbasza/s-curve-rt/actions/workflows/ci.yml/badge.svg)](https://github.com/abbasza/s-curve-rt/actions)
[![Crates.io](https://img.shields.io/crates/v/s-curve-rt.svg)](https://crates.io/crates/s-curve-rt)
[![Docs.rs](https://docs.rs/s-curve-rt/badge.svg)](https://docs.rs/s-curve-rt)
[![no_std](https://img.shields.io/badge/embedded-no__std-blue.svg)](https://docs.rust-embedded.org/book/)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-green.svg)](LICENSE)

`s-curve-rt` is a deterministic, **zero-heap-allocation** online third-order (7-segment jerk-limited) trajectory generator written in `#![no_std]` Rust. It is engineered for **1 kHz – 10 kHz hard real-time interrupt service routines (ISRs)** on bare-metal microcontrollers (ARM Cortex-M, RISC-V via RTIC/Zephyr) as well as high-throughput robotics control planes (`PREEMPT_RT` Linux, C++ flight controllers, Go microservices, and WebAssembly sandboxes).

---

## Architecture & Execution Pipeline

```mermaid
flowchart LR
    subgraph Input["Target Setpoint / Sensor Override"]
        CMD["Target (p_target, v_target)"]
        LIM["KinematicLimits (v_max, a_max, j_max)"]
    end

    subgraph Solver["s-curve-rt Core (#![no_std] | 0 Heap Allocs)"]
        SYNC["MultiDofSynchronizer<const N: usize>\n(Stack-Allocated Phase Stretching)"]
        PROF["7-Segment Analytical Solver\n(libm Halley Cube-Root + Bisection)"]
        ISR["1 kHz Online Step Integrator\n3rd-Order Taylor State Update"]
        SYNC --> PROF --> ISR
    end

    subgraph Targets["Cross-Language & Hardware Targets"]
        MCU["ARM Cortex-M / Zephyr / RTIC\n(thumbv7em-none-eabihf)"]
        CPP["C / C++ Flight & Actuator Control\n(cbindgen Zero-Copy C-ABI)"]
        WASM["Wasmtime / Browser Sandbox\n(wasm-bindgen)"]
    end

    CMD --> Solver
    LIM --> Solver
    ISR --> MCU
    ISR --> CPP
    ISR --> WASM
