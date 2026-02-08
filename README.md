# Vibe Engine (Simulation Core) 🦀

**Status:** 🚧 In Active Development

A dedicated physics and logic engine built with **Rust** and compiled to **WebAssembly (WASM)** via `wasm-pack`. This engine powers the _Vibe Heist_ project, handling performant 3D physics calculations off the main JavaScript thread.

## ⚡ Tech Stack

- **Language:** Rust (Edition 2024)
- **Physics:** Rapier3D (Simd-stable)
- **Math:** Nalgebra
- **Compilation:** WASM-Bindgen

## 🔧 Build Instructions

**Prerequisites:** Rust, Cargo, `wasm-pack`.

```bash
# Compile to WebAssembly
wasm-pack build --target web
```

## 🎯 Architecture

This module is strictly decoupled from the rendering layer. It accepts inputs (controller states) and returns a raw `Float32Array` of transform matrices, ensuring the simulation remains deterministic and framework-agnostic.
