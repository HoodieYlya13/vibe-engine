//! The sim ⇄ client boundary: snapshot writer, input reader, runtime
//! controls, and the dev/test surface. Everything the worker calls lives here
//! or is re-exported from a system module.

pub(crate) mod controls;
pub(crate) mod dev;
pub(crate) mod input;
pub(crate) mod snapshot;
