# Operation-Level Counting Rules

This document describes the rules used to derive operation-level
computational complexity for MPPT algorithms evaluated in the study
"Adaptive Gradient Descent–Based MPPT with Hardware-Oriented
Computational Complexity Benchmarking".

## Scope
The counting procedure targets a *single worst-case MPPT iteration*,
from acquisition of voltage/current samples to duty-cycle update.

The objective is to quantify intrinsic algorithmic complexity
independent of processor architecture, clock frequency, or software
optimization.

## Counted operations
The following elementary operations are counted explicitly:

- Addition (ADD)
- Subtraction (SUB)
- Multiplication (MUL)
- Division (DIV)
- Comparison (CMP)
- Absolute value (ABS)
- Logical operations (AND, OR, XOR)
- Bit-shift operations
- Maximum / minimum selection
- Exponential and logarithmic functions
- Power or exponentiation operations
- Lookup-table access (used to model FLC inference)
- Neural-network inference (modeled as matrix–vector operations)

## Counting rules
- Each arithmetic or logical operation is counted once per execution.
- Conditional branches are evaluated along their worst-case execution path.
- Loop bodies are expanded to their maximum number of executions per iteration.
- Memory access latency, pipeline depth, and instruction-level parallelism
  are not modeled explicitly.
- ADC sampling and PWM generation are excluded from operation counting,
  as they are common to all algorithms.

## Assumptions
- All operations are assumed to operate on fixed-point data.
- Bit-width is assumed identical across algorithms.
- The same rules are applied uniformly to all evaluated MPPT methods.

These rules ensure that reported complexity reflects *relative
algorithmic cost* rather than platform-specific execution details.
