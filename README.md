
# RV32I Pipelined Processor with Instruction Cache

## Overview

This repository contains an implementation of a RISC-V 32-bit (RV32I) pipelined processor with an instruction cache. The design aims to explore pipelining concepts, improve instruction fetch performance, and demonstrate basic cache implementation. It supports all instructions defined in the RV32I base ISA.

## Features

- **5-Stage Pipeline**:
  - Instruction Fetch (IF)
  - Instruction Decode (ID)
  - Execute (EX)
  - Memory Access (MEM)
  - Write Back (WB)
- **Instruction Cache**:
  - Configurable size and associativity.
  - Simple cache replacement policy.
- **Hazard Handling**:
  - Data forwarding for reducing stalls.
  - Basic branch prediction (static "always taken").
- **RV32I Compliance**: Supports all RV32I instructions.
- **Modular Design**: Easily extensible for future features.

---

## Directory Structure

```plaintext
.
├── src/
│   ├── rv32i_pipeline.v       # Main pipeline processor module
│   ├── instr_cache.v          # Instruction cache implementation
│   ├── control_unit.v         # Hazard detection and control logic
│   ├── alu.v                  # Arithmetic Logic Unit
│   ├── reg_file.v             # Register file implementation
│   ├── memory.v               # Data memory
│   └── utils.v                # Helper modules (e.g., muxes, sign extend)
├── testbench/
│   ├── tb_pipeline.v          # Testbench for the pipelined processor
│   ├── tb_instr_cache.v       # Testbench for the instruction cache
│   └── test_programs/         # Sample RISC-V programs for testing
├── docs/
│   ├── design_spec.md         # Detailed design documentation
│   └── pipeline_diagram.png   # Pipeline structure diagram
├── Makefile                   # Makefile for compiling and simulating
├── LICENSE                    # Project license
└── README.md                  # Project documentation
