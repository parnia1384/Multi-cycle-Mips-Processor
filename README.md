# Multi-Cycle MIPS Processor

## Overview
This is a Verilog implementation of a multi-cycle MIPS processor based on the design described in "Digital Design and Computer Architecture" by David Harris and Sarah Harris. Unlike single-cycle designs, this processor executes instructions over multiple clock cycles, reusing hardware components and allowing different instruction types to complete in varying cycle counts.

## Features
- **32-bit architecture** with 32 general-purpose registers
- **Multi-cycle execution**: Instructions complete in 3-5 clock cycles
- **Variable instruction timing**: Different instruction types take different cycles
- **Hardware reuse**: Single ALU used across multiple execution phases
- **Finite State Machine control**: Sequential control logic replaces combinational
- **Shared memory interface**: Single memory unit for both instructions and data
- **Instruction timing by type**:
  - R-type (ADD, SUB, AND, OR, SLT): 4 cycles
  - I-type (ADDI, ANDI, ORI, SLTI): 4 cycles
  - Load (LW): 5 cycles
  - Store (SW): 4 cycles
  - Branch (BEQ, BNE): 3 cycles
  - Jump (J): 3 cycles

## Architecture

### Multi-Cycle Execution Phases

```mermaid
stateDiagram-v2
    [*] --> Fetch
    Fetch --> Decode
    Decode --> MemAddr : SW/LW
    Decode --> ExecuteR : R-type
    Decode --> ExecuteI : I-type
    Decode --> Branch : BEQ/BNE
    Decode --> Jump : J
    
    MemAddr --> MemRead : LW
    MemAddr --> MemWrite : SW
    MemRead --> WriteBack
    MemWrite --> [*]
    
    ExecuteR --> ALUWriteBack
    ExecuteI --> ALUWriteBack
    ALUWriteBack --> [*]
    
    Branch --> [*]
    Jump --> [*]
    
    WriteBack --> [*]
```

### Datapath Diagram
```mermaid
flowchart TD
    %% Registers
    PC["PC<br/>Program Counter"]
    IR["IR<br/>Instruction Register"]
    MDR["MDR<br/>Memory Data Register"]
    A["A Register<br/>(rs value)"]
    B["B Register<br/>(rt value)"]
    ALUOut["ALUOut Register"]
    
    %% Memory
    InstrMem["Instruction<br/>Memory"]
    DataMem["Data<br/>Memory"]
    
    %% Functional Units
    ALU["ALU"]
    RF["Register File<br/>32 x 32-bit"]
    SE["Sign Extend"]
    Shift2["Shift Left 2"]
    
    %% Multiplexers
    MUX_PCSrc["MUX<br/>PC Source"]
    MUX_ALUSrcA["MUX<br/>ALUSrc A"]
    MUX_ALUSrcB["MUX<br/>ALUSrc B"]
    MUX_RegDst["MUX<br/>RegDst"]
    MUX_MemtoReg["MUX<br/>MemtoReg"]
    
    %% Control
    CU["Control Unit<br/>(Finite State Machine)"]
    
    %% Data Flow
    PC --> InstrMem
    InstrMem --> IR
    
    IR --> CU
    IR --> RF
    IR --> SE
    IR --> MUX_RegDst
    
    RF --> A
    RF --> B
    
    A --> MUX_ALUSrcA
    PC --> MUX_ALUSrcA
    ALUOut --> MUX_ALUSrcA
    
    B --> MUX_ALUSrcB
    SE --> MUX_ALUSrcB
    SE --> Shift2
    Shift2 --> MUX_ALUSrcB
    
    MUX_ALUSrcA --> ALU
    MUX_ALUSrcB --> ALU
    CU --> ALU
    ALU --> ALUOut
    
    ALUOut --> DataMem
    B --> DataMem
    DataMem --> MDR
    
    MDR --> MUX_MemtoReg
    ALUOut --> MUX_MemtoReg
    MUX_MemtoReg --> RF
    MUX_RegDst --> RF
    
    %% PC Update
    ALUOut --> MUX_PCSrc
    IR --> JumpCalc["Jump Address<br/>Calculation"]
    JumpCalc --> MUX_PCSrc
    PC --> Adder4["Adder<br/>PC + 4"]
    Adder4 --> MUX_PCSrc
    MUX_PCSrc --> PC
    
    classDef reg fill:#e1f5fe,stroke:#01579b
    classDef mem fill:#f3e5f5,stroke:#4a148c
    classDef alu fill:#e8f5e8,stroke:#2e7d32
    classDef mux fill:#fff3e0,stroke:#e65100
    classDef ctrl fill:#fce4ec,stroke:#c2185b
    
    class PC,IR,MDR,A,B,ALUOut reg
    class InstrMem,DataMem mem
    class ALU,RF,SE,Shift2,Adder4,JumpCalc alu
    class MUX_PCSrc,MUX_ALUSrcA,MUX_ALUSrcB,MUX_RegDst,MUX_MemtoReg mux
    class CU ctrl
```

### Control Unit: Finite State Machine
```mermaid
flowchart TD
    Start([Start]) --> S0["State 0: Fetch<br/>MemRead = 1, IorD = 0<br/>IRWrite = 1, ALUSrcA = 0<br/>ALUSrcB = 01, ALUOp = 00<br/>PCSrc = 00, PCWrite = 1"]
    
    S0 --> S1["State 1: Decode<br/>ALUSrcA = 0, ALUSrcB = 11<br/>ALUOp = 00"]
    
    S1 --> S2{"Opcode?"}
    
    S2 -- "LW/SW (100011/101011)" --> S3["State 2: MemAddr<br/>ALUSrcA = 1, ALUSrcB = 10<br/>ALUOp = 00"]
    S2 -- "R-type (000000)" --> S4["State 6: Execute<br/>ALUSrcA = 1, ALUSrcB = 00<br/>ALUOp = 10"]
    S2 -- "BEQ/BNE (000100/000101)" --> S5["State 8: Branch<br/>ALUSrcA = 1, ALUSrcB = 00<br/>ALUOp = 01, PCWriteCond = 1"]
    S2 -- "J (000010)" --> S6["State 9: Jump<br/>PCWrite = 1, PCSrc = 10"]
    
    S3 --> S7{"MemWrite or MemRead?"}
    S7 -- "LW" --> S8["State 3: MemRead<br/>MemRead = 1, IorD = 1"]
    S7 -- "SW" --> S9["State 5: MemWrite<br/>MemWrite = 1, IorD = 1"]
    
    S8 --> S10["State 4: WriteBack<br/>RegDst = 0, RegWrite = 1<br/>MemtoReg = 1"]
    
    S4 --> S11["State 7: ALUWriteBack<br/>RegDst = 1, RegWrite = 1<br/>MemtoReg = 0"]
    
    S10 --> Start
    S11 --> Start
    S9 --> Start
    S5 --> Start
    S6 --> Start
    
    classDef fetch fill:#e3f2fd,stroke:#0277bd
    classDef decode fill:#e8f5e8,stroke:#2e7d32
    classDef execute fill:#fff3e0,stroke:#ef6c00
    classDef memory fill:#f3e5f5,stroke:#7b1fa2
    classDef writeback fill:#fce4ec,stroke:#c2185b
    
    class S0 fetch
    class S1 decode
    class S3,S4,S5,S6,S7,S8,S9 execute
    class S8,S9 memory
    class S10,S11 writeback
```
### Execution Cycles by Instruction Type

| Instruction Type | Cycles | Stage 1 | Stage 2 | Stage 3 | Stage 4 | Stage 5 |
|------------------|--------|---------|---------|---------|---------|---------|
| **R-type** (ADD, SUB, AND, OR, SLT) | 4 | Fetch | Decode | Execute | WriteBack | - |
| **I-type** (ADDI, ANDI, ORI, SLTI) | 4 | Fetch | Decode | Execute | WriteBack | - |
| **Load** (LW) | 5 | Fetch | Decode | MemAddr | MemRead | WriteBack |
| **Store** (SW) | 4 | Fetch | Decode | MemAddr | MemWrite | - |
| **Branch** (BEQ, BNE) | 3 | Fetch | Decode | Branch | - | - |
| **Jump** (J) | 3 | Fetch | Decode | Jump | - | - |

### Control Signals Table

| Signal | Width | Description | Values/Meaning |
|--------|-------|-------------|----------------|
| **PCWrite** | 1-bit | Program Counter write enable | 0: Disable, 1: Enable |
| **PCWriteCond** | 1-bit | Conditional PC write (branch) | 0: Disable, 1: Enable if zero |
| **IorD** | 1-bit | Memory address source | 0: PC, 1: ALUOut |
| **MemRead** | 1-bit | Memory read enable | 0: Disable, 1: Enable |
| **MemWrite** | 1-bit | Memory write enable | 0: Disable, 1: Enable |
| **IRWrite** | 1-bit | Instruction Register write | 0: Disable, 1: Enable |
| **MemtoReg** | 1-bit | Register write data source | 0: ALUOut, 1: Memory Data |
| **PCSrc** | 2-bit | Next PC source | 00: ALU, 01: ALUOut, 10: Jump Address |
| **ALUOp** | 2-bit | ALU operation type | 00: Add, 01: Subtract, 10: Use funct field |
| **ALUSrcA** | 1-bit | ALU A input source | 0: PC, 1: A Register |
| **ALUSrcB** | 2-bit | ALU B input source | 00: B Register, 01: Constant 4, 10: Sign-extended offset, 11: Offset << 2 |
| **RegWrite** | 1-bit | Register File write enable | 0: Disable, 1: Enable |
| **RegDst** | 1-bit | Register destination select | 0: rt, 1: rd |

### State Machine Control Values

| State | ALUSrcA | ALUSrcB | ALUOp | MemRead | MemWrite | IRWrite | PCWrite | PCSrc | IorD | RegWrite | RegDst | MemtoReg |
|-------|---------|---------|-------|---------|----------|---------|---------|-------|------|----------|--------|----------|
| **Fetch** | 0 | 01 | 00 | 1 | 0 | 1 | 1 | 00 | 0 | 0 | X | X |
| **Decode** | 0 | 11 | 00 | 0 | 0 | 0 | 0 | XX | 0 | 0 | X | X |
| **MemAddr** | 1 | 10 | 00 | 0 | 0 | 0 | 0 | XX | 0 | 0 | X | X |
| **MemRead** | X | X | XX | 1 | 0 | 0 | 0 | XX | 1 | 0 | X | X |
| **MemWrite** | X | X | XX | 0 | 1 | 0 | 0 | XX | 1 | 0 | X | X |
| **ExecuteR** | 1 | 00 | 10 | 0 | 0 | 0 | 0 | XX | 0 | 0 | X | X |
| **ExecuteI** | 1 | 10 | 00 | 0 | 0 | 0 | 0 | XX | 0 | 0 | X | X |
| **ALUWriteBack** | X | X | XX | 0 | 0 | 0 | 0 | XX | 0 | 1 | 1 | 0 |
| **MemWriteBack** | X | X | XX | 0 | 0 | 0 | 0 | XX | 0 | 1 | 0 | 1 |
| **Branch** | 1 | 00 | 01 | 0 | 0 | 0 | 0 | 01 | 0 | 0 | X | X |
| **Jump** | X | X | XX | 0 | 0 | 0 | 1 | 10 | 0 | 0 | X | X |

**Legend:** X = Don't care, XX = Don't care (2-bit)
