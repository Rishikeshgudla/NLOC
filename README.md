# Design and Implementation of a Bio-Inspired Neural Localization (N-LOC) architecture for Visual place localization.

To design and implement a bio-inspired neural localization accelerator (N-LOC) on a single FPGA platform that recognizes predefined locations from structured visual landmark descriptors and orientation data received via UART, achieving deterministic low-latency place recognition with minimal hardware resources.



---

## Objectives:

1. TO study and analyze the bio-inspired N-LOC architecture for visual place localization.
2. Design and code the complete N-LOC Datapath in VHDLJVerilog comprising Signature Layer, Azimuth Layer, Spatial Working Memory, Place Cell Layer, and Winner-Takes-All network.
3. TO functionally verify the N -LOC architecture using RTL simulation.

---

## Proposal:
Bio-inspired neural architectures provide an altemative approach for visual place localization by representing spatial information through neural processing and stored place memories. However, implementing such computational architectures on conventional processors can introduce processing overhead and may not fully exploit the parallelism inherent in neural computations. FPGA-based implementation provides an opportunity to realize these operations as dedicated hardware, enabling parallel processing, deterministic execution, and potentially reduced latency and power consumption. Therefore, this project proposes the Design and implementation Of a Bio-Inspired Neural Localization (N -LOC) architecture for visual place recognition. the proposed system focuses on the hardware realization Of the core N-LOC architecture consisting Of the Signatule Layer (SL), Azimuth Layer (AL), Spatial Working Memory (SWM), Place Cell Layer (PCL), and Winner-Takes-All (WTA) decision module. Reference visual feature data and test input data are initially provided from a PC through a U ART-based interface, while orientation information is supplied as an input to the Azimuth Layer. The N -LOC architecture processes the input information, compares the current spatial representation with stored reference place memories, and identifies the corresponding location as a Place ID. The architecture will be developed using VHDL-based RTL design, with individual modules functionally verified simulation before integration and synthesis.

---

## Abstarct: 

The increasing demand for autonomous and intelligent navigation systems has created a need for efficient hardware architectures capable Of perfoming real-time visual localization with low latency and deterministic execution. Conventional implementations of neural and computationally intensive localization algorithms primarily rely on general-purpose processors, which may result in increased computational overhead and power consumption. FPGA-based hardware implementation provides an altenative by exploiting parallelism and dedicated hardware resources for efficient real-time processing. This project presents the design and FPGA implementation of a Bio-inspired Neural Architecture (N-LOC) for visual place localization. Inspired by the way the human brain recognizes and remembers places, the proposed system implements the core N-LOC processing architecture in synthesizable RTL. Ihe architecture consists of the Signature Layer (SL), Azimuth Layer (AL), spatial Working Memory (SWM), Place cell Layer (PCL), and Winner-Takes-A11 (WI'A) decision module. Reference visual feature data and test input data are provided to the FPGA through a UART-based interface from a PC, while orientation information is supplied as an input to the Azimuth Layer. N-LOC architecture processes the input representation, compares it with stored reference place infornation, and determines the most probable location as a corresponding Place 11). The proposed architecture is developed using VHDL RTL, functionally verified simulation, and synthesized for implementation on a suitable FPGA development platform. The implementation is evaluated in terms of functional correctness, processing latency, FPGA resource utilization, maximum operating frequency, and memory requirements. The developed N-LOC hardware processor serves as an independent FPGA-based localization core that can subsequently be integrated with the visual sensing and orientation subsystems of the complete autonomous vehicle localization system


## 👨‍💻 Author

**Rishikesh Gudla**

Electronics & Communication Engineering Student

Interested in:
- Aerospace Systems
- Satellite Communication
- Embedded Systems
- FPGA Design
- Python Development

---

## 📜 License

This project is licensed under the MIT License.

---

## ⭐ Support


This project is provided for educational and learning purposes📚. Feel free to modify, improve, and use it.
If you found this project useful, consider giving it a ⭐ on GitHub.
