# Deoxys Control

Real-time control framework for Franka Emika Panda robots. Desktop Python sends commands over ZMQ to a NUC running the C++ real-time controller.

## Project Structure

- `deoxys/deoxys/` — Python package (`pip install -e .` from `deoxys/`)
  - `franka_interface/franka_interface.py` — Main `FrankaInterface` class
  - `experimental/motion_utils.py` — High-level motion helpers
  - `utils/` — Config loading, transforms, IK, I/O devices, logging
  - `scripts/` — CLI entry points
  - `proto/` — Generated protobuf bindings
- `deoxys/config/` — YAML configs for robots, controllers, gripper
- `deoxys/examples/` — Usage examples (best reference for API patterns)
- `deoxys/franka-interface/` — C++ NUC-side real-time controller
- `deoxys/proto/franka-interface/` — Protobuf definitions (.proto files)

## Build

```bash
cd deoxys
# Desktop (Python)
pip install -r requirements.txt && pip install -e .
# OR
make -j build_deoxys=1

# NUC (C++)
make -j build_franka=1
```

## Architecture

- **Communication**: ZMQ PUB/SUB over TCP, protobuf serialization
- **Ports**: Arm control (5555/5556), gripper (5557/5558); second robot uses 5565/5566 and 5567/5568
- **Control flow**: Python `FrankaInterface` → ZMQ → C++ NUC controller → libfranka → robot
- **State flow**: Robot → libfranka → C++ publisher → ZMQ → Python state buffer

## Key Conventions

- Controller types are `UPPER_SNAKE_CASE` strings: `OSC_POSE`, `JOINT_POSITION`, etc.
- Config files use kebab-case: `osc-pose-controller.yml`
- Quaternions are `(x, y, z, w)` throughout
- Actions: last element is always gripper command (`>= 0` close, `< 0` open)
- Use `get_deoxys_example_logger()` for example scripts
- No automated test suite — test against hardware using `examples/` scripts

## CLI Entry Points

```bash
deoxys.get_controller_list    # List available controllers
deoxys.get_controller_info    # Controller details
deoxys.reset_joints           # Reset to home position
deoxys.print_robot_state      # Print current state
```
