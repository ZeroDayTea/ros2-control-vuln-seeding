# ROS2 Controller Vuln Seeding

Automated program repair of vulnerabilities in a ROS2 Controller

## Setup

Must be on Ubuntu 22.04 or higher for this to work. Has only been tested on Ubuntu 24.04.
```
./install_ros2_jazzy.sh
./copy_vuln.sh safe
./build.sh
```

## Running an example

```
./copy_vuln.sh safe[name of vuln]
colcon build --symlink-install --packages-select ros2_control_demo_example_7
./build_controllers.sh

[in one workspace]
./start_controllers.sh
[in a second workspace]
./controller.sh
[in a third workspace]
python3 voter.py
[in a fourth workspace]
./send_trajectory.sh

[afterwards]
./cleanup.sh
```

## Vulnerability Categories

### Easy
| Vulnerability | Description | Fix Type |
|---------------|-------------|----------|
| `bug_oob_array_access` | Off-by-one array access (`<=` instead of `<`) | Change operator |
| `bug_type_casting` | Incorrect type casting in delta calculation | Remove cast |
| `bug_type_casting_2` | Unnecessary type casting in interpolation | Remove cast |
| `bug_floating_point` | Floating Point Exception from accidental divizion by 0 | Add bounds check |
| `bug_interpolate_half_always` | Mathematical mistake. Always uses midpoint instead of scaling by delta | Replace formula |

### Medium
| Vulnerability | Description | Fix Type |
|---------------|-------------|----------|
| `vuln_stack_bof` | Stack buffer overflow in acceleration processing | Add bounds check |
| `vuln_segfault` | Null pointer dereference in feature allowing inspecting memory | Add null check/memory check |
| `vuln_infinite_loop` | Loop that increments but doesn't check negative values | Add validation |

### Hard
| Vulnerability | Description | Fix Type |
|---------------|-------------|----------|
| `vuln_heap_bof` | Heap buffer overflow progressively corrupting more chunk metadata | Complex bounds + memory management |
| `vuln_uaf` | Use-after-free between acceleration/effort processing | Clear data after free and use separate heap chunks |
| `vuln_fmtstr_crash` | Format string vulnerability reading invalid memory | Input sanitization |
| `vuln_fmtstr_leak` | Format string vulnerability leaking stack data | Input sanitization |

### Reference Implementations
| Implementation | Description |
|----------------|-------------|
| `safe` | Correct working implementation |
| `original_bug` | Original bug used for testing system |
