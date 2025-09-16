
# Moccer

## Introduction
Welcome to **Moccer**, an AI software tool designed for the [RoboCup Soccer Small Size League (SSL)](https://ssl.robocup.org/).

## System Requirements
Before using Moccer, ensure your system meets the following requirements:

- **Moccer-Sim**: A RoboCup SSL simulator — [GitHub Repository](https://github.com/mukuyo/Moccer-Sim)
- **Protocol Buffer Compiler**: `protoc` is required to compile protocol buffers.

## Getting Started
### 1. Build the Project
```bash
cd ~/ws/Moccer/
mkdir build && cd build
cmake ..
make
```

### 2. Run
```bash
make run
```

## Related Tools
Enhance your RoboCup SSL experience with these tools:

- [ssl-game-controller](https://github.com/RoboCup-SSL/ssl-game-controller): Official match controller.
- [ssl-autorefs](https://github.com/RoboCup-SSL/ssl-autorefs): Automated referee system.
- [Moccer-Sim](https://github.com/mukuyo/Moccer-Sim): SSL simulator.

## License
This project is licensed under the GNU General Public License v3 (GPL v3). See the [LICENSE](LICENSE) file for details.
