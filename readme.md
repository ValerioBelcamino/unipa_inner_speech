# My ROS 2 Package

This repository contains a ROS 2 package. To get started, follow the instructions below to set up your development environment and build the workspace.

## Installation

### Prerequisites

- **Operating System:** Ubuntu 22.04 or newer  
- **ROS 2 Distribution:** Humble Hawksbill  
  Make sure you have [ROS 2 Humble installed](https://docs.ros.org/en/humble/Installation.html).

### Setting Up the Workspace

If you don't already have a ROS 2 workspace, create one:

```bash
# Choose your workspace location
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Clone the Repository

Clone this repository inside the `src` directory:

```bash
git clone https://github.com/ValerioBelcamino/unipa_inner_speech
```

### Install Python Requirements

Go to the root of the workspace and install the required Python packages:

```bash
cd ~/ros2_ws
pip install -r src/unipa_inner_speech/requirements.txt
```

### Build the Workspace

Build the workspace using `colcon`:

```bash
colcon build
```

Then, source the setup file:

```bash
source install/setup.bash
```

## Usage

_TODO: Add usage instructions here._

## License

_TODO: Add license information here._
