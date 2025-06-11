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

### Launching the Full Architecture

To launch the entire system, use the main launch file provided in the `inner_speech` package:

```bash
ros2 launch inner_speech advisor.py
```

This launch file opens six terminal windows (or tabs), each running one of the system’s modules:

- **User Input**
- **Scope Detection**
- **Inner Speech**
- **Intent Recognition**
- **Query Generation**
- **Explainability**

---

### Switching Between Text and Speech Input

By default, the system uses the **text-based input** module. If you want to switch to **speech-based input** (using OpenAI's Whisper), the launch file supports this via a launch argument.

#### 🔹 Run with Text Input (default)

```bash
ros2 launch inner_speech advisor.py
```

#### 🔹 Run with Speech Input

```bash
ros2 launch inner_speech advisor.py use_audio:=true
```

> 💡 The speech input node runs a local instance of [Whisper](https://github.com/openai/whisper) for transcription.

---

### Important: Speech Dependencies Not Included

Due to the size and complexity of the dependencies (e.g., `whisper`, `torch`, etc.), **speech input dependencies are not included in `requirements.txt`**.  
If you want to use the speech-based input, you must install them manually:


> ⚠️ Make sure you have a working microphone and audio input permissions.

---

### Recommended Development & Debug Workflow

Since the launch system automatically closes terminals when any module crashes, we **strongly recommend manually launching each module** during development and testing.

You can do so using a multi-tab terminal tool (e.g. [Terminator](https://gnometerminator.blogspot.com/), `tmux`, etc.), and run the following commands in separate tabs:

```bash
# For text input
ros2 launch perception_modules user_input

# OR for speech input
ros2 run perception_nodes speech_recognition_node

# The rest of the modules
ros2 launch scope_detection scope_detection
ros2 launch inner_speech inner_speech
ros2 launch intent_recognition intent_recognition
ros2 launch query_generation query_generation
ros2 launch explainability explainability
```

This approach helps isolate issues and gives more control during integration and debugging.




