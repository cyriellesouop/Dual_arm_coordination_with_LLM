# llm_node_pkg

Voice-based LLM interface for the AURO robotic arm. Holds a natural language conversation with the user to determine which detected object they want retrieved, then publishes the selection to the controller. Also listens for a stop command while the arm is in motion.

---

## ROS2 Interface

### Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `/detected_objects` | `std_msgs/String` | JSON array of objects currently on the table. Triggers a new conversation. Only processed when the node is in the `WAITING_FOR_OBJECTS` state (i.e. arm is not already moving). |
| `/arm_status` | `std_msgs/String` | Arm state from the controller. `"moving"` activates the stop listener. `"ready"` resets the node to accept the next object list. |

### Published Topics

| Topic | Type | Description |
|---|---|---|
| `/selected_object` | `std_msgs/String` | JSON object of the user's confirmed selection. Published once the LLM has identified and confirmed the user's choice. |
| `/arm_stop` | `std_msgs/Bool` | Published as `true` if the user says "stop" while the arm is moving. The controller should pause the arm immediately on receipt. |

### Message Formats

**`/detected_objects`** (sent by the controller):
```json
[
  {"name": "apple",    "x": 0.10, "y": 0.20, "z": 0.05},
  {"name": "bottle",   "x": -0.20, "y": 0.30, "z": 0.12},
  {"name": "red ball", "x": 0.00, "y": 0.10, "z": 0.05}
]
```

**`/selected_object`** (published by this node):
```json
{"name": "apple", "x": 0.10, "y": 0.20, "z": 0.05}
```

**`/arm_status`** (sent by the controller):
- `"moving"`: arm has started moving toward the target
- `"ready"`: arm has returned to home position, ready for next selection

**`/arm_stop`** (published by this node):
- `true`: user issued a stop command, pause the arm immediately

---

## Node Behavior

### State Machine

The node cycles through three states:

- **WAITING_FOR_OBJECTS**: sits idle until `/detected_objects` arrives
- **CONVERSING**: runs the LLM conversation in a background thread until the user confirms a selection, then publishes to `/selected_object`
- **WAITING_FOR_ARM**: monitors `/arm_status`; a `"moving"` status starts the stop listener and a `"ready"` status announces completion and resets back to WAITING_FOR_OBJECTS

### Conversation Flow

1. Controller publishes detected objects to `/detected_objects`
2. Node announces available objects to the user via TTS
3. User speaks their selection; Whisper transcribes it
4. LLM handles the conversation, asks follow-up questions if needed (e.g. two red balls, which one?)
5. Once confirmed, the selected object is published to `/selected_object`
6. Node waits for `/arm_status: "moving"` then starts listening for a stop command
7. On `/arm_status: "ready"`, node announces completion and resets

### Controller Integration Checklist

The controller must:
- Publish a JSON array to `/detected_objects` when objects are detected and the arm is idle
- Publish `"moving"` to `/arm_status` after the arm begins moving toward the selected object
- Publish `"ready"` to `/arm_status` when the arm returns to its home position
- Subscribe to `/arm_stop` and immediately pause the arm when `true` is received

---

## Parameters

All parameters can be set via `config/llm_params.yaml` or overridden at launch.

| Parameter | Default | Description |
|---|---|---|
| `ollama_model` | `llama3` | Ollama model for conversation. Recommended: `llama3.2:3b` (fast on CPU). |
| `whisper_model` | `base` | Whisper model size. `tiny` is fastest, `medium` is most accurate, `base` is a good middle ground. |
| `use_voice` | `true` | `false` = keyboard input instead of microphone. Useful for testing. |
| `silence_duration` | `0.8` | Seconds of silence after speech before stopping the recording. |
| `noise_multiplier` | `2.5` | Speech threshold is ambient RMS multiplied by this value. Increase in loud environments. |
| `tts_rate` | `175` | TTS speech rate in words per minute (Windows/pyttsx3 only). |
| `tts_voice` | `Mark` | Partial TTS voice name (Windows/SAPI only). |
| `ollama_host` | `http://localhost:11434` | Ollama server URL. Works on WSL with mirrored networking enabled. |

---

## Dependencies

### System
```bash
sudo apt install -y espeak-ng alsa-utils pulseaudio-utils libasound2-plugins ffmpeg
```

### Python
```bash
pip install ollama faster-whisper pyttsx3 sounddevice scipy edge-tts
```

### Ollama (runs on Windows, accessible from WSL)
```bash
ollama pull llama3.2:3b
```

### WSL Audio Setup
WSL2 requires mirrored networking and WSLg for audio:

1. Add to `C:\Users\<you>\.wslconfig`:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
2. Restart WSL: run `wsl --shutdown` in PowerShell
3. Create `~/.asoundrc` in WSL:
   ```
   pcm.!default { type pulse }
   ctl.!default { type pulse }
   ```

WSLg provides PulseAudio automatically at `unix:/mnt/wslg/PulseServer`, no Windows-side PulseAudio install needed.

---

## Build and Run

```bash
# Build
cd ~/auro-final-project
source /opt/ros/humble/setup.bash
colcon build --packages-select llm_node_pkg
source install/setup.bash

# Run with voice input (default)
ros2 run llm_node_pkg llm_node --ros-args -p ollama_model:=llama3.2:3b

# Run with keyboard input (no mic needed)
ros2 run llm_node_pkg llm_node --ros-args -p use_voice:=false -p ollama_model:=llama3.2:3b

# Run via launch file
ros2 launch llm_node_pkg llm_node.launch.py
```

---

## Development Testing

`dev_run.sh` opens a 4-pane tmux session that simulates the full controller interface:

```bash
./dev_run.sh --voice       # voice input
./dev_run.sh               # keyboard input
```

| Pane | Contents |
|---|---|
| Left | `llm_node`: all conversation output and mic status |
| Top right | Mock object publisher, press Enter to inject `/detected_objects` |
| Middle right | Mock arm controller, auto-publishes `"moving"` then `"ready"`, handles `/arm_stop` |
| Bottom right | Live echo of `/selected_object` and `/arm_stop` |

---

## Standalone Testing (no ROS2 required)

```bash
# Voice input
python3 src/llm_node_pkg/scripts/llm_standalone.py --model llama3.2:3b

# Keyboard input
python3 src/llm_node_pkg/scripts/llm_standalone.py --model llama3.2:3b --no-voice
```
