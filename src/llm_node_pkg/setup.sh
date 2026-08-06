#!/usr/bin/env bash
# setup.sh - one-shot setup for llm_node_pkg
#
# Usage:
#   chmod +x setup.sh
#   ./setup.sh                        # installs llama3.2:3b (default, fast on CPU)
#   ./setup.sh --model llama3         # use a different model
#   ./setup.sh --no-conda             # skip conda, use pip directly
#
# Supports: Ubuntu 20.04 / 22.04 / 24.04 (including WSL)
# Ollama on Windows must be installed manually from https://ollama.com

set -e  # exit on any error

# Defaults
OLLAMA_MODEL="llama3.2:3b"
CONDA_ENV="auro"
FORCE_PIP=false

# Argument parsing
while [[ $# -gt 0 ]]; do
    case $1 in
        --model)   OLLAMA_MODEL="$2"; shift 2 ;;
        --no-conda) FORCE_PIP=true; shift ;;
        *) echo "Unknown argument: $1"; exit 1 ;;
    esac
done

# Helpers
info()    { echo -e "\033[1;34m[setup]\033[0m $*"; }
success() { echo -e "\033[1;32m[setup]\033[0m $*"; }
warn()    { echo -e "\033[1;33m[setup]\033[0m $*"; }
error()   { echo -e "\033[1;31m[setup]\033[0m $*" >&2; exit 1; }

# OS check
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" ]]; then
    error "Run this script inside WSL (Ubuntu) or Linux, not Windows directly."
fi

info "Starting llm_node_pkg setup on $(lsb_release -ds 2>/dev/null || uname -s)"
echo

# System dependencies
info "Installing system dependencies ..."
sudo apt-get update -qq
sudo apt-get install -y -qq \
    ffmpeg espeak portaudio19-dev \
    curl zstd \
    python3-venv python3-pip > /dev/null
success "System dependencies installed."
echo

# WSL audio setup (.asoundrc)
ASOUNDRC="$HOME/.asoundrc"
if [[ -f "$ASOUNDRC" ]]; then
    info ".asoundrc already exists, skipping."
else
    info "Creating ~/.asoundrc for WSL PulseAudio ..."
    cat > "$ASOUNDRC" <<'EOF'
pcm.!default { type pulse }
ctl.!default { type pulse }
EOF
    success "~/.asoundrc created."
    warn "WSL audio also requires networkingMode=mirrored in C:\\Users\\<you>\\.wslconfig"
    warn "Run 'wsl --shutdown' in PowerShell after setting that, then restart WSL."
fi
echo

# Python environment
CONDA_FOUND=false
if [[ "$FORCE_PIP" == false ]] && command -v conda &>/dev/null; then
    CONDA_FOUND=true
fi

VENV_DIR="$HOME/auro-venv"

if [[ "$CONDA_FOUND" == true ]]; then
    info "Conda detected."
    if conda env list | grep -q "^${CONDA_ENV} "; then
        info "Conda env '${CONDA_ENV}' already exists, updating it."
    else
        info "Creating conda env '${CONDA_ENV}' with Python 3.10 ..."
        conda create -y -n "$CONDA_ENV" python=3.10 > /dev/null
        success "Conda env '${CONDA_ENV}' created."
    fi
    info "Installing Python packages into conda env '${CONDA_ENV}' ..."
    conda run -n "$CONDA_ENV" pip install -q \
        ollama \
        faster-whisper \
        edge-tts \
        pyttsx3 \
        sounddevice \
        scipy
    success "Python packages installed into conda env '${CONDA_ENV}'."
    PIP_NOTE="conda run -n ${CONDA_ENV} python"
    ACTIVATE_NOTE="conda activate ${CONDA_ENV}"
else
    if [[ -d "$VENV_DIR" ]]; then
        info "Virtual environment already exists at ${VENV_DIR} — updating packages."
    else
        info "Creating Python virtual environment at ${VENV_DIR} ..."
        python3 -m venv "$VENV_DIR" --system-site-packages
        success "Virtual environment created."
    fi
    info "Installing Python packages into venv ..."
    "$VENV_DIR/bin/pip" install -q \
        ollama \
        faster-whisper \
        edge-tts \
        pyttsx3 \
        sounddevice \
        scipy
    success "Python packages installed into venv."

    # Add venv site-packages to PYTHONPATH in ~/.bashrc so ROS2 can find them
    PYTHON_VERSION=$("$VENV_DIR/bin/python" -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
    VENV_SITE="$VENV_DIR/lib/python${PYTHON_VERSION}/site-packages"
    BASHRC_LINE="export PYTHONPATH=\"${VENV_SITE}:\$PYTHONPATH\""
    if grep -qF "$VENV_SITE" "$HOME/.bashrc"; then
        info "PYTHONPATH already set in ~/.bashrc — skipping."
    else
        echo "" >> "$HOME/.bashrc"
        echo "# auro-venv site-packages for ROS2" >> "$HOME/.bashrc"
        echo "$BASHRC_LINE" >> "$HOME/.bashrc"
        success "Added venv to PYTHONPATH in ~/.bashrc."
        warn "Run 'source ~/.bashrc' or open a new terminal for this to take effect."
    fi

    PIP_NOTE="$VENV_DIR/bin/python"
    ACTIVATE_NOTE="source $VENV_DIR/bin/activate"
fi
echo

# GPU / CUDA check
GPU_STATUS="CPU only (no NVIDIA GPU detected)"
if command -v nvidia-smi &>/dev/null; then
    GPU_NAME=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1)
    CUDA_VERSION=$(nvidia-smi | grep -oP "CUDA Version: \K[0-9.]+" 2>/dev/null || echo "unknown")
    if [[ -n "$GPU_NAME" ]]; then
        GPU_STATUS="$GPU_NAME (CUDA $CUDA_VERSION), Ollama will use GPU automatically"
        success "NVIDIA GPU detected: $GPU_NAME (CUDA $CUDA_VERSION)"
        info "Ollama will use the GPU automatically — no extra config needed."
    fi
else
    # nvidia-smi not found — check if an NVIDIA GPU exists but has no drivers
    if lspci 2>/dev/null | grep -qi nvidia; then
        warn "NVIDIA GPU detected but drivers are not installed."
        warn "Ollama will fall back to CPU. To enable GPU acceleration:"
        warn "  https://docs.nvidia.com/cuda/cuda-installation-guide-linux/"
        warn "Install drivers first, then re-run this script."
        GPU_STATUS="NVIDIA GPU found but CUDA drivers missing — running on CPU"
    else
        info "No NVIDIA GPU detected — Ollama will run on CPU."
    fi
fi
echo

# Ollama
if command -v ollama &>/dev/null; then
    CURRENT_OLLAMA=$(ollama --version 2>/dev/null || echo "installed")
    info "Ollama already installed: ${CURRENT_OLLAMA}"
else
    info "Installing Ollama ..."
    curl -fsSL https://ollama.com/install.sh | sh
    success "Ollama installed: $(ollama --version 2>/dev/null || echo 'ok')"
fi
echo

# Pull model
info "Pulling Ollama model '${OLLAMA_MODEL}' (this may take a few minutes) ..."
ollama pull "$OLLAMA_MODEL"
success "Model '${OLLAMA_MODEL}' ready."
echo

# Done
echo "============================================================"
success "Setup complete!"
echo
echo "  Model pulled : ${OLLAMA_MODEL}"
echo "  GPU status   : ${GPU_STATUS}"
echo
echo "  Next steps:"
echo
echo "  Activate your environment first (required before every session):"
echo "       ${ACTIVATE_NOTE}"
echo
echo "  1. Standalone test (no ROS2 needed):"
echo "       python src/llm_node_pkg/scripts/llm_standalone.py \\"
echo "           --model ${OLLAMA_MODEL} --no-voice"
echo
echo "  2. ROS2 build (source ROS2 first):"
echo "       source /opt/ros/humble/setup.bash"
echo "       colcon build --packages-select llm_node_pkg"
echo "       source install/setup.bash"
echo "       ros2 run llm_node_pkg llm_node \\"
echo "           --ros-args -p use_voice:=false -p ollama_model:=${OLLAMA_MODEL}"
echo
echo "  See src/llm_node_pkg/README.md for full documentation."
echo "============================================================"
