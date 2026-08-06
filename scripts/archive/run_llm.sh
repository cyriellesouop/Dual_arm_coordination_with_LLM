#!/usr/bin/env bash
# run_llm.sh — Start the LLM node inside the Docker container.
#
# Before running this script, make sure Ollama is running on the HOST machine:
#   curl -fsSL https://ollama.com/install.sh | sh
#   ollama serve        (if not already running as a service)
#
# Then inside the container:
#   bash scripts/run_llm.sh

set -e

OLLAMA_HOST="http://localhost:11434"
MODEL="${1:-llama3.2:3b}"

# ── Check Ollama is reachable ─────────────────────────────────────────────────
echo "[llm] Checking Ollama at $OLLAMA_HOST ..."
if ! curl -sf "$OLLAMA_HOST" > /dev/null 2>&1; then
    echo ""
    echo "ERROR: Cannot reach Ollama at $OLLAMA_HOST"
    echo ""
    echo "On the HOST machine (outside Docker), run:"
    echo "  curl -fsSL https://ollama.com/install.sh | sh"
    echo "  ollama serve"
    echo ""
    exit 1
fi
echo "[llm] Ollama reachable."

# ── Pull model if ollama CLI is available ────────────────────────────────────
if command -v ollama &>/dev/null; then
    echo "[llm] Pulling model '$MODEL' (skipped if already cached) ..."
    ollama pull "$MODEL"
else
    echo "[llm] ollama CLI not in container — assuming model '$MODEL' is already pulled on host."
fi

# ── Source ROS2 and run ───────────────────────────────────────────────────────
echo "[llm] Starting LLM node (typed input, model=$MODEL) ..."
source /opt/ros/humble/setup.bash
source /auro-final-project/install/setup.bash

ros2 run llm_node_pkg llm_node \
    --ros-args \
    -p use_voice:=false \
    -p ollama_model:="$MODEL"
