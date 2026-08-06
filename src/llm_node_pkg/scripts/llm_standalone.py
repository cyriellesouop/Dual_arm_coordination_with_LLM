#!/usr/bin/env python3
# scripts/llm_standalone.py
#
# Standalone test for the LLM conversation node.
# No ROS2 required — run directly with Python on Windows or WSL.
#
# Usage:
#   python llm_standalone.py               # voice input (mic + TTS)
#   python llm_standalone.py --no-voice    # keyboard input, still speaks responses
#   python llm_standalone.py --model mistral --whisper tiny
#
# Prerequisites:
#   pip install ollama openai-whisper pyttsx3 sounddevice scipy
#   ollama pull llama3      (or whichever model you choose)
#   ollama serve            (must be running in a separate terminal)

import argparse
import json
import os
import sys

# Allow importing llm_node_pkg from the parent directory
# (works both when run directly and when installed as a ROS2 package)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from llm_node_pkg.llm_core import listen, run_conversation, speak

# Mock objects used for standalone testing (no controller running)
MOCK_OBJECTS = [
    {'name': 'apple',    'x':  0.10, 'y':  0.20, 'z': 0.05},
    {'name': 'bottle',   'x': -0.20, 'y':  0.30, 'z': 0.12},
    {'name': 'red ball', 'x':  0.00, 'y':  0.10, 'z': 0.05},
    {'name': 'red ball', 'x':  0.15, 'y':  0.10, 'z': 0.05},
]


def print_header(use_voice, model, whisper_size, ollama_host):
    print('=' * 60)
    print('  AURO Robotic Arm — LLM Interface (Standalone Test)')
    print('=' * 60)
    print(f'  Ollama model : {model}')
    print(f'  Ollama host  : {ollama_host}')
    print(f'  Input        : {"microphone (Whisper " + whisper_size + ")" if use_voice else "keyboard"}')
    print(f'  Output       : spoken (pyttsx3) + printed')
    print()
    print('  Mock objects on table:')
    for obj in MOCK_OBJECTS:
        print(f'    {obj["name"]:12s}  x={obj["x"]:+.2f}  y={obj["y"]:+.2f}  z={obj["z"]:+.2f}')
    print('=' * 60)
    print()


def main():
    parser = argparse.ArgumentParser(
        description='Standalone LLM node test — no ROS2 needed.'
    )
    parser.add_argument(
        '--no-voice', action='store_true',
        help='Use keyboard input instead of microphone (useful for initial testing)',
    )
    parser.add_argument(
        '--model', default='llama3',
        help='Ollama model to use (default: llama3)',
    )
    parser.add_argument(
        '--whisper', default='base',
        choices=['tiny', 'base', 'small', 'medium'],
        help='Whisper model size — tiny is fastest, medium is most accurate (default: base)',
    )
    parser.add_argument(
        '--silence-duration', type=float, default=0.8,
        help='Seconds of silence after speech before stopping recording (default: 0.8)',
    )
    parser.add_argument(
        '--noise-multiplier', type=float, default=2.5,
        help='Speech threshold = ambient_rms * this value. '
             'Increase (e.g. 3.5) if a loud server fan triggers false detection (default: 2.5)',
    )
    parser.add_argument(
        '--tts-voice', default='Mark',
        help='Partial name of the Windows TTS voice to use (default: Mark). '
             'Run with --list-voices to see all available voices.',
    )
    parser.add_argument(
        '--list-voices', action='store_true',
        help='Print all available TTS voices and exit.',
    )
    parser.add_argument(
        '--ollama-host', default='http://localhost:11434',
        help='Ollama server URL (default: http://localhost:11434). '
             'From WSL use this same default if you have mirrored networking, '
             'or set to http://<windows-host-ip>:11434 otherwise.',
    )
    args = parser.parse_args()

    if args.list_voices:
        import pyttsx3
        engine = pyttsx3.init()
        for v in engine.getProperty('voices'):
            print(v.name)
        return

    use_voice = not args.no_voice
    print_header(use_voice, args.model, args.whisper, args.ollama_host)

    while True:
        try:
            selected = run_conversation(
                objects=MOCK_OBJECTS,
                model=args.model,
                whisper_model_size=args.whisper,
                silence_duration=args.silence_duration,
                noise_multiplier=args.noise_multiplier,
                tts_voice=args.tts_voice,
                use_voice=use_voice,
                ollama_host=args.ollama_host,
            )
        except KeyboardInterrupt:
            print('\n[Interrupted — exiting]')
            sys.exit(0)
        except RuntimeError as exc:
            print(f'\n[Error] {exc}', file=sys.stderr)
            sys.exit(1)

        print()
        print('=' * 60)
        print('  SELECTED OBJECT:')
        print(f'  {json.dumps(selected, indent=4)}')
        print('=' * 60)
        print()
        print('  (In the full system this JSON is published to /selected_object)')
        print()

        # Ask whether the user wants to go again
        again_prompt = 'Would you like to select another object?'
        if use_voice:
            speak(again_prompt)
            again = listen(whisper_model_size=args.whisper)
        else:
            again = input(f'{again_prompt} (yes/no): ').strip()

        yes_words = {'yes', 'yeah', 'sure', 'yep', 'another', 'more', 'ok', 'okay', 'please'}
        if not any(w in again.lower() for w in yes_words):
            speak('Goodbye!')
            break

        print()


if __name__ == '__main__':
    main()
