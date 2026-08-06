# llm_node_pkg/llm_core.py
#
# Shared LLM/STT/TTS logic used by both the standalone test script
# and the ROS2 LLM node.
#
# Dependencies (pip install):
#   ollama faster-whisper pyttsx3 sounddevice scipy edge-tts
# TTS backend: edge-tts (Linux/WSL) or pyttsx3 (Windows)
# STT backend: faster-whisper (4x faster than openai-whisper on CPU)

import json
import os
import re
import sys
import tempfile
import threading
import warnings

import ollama

warnings.filterwarnings('ignore', category=UserWarning)

# Suppress HuggingFace Hub unauthenticated token warning
os.environ.setdefault('HF_HUB_VERBOSITY', 'error')

# Serialize all TTS calls so multiple threads never speak simultaneously
_tts_lock = threading.Lock()

# ---------------------------------------------------------------------------
# Lazy-loaded singletons
# ---------------------------------------------------------------------------

_tts_engine = None
_whisper_model = None        # faster-whisper model for conversation (base)
_whisper_tiny_model = None   # faster-whisper tiny model for keyword detection
_calibrated_threshold = None


# ---------------------------------------------------------------------------
# TTS
# ---------------------------------------------------------------------------

def _find_onecore_voice_id(name):
    if sys.platform != 'win32':
        return None
    try:
        import winreg
        base = r'SOFTWARE\Microsoft\Speech_OneCore\Voices\Tokens'
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, base) as key:
            i = 0
            while True:
                try:
                    subkey_name = winreg.EnumKey(key, i)
                    if name.lower() in subkey_name.lower():
                        return f'HKEY_LOCAL_MACHINE\\{base}\\{subkey_name}'
                    i += 1
                except OSError:
                    break
    except OSError:
        pass
    return None


def _get_tts_engine(rate=175, voice_name=''):
    global _tts_engine
    if _tts_engine is None:
        try:
            import pyttsx3
            _tts_engine = pyttsx3.init()
            _tts_engine.setProperty('rate', rate)
            if voice_name and sys.platform == 'win32':
                voices = _tts_engine.getProperty('voices')
                match = next(
                    (v for v in voices if voice_name.lower() in v.name.lower()), None
                )
                if match:
                    _tts_engine.setProperty('voice', match.id)
                    print(f'[TTS] Using voice: {match.name}')
                else:
                    onecore_id = _find_onecore_voice_id(voice_name)
                    if onecore_id:
                        _tts_engine.setProperty('voice', onecore_id)
                        print(f'[TTS] Using OneCore voice: {voice_name}')
                    else:
                        available = [v.name for v in voices]
                        print(
                            f'[TTS] Voice "{voice_name}" not found. '
                            f'Available: {available}',
                            file=sys.stderr,
                        )
        except Exception as exc:
            print(f'[TTS] pyttsx3 init failed: {exc}', file=sys.stderr)
    return _tts_engine


def _generate_mp3(text):
    # runs edge-tts and returns the path to the generated mp3
    import subprocess
    tmp_path = None
    with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as tmp:
        tmp_path = tmp.name
    subprocess.run(
        ['edge-tts', '--voice', 'en-US-AriaNeural', '--text', text, '--write-media', tmp_path],
        check=True, capture_output=True,
    )
    return tmp_path


def _speak_linux(text, interrupt_event=None):
    # speaks via edge-tts + ffplay on Linux/WSL, kills ffplay if interrupt_event is set
    import subprocess
    import shutil
    if shutil.which('ffplay') is None:
        print('[TTS] ffplay not found, install ffmpeg for audio output.', file=sys.stderr)
        return
    tmp_path = None
    try:
        tmp_path = _generate_mp3(text)
        if interrupt_event and interrupt_event.is_set():
            return
        proc = subprocess.Popen(
            ['ffplay', '-nodisp', '-autoexit', tmp_path],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        if interrupt_event:
            while proc.poll() is None:
                if interrupt_event.is_set():
                    proc.kill()
                    proc.wait()
                    return
                threading.Event().wait(0.05)
        else:
            proc.wait()
    except Exception as exc:
        print(f'[TTS] edge-tts error: {exc}', file=sys.stderr)
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


def speak(text, tts_rate=175, tts_voice='', print_text=True):
    # prints and speaks text, serialized so multiple threads don't overlap
    if print_text:
        print(f'[Robot]: {text}')
    with _tts_lock:
        if sys.platform != 'win32':
            _speak_linux(text)
            return
        engine = _get_tts_engine(tts_rate, tts_voice)
        if engine is not None:
            try:
                engine.say(text)
                engine.runAndWait()
            except Exception as exc:
                print(f'[TTS] speak error: {exc}', file=sys.stderr)



# ---------------------------------------------------------------------------
# STT (faster-whisper)
# ---------------------------------------------------------------------------

def _get_whisper_model(model_size='base'):
    # loads and caches the base Whisper model used for conversation
    global _whisper_model
    if _whisper_model is None:
        from faster_whisper import WhisperModel
        print(f'[STT] Loading Whisper {model_size} model ...')
        _whisper_model = WhisperModel(model_size, device='cpu', compute_type='int8')
        print(f'[STT] Whisper {model_size} ready.')
    return _whisper_model


def _get_whisper_tiny():
    # loads and caches the tiny Whisper model used for keyword detection
    global _whisper_tiny_model
    if _whisper_tiny_model is None:
        from faster_whisper import WhisperModel
        _whisper_tiny_model = WhisperModel('tiny', device='cpu', compute_type='int8')
    return _whisper_tiny_model


def prewarm_models(whisper_model_size='base', sample_rate=16000, noise_multiplier=2.5):
    # loads both Whisper models and calibrates ambient noise in the background at startup
    threading.Thread(target=_get_whisper_model, args=(whisper_model_size,), daemon=True).start()
    threading.Thread(target=_get_whisper_tiny, daemon=True).start()
    threading.Thread(target=_calibrate, args=(sample_rate, noise_multiplier), daemon=True).start()


def _transcribe(audio_path, model_size='base', tiny=False):
    # transcribes a WAV file and returns the stripped text
    try:
        model = _get_whisper_tiny() if tiny else _get_whisper_model(model_size)
        segments, _ = model.transcribe(audio_path, beam_size=1)
        return ' '.join(s.text for s in segments).strip()
    except Exception as exc:
        print(f'[STT] Transcription failed: {exc}', file=sys.stderr)
        return ''


def _calibrate(sample_rate, noise_multiplier):
    # records 0.5s of ambient noise and sets the global VAD threshold
    global _calibrated_threshold
    if _calibrated_threshold is not None:
        return _calibrated_threshold
    try:
        import sounddevice as sd
        import numpy as np
        print('[STT] Calibrating to ambient noise ...')
        ambient = sd.rec(int(0.5 * sample_rate), samplerate=sample_rate, channels=1, dtype='int16')
        sd.wait()
        ambient_rms = float(np.sqrt(np.mean(ambient.astype(np.float32) ** 2)))
        _calibrated_threshold = max(200.0, ambient_rms * noise_multiplier)
        print(f'[STT] Ambient RMS: {ambient_rms:.0f}  |  Threshold: {_calibrated_threshold:.0f}')
    except Exception as exc:
        print(f'[STT] Calibration failed: {exc}', file=sys.stderr)
        _calibrated_threshold = 300.0
    return _calibrated_threshold


def listen(
    whisper_model_size='base',
    sample_rate=16000,
    silence_duration=0.8,
    max_duration=30,
    noise_multiplier=2.5,
    on_speech_start=None,
):
    # records from mic using VAD, transcribes with Whisper, falls back to keyboard if audio is unavailable
    try:
        import sounddevice as sd
        import scipy.io.wavfile as wav_io
        import numpy as np
    except ImportError as exc:
        print(f'[STT] Missing audio dependency ({exc}). Falling back to keyboard.', file=sys.stderr)
        return input('You (type): ').strip()

    # Pre-load model while we wait for speech (hides load time on first call)
    threading.Thread(target=_get_whisper_model, args=(whisper_model_size,), daemon=True).start()

    threshold = _calibrate(sample_rate, noise_multiplier)

    chunk_size = 1024
    silent_chunks_needed = int(silence_duration * sample_rate / chunk_size)
    max_chunks = int(max_duration * sample_rate / chunk_size)

    audio_chunks = []
    speech_started = False
    silent_chunk_count = 0

    print('[STT] Listening ...')
    try:
        with sd.InputStream(samplerate=sample_rate, channels=1, dtype='int16') as stream:
            for _ in range(max_chunks):
                chunk, _ = stream.read(chunk_size)
                rms = float(np.sqrt(np.mean(chunk.astype(np.float32) ** 2)))

                if rms > threshold:
                    if not speech_started:
                        speech_started = True
                        print('[STT] Recording ...')
                        if on_speech_start:
                            on_speech_start()
                    silent_chunk_count = 0
                    audio_chunks.append(chunk.copy())
                elif speech_started:
                    audio_chunks.append(chunk.copy())
                    silent_chunk_count += 1
                    if silent_chunk_count >= silent_chunks_needed:
                        break
    except Exception as exc:
        print(f'[STT] Recording failed ({exc}). Falling back to keyboard.', file=sys.stderr)
        return input('You (type): ').strip()

    if not audio_chunks:
        print('[STT] No speech detected.')
        return ''

    audio = np.concatenate(audio_chunks, axis=0)
    tmp_path = None
    try:
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
            tmp_path = tmp.name
        wav_io.write(tmp_path, sample_rate, audio)
        text = _transcribe(tmp_path, model_size=whisper_model_size)
        print(f'[You said]: {text}')
        return text
    finally:
        if tmp_path and os.path.exists(tmp_path):
            os.unlink(tmp_path)


def listen_for_keyword(
    keyword,
    cancel_event,
    clip_duration=1.5,
    sample_rate=16000,
    noise_multiplier=2.5,
):
    # records short clips and checks for keyword, returns True if detected or False if cancelled
    try:
        import sounddevice as sd
        import scipy.io.wavfile as wav_io
        import numpy as np
    except ImportError as exc:
        print(f'[STT] Missing audio dependency: {exc}', file=sys.stderr)
        return False

    # Pre-load tiny model and calibrate (both instant if prewarm_models() was called)
    _get_whisper_tiny()
    _calibrate(sample_rate, noise_multiplier)

    clip_samples = int(clip_duration * sample_rate)
    print(f'[STT] Listening for "{keyword}" ...')

    while not cancel_event.is_set():
        try:
            audio = sd.rec(clip_samples, samplerate=sample_rate, channels=1, dtype='int16')
            sd.wait()
        except Exception as exc:
            print(f'[STT] Clip recording failed: {exc}', file=sys.stderr)
            break

        if cancel_event.is_set():
            break

        # skip silent clips, only transcribe if there's actual speech energy
        import numpy as np
        rms = float(np.sqrt(np.mean(audio.astype(np.float32) ** 2)))
        if rms < (_calibrated_threshold or 300.0):
            continue

        print('[STT] Recording ...')
        tmp_path = None
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
                tmp_path = tmp.name
            wav_io.write(tmp_path, sample_rate, audio)
            text = _transcribe(tmp_path, tiny=True)
            if keyword.lower() in text.lower():
                print(f'[STT] Keyword detected: "{text}"')
                return True
        finally:
            if tmp_path and os.path.exists(tmp_path):
                os.unlink(tmp_path)

    return False


# ---------------------------------------------------------------------------
# LLM conversation
# ---------------------------------------------------------------------------

def parse_llm_selection(text):
    # scans LLM reply for a JSON object with a name field, returns dict or None
    for match in re.findall(r'\{[^{}]+\}', text):
        try:
            obj = json.loads(match)
            if 'name' in obj:
                return {'name': str(obj['name'])}
        except (json.JSONDecodeError, ValueError, TypeError):
            continue
    return None


def _build_system_prompt(objects):
    # Only pass object names to the LLM — no coordinates, no types.
    # The arm controller already has the full object data; the LLM just
    # needs to help the user pick one by name.
    name_list = [obj['name'] if isinstance(obj, dict) else str(obj) for obj in objects]
    names_str = ', '.join(name_list)
    return (
        'You are the voice interface for a robotic arm assistant. '
        'Your job is to help a user pick an object from a table so the arm can retrieve it.\n\n'
        'IMPORTANT: The ONLY objects on the table are: ' + names_str + '. '
        'There are EXACTLY ' + str(len(name_list)) + ' objects. '
        'Do NOT add, invent, or duplicate any objects.\n\n'
        'Your instructions:\n'
        '1. Announce the available objects in ONE short sentence. '
        'Just list the object names simply. '
        'If there are duplicates, say the count (e.g. "two red balls"). '
        'Example: "I see an orange and a red ball. What would you like?"\n'
        '2. If the user asks for something not on the list, politely say it was not detected '
        'and ask them to choose something that is available.\n'
        '3. IMPORTANT: Only output a selection when the user has clearly and explicitly named a '
        'specific object. Do NOT assume or guess — if the user says something vague like '
        '"yeah", "sure", "okay", or anything unclear, ask them to clarify what they want.\n'
        '4. Once the user has clearly named an object, respond with ONLY a JSON object '
        'and absolutely nothing else — no explanation, no extra text:\n'
        'EXAMPLE: {"name": "orange"}\n\n'
        'The JSON must contain ONLY the "name" field. Do NOT include coordinates, '
        'numbers, or any other fields.\n\n'
        'Keep every response to 1-2 sentences maximum. Be natural and brief.'
    )


def run_conversation(
    objects,
    model='llama3',
    whisper_model_size='base',
    silence_duration=0.8,
    noise_multiplier=2.5,
    tts_rate=175,
    tts_voice='',
    use_voice=True,
    ollama_host=None,
):
    # runs a multi-turn conversation until the user picks an object, returns {name, x, y, z}
    if ollama_host is None:
        ollama_host = os.environ.get('OLLAMA_HOST', 'http://localhost:11434')

    client = ollama.Client(host=ollama_host)

    messages = [
        {'role': 'system', 'content': _build_system_prompt(objects)},
        {'role': 'user', 'content': 'Hello, what objects are on the table?'},
    ]

    user_has_spoken = False

    while True:
        try:
            stream = client.chat(model=model, messages=messages, stream=True)
            reply = ''
            print('[Robot]: ', end='', flush=True)
            for chunk in stream:
                token = chunk['message']['content']
                print(token, end='', flush=True)
                reply += token
            print()
        except Exception as exc:
            raise RuntimeError(
                f'Ollama unreachable at {ollama_host}: {exc}\n'
                'Ensure Ollama is running.\n'
                'From WSL, try: curl http://localhost:11434'
            ) from exc

        messages.append({'role': 'assistant', 'content': reply})

        # only check for a selection after the user has spoken, because small models
        # like llama3.2:3b sometimes emit JSON in their opening greeting
        if user_has_spoken:
            selection = parse_llm_selection(reply)
            if selection:
                msg = f"Got it. I'll retrieve the {selection['name']} for you."
                if use_voice:
                    speak(msg, tts_rate, tts_voice)
                else:
                    print(f'[Robot]: {msg}')
                return selection

        # In text mode the reply was already streamed to the console,
        # so calling speak() here would just block the input prompt.
        if use_voice:
            speak(reply, tts_rate, tts_voice, print_text=False)

        if use_voice:
            user_input = listen(
                whisper_model_size=whisper_model_size,
                silence_duration=silence_duration,
                noise_multiplier=noise_multiplier,
            )
        else:
            try:
                user_input = input('You: ').strip()
            except EOFError:
                user_input = ''

        user_has_spoken = True

        if not user_input:
            user_input = 'Sorry, I did not catch that.'

        messages.append({'role': 'user', 'content': user_input})
