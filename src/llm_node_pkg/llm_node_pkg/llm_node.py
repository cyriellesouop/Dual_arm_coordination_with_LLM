# llm_node.py
# ROS2 node that runs the voice conversation loop and publishes the
# user's object selection to /selected_object.

import json
import threading
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from llm_node_pkg.llm_core import listen_for_keyword, prewarm_models, run_conversation, speak


class State(Enum):
    WAITING_FOR_OBJECTS = auto()
    CONVERSING = auto()
    WAITING_FOR_ARM = auto()


class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')

        self.declare_parameter('ollama_model', 'llama3')
        self.declare_parameter('whisper_model', 'base')
        self.declare_parameter('silence_duration', 0.8)
        self.declare_parameter('noise_multiplier', 2.5)
        self.declare_parameter('tts_rate', 175)
        self.declare_parameter('tts_voice', 'Mark')
        self.declare_parameter('use_voice', True)
        self.declare_parameter('ollama_host', 'http://localhost:11434')

        self.ollama_model     = self.get_parameter('ollama_model').value
        self.whisper_model    = self.get_parameter('whisper_model').value
        self.silence_duration = self.get_parameter('silence_duration').value
        self.noise_multiplier = self.get_parameter('noise_multiplier').value
        self.tts_rate         = self.get_parameter('tts_rate').value
        self.tts_voice        = self.get_parameter('tts_voice').value
        self.use_voice        = self.get_parameter('use_voice').value
        self.ollama_host      = self.get_parameter('ollama_host').value

        self._state = State.WAITING_FOR_OBJECTS
        self._state_lock = threading.Lock()
        self._current_objects = []

        # Stop listener cancel signal
        self._stop_cancel = threading.Event()

        # Latched QoS so a late-starting LLM node still receives the last
        # /detected_objects message published by the controller.
        qos_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._qos_latched = qos_latched
        self._objects_sub = self.create_subscription(
            String, '/detected_objects', self._on_objects_received, qos_latched
        )
        self.create_subscription(String, '/arm_status', self._on_arm_status, 10)

        self._selection_pub = self.create_publisher(String, '/selected_object', 10)
        self._stop_pub = self.create_publisher(Bool, '/arm_stop', 10)

        self.get_logger().info(
            f'LLM node ready | model={self.ollama_model} '
            f'whisper={self.whisper_model} voice={self.use_voice} '
            f'ollama_host={self.ollama_host}'
        )

        # Pre-load Whisper models in background so they are ready before first use
        if self.use_voice:
            prewarm_models(self.whisper_model)

    def _on_objects_received(self, msg):
        with self._state_lock:
            if self._state != State.WAITING_FOR_OBJECTS:
                return

            try:
                objects = json.loads(msg.data)
            except json.JSONDecodeError as exc:
                self.get_logger().error(f'Bad /detected_objects JSON: {exc}')
                return

            if not objects:
                self.get_logger().warn('Received empty object list, ignoring.')
                return

            self._current_objects = objects
            self._state = State.CONVERSING

        thread = threading.Thread(target=self._conversation_thread, daemon=True)
        thread.start()

    def _on_arm_status(self, msg):
        status = msg.data.strip().lower()

        if status == 'moving':
            with self._state_lock:
                if self._state == State.WAITING_FOR_ARM:
                    self.get_logger().info('Arm moving, starting stop listener.')
                    self._stop_cancel.clear()
                    thread = threading.Thread(target=self._stop_listener_thread, daemon=True)
                    thread.start()

        elif status == 'ready':
            # Cancel any active stop listener
            self._stop_cancel.set()
            with self._state_lock:
                if self._state == State.WAITING_FOR_ARM:
                    self.get_logger().info('Arm ready, waiting for next object list.')
                    speak(
                        'The arm has finished. Ready for your next selection.',
                        self.tts_rate,
                    )
                    self._state = State.WAITING_FOR_OBJECTS
                    self._current_objects = []

                    # Re-create the /detected_objects subscription so the
                    # TRANSIENT_LOCAL latched message is delivered again.
                    # Without this, the LLM waits forever if the scene hasn't
                    # changed (same objects still on the table).
                    self.destroy_subscription(self._objects_sub)
                    self._objects_sub = self.create_subscription(
                        String, '/detected_objects',
                        self._on_objects_received, self._qos_latched,
                    )

    def _conversation_thread(self):
        self.get_logger().info(
            f'Starting conversation ({len(self._current_objects)} objects detected).'
        )
        try:
            selected = run_conversation(
                objects=self._current_objects,
                model=self.ollama_model,
                whisper_model_size=self.whisper_model,
                silence_duration=self.silence_duration,
                noise_multiplier=self.noise_multiplier,
                tts_rate=self.tts_rate,
                tts_voice=self.tts_voice,
                use_voice=self.use_voice,
                ollama_host=self.ollama_host,
            )
        except Exception as exc:
            self.get_logger().error(f'Conversation failed: {exc}')
            with self._state_lock:
                self._state = State.WAITING_FOR_OBJECTS
                # Re-create subscription to receive the latched message again
                self.destroy_subscription(self._objects_sub)
                self._objects_sub = self.create_subscription(
                    String, '/detected_objects',
                    self._on_objects_received, self._qos_latched,
                )
            return

        # The LLM now returns only {"name": "..."}.  Look up the full
        # object (with coordinates) from the stored camera data so we
        # never rely on the LLM for numbers.
        selected_name = selected.get('name', '').strip().lower()
        full_object = None
        for obj in self._current_objects:
            if obj.get('name', '').strip().lower() == selected_name:
                full_object = obj
                break

        if full_object is None:
            self.get_logger().warn(
                f'LLM selected "{selected_name}" but no match in current objects. '
                f'Available: {[o.get("name") for o in self._current_objects]}'
            )
            with self._state_lock:
                self._state = State.WAITING_FOR_OBJECTS
                self.destroy_subscription(self._objects_sub)
                self._objects_sub = self.create_subscription(
                    String, '/detected_objects',
                    self._on_objects_received, self._qos_latched,
                )
            return

        out_msg = String()
        out_msg.data = json.dumps(full_object)
        self._selection_pub.publish(out_msg)
        self.get_logger().info(f'Published /selected_object: {out_msg.data}')

        with self._state_lock:
            self._state = State.WAITING_FOR_ARM

    def _stop_listener_thread(self):
        # runs concurrently with arm motion, cancelled when arm reports ready
        self.get_logger().info('Stop listener active.')

        if self.use_voice:
            # Start listening right away so the user can say stop during the announcement
            result_holder = [False]
            def _listen():
                result_holder[0] = listen_for_keyword(
                    keyword='stop',
                    cancel_event=self._stop_cancel,
                    noise_multiplier=self.noise_multiplier,
                )
            listener = threading.Thread(target=_listen, daemon=True)
            listener.start()

            # Announce while the listener is already running
            speak('Arm moving. Say stop to pause.', self.tts_rate)

            listener.join()
            detected = result_holder[0]
        else:
            speak('Arm moving. Type stop to pause.', self.tts_rate)
            detected = False
            while not self._stop_cancel.is_set():
                try:
                    user_input = input('Type "stop" to pause the arm: ').strip()
                except EOFError:
                    break
                if 'stop' in user_input.lower():
                    detected = True
                    break

        if detected and not self._stop_cancel.is_set():
            self.get_logger().info('Stop command detected, publishing to /arm_stop.')
            stop_msg = Bool()
            stop_msg.data = True
            self._stop_pub.publish(stop_msg)
            speak('Stopping the arm.', self.tts_rate)


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
