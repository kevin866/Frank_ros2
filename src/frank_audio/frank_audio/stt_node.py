import queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import scipy.signal as sps

import numpy as np
import sounddevice as sd
import webrtcvad
import collections
import threading
from faster_whisper import WhisperModel


# ── Intent mapping ─────────────────────────────────────────────────────────────
# Add more commands here as you expand. Each entry is a list of phrases that
# map to a single canonical intent token.
INTENT_MAP = {
    'track_face': [
        'come here', 'come to me', 'follow me', 'follow',
        'come', 'get here', 'come over', 'approach me', 'track face',
    ],
    'find_face': [
        'find me', 'look for me', 'search for me', 'find face',
        'where am i', 'look around',
    ],
    'stop': [
        'stop', 'halt', 'freeze', 'stay', 'stay there',
    ],
    'go_home': [
        'go home', 'return home', 'home', 'go back',
    ],
}


def classify_intent(text: str) -> tuple[str, str]:
    """
    Returns (intent, raw_text).
    intent is a canonical token like 'follow', or 'unknown' if no match.
    """
    lowered = text.lower().strip()
    for intent, phrases in INTENT_MAP.items():
        for phrase in phrases:
            if phrase in lowered:
                return intent, text
    return 'unknown', text


class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter('model_size', 'tiny.en')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('sample_rate', 16000)        # whisper/VAD target rate
        self.declare_parameter('vad_aggressiveness', 2)
        self.declare_parameter('silence_timeout', 1.0)
        self.declare_parameter('min_speech_duration', 0.4)
        self.declare_parameter('native_sample_rate', 44100) # actual mic rate
        self.declare_parameter('mic_device_index', -1)      # -1 = system default

        model_size               = self.get_parameter('model_size').get_parameter_value().string_value
        device                   = self.get_parameter('device').get_parameter_value().string_value
        self.sample_rate         = self.get_parameter('sample_rate').get_parameter_value().integer_value
        vad_aggressiveness       = self.get_parameter('vad_aggressiveness').get_parameter_value().integer_value
        self.silence_timeout     = self.get_parameter('silence_timeout').get_parameter_value().double_value
        self.min_speech_duration = self.get_parameter('min_speech_duration').get_parameter_value().double_value
        self.native_sample_rate  = self.get_parameter('native_sample_rate').get_parameter_value().integer_value
        mic_device               = self.get_parameter('mic_device_index').get_parameter_value().integer_value

        # ── publishers ────────────────────────────────────────────────────────
        self.pub_intent = self.create_publisher(String, '/frank/intent', 10)
        self.pub_raw    = self.create_publisher(String, '/voice/raw_text', 10)

        # ── Whisper ───────────────────────────────────────────────────────────
        self.get_logger().info(f'Loading Whisper model: {model_size} on {device}')
        self.model = WhisperModel(
            model_size,
            device=device,
            compute_type='int8',   # int8 is fastest on CPU / Jetson
        )
        self.get_logger().info('Whisper model loaded')

        # ── VAD setup ─────────────────────────────────────────────────────────
        self.vad = webrtcvad.Vad(vad_aggressiveness)
        self.frame_duration_ms = 30          # webrtcvad supports 10/20/30ms
        self.frame_size = int(self.native_sample_rate * self.frame_duration_ms / 1000)


        # Ring buffer of recent frames to catch speech onset
        num_padding_frames = int(300 / self.frame_duration_ms)  # 300ms padding
        self.ring_buffer = collections.deque(maxlen=num_padding_frames)

        self.voiced_frames = []
        self.triggered = False
        self.silence_frames = 0
        self.silence_limit = int(self.silence_timeout * 1000 / self.frame_duration_ms)

        # ── audio thread ──────────────────────────────────────────────────────
        # self._lock = threading.Lock()
        # self._audio_buffer = []
        self._audio_queue = queue.Queue()

        # start processing thread
        self._processing_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self._processing_thread.start()

        # stream stays the same
        self.stream = sd.InputStream(
            device=mic_device if mic_device >= 0 else None,
            samplerate=self.native_sample_rate,
            channels=1,
            dtype='int16',
            blocksize=self.frame_size,
            callback=self._audio_callback,
        )
        self.stream.start()

    def _audio_callback(self, indata, frames, time_info, status):
        # callback now just queues raw data — no processing here
        if status:
            self.get_logger().warn(f'Audio status: {status}')
        self._audio_queue.put(indata.copy())

    def _processing_loop(self):
        vad_frame_size = int(self.sample_rate * self.frame_duration_ms / 1000)

        while rclpy.ok():
            try:
                indata = self._audio_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            # resample from native rate to 16000Hz
            num_target = int(len(indata) * self.sample_rate / self.native_sample_rate)
            resampled = sps.resample(indata[:, 0], num_target).astype(np.int16)

            # pad or trim to exact VAD frame size
            if len(resampled) < vad_frame_size:
                resampled = np.pad(resampled, (0, vad_frame_size - len(resampled)))
            else:
                resampled = resampled[:vad_frame_size]

            frame = resampled.tobytes()
            is_speech = self.vad.is_speech(frame, self.sample_rate)

            if not self.triggered:
                self.ring_buffer.append((frame, is_speech))
                num_voiced = sum(1 for _, s in self.ring_buffer if s)
                if num_voiced > 0.6 * self.ring_buffer.maxlen:
                    self.triggered = True
                    self.voiced_frames = [f for f, _ in self.ring_buffer]
                    self.ring_buffer.clear()
                    self.silence_frames = 0
            else:
                self.voiced_frames.append(frame)
                if not is_speech:
                    self.silence_frames += 1
                    if self.silence_frames > self.silence_limit:
                        audio_data = b''.join(self.voiced_frames)
                        self._transcribe_and_publish(audio_data)
                        self.triggered = False
                        self.voiced_frames = []
                        self.silence_frames = 0
                else:
                    self.silence_frames = 0

    def _transcribe_and_publish(self, audio_bytes: bytes):
        # check minimum duration
        num_samples = len(audio_bytes) // 2  # int16 = 2 bytes
        duration = num_samples / self.sample_rate
        if duration < self.min_speech_duration:
            return

        # convert to float32 numpy array for Whisper
        audio_np = np.frombuffer(audio_bytes, dtype=np.int16).astype(np.float32) / 32768.0

        try:
            segments, _ = self.model.transcribe(
                audio_np,
                language='en',
                beam_size=1,       # beam_size=1 is fastest (greedy)
                vad_filter=True,   # Whisper's built-in VAD as second pass
            )
            text = ' '.join(seg.text.strip() for seg in segments).strip()
        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')
            return

        if not text:
            return

        self.get_logger().info(f'Heard: "{text}"')

        # publish raw transcript
        raw_msg = String()
        raw_msg.data = text
        self.pub_raw.publish(raw_msg)

        # classify intent and publish
        intent, _ = classify_intent(text)
        intent_msg = String()
        intent_msg.data = intent
        self.pub_intent.publish(intent_msg)
        self.get_logger().info(f'Intent: {intent}')

    # ── cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self.stream.stop()
        self.stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()