#!/usr/bin/env python3
"""
llm_intent_node.py
------------------
Subscribes to /voice/raw_text (raw Whisper transcription),
sends it to a local Ollama LLM, and publishes a structured
intent token to /frank/intent.

Pipeline:
  stt_node → /voice/raw_text → [this node] → /frank/intent → behavior_manager
"""

import json
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Valid intent tokens — must match behavior_manager_node.py exactly
# ---------------------------------------------------------------------------
VALID_INTENTS = {"track_face", "find_face", "stop", "go_home"}
FALLBACK_INTENT = "stop"

# ---------------------------------------------------------------------------
# System prompt — tells the LLM exactly what to do and what to output
# ---------------------------------------------------------------------------
SYSTEM_PROMPT = """You are the intent classifier for a mobile robot called Frank.
Frank can perform exactly these actions:

  track_face  — turn and face a person, follow them if they move
  find_face   — spin and search the room until a face is detected
  stop        — stop all movement immediately
  go_home     — return to the home/docking position

Your job: read a voice command and reply with ONLY one of those four tokens.
No punctuation, no explanation, no extra words — just the single token.

If the command is ambiguous or unknown, reply: stop

Examples:
  "come here"         → track_face
  "follow me please"  → track_face
  "look around"       → find_face
  "can you find me"   → find_face
  "halt"              → stop
  "freeze"            → stop
  "go back to base"   → go_home
  "what time is it"   → stop
"""


class LLMIntentNode(Node):
    def __init__(self):
        super().__init__("llm_intent_node")

        # Parameters — override via ROS params or launch file
        self.declare_parameter("ollama_url", "http://localhost:11434/api/chat")
        self.declare_parameter("model", "llama3.2:3b")
        self.declare_parameter("timeout_sec", 8.0)

        self.ollama_url = self.get_parameter("ollama_url").value
        self.model = self.get_parameter("model").value
        self.timeout_sec = self.get_parameter("timeout_sec").value

        self.sub = self.create_subscription(
            String, "/voice/raw_text", self.on_raw_text, 10
        )
        self.pub = self.create_publisher(String, "/frank/intent", 10)

        self.get_logger().info('Warming up Ollama model...')
        self._warmup()
        self.get_logger().info('Ollama ready.')
        
    def _warmup(self):
        """Fire a dummy request at startup to load the model into RAM."""
        payload = {
            "model": self.model,
            "keep_alive": -1,          # pin in RAM indefinitely
            "stream": False,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user",   "content": "ready"},
            ],
        }
        try:
            data = json.dumps(payload).encode("utf-8")
            req = urllib.request.Request(
                self.ollama_url, data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=30.0) as resp:
                resp.read()
        except Exception as e:
            self.get_logger().warn(f'Warmup failed: {e} — first inference may be slow')
        

    # ------------------------------------------------------------------
    def on_raw_text(self, msg: String):
        raw = msg.data.strip()
        if not raw:
            return

        self.get_logger().info(f"Heard: '{raw}'")
        intent = self.classify(raw)
        self.get_logger().info(f"Intent: {intent}")

        out = String()
        out.data = intent
        self.pub.publish(out)

    # ------------------------------------------------------------------
    def classify(self, text: str) -> str:
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user",   "content": text},
            ],
            "stream": False,
        }

        try:
            data = json.dumps(payload).encode("utf-8")
            req = urllib.request.Request(
                self.ollama_url,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=self.timeout_sec) as resp:
                body = json.loads(resp.read().decode("utf-8"))

            token = body["message"]["content"].strip().lower()

            if token not in VALID_INTENTS:
                self.get_logger().warn(
                    f"LLM returned unexpected token '{token}', falling back to '{FALLBACK_INTENT}'"
                )
                return FALLBACK_INTENT

            return token

        except urllib.error.URLError as e:
            self.get_logger().error(f"Ollama unreachable: {e} — falling back to '{FALLBACK_INTENT}'")
            return FALLBACK_INTENT

        except (KeyError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Bad response from Ollama: {e} — falling back to '{FALLBACK_INTENT}'")
            return FALLBACK_INTENT

        except TimeoutError:
            self.get_logger().error(f"Ollama timed out after {self.timeout_sec}s — falling back to '{FALLBACK_INTENT}'")
            return FALLBACK_INTENT


# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = LLMIntentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()