#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import sounddevice as sd
from vosk import Model, KaldiRecognizer
import json
import os
import time

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__("voice_command_node")

        # -------------------------------------------------------------
        # Model path
        # -------------------------------------------------------------
        self.declare_parameter(
            'model_path',
            '/home/surendar/Drone_pro/ros2_arm_ws/src/voic_reg/models/vosk/vosk-model-small-en-in-0.4' #vosk-model-en-in-0.5
        )
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        if not os.path.isdir(model_path):
            self.get_logger().error(f"❌ Vosk model not found: {model_path}")
            raise SystemExit("Model missing")

        try:
            self.model = Model(model_path)
            self.get_logger().info(f"✅ Loaded Vosk model: {model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load Vosk model: {e}")
            raise SystemExit("Model load fail")

        # -------------------------------------------------------------
        # KEYWORD ONLY recognizer  (BEST FOR COMMANDS)
        # -------------------------------------------------------------
        kws = '["up","down","left","right","pick","release","normal"]'
        self.rec = KaldiRecognizer(self.model, 16000, kws)

        # -------------------------------------------------------------
        # Publishers
        # -------------------------------------------------------------
        self.pub = {
            "up": self.create_publisher(Bool, "/up", 10),
            "down": self.create_publisher(Bool, "/down", 10),
            "left": self.create_publisher(Bool, "/left", 10),
            "right": self.create_publisher(Bool, "/right", 10),
            "pick": self.create_publisher(Bool, "/pick", 10),
            "release": self.create_publisher(Bool, "/release", 10),
            "normal": self.create_publisher(Bool, "/normal", 10),
        }

        # -------------------------------------------------------------
        # Start microphone
        # -------------------------------------------------------------
        try:
            self.stream = sd.RawInputStream(
                samplerate=16000,
                blocksize=1024,  # FAST
                dtype='int16',
                channels=1,
                callback=self.audio_callback
            )
            self.stream.start()
            self.get_logger().info("🎤 Voice Recognition Started in KEYWORD MODE...")
        except Exception as e:
            self.get_logger().error(f"❌ Microphone error: {e}")
            raise SystemExit("Mic failure")

    # =================================================================
    # Microphone Callback
    # =================================================================
    def audio_callback(self, indata, frames, time_val, status):
        if status:
            self.get_logger().warn(f"Audio status: {status}")

        data_bytes = bytes(indata)

        if self.rec.AcceptWaveform(data_bytes):
            res = json.loads(self.rec.Result())
            text = res.get("text", "").strip().lower()

            if text:
                self.get_logger().info(f"🗣️ Heard: {text}")
                self.process_voice(text)

    # =================================================================
    # ACCENT-FRIENDLY COMMAND DETECTION
    # =================================================================
    def process_voice(self, text):

        # Indian-accent pronunciation variations
        PICK_WORDS = ["pick", "pic", "pik", "peak", "peek", "pikk"]
        RELEASE_WORDS = ["release", "rilis", "reliz", "relis", "lace", "leas"]

        if text in PICK_WORDS:
            self.trigger("pick")
            return

        if text in RELEASE_WORDS:
            self.trigger("release")
            return

        # Direct matches for easy words
        if text in ["up", "down", "left", "right", "normal"]:
            self.trigger(text)

    # =================================================================
    # PUBLISH TRUE → WAIT → FALSE
    # =================================================================
    def trigger(self, cmd):
        self.get_logger().info(f"➡️ Voice Trigger: {cmd}")

        msg = Bool()
        msg.data = True
        self.pub[cmd].publish(msg)

        time.sleep(0.3)

        msg.data = False
        self.pub[cmd].publish(msg)


# =================================================================
# MAIN
# =================================================================
def main(args=None):
    rclpy.init(args=args)
    node = VoiceCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Shutting down voice node...")
    finally:
        try: node.destroy_node()
        except: pass

        try: rclpy.shutdown()
        except: pass


if __name__ == "__main__":
    main()

