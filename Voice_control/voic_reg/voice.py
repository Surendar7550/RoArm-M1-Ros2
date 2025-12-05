#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import sounddevice as sd
from vosk import Model, KaldiRecognizer
import json
import os
import time

# Voice command keywords
VOICE_CMDS = ["up", "down", "left", "right", "pick", "release", "normal"]

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__("voice_command_node")

        # -------------------------------------------------------------
        # Declare model path parameter
        # -------------------------------------------------------------
        self.declare_parameter(
            'model_path',
            '/home/surendar/Drone_pro/ros2_arm_ws/src/voic_reg/models/vosk/vosk-model-small-en-in-0.4' #vosk-model-en-in-0.5
        )
        model_path = self.get_parameter('model_path').get_parameter_value().string_value

        # -------------------------------------------------------------
        # Check model directory
        # -------------------------------------------------------------
        if not os.path.isdir(model_path):
            self.get_logger().error(f"❌ Vosk model not found: {model_path}")
            raise SystemExit("Model missing - cannot start STT")

        try:
            self.model = Model(model_path)
            self.get_logger().info(f"✅ Loaded Vosk model: {model_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to load Vosk model: {e}")
            raise SystemExit("Vosk model load failed")

        # Create recognizer
        self.rec = KaldiRecognizer(self.model, 16000)

        # -------------------------------------------------------------
        # Publishers for each voice command
        # -------------------------------------------------------------
        self.pub = {
            cmd: self.create_publisher(Bool, f"/{cmd}", 10)
            for cmd in VOICE_CMDS
        }

        # -------------------------------------------------------------
        # Start microphone stream
        # -------------------------------------------------------------
        try:
            self.stream = sd.RawInputStream(
                samplerate=16000,
                blocksize=8000,
                dtype='int16',
                channels=1,
                callback=self.audio_callback
            )
            self.stream.start()
            self.get_logger().info("🎤 Voice Recognition Started...")
        except Exception as e:
            self.get_logger().error(f"❌ Microphone access error: {e}")
            raise SystemExit("Microphone failed")

    # =================================================================
    # Microphone Callback  (FIXED: convert buffer → bytes)
    # =================================================================
    def audio_callback(self, indata, frames, time_val, status):
        if status:
            self.get_logger().warn(f"Audio status: {status}")

        # FIX: Vosk needs bytes, not buffer
        data_bytes = bytes(indata)

        if self.rec.AcceptWaveform(data_bytes):
            result = json.loads(self.rec.Result())
            text = result.get("text", "").strip().lower()

            if text:
                self.get_logger().info(f"🗣️ Heard: {text}")
                self.process_voice(text)

    # =================================================================
    # Process recognized text
    # =================================================================
    def process_voice(self, text):
        for cmd in VOICE_CMDS:
            if cmd in text:
                self.trigger_command(cmd)
                return

    # =================================================================
    # Publish True for 0.4 sec then False
    # =================================================================
    def trigger_command(self, cmd):
        self.get_logger().info(f"➡️ Voice Trigger: {cmd}")

        msg = Bool()

        # Press (True)
        msg.data = True
        self.pub[cmd].publish(msg)

        time.sleep(0.4)

        # Release (False)
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
        try:
            node.destroy_node()
        except:
            pass

        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()

