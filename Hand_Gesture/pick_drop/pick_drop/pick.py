import cv2
import mediapipe as mp
import numpy as np
import tflite_runtime.interpreter as tflite
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
 
# ----------------------------
# Load TFLite model
# ----------------------------
MODEL_PATH = "/home/surendar/Drone_pro/ros2_arm_ws/src/pick_drop/pick_drop/gesture_model_int8.tflite"
interpreter = tflite.Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()[0]
output_details = interpreter.get_output_details()[0]
in_scale, in_zero_point = input_details['quantization']
out_scale, out_zero_point = output_details['quantization']
 
# Match dataset label order exactly
CLASSES = [
    "", "release", "down", "pick", "left",
    "right", "normal", "unknown", "up", ""
]
 
# ----------------------------
# Mediapipe Hands
# ----------------------------
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(max_num_hands=1,
                       min_detection_confidence=0.7,
                       min_tracking_confidence=0.7)
 
# ----------------------------
# Preprocessing
# ----------------------------
def preprocess_landmarks(landmarks):
    pts = np.array([[lm.x, lm.y, lm.z] for lm in landmarks], dtype=np.float32)
    wrist = pts[0]
    pts_rel = pts - wrist
    max_val = np.max(np.abs(pts_rel))
    if max_val == 0:
        max_val = 1.0
    pts_norm = pts_rel / max_val
    row = pts_norm.flatten().astype(np.float32).reshape(1, -1)
    if input_details['dtype'] == np.int8:
        row = (row / in_scale + in_zero_point).round().astype(np.int8)
    return row
 
# ----------------------------
# Prediction
# ----------------------------
def predict(landmarks, threshold=0.6):
    input_data = preprocess_landmarks(landmarks)
    interpreter.set_tensor(input_details['index'], input_data)
    interpreter.invoke()
    output = interpreter.get_tensor(output_details['index'])[0]
    if output_details['dtype'] == np.int8:
        output = (output.astype(np.float32) - out_zero_point) * out_scale
    pred_idx = np.argmax(output)
    confidence = output[pred_idx]
    if confidence < threshold:
        return "unknown", confidence
    return CLASSES[pred_idx], confidence
 
# ----------------------------
# ROS 2 Node
# ----------------------------
class GesturePublisher(Node):
    def __init__(self):
        super().__init__('gesture_publisher')
 
        # Create publisher for each valid gesture (skip unknown & empty)
        self.gesture_publishers = {
            gesture: self.create_publisher(Bool, f'/{gesture}', 10)
            for gesture in CLASSES if gesture not in ["", "unknown"]
        }
 
        self.timer = self.create_timer(0.1, self.process_frame)
        self.cap = cv2.VideoCapture(0)
        self.last_detected = "unknown"
 
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return
 
        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)
 
        detected_gesture = "unknown"
 
        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                gesture, conf = predict(hand_landmarks.landmark)
                detected_gesture = gesture
                cv2.putText(frame, f"{gesture} ({conf:.2f})", (10, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                self.get_logger().info(f"Detected Gesture: {gesture} ({conf:.2f})")
        else:
            cv2.putText(frame, "unknown", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
 
        # ----------------------------
        # Publish TRUE for detected gesture, FALSE for others
        # ----------------------------
        for gesture, publisher in self.gesture_publishers.items():
            msg = Bool()
            msg.data = (gesture == detected_gesture)
            publisher.publish(msg)
 
        self.last_detected = detected_gesture
 
        cv2.imshow("Hand Gesture Recognition", frame)
        if cv2.waitKey(1) & 0xFF == 27:
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()
 
# ----------------------------
# Main
# ----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = GesturePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
