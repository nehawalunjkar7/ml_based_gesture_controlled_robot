import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import cv2
import mediapipe as mp
import numpy as np
import joblib

class GesturePublisher(Node):
    def __init__(self):
        super().__init__('gesture_publisher')

        # Publisher to publish commands as strings
        self.publisher_ = self.create_publisher(String, '/gesture_cmds', 10)

        base_dir = os.path.dirname(__file__)

        model_path = os.path.join(base_dir, 'gesture_classifier.joblib')
        label_encoder_path = os.path.join(base_dir, 'label_encoder.joblib')
        scaler_path = os.path.join(base_dir, 'scaler.joblib')

        # Load the model, label encoder, and scaler
        self.model = joblib.load(model_path)
        self.le = joblib.load(label_encoder_path)
        self.scaler = joblib.load(scaler_path)

        # MediaPipe hands setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
        self.mp_drawing = mp.solutions.drawing_utils

        # OpenCV Video capture
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open webcam")
            rclpy.shutdown()
            return

        # Probability threshold for unknown gesture detection
        self.proba_threshold = 0.5

        self.running = True

        self.timer = self.create_timer(0.1, self.timer_callback)

    def extract_landmarks(self, results):
        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            landmarks = []
            for lm in hand_landmarks.landmark:
                landmarks.extend([lm.x, lm.y, lm.z])
            return landmarks
        return None

    def timer_callback(self):
        if not self.running:
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture')
            return

        # Prepare image for MediaPipe processing
        image = cv2.cvtColor(cv2.flip(frame, 1), cv2.COLOR_BGR2RGB)
        image.flags.writeable = False
        results = self.hands.process(image)
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        landmarks = self.extract_landmarks(results)

        if landmarks:
            X = np.array(landmarks).reshape(1, -1)
            X = self.scaler.transform(X)

            proba = self.model.predict_proba(X)
            max_proba = np.max(proba)

            if max_proba < self.proba_threshold:
                prediction = "Unknown gesture"
            else:
                pred_class_index = np.argmax(proba)
                pred_label = self.le.inverse_transform([pred_class_index])
                prediction = pred_label[0]

            msg = String()
            msg.data = str(prediction)
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published command: {prediction}")

            # Draw landmarks and predicted command on image
            self.mp_drawing.draw_landmarks(image, results.multi_hand_landmarks[0], self.mp_hands.HAND_CONNECTIONS)
            cv2.putText(image, f"Command: {prediction}", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            pass

        cv2.imshow("Hand Gesture to Command", image)
        if cv2.waitKey(1) & 0xFF == 27:
            self.get_logger().info("ESC pressed, shutting down...")
            self.running = False
            self.cleanup()
            rclpy.shutdown()

    def cleanup(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = GesturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down...")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
