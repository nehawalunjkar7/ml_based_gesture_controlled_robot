import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GestureRecognitionNode(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')
        self.publisher = self.create_publisher(String, '/gesture_cmds', 10)
        self.timer = self.create_timer(0.1, self.detect_gesture)
        self.cap = cv2.VideoCapture(0)
        self.hands = mp.solutions.hands.Hands(
            min_detection_confidence=0.7, 
            min_tracking_confidence=0.7
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands
        self.running = True
        self.debug = False   # <-- DEBUG MODE: Set to False to hide finger count display

    def detect_gesture(self):
        if not self.running:
            return

        ret, frame = self.cap.read()
        if self.debug:
            frame = cv2.flip(frame, 1)  # Optional: mirror for visual comfort
            
        if not ret:
            self.get_logger().warn('Failed to capture image')
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(frame_rgb)

        gesture = "No hand detected"
        if results.multi_hand_landmarks:
            # For now, just consider first detected hand
            hand_landmarks = results.multi_hand_landmarks[0]
            self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
            finger_states = self.get_finger_states(hand_landmarks)
            gesture = self.classify_gesture(finger_states, hand_landmarks)
            self.publish_gesture(gesture)

        # Overlay gesture text on frame
        cv2.putText(frame, f'Gesture: {gesture}', (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        if self.debug and results.multi_hand_landmarks:
            cv2.putText(frame, f'Fingers: {finger_states}', (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            print(f"Finger States: {finger_states}")


        cv2.imshow('Gesture Recognition', frame)

        # Exit cleanly on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.running = False
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def get_finger_states(self, hand_landmarks):
        fingers = {}

        # Thumb (x comparison because it's sideways)
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP]
        fingers['thumb'] = thumb_tip.x < thumb_ip.x  # right hand assumption

        # Other fingers (y comparison: tip above pip)
        tips = [
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            self.mp_hands.HandLandmark.RING_FINGER_TIP,
            self.mp_hands.HandLandmark.PINKY_TIP
        ]

        pips = [
            self.mp_hands.HandLandmark.INDEX_FINGER_PIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP,
            self.mp_hands.HandLandmark.RING_FINGER_PIP,
            self.mp_hands.HandLandmark.PINKY_PIP
        ]

        labels = ['index', 'middle', 'ring', 'pinky']

        for tip, pip, label in zip(tips, pips, labels):
            fingers[label] = hand_landmarks.landmark[tip].y < hand_landmarks.landmark[pip].y

        return fingers


    def classify_gesture(self, finger_states, hand_landmarks):
        thumb = finger_states['thumb']
        index = finger_states['index']
        middle = finger_states['middle']
        ring = finger_states['ring']
        pinky = finger_states['pinky']

        # Stop: All fingers except thumb extended
        if index and middle and ring and pinky and not thumb:
            return 'stop'

        # Turn Left: Only index finger extended
        if index and not middle and not ring and not pinky and not thumb:
            return 'turn_left'

        # Turn Left + Forward: Index + Thumb up
        if index and not middle and not ring and not pinky and thumb:
            # Confirm thumb is pointing upward
            thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
            thumb_ip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP]
            if thumb_tip.y < thumb_ip.y:
                return 'turn_left_forward'

        # Two fingers: Forward constant
        if index and middle and not ring and not pinky:
            return 'forward_constant'

        # Three fingers: Accelerate
        if index and middle and ring and not pinky:
            return 'forward_accelerate'

        # Thumb down
        if thumb and not index and not middle and not ring and not pinky:
            thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
            thumb_ip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP]
            if thumb_tip.y > thumb_ip.y:
                return 'slow_down_or_reverse'

        return 'unknown'

    def publish_gesture(self, gesture):
        msg = String()
        msg.data = gesture
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GestureRecognitionNode()
    rclpy.spin(node)
    node.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
