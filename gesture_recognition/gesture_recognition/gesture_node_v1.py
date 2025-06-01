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

    def detect_gesture(self):
        if not self.running:
            return

        ret, frame = self.cap.read()
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
            extended_fingers = self.count_extended_fingers(hand_landmarks)
            gesture = self.classify_gesture(extended_fingers)
            self.publish_gesture(gesture)

        # Overlay gesture text on frame
        cv2.putText(frame, f'Gesture: {gesture}', (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('Gesture Recognition', frame)

        # Exit cleanly on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.running = False
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def count_extended_fingers(self, hand_landmarks):
        extended_fingers = 0

        # Thumb: check relative x coordinate because thumb points sideways
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP]

        if thumb_tip.x < thumb_ip.x:  # Assuming right hand
            extended_fingers += 1

        # Fingers tips and dips
        finger_tips = [
            self.mp_hands.HandLandmark.INDEX_FINGER_TIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
            self.mp_hands.HandLandmark.RING_FINGER_TIP,
            self.mp_hands.HandLandmark.PINKY_TIP
        ]

        finger_dips = [
            self.mp_hands.HandLandmark.INDEX_FINGER_DIP,
            self.mp_hands.HandLandmark.MIDDLE_FINGER_DIP,
            self.mp_hands.HandLandmark.RING_FINGER_DIP,
            self.mp_hands.HandLandmark.PINKY_DIP
        ]

        for tip, dip in zip(finger_tips, finger_dips):
            if hand_landmarks.landmark[tip].y < hand_landmarks.landmark[dip].y:
                extended_fingers += 1

        return extended_fingers

    def classify_gesture(self, extended_fingers):
        if extended_fingers == 0:
            return 'stop'
        elif extended_fingers == 1:
            return 'left'
        elif extended_fingers == 2:
            return 'right'
        elif extended_fingers == 3:
            return 'forward'
        else:
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
