import cv2
import mediapipe as mp
import numpy as np
import math
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DEBUG = False  # Set to False for production mode

class GesturePublisher(Node):
    def __init__(self):
        super().__init__('gesture_recognition_node')
        self.publisher_ = self.create_publisher(String, '/gesture_cmds', 10)


@dataclass
class FingerInfo:
    extended: bool
    angle: float
    direction: str = "None"  # 'up', 'down', 'left', 'right'

GESTURE_MEANINGS = {
    'palm': 'Stop',
    'fist': 'Reduce speed',
    'index_up': 'Continue moving forward',
    'index_left': 'Turn left',
    'index_right': 'Turn right',
    'index_middle_up': 'Increase speed',
    'unknown': 'Unknown gesture'
}


def angle_between_vectors(vec1, vec2):
    v1 = np.array(vec1)
    v2 = np.array(vec2)
    dot_prod = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)

    if norm_v1 == 0 or norm_v2 == 0:
        return 0.0

    cos_theta = dot_prod / (norm_v1 * norm_v2)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)  # For numerical stability
    angle_rad = math.acos(cos_theta)
    return math.degrees(angle_rad)

def get_direction(mcp, tip):
    """
    Calculate the 2D direction of the finger from MCP to TIP,
    and return one of: 'Right', 'Left', 'Up', 'Down'.
    """
    dx = tip.x - mcp.x
    dy = tip.y - mcp.y

    angle = math.degrees(math.atan2(-dy, dx))  # negative dy because y axis is inverted in images
    # Angle is between -180 and 180, 0 is right

    if -45 <= angle <= 45:
        return "Right"
    elif 45 < angle <= 135:
        return "Up"
    elif angle > 135 or angle < -135:
        return "Left"
    else:
        return "Down"

def is_finger_extended(wrist, mcp, dip, tip, angle_threshold=35) -> FingerInfo:
    base_vector = [mcp.x - wrist.x, mcp.y - wrist.y, mcp.z - wrist.z]
    tip_vector = [tip.x - dip.x, tip.y - dip.y, tip.z - dip.z]
    angle = angle_between_vectors(base_vector, tip_vector)
    extended = angle < angle_threshold
    direction = get_direction(mcp, tip) if extended else "None"
    return FingerInfo(extended=extended, angle=angle, direction=direction)

def get_hand_finger_status(landmarks, mp_hands) -> dict:
    lm = landmarks.landmark
    wrist = lm[mp_hands.HandLandmark.WRIST]

    return {
        'thumb': is_finger_extended(wrist, lm[mp_hands.HandLandmark.THUMB_MCP],
                                           lm[mp_hands.HandLandmark.THUMB_IP],
                                           lm[mp_hands.HandLandmark.THUMB_TIP]),

        'index': is_finger_extended(wrist, lm[mp_hands.HandLandmark.INDEX_FINGER_MCP],
                                           lm[mp_hands.HandLandmark.INDEX_FINGER_DIP],
                                           lm[mp_hands.HandLandmark.INDEX_FINGER_TIP]),

        'middle': is_finger_extended(wrist, lm[mp_hands.HandLandmark.MIDDLE_FINGER_MCP],
                                            lm[mp_hands.HandLandmark.MIDDLE_FINGER_DIP],
                                            lm[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]),

        'ring': is_finger_extended(wrist, lm[mp_hands.HandLandmark.RING_FINGER_MCP],
                                          lm[mp_hands.HandLandmark.RING_FINGER_DIP],
                                          lm[mp_hands.HandLandmark.RING_FINGER_TIP]),

        'pinky': is_finger_extended(wrist, lm[mp_hands.HandLandmark.PINKY_MCP],
                                           lm[mp_hands.HandLandmark.PINKY_DIP],
                                           lm[mp_hands.HandLandmark.PINKY_TIP]),
    }

def detect_named_gesture(finger_status: dict) -> str:
    """
    Returns one of the 6 named gestures.
    """
    f = finger_status  # shorthand

    # Gesture 1: Palm (all fingers except thumb extended)
    if all(f[fn].extended for fn in ['index', 'middle', 'ring', 'pinky']):
        return 'palm'

    # Gesture 2: Fist
    if all(not f[fn].extended for fn in ['thumb', 'index', 'middle', 'ring', 'pinky']):
        return 'fist'

    # Gesture: index_up → only index extended AND pointing up
    if (not f['thumb'].extended and
        f['index'].extended and f['index'].direction == 'Up' and
        all(not f[fn].extended for fn in ['middle', 'ring', 'pinky'])):
        return 'index_up'

    # Gesture: index_left → only index extended AND pointing left
    if (not f['thumb'].extended and
        f['index'].extended and f['index'].direction == 'Left' and
        all(not f[fn].extended for fn in ['middle', 'ring', 'pinky'])):
        return 'index_left'

    # Gesture: index_right → only index extended AND pointing right
    if (not f['thumb'].extended and
        f['index'].extended and f['index'].direction == 'Right' and
        all(not f[fn].extended for fn in ['middle', 'ring', 'pinky'])):
        return 'index_right'

    # Gesture: index_middle_up → index and middle extended + pointing up
    if (not f['thumb'].extended and
        f['index'].extended and f['index'].direction == 'Up' and
        f['middle'].extended and f['middle'].direction == 'Up' and
        all(not f[fn].extended for fn in ['ring', 'pinky'])):
        return 'index_middle_up'

    return 'unknown'


def main():
    rclpy.init()
    node = GesturePublisher()
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(static_image_mode=False,
                           max_num_hands=1,
                           min_detection_confidence=0.7,
                           min_tracking_confidence=0.7)
    mp_draw = mp.solutions.drawing_utils

    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break

        frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb)

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            finger_status = get_hand_finger_status(hand_landmarks, mp_hands)
            gesture = detect_named_gesture(finger_status)
            meaning = GESTURE_MEANINGS.get(gesture, "Unknown gesture")

            # Publish the gesture meaning
            msg = String()
            msg.data = meaning
            node.publisher_.publish(msg)
            if DEBUG:
                print(f"[Gesture Node] Published: {meaning}")  # Optional debug log

            # Always show high-level gesture
            cv2.putText(frame, f"Gesture: {meaning}", (10, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            if DEBUG:
                # Additionally show detailed finger info
                text_vertical_pos = 100  # Start lower to avoid overlapping gesture text
                cv2.putText(frame, f"Gesture: {gesture}", (10, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
                for name, info in finger_status.items():
                    status_text = f"{name}: {'Extended' if info.extended else 'Bent'} | {info.angle:.1f}° | {info.direction}"
                    cv2.putText(frame, status_text, (10, text_vertical_pos),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (0, 255, 0) if info.extended else (0, 0, 255), 2)
                    text_vertical_pos += 30


        cv2.imshow("Finger Extension and Direction Viewer", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
