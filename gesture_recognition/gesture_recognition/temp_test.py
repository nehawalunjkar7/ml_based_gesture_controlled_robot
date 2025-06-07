import cv2
import mediapipe as mp
import numpy as np
import math

def angle_between_vectors(vec1, vec2):
    v1 = np.array(vec1)
    v2 = np.array(vec2)
    dot_prod = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)

    if norm_v1 == 0 or norm_v2 == 0:
        return 0.0

    cos_theta = dot_prod / (norm_v1 * norm_v2)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)  # for stability
    angle_rad = math.acos(cos_theta)
    return math.degrees(angle_rad)


def is_index_finger_extended(wrist, mcp, dip, tip, angle_threshold=35):
    """
    Returns True if angle between wrist->MCP and DIP->TIP is below threshold (i.e. finger is straight).
    """
    base_vector = [mcp.x - wrist.x, mcp.y - wrist.y, mcp.z - wrist.z]
    tip_vector = [tip.x - dip.x, tip.y - dip.y, tip.z - dip.z]
    angle = angle_between_vectors(base_vector, tip_vector)

    print(f"[DEBUG] Index Finger Angle: {angle:.2f}°")
    return angle < angle_threshold, angle


def main():
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

        gesture = "No hand"
        angle_display = ""

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            lm = hand_landmarks.landmark
            wrist = lm[mp_hands.HandLandmark.WRIST]
            index_mcp = lm[mp_hands.HandLandmark.INDEX_FINGER_MCP]
            index_dip = lm[mp_hands.HandLandmark.INDEX_FINGER_DIP]
            index_tip = lm[mp_hands.HandLandmark.INDEX_FINGER_TIP]

            extended, angle = is_index_finger_extended(wrist, index_mcp, index_dip, index_tip)
            angle_display = f"Angle: {angle:.1f}°"

            if extended:
                gesture = "Index: Extended"
            else:
                gesture = "Index: Bent"

        # Overlay text
        cv2.putText(frame, gesture, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, angle_display, (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        cv2.imshow("Index Finger Angle Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
