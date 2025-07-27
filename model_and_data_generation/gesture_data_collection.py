import cv2
import mediapipe as mp
import numpy as np
import csv
import argparse
import re
import os

def capture_hand_gestures(gesture_label="gesture", save_data=True, max_samples=None):
    # MediaPipe setup
    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils
    hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)

    # Webcam
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Cannot access the webcam.")
        return

    filename = f"data/{gesture_label}_data.csv"
    sample_count = 0
    csv_writer = None
    csv_file = None

    try:
        if save_data:
            os.makedirs("data", exist_ok=True)
            # Always overwrite existing file
            csv_file = open(filename, 'w', newline='')
            csv_writer = csv.writer(csv_file)
            print(f"[INFO] Writing data to: {filename}")

        while cap.isOpened():
            success, image = cap.read()
            if not success:
                print("Failed to read from webcam. Exiting...")
                break

            image = cv2.flip(image, 1)
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = hands.process(rgb_image)

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                    landmarks = [coord for lm in hand_landmarks.landmark for coord in (lm.x, lm.y, lm.z)]
                    sample_count += 1

                    # Save to CSV
                    if save_data and csv_writer:
                        csv_writer.writerow(landmarks + [gesture_label])

                    print(f"Sample {sample_count} recorded for '{gesture_label}'")

            # Display window
            cv2.putText(image, f"Samples: {sample_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('MediaPipe Hands', image)

            if cv2.waitKey(5) & 0xFF == 27:
                print("Esc pressed. Exiting...")
                break

            if max_samples and sample_count >= max_samples:
                print(f"Reached max samples: {max_samples}")
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()
        if csv_file:
            csv_file.close()
            print(f"[INFO] Data saved to: {filename}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture hand gesture landmarks using MediaPipe.")
    parser.add_argument('--label', type=str, default='Turn right', help='Label for the gesture')
    parser.add_argument('--no-save', action='store_false', dest='save_data', help='Do not save landmarks to CSV')
    parser.add_argument('--max', type=int, default=None, help='Max number of samples to capture')

    args = parser.parse_args()

    capture_hand_gestures(
        gesture_label=args.label,
        save_data=args.save_data, 
        max_samples=args.max
    )

