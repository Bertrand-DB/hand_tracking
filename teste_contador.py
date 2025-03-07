import cv2
from HandTrack import HandTrack
from Gestures import Gestures

tracker = HandTrack(0)
left_hand = Gestures(False,(1280,720), 78.6, (6,8))
right_hand = Gestures(True,(1280,720), 78.6, (6,8))

while True:
    contador = 0

    if tracker.process_image():
        tracker.draw_hand(show_landmark_id=True)
        tracker.show_image()

        left, right = tracker.get_hands_coordinates()
        
        if left: 
            left_hand.frame_update(left)
            contador += 5 - len(left_hand.fingers_flexed())

        if right: 
            right_hand.frame_update(right)
            contador += 5 - len(right_hand.fingers_flexed())

    print(contador)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  #ESC para sair
        break
