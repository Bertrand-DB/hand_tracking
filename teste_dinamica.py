import cv2
from HandTrack import HandTrack
from Gestures import *

tracker = HandTrack(0)
right_hand = Gestures(True,(1280,720), 78.6, (6,8))
left_hand = Gestures(False,(1280,720), 78.6, (6,8))

while True:
    if tracker.process_image():

        tracker.draw_hand(show_landmark_id=True)
        tracker.show_image()
        left_coord , right_coord = tracker.get_hands_coordinates()
        
        if left_coord:
            left_hand.frame_update(left_coord)
            deslocamento = left_hand.deslocation()
            deslocamento_str = ", ".join(f"{d:.2f}" for d in deslocamento)
            print(deslocamento_str)

        if right_coord:
            right_hand.frame_update(right_coord)
            angulos = (right_hand.estimate_angle_x(), right_hand.estimate_angle_y(), right_hand.estimate_angle_z())
            angulos_str = ", ".join(f"{a:.2f}" for a in angulos)
            print(angulos_str)


    key = cv2.waitKey(1) & 0xFF
    if key == 27:  #ESC para sair
        break
