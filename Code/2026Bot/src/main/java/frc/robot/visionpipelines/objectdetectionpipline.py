import cv2
import numpy as np

def runPipeline(image, ll_robot):
    detections = ll_robot.get_target_list()
    clump_estimates = []

    AREA_AT_ZERO = 1.4 
    SLOPE = 0.015 

    for det in detections:
        ty = det[2]
        area = det[3]
        x1, y1 = int(det[6]), int(det[7])

        expected_single_area = max(0.4, AREA_AT_ZERO - (ty * SLOPE))
        estimate = max(1, int(round(area / expected_single_area)))
        clump_estimates.append(estimate)

        cv2.putText(image, str(estimate), (x1, y1 - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

    return [sum(clump_estimates)], image