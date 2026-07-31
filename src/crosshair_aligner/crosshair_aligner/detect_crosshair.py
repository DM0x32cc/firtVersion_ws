import cv2
import numpy as np

def detect_crosshair(image):
    # Resize to 640x480 for speed
    image = cv2.resize(image, (640, 480))

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0.0, 0.0, False

    max_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(max_contour) < 100:
        return 0.0, 0.0, False

    mask = np.zeros_like(gray)
    cv2.drawContours(mask, [max_contour], -1, 255, -1)

    ys, xs = np.where(mask == 255)
    intensities = gray[ys, xs].astype(np.float64)

    total_intensity = np.sum(intensities)
    if total_intensity == 0:
        return 0.0, 0.0, False

    cx = np.sum(xs * intensities) / total_intensity
    cy = np.sum(ys * intensities) / total_intensity

    return cx, cy, True
