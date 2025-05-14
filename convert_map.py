from PIL import Image
import numpy as np
import cv2

def image_to_obstacle_map(image_path, size=(500, 500)):
    # Load image and convert to grayscale
    image = Image.open(image_path).convert("L")
    image_array = np.array(image)
    
    # Resize to target size
    image = image.resize(size, Image.BILINEAR)
    
    # Convert to numpy array
    image_array = np.array(image)
    
    # Threshold: nếu pixel tối (gần 0) thì coi là obstacle (1), ngược lại là free (0)
    threshold = 128
    obstacle_map = (image_array < threshold).astype(np.uint8)
    for i in range(500):
        obstacle_map[i][0] = 0
        obstacle_map[0][i] = 0
        obstacle_map[size[0] - 1][i] = 0
        obstacle_map[i][size[1] - 1] = 0

    # OpenCV yêu cầu ảnh nhị phân có giá trị 0 và 255
    binary_map = obstacle_map * 255

    # Tìm các contours (đường viền) của các vùng có giá trị 255
    contours, _ = cv2.findContours(binary_map, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Chuyển contours thành danh sách các polygon (danh sách các điểm)
    polygons = []
    for contour in contours:
        polygon = [(int(p[0][1]), int(p[0][0])) for p in contour]  # (row, col)
        polygons.append(polygon)

    return size, polygons

