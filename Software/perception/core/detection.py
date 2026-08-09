import cv2
import numpy as np

def detect_edges(data: np.ndarray, algorithm: str = "sobel",
                   ksize: int = 3, low: int = 100, high: int = 200) -> np.ndarray:
    """
    Detect images in an input image with various algorithms.
    Args:
        data: image pixel data
        algorithm: edge detection algorithm, from "sobel", "scharr", "laplacian", or "canny"
        ksize: kernel size of "sobel" and "canny"
        low: lower intensity threshold for valid edge
        high: higher intensity threshold for valid edge
    Returns:
        edges: image of binary-masked detected edges
    """
    match (algorithm):
        case "sobel":
            sobelx = cv2.Sobel(data, cv2.CV_64F, 1, 0, ksize=ksize)
            sobely = cv2.Sobel(data, cv2.CV_64F, 0, 1, ksize=ksize)
            edges = cv2.convertScaleAbs(cv2.magnitude(sobelx, sobely))
            edges = cv2.cvtColor(edges, cv2.COLOR_BGR2GRAY)
            _, edges = cv2.threshold(edges, low, high, cv2.THRESH_BINARY)
        case "scharr":
            scharrx = cv2.Scharr(data, cv2.CV_64F, 1, 0)
            scharry = cv2.Scharr(data, cv2.CV_64F, 0, 1)
            edges = cv2.convertScaleAbs(cv2.magnitude(scharrx, scharry))
            edges = cv2.cvtColor(edges, cv2.COLOR_BGR2GRAY)
            _, edges = cv2.threshold(edges, low, high, cv2.THRESH_BINARY)
        case "laplacian":
            laplacian = cv2.Laplacian(data, cv2.CV_64F)
            edges = cv2.convertScaleAbs(laplacian)
            edges = cv2.cvtColor(edges, cv2.COLOR_BGR2GRAY)
            _, edges = cv2.threshold(edges, low, high, cv2.THRESH_BINARY)
        case "canny":
            smoothed = cv2.GaussianBlur(data, (ksize, ksize), 1.4)
            edges = cv2.Canny(smoothed, threshold1=low, threshold2=high)
    return edges

def detect_contours(edge_data: np.ndarray) -> list[np.ndarray]:
    """
    Detect contours from an edge binary mask.
    Args:
        edge_data: input edge binary mask of an image
    Returns:
        contours: list of contour images
    """
    contours, _ = cv2.findContours(edge_data, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return contours

def detect_squares(contour_data: list[np.ndarray], cepsilon: float = 0.02,
                   area_thresh: float = 500, fill_thresh: float = 0.5) -> list[np.ndarray]:
    """
    Detect squares from a list of contours.
    Args:
        contour_data: input list of contours
        cepsilon: constant term to multiply `epsilon` by, lower is stricter
        area_thresh: area in number of pixels squared that a contour must be to be considered a square
        fill_thresh: ratio of pixels within contour vs pixels in contour bounding box that a contour
                     must fulfill to be considered a square
    Returns:
        squares: detected squares from the list of contours
    """
    squares = []
    for contour in contour_data:
        epsilon = cepsilon * cv2.arcLength(contour, True)
        corners = cv2.approxPolyDP(contour, epsilon, True)
        if len(corners) == 4 and cv2.isContourConvex(corners):
            area = cv2.contourArea(corners)
            _, _, w, h = cv2.boundingRect(corners)
            fill_ratio = area / (w * h)
            if area > area_thresh and fill_ratio > fill_thresh:
                squares.append(corners)
    return squares

def solve_homography(square_data: np.ndarray, source_img: np.ndarray):
    corners = square_data.reshape(4, 2)
    s = corners.sum(axis=1)
    d = corners[:, 0] - corners[:, 1]
    ordered_corners = np.array([
        corners[np.argmin(s)],  # top-left
        corners[np.argmax(d)],  # top-right
        corners[np.argmax(s)],  # bottom-right
        corners[np.argmin(d)],  # bottom-left
    ], dtype=np.float32)

    dst = np.array([[0, 0],[199, 0], [199, 199], [0,  199]], dtype=np.float32)
    H = cv2.getPerspectiveTransform(ordered_corners, dst)

    warped = cv2.warpPerspective(source_img, H, (200, 200))
    return warped

def solve_microtag_fiducial():
    pass