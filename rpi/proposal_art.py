from datetime import datetime
import cv2
import numpy as np

class ProposalArt:
    def __init__(self, file):
        self.orginal = cv2.imread(file)

    def _edge_mask(self, img, line_size, blur_value):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.medianBlur(gray, blur_value)
        edges = cv2.adaptiveThreshold(gray_blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, line_size, blur_value)
        return edges

    def _color_quantization(self, img, k):
        # Transform the image
        data = np.float32(img).reshape((-1, 3))

        # Determine criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.001)

        # Implementing K-Means
        ret, label, center = cv2.kmeans(data, k, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        center = np.uint8(center)
        result = center[label.flatten()]
        result = result.reshape(img.shape)
        return result

    def create(self, out_file = "/home/pi/camera/proposal_art.jpg", line_size = 7, blur_value = 7, color_palette = 9):
        edges = self._edge_mask(self.orginal, line_size, blur_value)
        quantized = self._color_quantization(self.orginal, color_palette)
        cartoon = cv2.bitwise_and(quantized, quantized, mask=edges)
        cv2.imwrite(out_file, cartoon)
        return cartoon 


def main():
    proposal_art = ProposalArt("/home/pi/camera/proposal_art_source.jpg")
    proposal_art.create(line_size = 7, blur_value = 9, color_palette = 5)

if __name__ == '__main__':
    main()
