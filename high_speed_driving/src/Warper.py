import cv2
import numpy as np

class Warper:
    def __init__(self):
	# img의 가로 / 세로
        h = 480
        w = 640
        print("h : " ,h)
        print("w : " ,w)
         
        # distort src(INPUT) to dst(OUTPUT) 
        src = np.float32([ # 4개의 원본 좌표 점
            [w * 1.5, h * 1.3], # [960, 624]
            [w * (-0.1), h * 1.3], # [-64.0, 624]
            [0, h * 0.62], # [0, 297.6]
            [w, h * 0.62], # [640, 297.6]
        ])
        dst = np.float32([ # 4개의 결과 좌표 점 - 투시 변환 후의 이미지에서 'src' 좌표가 매핑되는 위치를 나타냄.
            [w * 0.65, h * 0.98], # [416, 470.4]
            [w * 0.35, h * 0.98], # [224, 470.4]
            [w * (-0.3), 0], # [-192, 0]
            [w * 1.3, 0], # [832, 0]
        ])
        
        

        self.M = cv2.getPerspectiveTransform(src, dst) # self.M : 투시변환 행렬(src to dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src) # self.Minv : 투시변환 행렬(dst to src)

    # 이미지를 투시 변환하여 변형된 이미지를 반환 - img -> self.M 투시 변환 행렬을 적용하여 변형 -> 변형된 이미지의 크기는 원본 이미지의 가로(img.shape[1])와 세로(img.shape[0]) 크기로 설정
    def warp(self, img): 
        return cv2.warpPerspective(
            img,
            self.M, 
            (img.shape[1], img.shape[0]), # img w, h
            flags=cv2.INTER_LINEAR # 이미지 보간 방법을 선형 보간으로 설정
        )
    # 이미지를 역 투시 변환하여 원본 이미지로 복원 - img -> self.Minv 역 투시 변환 행렬을 적용하여 원본 이미지로 역변환 -> 원본 이미지의 가로(img.shape[1])와 세로(img.shape[0]) 크기
    def unwarp(self, img):
        return cv2.warpPersective(
            img,
            self.Minv,
            (img.shape[1], img.shape[0]),
            flags=cv2.INTER_LINEAR # 이미지 보간 방법을 선형 보간으로 설정
        )