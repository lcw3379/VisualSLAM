
보통의 visual slam은 depth sensor가 내장된 RGBD카메라를 사용해서 이루어진다. 하지만 이러한 카메라들은 매우 비싸다는 단점이 있다... 학생으로서 최소 20만원 하는 비싼 카메라를 살 수가 없어서 다른 방법이 없을까 여러 방면으로 찾아보았다. 

그 결과로 다이소에서 5,000원이면 살 수 있는, 매우 저렴한 카메라로 visual slam을 하려 한 경험을 정리하였다. 

# 1. ai기반 depth estimation model 사용
MiDas라고 하는 ai기반의 모델을 사용해서 깊이 이미지를 추정하고 point cloud를 생성해 보았지만, 여러 단점이 있었다.
(point cloud 만든 거 사진) 

1. 무거운 모델을 사용하면 더 정확성이 뛰어난 깊이 이미지를 얻지만, 그만큼 그래픽 카드 연산량이 많아진다. 실시간 visual slam에 활용할 건데 프로그램이 너무 무거워져 활용이 힘들었다.<br/>
2. 그렇다고 가벼운 모델을 사용하면 깊이 이미지의 정확도가 낮고, 노이즈 때문에 비슷한 raw이미지라도 생성된 깊이 이미지가 서로 다르다. 이렇다 보니 부정확한 깊이 이미지와 point cloud가 만들어졌다. 

특히 가벼운 모델을 사용하여 rtab_map을 실행하였더니 rtab속 quality가 매우 낮아 odometry 생성이 불가능했다. 

그래서 일단 ai모델에 기반한 방법은 보류하고, stereo 카메라로 시선을 돌렸다. 

# 2. stereo camera 제작
(스테레오 기본 원리 설명)


![카메라](https://github.com/user-attachments/assets/5f7e87ce-90c1-4ccc-9919-559c3ad3fd95)

다이소 카메라 두 대를 연결하였다. 참으로 허접하기 짝이 없다.

![Honeycam 2024-07-23 09-19-23](https://github.com/user-attachments/assets/d305e368-e9d1-4aec-b924-bd2049016d0d)

스테레오 카메라 보정을 하여 각 카메라의 camera matrix, distortion, r, p를 얻고, 이렇게 얻은 파라미터로 양쪽 이미지를 rectify하여 ros로 퍼블리싱한다. 그 후 rtab-map을 스테레오 모드로 실행한다. 

책장처럼 밀도있고 복잡한 부분은 특징점을 잘 찾지만, 벽, 냉장고 등 밀도가 낮고 평평한 부분은 특징점을 찾지 못하는 단점이 있다. 사용한 카메라의 시야각이 60°인데 

가장 중요한 부분은 스테레오 카메라 보정, ros2 내부 프레임들간의 정확한 tranform을 해야 slam이 가능하다는 것이다. 

솔직히 나도 잘 될지는 몰랐지만, 제작비 12000원의 스테레오 카메라로서는 생각보다 좋은 성능을 보여준 것 같았다. 

훗날 자금에 여유가 생긴다면 rgbd 카메라를 이용한 3d reconstruction 또한 해보고 싶다. rtabmap 에서 visual odometry에 쓰인 이미지들을 묶어서 저장해 데이터셋을 만들 수 있게 해 주니 depth map과 같이 저장해서 colmap등의 프로그램으로 3d reconstruction을 하면 될 것이다.
