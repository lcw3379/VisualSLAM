
보통의 visual slam은 depth sensor가 내장된 RGBD카메라를 사용해서 이루어진다. 하지만 이러한 카메라들은 매우 비싸다는 단점이 있다... 학생으로서 최소 20만원 하는 비싼 카메라를 살 수가 없어서 다른 방법이 없을까 여러 방면으로 찾아보았다. 

그 결과로 다이소에서 5,000원이면 살 수 있는, 매우 저렴한 카메라로 visual slam을 하려 한 경험을 정리하였다. 

# 1. AI기반 depth estimation model 사용
MiDas라고 하는 ai기반의 모델을 사용해서 깊이 이미지를 추정하고 테스트로 point cloud를 생성해 보았다.

![002](https://github.com/user-attachments/assets/0e19d812-be6e-4778-9257-b1a62d443ad2)

MiDas를 이용하여 Depth Map을 생성하였다.

![point1](https://github.com/user-attachments/assets/75b24b24-a844-4042-9dfb-94229aad4b65)

![point2](https://github.com/user-attachments/assets/82e2665a-be75-4861-8e17-5a3664dbb9ff)

point cloud까지 생성해 본 모습이다. 3D Reconstruction 예제로 나온 다른 데이터셋에서는 Point Cloud가 정확하게 생성되었는데, AI 기반 Depth Map으로 생성을 하니 중앙 부분은 볼만하게 나오지만 이미지의 끝으로 갈수록 Point Cloud 생성이 불안한 모습을 볼 수 있었다.

일단 이 모델을 사용하여 실시간 SLAM을 해야 하므로 rtab_map에서 실행해 보았다.

![single_rtab](https://github.com/user-attachments/assets/d506094c-c26b-45f5-9f63-6f91a7b45910)


하지만 실시간 상황에서는 depth map의 편차가 커서인지 quality가 낮아 Visual SLAM이 불가능했다.

무거운 모델을 사용하면 더 정확성이 뛰어난 깊이 이미지를 얻지만, 그만큼 그래픽 카드 연산량이 많아진다. 실시간 visual slam에 활용할 건데 프로그램이 너무 무거워져 활용이 힘들었다.<br/>
그렇다고 가벼운 모델을 사용하면 깊이 이미지의 정확도가 낮고, 노이즈 때문에 비슷한 raw이미지라도 생성된 깊이 이미지가 서로 다르다. 이렇다 보니 부정확한 깊이 이미지와 point cloud가 만들어졌다. 

특히 가벼운 모델을 사용하여 rtab_map을 실행하였더니 rtab속 quality가 매우 낮아 odometry 생성이 불가능했다. 

그래서 일단 ai모델에 기반한 방법은 보류하기로 했다. 훗날 3D Reconstruction에 사용할 수도 있겠지만 지금은 Visual SLAM을 목표로 하기 때문에 다른 방법을 찾았다.


# 2. Stereo Vision
이론적 배경은 해당 pdf를 참고했다. http://www.cs.toronto.edu/~fidler/slides/2015/CSC420/lecture12_hres.pdf

OpenCV 공식 홈페이지도 도움이 되었다. https://docs.opencv.org/3.4/dd/d53/tutorial_py_depthmap.html

![stereo1](https://github.com/user-attachments/assets/e1d89ce7-cfec-4994-adc7-eb3a0119a598)

기본적으로 두 개의 카메라의 위치 차이를 통해서 깊이를 추정할 수 있다는 이론이다.

![stereo2](https://github.com/user-attachments/assets/b33b5455-d613-43a7-b009-d7495fc93d8d)![stereo3](https://github.com/user-attachments/assets/c30f0a19-ab79-47e2-94e1-6603c15a84de)


빨간 삼각형과 파란 삼각형의 닮음비를 이용해서, 카메라로부터 P까지의 깊이를 알아낼 수 있다. 

![stereo4](https://github.com/user-attachments/assets/d7c46963-5f77-4ad7-bc8c-417ae53708b1)

스테레오 카메라는 먼저 두 이미지 간의 픽셀 차이를 통해서 Disparity Map을 만들고, Disparity Map과 위쪽 이론의 깊이 추정을 토대로 Depth Map을 만든다. 여기엔 스테레오 카메라의 정확한 보정이 필요하다고 한다. 


# 3. Stereo Camera 제작

![카메라](https://github.com/user-attachments/assets/5f7e87ce-90c1-4ccc-9919-559c3ad3fd95)

스테레오 카메라를 제작하고 Rtab_Map을 사용하기 위하여 다이소 카메라를 하나 더 구입한 후 서로 연결하였다. 참으로 허접하기 짝이 없다.

![Honeycam 2024-07-23 09-19-23](https://github.com/user-attachments/assets/d305e368-e9d1-4aec-b924-bd2049016d0d)

실행과정은 다음과 같다.

  1. ros2 launch mycam stp_stereo.launch.py
  2. ros2 run mycam camerapub
  3. ros2 launch rtabmap_launch rtabmap.launch.py stereo:=true queue_size:=30 frame_id:=base_link approx_sync:=true


스테레오 카메라 보정을 하여 각 카메라의 camera matrix, distortion, r, p를 얻고, 이렇게 얻은 파라미터로 양쪽 이미지를 rectify하여 ros로 퍼블리싱한다. 또한 카메라 파라미터 정보 또한 CameraInfo 타입으로 퍼블리싱한다. 그 후 rtab-map을 스테레오 모드로 실행한다. 

책장처럼 밀도있고 복잡한 부분은 특징점을 잘 찾지만, 벽, 냉장고 등 밀도가 낮고 평평한 부분은 특징점을 찾지 못하는 단점이 있다. 사용한 카메라의 시야각이 매우 좁은 것 같아 더 좋은 카메라를 사용해야 더 좋은 SLAM이 가능해 보인다는 점이 조금은 아쉬웠다.

가장 중요한 부분이라고 생각한 것은 스테레오 카메라 보정이다. MATLAB, OPENCV, ROS패키지를 사용해서 해 보았는데 ROS패키지가 이미지를 일일이 저장할 필요가 없어 가장 편하다고 생각했다. 또한 구석에서 체커보드를 인식할수록 좋은 보정 데이터를 얻을 수 있다고 한다.

그리고 ros2 내부 프레임들간의 정확한 tranform을 해야 slam이 가능하다는 것이다. 각 토픽 프레임들간의 정확한 연결 없이는 불가능했다.


그럼에도 불구하고, 카메라의 성능이나 여타 다른 문제 때문인지, 3D Map 생성이 카메라에서 발산하는 형태로 만들어진다.

![Screenshot from 2024-08-27 16-35-05](https://github.com/user-attachments/assets/13315034-7a0b-4149-99fb-9f2735b17fdd)


결국 저렴한 가격의 카메라로 제작한 stereo camera는 visual odometry만 대략적으로 되는 것처럼 보이고, 실제로 정확한 3D MAP 생성을 위해선 Depth Sensor를 가진 카메라가 필수로 보인다. Rtab-map의 사용법을 공부한 걸 의의로 두어야 겠다.
