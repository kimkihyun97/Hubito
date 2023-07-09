UWB의 노이즈는 거의 미미한 수준이며 매우 정확한거 같습니다.
하지만 각도 부분에서 간혹 값이 튀는 경우가 있지만 크게 문제되지는 않습니다.
following은 순수 uwb로만 진행됩니다.
사람이 빠르게 이동하거나 로봇의 뒤로 움직일 경우 완벽한 위치 파악이 되지 않아 following이 불가합니다.
anchor의 추가 부착이 필요한거 같습니다.

칼만필터를 계산하기 위해 ros message filter를 사용해 depth camera imu와 UWB(기존 uwb에 시간을 추가)를 동시에 받아 콜백함수를 실행합니다.

칼만필터의 경우 잘 작동하는 지 확인하지 못하였습니다.

참고 자료 입니다.
A Human-Tracking Robot Using Ultra Wideband Technology
-Tao Feng; Yao Yu; Lin Wu; Yanru Bai; Ziang Xiao; Zhen Lu

https://github.com/nooploop-dev/autorobo_a

많이 부족한 코드입니다. 



