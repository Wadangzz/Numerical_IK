# 🤖 Lie Group 기반 수치해석 역기구학 (Numerical IK)
![example](https://github.com/user-attachments/assets/889392eb-6135-4069-9b4d-3f3a21fa743e)

<p align="center">
  <img src="scara_animation.gif" width="30%">
  <img src="scara_animation2.gif" width="30%">
  <img src="UR5.gif" width="30%">
</p>

Lie group 연산을 활용한 Newton-Raphson Invese Kinematics Python 프로젝트입니다.

1~2년 전 프로그래밍에 무지할 때 Excel을 활용해서 구현하여 교내 경진대회에서 우수상을 수상했습니다.   
Python을 익히면서 구조화된 모듈로 재구성하고 여러 로봇에 범용 적용 가능하도록 확장하고 있습니다.

---

## ✨ 주요 기능

* ✅ Newton-Raphson 방식의 수치해석 역기구학 (Numerical IK)

* ✅ SE(3), se(3) 기반 Matrix exponential, logarithm 연산

* ✅ SCARA, Barret WAM 등 다양한 로봇 구조 모듈화, 추후 추가 가능

* ✅ 5차 다항식 기반 time scaling trajectory 생성

* ✅ 관절 각도, 속도, 가속도 시각화

* 🚧 피드백 제어 루프, 실시간 제어 연동 예정

---

## 📦 구성 모듈

* `MyRobotMath.py` – Lie Group, Lie algebra 연산(Skew-symmetric, Jacobian, twist, Matrix exponential, logarithm 등)

* `Robot.py` – SCARA, BarretWAM 등 Robot 클래스 정의, 각도 단위 변환 및 정규화 메서드 + 초기값 호출 메서드 포함

* `SCARA.py` – SCARA IK 계산, joint space trajectory 생성

* `WAM.py` – barretWAM IK 계산, joint space trajectory 생성

* `UR5.py` – UR5 IK 계산, joint space trajectory 생성, Unity 시각화

* ~~`visualization.py` - Robot Link, Joint, End-Effector 시각화~~ 왠만하면 사용하지 않는 게 좋을 듯

---

## 🚀 로드맵

- [x] Support SCARA and 7DOF BarretWAM
- [x] 5th-order trajectory generation
- [X] 3D animation with Matplotlib or Unity integration
- [ ] Feedback linearization or impedance controller
- [ ] Export to CSV / ROS message

---

## 🛠️ 실행 환경

* Python 3.9
* `numpy`, `scipy`, `matplotlib`

```bash
pip install numpy scipy matplotlib
```
---

## 🔗 License

MIT License. Free to use, modify, and learn from.

---
