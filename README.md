# 🤖 Lie Group 기반 수치해석 역기구학 (Numerical IK)
![example](https://github.com/user-attachments/assets/889392eb-6135-4069-9b4d-3f3a21fa743e)

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

* `SCARA.py` – SCARA IK 계산, joint space trajectory 생성 및 시각화

* `WAM.py` – barretWAM IK 계산, joint space trajectory 생성 및 시각화


---

## 🧠 Lie Group

* ✅ 좌표계에 구애받지 않는 수식 표현

* ✅ 로봇 모션 및 제어 전반에 걸쳐 활용 가능한 유연한 구조

* ✅ 추후 SLAM이나 최적화 문제로의 확장에 바로 적용 가능

---

## 🚀 로드맵

- [x] Support SCARA and 7DOF BarretWAM
- [x] 5th-order trajectory generation
- [ ] Feedback linearization or impedance controller
- [ ] 3D animation with Matplotlib or Unity integration
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
