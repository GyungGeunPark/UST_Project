# PICO OS 6 Early Access 신청 이메일

## 수신: developer@picoxr.com
## 제목: [OS 6 Early Access Request] Robot Teleoperation - XRoboToolkit + CloudXR Spatial Multitasking

---

Dear PICO Developer Relations Team,

I am writing to request access to the **PICO OS 6 Early Access Program** for our robotics teleoperation research project at the University of Science and Technology (UST), South Korea.

### Why We Need PICO OS 6

Our project requires **two XR applications running simultaneously** on the PICO 4 Ultra:

1. **XRoboToolkit App** - Full-body + hand tracking data capture (sends to PC via gRPC)
2. **CloudXR.js** (web browser) - Receives VR rendering stream from NVIDIA Isaac Lab simulation

On **PICO OS 5 (current: 5.15.x)**, only one immersive XR app can run at a time, which makes it impossible to have both VR visualization and full-body tracking active simultaneously. We believe **PICO OS 6's Spatial Engine and Shared Space** will solve this by enabling multiple XR apps to coexist.

### Project Overview

We are developing a **GPU-accelerated humanoid robot teleoperation system** for kitchen object sorting tasks:

- **Robot**: Unitree G1 + INSPIRE 5-finger dexterous hand (38-DOF control)
- **Simulation**: NVIDIA Isaac Lab 2.3.0 on RTX PRO 6000 (96GB VRAM)
- **Body Tracking**: PICO 4 Ultra + XRoboToolkit (full-body + hand 26-joint)
- **Finger Tracking**: UDCAP VR Gloves via VMC protocol (12-DOF per hand)
- **VR Rendering**: CloudXR 6.0.1 WebRTC streaming to PICO browser
- **Learning Pipeline**: BC-RNN + HG-DAgger corrective teaching + ensemble uncertainty

### Current Architecture

```
PICO 4 Ultra
  ├── XRoboToolkit App → gRPC → PC (RoboticsServiceProcess)
  └── CloudXR.js (browser) ← WebRTC ← PC (CloudXR Runtime 6.0.1)

PC (Ubuntu, RTX PRO 6000)
  ├── NVIDIA Isaac Lab (G1 Kitchen Sorting Environment)
  ├── CloudXR Runtime 6.0.1 (VR frame encoding)
  └── XRoboToolkit PC Service (tracking data reception)

Windows Mini-PC
  └── UDCAP Gloves → VMC/OSC UDP → PC (finger tracking)
```

### What We Are Requesting

1. **PICO OS 6 preview build for PICO 4 Ultra** - to test CloudXR.js + XRoboToolkit simultaneous execution in Shared Space
2. **OS 6 SDK access** - for potential integration optimization
3. **Technical guidance** - on whether Spatial Engine supports our use case (two XR data streams: one rendering input, one tracking output)

### Our Hardware

- PICO 4 Ultra (current OS: 5.15.x)
- PICO Motion Trackers (for full-body tracking via XRoboToolkit)

### Organization Information

- **Organization**: University of Science and Technology (UST) / [연구실명]
- **Location**: [도시], South Korea
- **Developer Console Account**: [PICO 개발자 계정 이메일]
- **Project Duration**: March 2026 - ongoing

We would be happy to provide detailed feedback on PICO OS 6 performance for robotics teleoperation use cases, which we believe is a valuable and underexplored application of the Spatial Engine.

Thank you for your consideration.

Best regards,
[이름]
[직책/학과]
University of Science and Technology (UST)
[이메일]
[전화번호]
