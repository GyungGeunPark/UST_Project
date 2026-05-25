 실행 가이드                                                                                                                                                                                                                            
                                                                                                                                                                                                                                         
  방안 3: 모니터 뷰 (즉시 사용 가능, 추천)                                                                                                                                                                                               
                                                                                                                                                                                                                                         
  VR 렌더링 없이 PC 모니터에서 시뮬레이션을 보며 PICO로 전신 제어합니다.                                                                                                                                                                 
                                                                                                                                                                                                                                         
  ┌─────────────────────────────────────────────────────────┐                                                                                                                                                                            
  │  PICO 4 Ultra (XRoboToolkit 실행)                       │
  │    └── gRPC → RoboticsServiceProcess (PC)              │                                                                                                                                                                             
  │                └── ConsoleDemo (subprocess)             │                                                                                                                                                                            
  │                     └── unified_bridge.py (TCP:8889)   │                                                                                                                                                                             
  │                          └── Isaac Lab (PC 모니터)     │                                                                                                                                                                             
  │                                                         │                                                                                                                                                                            
  │  Windows 미니PC (UDCAP 글러브)                         │                                                                                                                                                                             
  │    └── VMC UDP:39539 → unified_bridge.py               │                                                                                                                                                                             
  └─────────────────────────────────────────────────────────┘                                                                                                                                                                            
  
  Step 1: 호스트에서 환경 설정 + 프로세스 시작                                                                                                                                                                                           
  # 호스트 (Ubuntu 24.04)                                   
  source ust_ws/ust_260220/setup_pico_env.sh                                                                                                                                                                                             
  이것으로 자동 시작됨:                                                                                                                                                                                                                  
  - RoboticsServiceProcess (gRPC 서버)
  - unified_bridge.py (ConsoleDemo subprocess + UDCAP 수신 + TCP 출력)                                                                                                                                                                   
                                                                                                                                                                                                                                         
  Step 2: PICO 연결                                                                                                                                                                                                                      
  - PICO에서 XRoboToolkit App 실행                                                                                                                                                                                                       
  - PC Service IP에 호스트 IP 입력                                                                                                                                                                                                       
  - "device connect" 메시지 확인                                                                                                                                                                                                         
                                                                                                                                                                                                                                         
  Step 3: UDCAP 연결 (선택)                                                                                                                                                                                                              
  - Windows 미니PC에서 UDCAP 소프트웨어 실행                                                                                                                                                                                             
  - VMC 대상: 호스트IP:39539                                                                                                                                                                                                             
                                                            
  Step 4: Isaac Lab 실행 (Docker)                                                                                                                                                                                                        
  # Docker 내부                                             
  ./isaaclab.sh -p ust_ws/ust_260220/scripts/run_teleop.py \
      --teleop_device pico --render_mode monitor                                                                                                                                                                                         
  - XR 없이 실행되어 Isaac Lab GUI 창에 시뮬레이션이 표시됩니다
  - [XRT] 첫 번째 트래킹 데이터 수신! 메시지가 나오면 정상                                                                                                                                                                               
                                                                                                                                                                                                                                         
  UDCAP 없이 실행 (XRT 핸드 트래킹만):                                                                                                                                                                                                   
  ./isaaclab.sh -p ust_ws/ust_260220/scripts/run_teleop.py \                                                                                                                                                                             
      --teleop_device pico_no_udcap --render_mode monitor                                                                                                                                                                                
                                                                                                                                                                                                                                         
  ---                                                                                                                                                                                                                                    
  방안 1: PICO Connect (SteamVR PCVR + VR 뷰)                                                                                                                                                                                            
                                                                                                                                                                                                                                         
  PICO Connect로 VR 화면을 스트리밍하면서 XRoboToolkit으로 트래킹합니다.                                                                                                                                                                 
                                                                                                                                                                                                                                         
  ┌─────────────────────────────────────────────────────────┐
  │  PICO 4 Ultra                                           │                                                                                                                                                                            
  │    ├── PICO Connect (시스템 서비스) ←→ SteamVR (PC)    │                                                                                                                                                                             
  │    │     └── VR 렌더링 스트리밍 (WiFi 6E / USB-C)     │                                                                                                                                                                              
  │    └── XRoboToolkit App (포그라운드)                    │                                                                                                                                                                            
  │          └── gRPC → RoboticsServiceProcess (PC)        │                                                                                                                                                                             
  │               └── unified_bridge.py → Isaac Lab        │                                                                                                                                                                             
  └─────────────────────────────────────────────────────────┘                                                                                                                                                                            
                                                            
  전제 조건:                                                                                                                                                                                                                             
  1. PC에 Steam + SteamVR 설치                              
  2. PC에 PICO Connect 소프트웨어 설치 (PICO 공식 사이트)                                                                                                                                                                                
  3. PICO 4 Ultra에서 PICO Connect 활성화 (설정 → 일반)  
                                                                                                                                                                                                                                         
  Step 1: SteamVR 시작 (PC)                                                                                                                                                                                                              
  # SteamVR 실행 (백그라운드)                                                                                                                                                                                                            
  steam steam://rungameid/250820 &                                                                                                                                                                                                       
                                                                                                                                                                                                                                         
  Step 2: PICO Connect 연결
  - PICO 설정 → 일반 → PICO Connect → PC 연결                                                                                                                                                                                            
  - USB-C 또는 WiFi로 연결 (WiFi 6E 라우터 권장)            
  - PC의 PICO Connect 앱에서 "연결됨" 확인                                                                                                                                                                                               
                                                                                                                                                                                                                                         
  Step 3: XRoboToolkit + 브릿지 시작
  # 호스트 (Ubuntu 24.04)                                                                                                                                                                                                                
  source ust_ws/ust_260220/setup_pico_connect_env.sh        
                                                                                                                                                                                                                                         
  Step 4: PICO에서 XRoboToolkit App 실행                                                                                                                                                                                                 
  - PICO Connect는 시스템 서비스로 계속 동작
  - XRoboToolkit은 트래킹 데이터만 전송                                                                                                                                                                                                  
  - 핵심: PICO Connect는 OS 레벨 서비스이므로 XRoboToolkit과 공존 가능
                                                                                                                                                                                                                                         
  Step 5: Isaac Lab 실행 (Docker)                                                                                                                                                                                                        
  ./isaaclab.sh -p ust_ws/ust_260220/scripts/run_teleop.py \                                                                                                                                                                             
      --teleop_device pico --render_mode pico_connect                                                                                                                                                                                    
                                                                                                                                                                                                                                         
  ---                                                                                                                                                                                                                                    
  변경 파일 요약
                                                                                                                                                                                                                                         
  ┌────────────────────────────┬─────────────────────────────────────────────────────────────┐
  │            파일            │                          변경 내용                          │
  ├────────────────────────────┼─────────────────────────────────────────────────────────────┤
  │ teleop/unified_bridge.py   │ ConsoleDemo subprocess 모드 추가 (TCP 7777 → stdout 파이프) │
  ├────────────────────────────┼─────────────────────────────────────────────────────────────┤
  │ scripts/run_teleop.py      │ --render_mode 플래그 추가 (monitor/pico_connect/cloudxr)    │                                                                                                                                           
  ├────────────────────────────┼─────────────────────────────────────────────────────────────┤                                                                                                                                           
  │ kitchen_sorting_env_cfg.py │ KitchenSortingG1MonitorEnvCfg 추가, bridge_port 8888 통일   │                                                                                                                                           
  ├────────────────────────────┼─────────────────────────────────────────────────────────────┤                                                                                                                                           
  │ __init__.py                │ Isaac-KitchenSorting-G1-InspireFTP-Monitor-v0 등록          │
  ├────────────────────────────┼─────────────────────────────────────────────────────────────┤                                                                                                                                           
  │ setup_pico_env.sh          │ ConsoleDemo 별도실행 제거, 안내문 업데이트                  │
  ├────────────────────────────┼─────────────────────────────────────────────────────────────┤                                                                                                                                           
  │ setup_pico_connect_env.sh  │ 신규 - SteamVR + PICO Connect 설정 스크립트                 │
  └────────────────────────────┴─────────────────────────────────────────────────────────────┘                                                                                                                                           
                                                            
  트러블슈팅                                                                                                                                                                                                                             
                                                            
  [XRT] ConsoleDemo 바이너리를 찾을 수 없습니다                                                                                                                                                                                          
  # 수동 경로 지정                                          
  python3 teleop/unified_bridge.py --console_demo /path/to/ConsoleDemo                                                                                                                                                                   
                                                                      
  [PICODevice] 브릿지 연결 시도 후 연결 안됨                                                                                                                                                                                             
  - 호스트에서 unified_bridge.py가 실행 중인지 확인: pgrep -af unified_bridge                                                                                                                                                            
  - 포트 확인: ss -tlnp | grep 8888                                                                                                                                                                                                      
                                                                                                                                                                                                                                         
  PICO Connect에서 XRoboToolkit 동시 실행 불가 시                                                                                                                                                                                        
  - PICO Connect를 먼저 연결 → 상태바에서 연결 유지 확인                                                                                                                                                                                 
  - 그 후 XRoboToolkit App 실행                                                                                                                                                                                                          
  - PICO OS 버전 5.8+ 필요 (멀티태스킹 지원)                                                          
