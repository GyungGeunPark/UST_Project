"""
setup/apply_omnigraph.py
OmniGraph 적용 메인 스크립트

Isaac Sim 씬에 OmniGraph를 적용하는 메인 스크립트입니다.

사용법:
    1. Isaac Sim에서 로봇 씬 로드
    2. Script Editor에서 이 스크립트 실행
    3. USD 저장 (File → Save As)
    4. 이후 USD 열고 Play만 누르면 자동 실행

Isaac Sim Script Editor에서 실행:
    exec(open("/home/isaac/ust_ws/ust_project1/scripts/setup/apply_omnigraph.py").read())

호환성:
    - Isaac Sim 4.5.0+: isaacsim.* 확장 사용
    - Isaac Sim 4.2 이하: omni.isaac.* 확장 사용
    - 자동 버전 감지 및 적절한 노드 타입 선택

Author: UST Robotics Project
Date: 2024
"""

import os
import sys
import importlib

# 모듈 경로 추가 (Isaac Sim Script Editor에서 __file__이 제대로 작동하지 않을 수 있음)
# 하드코딩된 경로 사용
SCRIPTS_DIR = "/home/isaac/ust_ws/ust_project1/scripts"

# 대안: __file__이 작동하면 사용
try:
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    _project_root = os.path.dirname(os.path.dirname(_script_dir))
    _scripts_dir = os.path.join(_project_root, "scripts")
    if os.path.exists(_scripts_dir) and "ust_project1" in _scripts_dir:
        SCRIPTS_DIR = _scripts_dir
except Exception:
    pass

print(f"[Setup] Using SCRIPTS_DIR: {SCRIPTS_DIR}")

if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)


def _clear_pycache():
    """
    __pycache__ 디렉토리의 .pyc 파일 무효화

    이전에 컴파일된 바이트코드가 문제를 일으킬 수 있으므로
    importlib을 사용하여 캐시를 무효화합니다.
    """
    try:
        import importlib
        # 캐시 무효화 (Python 3.3+)
        importlib.invalidate_caches()
        print("  [Cache] Invalidated import caches")
    except Exception as e:
        print(f"  [!] Could not invalidate caches: {e}")


def _reload_omnigraph_modules():
    """
    OmniGraph 모듈을 강제로 리로드하여 캐시 문제 해결

    Isaac Sim Script Editor에서 모듈이 캐시되어 이전 버전의
    API 감지 결과가 유지되는 문제를 해결합니다.
    """
    # 먼저 import 캐시 무효화
    _clear_pycache()

    modules_to_delete = [
        "omnigraph",
        "omnigraph.graph_builder",
        "omnigraph.ik_graph",
        "omnigraph.differential_drive_graph",
        "omnigraph.teleoperation_graph",
        "omnigraph.master_graph",
    ]

    # 먼저 모듈 완전히 삭제
    for module_name in modules_to_delete:
        if module_name in sys.modules:
            try:
                del sys.modules[module_name]
                print(f"  [Delete] {module_name}")
            except Exception as e:
                print(f"  [!] Could not delete {module_name}: {e}")

    # 다시 임포트
    try:
        import omnigraph.graph_builder
        import omnigraph.ik_graph
        import omnigraph.differential_drive_graph
        import omnigraph.teleoperation_graph
        import omnigraph.master_graph
        print("  [Import] All omnigraph modules reimported")
    except Exception as e:
        print(f"  [!] Could not reimport modules: {e}")
        import traceback
        traceback.print_exc()


def ensure_extensions_loaded() -> tuple:
    """
    필요한 Isaac Sim 확장들이 로드되었는지 확인하고 로드

    Isaac Sim 4.5.0+와 이전 버전 모두 지원합니다.

    Returns:
        (성공 여부, 새 API 사용 여부) 튜플
    """
    use_new_api = False

    try:
        import omni.kit.app

        ext_manager = omni.kit.app.get_app().get_extension_manager()

        # Isaac Sim 4.5.0+ 새 확장 이름
        # 참고: wheeled_robots는 버전에 따라 다른 이름일 수 있음
        new_extensions = [
            "isaacsim.core.nodes",
            "isaacsim.ros2.bridge",
        ]

        # wheeled_robots 확장 - 여러 이름 시도
        wheeled_robot_extensions = [
            "isaacsim.robot.wheeled_robots",
            "isaacsim.wheeled_robots",
        ]

        # 레거시 확장 이름 (Isaac Sim 4.2 이하)
        legacy_extensions = [
            "omni.isaac.core_nodes",
            "omni.isaac.ros2_bridge",
            "omni.isaac.wheeled_robots",
        ]

        # 공통 확장
        common_extensions = [
            "omni.graph.action",
            "omni.graph.nodes",
            "omni.graph.scriptnode",
        ]

        # 새 확장이 있는지 먼저 확인
        for ext_name in new_extensions:
            if ext_manager.is_extension_enabled(ext_name):
                use_new_api = True
                break

        # wheeled_robots 확장 확인 (여러 이름 시도)
        wheeled_ext_found = None
        for ext_name in wheeled_robot_extensions:
            if ext_manager.is_extension_enabled(ext_name):
                wheeled_ext_found = ext_name
                use_new_api = True
                break

        # 확장 목록 결정
        if use_new_api:
            required_extensions = new_extensions.copy()
            if wheeled_ext_found:
                required_extensions.append(wheeled_ext_found)
            required_extensions.extend(common_extensions)
            print("[Setup] Using Isaac Sim 4.5.0+ extensions (isaacsim.*)")
        else:
            # 새 확장 로드 시도
            try:
                for ext_name in new_extensions:
                    if not ext_manager.is_extension_enabled(ext_name):
                        ext_manager.set_extension_enabled_immediate(ext_name, True)

                # wheeled_robots 확장 로드 시도 (여러 이름)
                for ext_name in wheeled_robot_extensions:
                    try:
                        if not ext_manager.is_extension_enabled(ext_name):
                            ext_manager.set_extension_enabled_immediate(ext_name, True)
                        wheeled_ext_found = ext_name
                        print(f"[Setup] Enabled wheeled robots extension: {ext_name}")
                        break
                    except Exception:
                        continue

                use_new_api = True
                required_extensions = new_extensions.copy()
                if wheeled_ext_found:
                    required_extensions.append(wheeled_ext_found)
                required_extensions.extend(common_extensions)
                print("[Setup] Enabled Isaac Sim 4.5.0+ extensions")
            except Exception:
                # 새 확장이 없으면 레거시 사용
                required_extensions = legacy_extensions + common_extensions
                print("[Setup] Using legacy extensions (omni.isaac.*)")

        # 확장 활성화
        enabled_count = 0
        for ext_name in required_extensions:
            try:
                if not ext_manager.is_extension_enabled(ext_name):
                    ext_manager.set_extension_enabled_immediate(ext_name, True)
                    print(f"  [+] Enabled: {ext_name}")
                    enabled_count += 1
                else:
                    print(f"  [OK] Already enabled: {ext_name}")
            except Exception as e:
                print(f"  [!] Could not enable {ext_name}: {e}")

        if enabled_count > 0:
            print(f"[Setup] Enabled {enabled_count} extensions")

        return True, use_new_api

    except Exception as e:
        print(f"[Setup] Extension loading failed: {e}")
        print("[Setup] Continuing anyway - extensions may already be loaded")
        return False, use_new_api


def apply_omnigraph_to_scene(
    output_usd_path: str = None,
    config: dict = None,
    enable_ik: bool = True,
    enable_diff_drive: bool = True,
    enable_teleop: bool = True
) -> dict:
    """
    현재 씬에 OmniGraph 적용

    Isaac Sim 내에서 호출하여 OmniGraph를 생성하고 USD에 저장합니다.
    Isaac Sim 버전을 자동으로 감지하여 적절한 노드 타입을 사용합니다.

    Args:
        output_usd_path: USD 저장 경로 (선택, None이면 저장하지 않음)
        config: 커스텀 설정 딕셔너리 (선택)
        enable_ik: IK 그래프 활성화
        enable_diff_drive: 차동 구동 그래프 활성화
        enable_teleop: 텔레오퍼레이션 그래프 활성화

    Returns:
        각 그래프의 생성 결과 딕셔너리

    Example:
        # Script Editor에서 실행
        from setup.apply_omnigraph import apply_omnigraph_to_scene

        results = apply_omnigraph_to_scene(
            output_usd_path="/home/isaac/ust_ws/ust_project1/usd/scene_with_omnigraph.usd",
            enable_teleop=True
        )
    """
    print("\n" + "=" * 70)
    print(" OmniGraph Teleoperation System Setup")
    print(" UST Robotics Project")
    print(" (Auto-detects Isaac Sim version)")
    print("=" * 70)

    # Step 1: 필요한 확장 로드
    print("\n[Step 1] Loading required extensions...")
    success, use_new_api = ensure_extensions_loaded()

    # Step 2: 모듈 리로드 및 캐시 리셋
    print("\n[Step 2] Reloading modules and resetting cache...")
    _reload_omnigraph_modules()

    # 리로드 후 모듈에서 직접 가져오기 (캐시된 참조 사용 방지)
    try:
        import omnigraph.graph_builder as gb_module
        ROS2NodeFactory = gb_module.ROS2NodeFactory

        # 캐시 리셋 후 API 버전 강제 설정
        ROS2NodeFactory.reset_cache()
        if use_new_api:
            ROS2NodeFactory.force_new_api(True)
            print("[Setup] Forced ROS2NodeFactory to use new API (isaacsim.*)")
        else:
            ROS2NodeFactory.force_new_api(False)
            print("[Setup] Forced ROS2NodeFactory to use legacy API (omni.isaac.*)")

        # 실제 사용 가능한 노드 타입 확인 (디버깅용)
        try:
            available = ROS2NodeFactory._get_available_node_types()
            if available:
                ros2_nodes = [t for t in available if 'ros2' in t.lower()]
                core_nodes = [t for t in available if 'isaac' in t.lower() and 'articulation' in t.lower()]
                wheeled_nodes = [t for t in available if 'wheeled' in t.lower() or 'differential' in t.lower()]

                print(f"[Setup] Found {len(ros2_nodes)} ROS2 nodes, {len(core_nodes)} articulation nodes, {len(wheeled_nodes)} wheeled robot nodes")

                # 주요 노드 타입 확인
                key_nodes = ["context", "subscribe_twist", "articulation_controller", "differential_controller"]
                print("[Setup] Key node type mappings:")
                for key in key_nodes:
                    resolved = ROS2NodeFactory.get_node_type(key)
                    exists = resolved in available
                    status = "OK" if exists else "NOT FOUND"
                    print(f"  {key}: {resolved} [{status}]")

                # ROS2 관련 노드 타입 샘플 출력
                print("[Setup] Sample ROS2 node types found:")
                for t in sorted(ros2_nodes)[:10]:
                    print(f"  {t}")
            else:
                print("[Setup] Could not retrieve available node types")
        except Exception as debug_err:
            print(f"[Setup] Debug info collection failed: {debug_err}")

    except (ImportError, AttributeError) as e:
        print(f"[WARNING] Could not reset ROS2NodeFactory cache: {e}")

    # Step 3: 마스터 그래프 모듈 임포트
    print("\n[Step 3] Importing master graph module...")
    try:
        from omnigraph.master_graph import MasterControlGraph
    except ImportError:
        print("[ERROR] Failed to import omnigraph module")
        print(f"[ERROR] SCRIPTS_DIR: {SCRIPTS_DIR}")
        print("[ERROR] Make sure all modules are properly installed")
        return {}

    # 기본 설정
    default_config = {
        # 로봇 경로 (씬에 맞게 수정 필요)
        "manipulator_path": "/World/Robot/open_manipulator_x",
        "mobile_base_path": "/World/Robot/MobileBase",
        "chassis_path": "/World/Robot/MobileBase/base_link",
        "ik_target_path": "/World/IK_Target",

        # IK 설정
        "config_dir": "/home/isaac/ust_ws/ust_project1/config",
        "robot_desc_file": "open_x1_des.yaml",
        "urdf_file": "open_manipulator_x.urdf",
        "end_effector_frame": "end_effector_link",
        "ik_target_initial_position": [0.2, 0.0, 0.15],

        # 차동 구동 설정
        "wheel_radius": 0.05,
        "wheel_distance": 0.3,
        "left_wheel_joint": "left_wheel_joint",
        "right_wheel_joint": "right_wheel_joint",
        "max_linear_velocity": 1.0,
        "max_angular_velocity": 2.0,

        # 그리퍼 설정
        "gripper_joint_name": "gripper_left_joint",

        # 활성화 플래그
        "enable_ik": enable_ik,
        "enable_diff_drive": enable_diff_drive,
        "enable_teleoperation": enable_teleop,
        "enable_mobile_in_teleop": True,
        "enable_gripper_in_teleop": True,

        # ROS2 설정
        "namespace": "",
    }

    # 커스텀 설정 병합
    if config:
        default_config.update(config)

    # 마스터 그래프 생성
    master = MasterControlGraph(default_config)

    # 모든 그래프 생성
    results = master.create_all()

    # 결과 출력
    print("\n" + "=" * 70)
    print(" Setup Complete!")
    print("=" * 70)

    for name, success in results.items():
        status = "[OK]    " if success else "[FAILED]"
        print(f"  {status} {name}")

    # USD 저장
    if output_usd_path:
        master.save_to_usd(output_usd_path)
        print(f"\n  USD saved to: {output_usd_path}")

    # 다음 단계 안내
    _print_next_steps()

    return results


def apply_from_config(yaml_path: str, output_usd_path: str = None) -> dict:
    """
    YAML 설정 파일에서 OmniGraph 적용

    Args:
        yaml_path: robot_params.yaml 경로
        output_usd_path: USD 저장 경로

    Returns:
        생성 결과 딕셔너리
    """
    # 확장 로드 및 캐시 리셋
    success, use_new_api = ensure_extensions_loaded()
    _reload_omnigraph_modules()

    try:
        from omnigraph.graph_builder import ROS2NodeFactory
        ROS2NodeFactory.reset_cache()
        ROS2NodeFactory.force_new_api(use_new_api)
    except ImportError:
        pass

    try:
        from omnigraph.master_graph import MasterControlGraph
    except ImportError:
        print("[ERROR] Failed to import omnigraph module")
        return {}

    master = MasterControlGraph.from_yaml(yaml_path)
    results = master.create_all()

    if output_usd_path:
        master.save_to_usd(output_usd_path)

    _print_next_steps()

    return results


def _print_next_steps():
    """다음 단계 안내 출력"""
    print("""
=" * 70
 다음 단계
=" * 70

1. USD 파일 저장:
   File → Save As → ust_project1_with_omnigraph.usd

2. Isaac Sim 재시작 후 USD 열기:
   File → Open → ust_project1_with_omnigraph.usd

3. Play 버튼 클릭:
   - OmniGraph가 자동으로 실행됩니다
   - ROS2 토픽이 활성화됩니다

4. ROS2 테스트:
   # 토픽 목록 확인
   ros2 topic list

   # Joint States 확인
   ros2 topic echo /joint_states

   # 모바일 베이스 제어 테스트
   ros2 topic pub /cmd_vel geometry_msgs/Twist \\
     "{linear: {x: 0.1}, angular: {z: 0.1}}"

5. Quest2ROS 텔레오퍼레이션:
   - Meta Quest에서 Quest2ROS 앱 실행
   - 동일 네트워크 연결 확인
   - VR 컨트롤러로 로봇 제어

=" * 70
""")


# =============================================================================
# 단독 실행 시
# =============================================================================

def main():
    """메인 함수 (Script Editor에서 직접 실행용)"""
    # 기본 설정으로 실행
    results = apply_omnigraph_to_scene(
        output_usd_path="/home/isaac/ust_ws/ust_project1/usd/ust_project1_with_omnigraph.usd",
        enable_ik=True,
        enable_diff_drive=True,
        enable_teleop=True
    )

    print("\n" + "=" * 70)
    print(" 적용 완료!")
    print("=" * 70)

    return results


# Script Editor에서 직접 실행 시
if __name__ == "__main__":
    main()


# =============================================================================
# Quick Access Functions (Script Editor용)
# =============================================================================

def quick_ik_only():
    """IK 그래프만 빠르게 적용"""
    return apply_omnigraph_to_scene(
        enable_ik=True,
        enable_diff_drive=False,
        enable_teleop=False
    )


def quick_mobile_only():
    """모바일 베이스 그래프만 빠르게 적용"""
    return apply_omnigraph_to_scene(
        enable_ik=False,
        enable_diff_drive=True,
        enable_teleop=False
    )


def quick_teleop_only():
    """텔레오퍼레이션 그래프만 빠르게 적용"""
    return apply_omnigraph_to_scene(
        enable_ik=False,
        enable_diff_drive=False,
        enable_teleop=True
    )


def quick_full_setup():
    """모든 그래프 빠르게 적용 및 저장"""
    return apply_omnigraph_to_scene(
        output_usd_path="/home/isaac/ust_ws/ust_project1/usd/ust_project1_with_omnigraph.usd",
        enable_ik=True,
        enable_diff_drive=True,
        enable_teleop=True
    )
