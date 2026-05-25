"""
setup/verify_setup.py
OmniGraph 설정 검증 스크립트

OmniGraph가 올바르게 설정되었는지 검증합니다.

Usage:
    from setup.verify_setup import verify_omnigraph_setup
    verify_omnigraph_setup()

Author: UST Robotics Project
Date: 2024
"""

from typing import Dict, List, Optional, Any


def verify_omnigraph_setup(verbose: bool = True) -> Dict[str, bool]:
    """
    OmniGraph 설정 검증

    모든 그래프와 노드가 올바르게 설정되었는지 확인합니다.

    Args:
        verbose: 상세 출력 여부

    Returns:
        검증 결과 딕셔너리
    """
    results = {
        "stage_loaded": False,
        "ik_graph": False,
        "diff_drive_graph": False,
        "teleop_graph": False,
        "ik_target": False,
        "ros2_context": False,
    }

    if verbose:
        print("\n" + "=" * 60)
        print("OmniGraph Setup Verification")
        print("=" * 60)

    try:
        import omni.graph.core as og
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            if verbose:
                print("[ERROR] No stage loaded")
            return results

        results["stage_loaded"] = True
        if verbose:
            print("[OK] Stage loaded")

        # 검증할 그래프 목록
        graphs_to_check = {
            "ik_graph": "/World/IK_Controller_Graph",
            "diff_drive_graph": "/World/DifferentialDrive_Graph",
            "teleop_graph": "/World/Teleoperation_Graph",
        }

        for key, graph_path in graphs_to_check.items():
            result = _verify_graph(graph_path, verbose)
            results[key] = result

        # IK Target Xform 확인
        ik_target_path = "/World/IK_Target"
        ik_prim = stage.GetPrimAtPath(ik_target_path)
        if ik_prim.IsValid():
            results["ik_target"] = True
            if verbose:
                print(f"[OK] IK Target exists at {ik_target_path}")
        else:
            if verbose:
                print(f"[MISSING] IK Target not found at {ik_target_path}")

        # ROS2 Context 확인 (어느 그래프에든 있으면 OK)
        for graph_path in graphs_to_check.values():
            context_path = f"{graph_path}/ROS2Context"
            context_prim = stage.GetPrimAtPath(context_path)
            if context_prim.IsValid():
                results["ros2_context"] = True
                if verbose:
                    print(f"[OK] ROS2 Context found")
                break

        if not results["ros2_context"] and verbose:
            print("[WARNING] ROS2 Context not found in any graph")

        # 결과 요약
        if verbose:
            _print_summary(results)

        return results

    except Exception as e:
        if verbose:
            print(f"[ERROR] Verification failed: {e}")
        return results


def _verify_graph(graph_path: str, verbose: bool) -> bool:
    """개별 그래프 검증"""
    try:
        import omni.graph.core as og

        graph = og.get_graph_by_path(graph_path)
        if graph is None:
            if verbose:
                print(f"[SKIP] Graph not found: {graph_path}")
            return False

        is_valid = graph.is_valid()

        if verbose:
            print(f"\n--- {graph_path} ---")
            print(f"  Valid: {is_valid}")
            print(f"  Evaluator: {graph.get_evaluator_name()}")

            # 노드 확인
            nodes = graph.get_nodes()
            print(f"  Nodes: {len(nodes)}")

            invalid_nodes = []
            for node in nodes:
                node_path = node.get_prim_path()
                node_name = node_path.split("/")[-1]
                if not node.is_valid():
                    invalid_nodes.append(node_name)
                    print(f"    [INVALID] {node_name}")
                else:
                    print(f"    [OK] {node_name}")

            if invalid_nodes:
                print(f"  WARNING: {len(invalid_nodes)} invalid nodes")

        return is_valid

    except Exception as e:
        if verbose:
            print(f"[ERROR] Failed to verify {graph_path}: {e}")
        return False


def _print_summary(results: Dict[str, bool]):
    """결과 요약 출력"""
    print("\n" + "=" * 60)
    print("Verification Summary")
    print("=" * 60)

    total = len(results)
    passed = sum(results.values())

    for key, success in results.items():
        status = "[PASS]" if success else "[FAIL]"
        print(f"  {status} {key}")

    print("-" * 60)
    print(f"  Total: {passed}/{total} checks passed")

    if passed == total:
        print("\n  [SUCCESS] All checks passed!")
        print("  OmniGraph is properly configured.")
    elif passed >= total - 2:
        print("\n  [WARNING] Some optional components missing.")
        print("  Core functionality may still work.")
    else:
        print("\n  [ERROR] Critical components missing.")
        print("  Please re-run apply_omnigraph.py")

    print("=" * 60)


def print_graph_info(graph_path: str = None):
    """
    그래프 상세 정보 출력

    Args:
        graph_path: 그래프 경로 (None이면 모든 그래프)
    """
    try:
        import omni.graph.core as og
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            print("[ERROR] No stage loaded")
            return

        if graph_path:
            graphs = [graph_path]
        else:
            graphs = [
                "/World/IK_Controller_Graph",
                "/World/DifferentialDrive_Graph",
                "/World/Teleoperation_Graph",
            ]

        for gp in graphs:
            graph = og.get_graph_by_path(gp)
            if graph is None:
                print(f"\n[NOT FOUND] {gp}")
                continue

            print(f"\n{'=' * 60}")
            print(f"Graph: {gp}")
            print(f"{'=' * 60}")
            print(f"  Valid: {graph.is_valid()}")
            print(f"  Evaluator: {graph.get_evaluator_name()}")

            nodes = graph.get_nodes()
            print(f"\n  Nodes ({len(nodes)}):")

            for node in nodes:
                path = node.get_prim_path()
                name = path.split("/")[-1]
                node_type = node.get_type_name()

                print(f"\n    [{name}]")
                print(f"      Type: {node_type}")
                print(f"      Valid: {node.is_valid()}")

                # 연결 정보
                attrs = node.get_attributes()
                inputs = [a for a in attrs if a.get_port_type().name == "ATTRIBUTE_PORT_TYPE_INPUT"]
                outputs = [a for a in attrs if a.get_port_type().name == "ATTRIBUTE_PORT_TYPE_OUTPUT"]

                print(f"      Inputs: {len(inputs)}, Outputs: {len(outputs)}")

    except Exception as e:
        print(f"[ERROR] {e}")


def check_ros2_topics():
    """
    ROS2 토픽 정보 출력 (참조용)

    실제 토픽 확인은 터미널에서:
        ros2 topic list
    """
    expected_topics = {
        "Publishers": [
            "/joint_states",
            "/odom",
            "/tf",
            "/clock",
        ],
        "Subscribers": [
            "/cmd_vel",
            "/joint_command",
            "/q2r_right_hand_pose",
            "/q2r_left_hand_twist",
        ],
    }

    print("\n" + "=" * 60)
    print("Expected ROS2 Topics")
    print("=" * 60)

    for category, topics in expected_topics.items():
        print(f"\n  {category}:")
        for topic in topics:
            print(f"    - {topic}")

    print("\n  Verify with:")
    print("    ros2 topic list")
    print("    ros2 topic echo <topic_name>")
    print("=" * 60)


def run_full_verification():
    """전체 검증 실행"""
    print("\n" + "#" * 70)
    print("#  OmniGraph Full Verification")
    print("#" * 70)

    # 1. 설정 검증
    results = verify_omnigraph_setup(verbose=True)

    # 2. 상세 정보 출력
    print_graph_info()

    # 3. ROS2 토픽 정보
    check_ros2_topics()

    return results


# =============================================================================
# 단독 실행 시
# =============================================================================

if __name__ == "__main__":
    run_full_verification()
