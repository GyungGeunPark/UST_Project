"""
debug_node_types.py
Isaac Sim에서 사용 가능한 OmniGraph 노드 타입 및 속성 확인

Isaac Sim Script Editor에서 실행:
    exec(open("/home/isaac/ust_ws/ust_project1/scripts/setup/debug_node_types.py").read())
"""


def get_all_node_types():
    """OmniGraph에서 사용 가능한 모든 노드 타입 가져오기"""
    try:
        import omni.graph.core as og
        # 여러 API 시도
        if hasattr(og, 'get_all_node_types'):
            return og.get_all_node_types()
        elif hasattr(og, 'get_registered_nodes'):
            return og.get_registered_nodes()
        else:
            # NodeTypeRegistry 사용
            registry = og.get_node_type_registry()
            if registry:
                return list(registry.get_all_node_type_names())
    except Exception as e:
        print(f"Error getting node types: {e}")

    # 대안: OmniGraph Controller를 통해 노드 타입 확인
    try:
        import omni.graph.core as og
        # 노드 타입 레지스트리에서 가져오기
        return [str(t) for t in og.get_node_types_with_interface()]
    except Exception:
        pass

    return []


def get_node_attributes(node_type: str):
    """특정 노드 타입의 입력/출력 속성 확인"""
    try:
        import omni.graph.core as og

        # 임시 그래프 생성
        graph_path = "/World/__temp_debug_graph__"

        # 기존 그래프 삭제
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        if stage and stage.GetPrimAtPath(graph_path):
            stage.RemovePrim(graph_path)

        # 노드 생성 시도
        keys = og.Controller.Keys
        graph, (node,), _, _ = og.Controller.edit(
            {"graph_path": graph_path, "evaluator_name": "execution"},
            {keys.CREATE_NODES: [("TestNode", node_type)]}
        )

        # 속성 가져오기
        inputs = []
        outputs = []

        for attr in node.get_attributes():
            attr_name = attr.get_name()
            attr_type = str(attr.get_resolved_type())
            if attr_name.startswith("inputs:"):
                inputs.append((attr_name, attr_type))
            elif attr_name.startswith("outputs:"):
                outputs.append((attr_name, attr_type))

        # 임시 그래프 삭제
        stage.RemovePrim(graph_path)

        return inputs, outputs

    except Exception as e:
        print(f"Error getting attributes for {node_type}: {e}")
        return [], []


def print_node_attributes(node_type: str):
    """노드 속성 출력"""
    print(f"\n{'='*60}")
    print(f" Node: {node_type}")
    print(f"{'='*60}")

    inputs, outputs = get_node_attributes(node_type)

    if inputs:
        print(f"\n  INPUTS ({len(inputs)}):")
        for name, typ in sorted(inputs):
            print(f"    {name}: {typ}")
    else:
        print(f"\n  INPUTS: (none or error)")

    if outputs:
        print(f"\n  OUTPUTS ({len(outputs)}):")
        for name, typ in sorted(outputs):
            print(f"    {name}: {typ}")
    else:
        print(f"\n  OUTPUTS: (none or error)")


def list_node_types(filter_pattern: str = None):
    """사용 가능한 노드 타입 목록 출력"""
    all_types = get_all_node_types()

    if filter_pattern:
        filtered = [t for t in all_types if filter_pattern.lower() in t.lower()]
        print(f"\n=== Node types containing '{filter_pattern}' ({len(filtered)} found) ===")
        for t in sorted(filtered):
            print(f"  {t}")
    else:
        print(f"\n=== All node types ({len(all_types)} total) ===")
        for t in sorted(all_types)[:100]:
            print(f"  {t}")
        if len(all_types) > 100:
            print(f"  ... and {len(all_types) - 100} more")

    return filtered if filter_pattern else all_types


def check_ros2_nodes():
    """ROS2 관련 노드 확인"""
    print("\n" + "=" * 60)
    print(" ROS2 Bridge Nodes")
    print("=" * 60)

    # isaacsim.ros2.bridge 노드 확인
    new_ros2 = list_node_types("isaacsim.ros2")

    # omni.isaac.ros2_bridge 노드 확인
    old_ros2 = list_node_types("omni.isaac.ros2")

    return new_ros2, old_ros2


def check_core_nodes():
    """Core 노드 확인"""
    print("\n" + "=" * 60)
    print(" Core Nodes")
    print("=" * 60)

    # isaacsim.core.nodes 확인
    new_core = list_node_types("isaacsim.core")

    # omni.isaac.core_nodes 확인
    old_core = list_node_types("omni.isaac.core_nodes")

    return new_core, old_core


def check_wheeled_robots():
    """Wheeled Robot 노드 확인"""
    print("\n" + "=" * 60)
    print(" Wheeled Robot Nodes")
    print("=" * 60)

    # isaacsim.wheeled_robots 확인
    new_wheeled = list_node_types("isaacsim.wheeled")

    # omni.isaac.wheeled_robots 확인
    old_wheeled = list_node_types("omni.isaac.wheeled")

    # DifferentialController 관련 확인
    diff = list_node_types("Differential")

    return new_wheeled, old_wheeled, diff


def check_graph_nodes():
    """일반 그래프 노드 확인"""
    print("\n" + "=" * 60)
    print(" Graph Nodes")
    print("=" * 60)

    # omni.graph.nodes 확인
    graph_nodes = list_node_types("omni.graph.nodes")

    # ReadPrim 관련 확인
    read_prim = list_node_types("ReadPrim")

    # Script 노드 확인
    script = list_node_types("Script")

    return graph_nodes, read_prim, script


def check_subscribe_nodes():
    """Subscribe 노드 확인"""
    print("\n" + "=" * 60)
    print(" Subscribe Nodes")
    print("=" * 60)

    # Subscribe 관련 확인
    subscribe = list_node_types("Subscribe")

    # Pose 관련 확인
    pose = list_node_types("Pose")

    return subscribe, pose


def check_critical_node_attributes():
    """핵심 노드들의 속성 확인 (문제 해결용)"""
    print("\n" + "=" * 60)
    print(" Critical Node Attributes Check")
    print("=" * 60)

    # 문제가 된 노드들
    critical_nodes = [
        "isaacsim.core.nodes.IsaacArticulationController",
        "isaacsim.core.nodes.IsaacReadSimulationTime",
        "isaacsim.ros2.bridge.ROS2Context",
        "isaacsim.ros2.bridge.ROS2SubscribeTwist",
        "isaacsim.robot.wheeled_robots.DifferentialController",
    ]

    # PoseStamped 관련 노드 찾기
    all_types = get_all_node_types()
    pose_nodes = [t for t in all_types if 'pose' in t.lower() and 'subscribe' in t.lower()]
    if pose_nodes:
        print(f"\n  Found Pose Subscribe nodes: {pose_nodes}")
        critical_nodes.extend(pose_nodes[:2])  # 처음 2개만

    for node_type in critical_nodes:
        if node_type in all_types:
            print_node_attributes(node_type)
        else:
            print(f"\n  [SKIP] {node_type} - not found")


def main():
    """메인 함수"""
    print("\n" + "=" * 60)
    print(" Isaac Sim OmniGraph Node Types Debug")
    print("=" * 60)

    check_ros2_nodes()
    check_core_nodes()
    check_wheeled_robots()
    check_graph_nodes()
    check_subscribe_nodes()

    # 핵심 노드 속성 확인
    check_critical_node_attributes()

    # 특정 노드 타입 확인
    print("\n" + "=" * 60)
    print(" Specific Node Type Checks")
    print("=" * 60)

    specific_checks = [
        "ROS2Context",
        "ROS2SubscribeTwist",
        "ROS2SubscribePose",
        "ROS2PublishJoint",
        "ArticulationController",
        "OnPlaybackTick",
    ]

    for check in specific_checks:
        results = list_node_types(check)
        if not results:
            print(f"  WARNING: No nodes found for '{check}'")


if __name__ == "__main__":
    main()
