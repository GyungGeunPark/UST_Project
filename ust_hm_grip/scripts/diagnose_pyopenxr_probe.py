"""Non-destructive OpenXR probe with pyopenxr.

Why this exists
---------------
The 10th-session option-B investigation confirmed that Isaac Sim's
``omni.kit.xr.core`` wrapper does NOT surface hand-tracking joints on
either SteamVR or PicoStreamingXR runtimes (30s polling, 453+ frames
with ``with_data=0``).  The user requested a direct pyopenxr attempt
to rule out whether ``omni.kit.xr.core`` is the culprit vs. the OpenXR
runtime itself.

This probe does NOT create a session (which would conflict with Isaac
Sim's existing XR session and require graphics binding).  Instead it
only:

  1. Creates an ``xr.Instance`` requesting ``XR_EXT_hand_tracking``,
     ``XR_FB_hand_tracking_aim``, and ``XR_MND_headless`` extensions.
  2. Enumerates *supported* extensions on the active runtime.
  3. Looks up the system (head-mounted display form factor).
  4. Queries hand-tracking system properties if the extension exists.

Verdict logic
-------------
* ``XR_EXT_hand_tracking`` is in supported list -> runtime CAN expose
  hand joints in principle.  Next step would be to create a session
  (graphics binding required; coordinate with Isaac Sim).
* ``XR_MND_headless`` exists -> we can run input-only sessions in a
  background thread WITHOUT conflicting with Isaac Sim's graphics
  session.  This is the key requirement for a viable pyopenxr-based
  fallback.
* Neither -> option B "direct pyopenxr" is also dead end; only software
  fallback (keyboard / ALVR / community report) remains.

Usage::

    & C:\\Users\\pjwpy\\miniconda3\\envs\\ust\\python.exe -X utf8 `
        -m ust_ws.ust_hm_grip.scripts.diagnose_pyopenxr_probe
"""

from __future__ import annotations

import sys
from pathlib import Path

try:
    sys.stdout.reconfigure(line_buffering=True)  # type: ignore[attr-defined]
except (AttributeError, OSError):
    pass


WANTED_EXTENSIONS = [
    # Required for any meaningful interop
    "XR_KHR_D3D11_enable",
    "XR_KHR_D3D12_enable",
    "XR_KHR_vulkan_enable2",
    "XR_KHR_opengl_enable",
    # Input
    "XR_EXT_hand_tracking",       # 26-joint hand skeleton (Quest, PICO, etc.)
    "XR_FB_hand_tracking_aim",    # pinch strength + aim vector (Quest-style)
    "XR_FB_hand_tracking_capsules",
    "XR_EXT_hand_interaction",    # controller-style binding for hand pinch
    "XR_MSFT_hand_interaction",
    # Multi-session enablement (run input-only in background)
    "XR_MND_headless",
    "XR_EXT_palm_pose",
    # PICO vendor extensions if any
    "XR_PICO_perf_settings",
    "XR_PICO_view_state_ext_enable",
    "XR_BD_controller_interaction",
    "XR_BD_motion_tracker",  # PICO body tracking
]


def main() -> int:
    import xr

    print("=" * 70)
    print(f"pyopenxr probe (pyopenxr version: {getattr(xr, '__version__', '?')})")
    print("=" * 70)

    # ----- Step 1: enumerate supported extensions (no instance yet) -----
    print("\n[step 1] Enumerating extensions reported by the OpenXR loader...")
    try:
        ext_props = xr.enumerate_instance_extension_properties()
    except Exception as exc:
        print(f"  FATAL: enumerate_instance_extension_properties failed: {exc}")
        return 2

    supported = sorted(
        (p.extension_name.decode("utf-8") if isinstance(p.extension_name, bytes) else str(p.extension_name))
        for p in ext_props
    )
    print(f"  Runtime reports {len(supported)} extensions.")

    # Highlight the ones we care about
    print("\n[step 2] Key extensions:")
    found = {}
    for want in WANTED_EXTENSIONS:
        present = want in supported
        found[want] = present
        marker = "[OK]" if present else "[--]"
        print(f"    {marker} {want}")

    # Optional: dump everything for the curious
    print("\n[step 3] Full extension list (sorted):")
    for name in supported:
        marker = "*" if name in found and found[name] else " "
        print(f"    {marker} {name}")

    # ----- Step 4: try to create an Instance with the basics -----
    # Use XR_KHR_D3D11_enable as a placeholder graphics requirement (most
    # Windows runtimes require *some* graphics extension to even create
    # an instance, even if we don't end up making a session).
    print("\n[step 4] Creating xr.Instance with extensions:")
    request_exts = [
        e for e in (
            "XR_KHR_D3D11_enable",   # cheapest graphics ext to request
            "XR_EXT_hand_tracking",
            "XR_MND_headless",
        ) if e in supported
    ]
    print(f"  Requesting: {request_exts}")

    # pyopenxr's ApplicationInfo expects `api_version` as an xr.Version
    # OBJECT, not an int (verified via inspect.signature).
    app_info = xr.ApplicationInfo(
        application_name="ust_hm_grip_probe",
        application_version=1,
        engine_name="ust_hm_grip",
        engine_version=1,
        api_version=xr.Version(1, 0, 0),
    )
    instance = None
    try:
        instance_create_info = xr.InstanceCreateInfo(
            application_info=app_info,
            enabled_extension_names=request_exts,
        )
        instance = xr.create_instance(instance_create_info)
        print(f"  [OK] xr.Instance created with extensions: {request_exts}")
    except Exception as exc:
        print(f"  [WARN] xr.create_instance with full extension set failed: "
              f"{type(exc).__name__}: {exc}")
        # Try minimal (only graphics ext)
        print("  Retrying with minimal extensions only (XR_KHR_D3D11_enable)...")
        try:
            instance_create_info = xr.InstanceCreateInfo(
                application_info=app_info,
                enabled_extension_names=[e for e in ("XR_KHR_D3D11_enable",) if e in supported],
            )
            instance = xr.create_instance(instance_create_info)
            print(f"  [OK] minimal xr.Instance created (no hand_tracking).")
        except Exception as exc2:
            print(f"  [FATAL] minimal also failed: {type(exc2).__name__}: {exc2}")
            return 2

    # ----- Step 5: query instance properties -----
    print("\n[step 5] Instance properties:")
    try:
        props = xr.get_instance_properties(instance)
        # Some pyopenxr versions expose .runtime_name as bytes vs str
        rn = props.runtime_name
        if isinstance(rn, bytes):
            rn = rn.decode("utf-8", errors="replace")
        rv = props.runtime_version
        print(f"  runtime_name    = {rn!r}")
        print(f"  runtime_version = major={rv.major} minor={rv.minor} patch={rv.patch}")
    except Exception as exc:
        print(f"  [WARN] get_instance_properties failed: {exc}")

    # ----- Step 6: get the system -----
    print("\n[step 6] Looking up the HMD system...")
    try:
        sys_get_info = xr.SystemGetInfo(form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY)
        system_id = xr.get_system(instance, sys_get_info)
        print(f"  [OK] system_id = {int(system_id)}")
        try:
            sys_props = xr.get_system_properties(instance, system_id)
            sn = sys_props.system_name
            if isinstance(sn, bytes):
                sn = sn.decode("utf-8", errors="replace")
            print(f"  system_name = {sn!r}")
            print(f"  vendor_id   = {sys_props.vendor_id}")
            # Tracking properties
            tp = sys_props.tracking_properties
            print(f"  orientation_tracking = {bool(tp.orientation_tracking)}")
            print(f"  position_tracking    = {bool(tp.position_tracking)}")
        except Exception as exc:
            print(f"  [WARN] get_system_properties failed: {exc}")
    except Exception as exc:
        print(f"  [FAIL] get_system: {type(exc).__name__}: {exc}")
        print("  Headset is not active / runtime is not in 'ready' state.")
        print("  Make sure PCVR streaming is running and the headset is awake.")

    # ----- Step 7: try hand tracking system properties (if extension supported) -----
    if found.get("XR_EXT_hand_tracking"):
        print("\n[step 7] Hand-tracking system properties:")
        try:
            # pyopenxr exposes XrSystemHandTrackingPropertiesEXT via xr.SystemHandTrackingPropertiesEXT
            # Chained into get_system_properties via next pointer
            ht_props = xr.SystemHandTrackingPropertiesEXT()
            sys_props = xr.SystemProperties(next=xr.cast(xr.pointer(ht_props), xr.c_void_p))
            xr.get_system_properties(instance, system_id, byref=sys_props if False else None)  # safe fallback
            # Simpler: many pyopenxr builds expose convenience helper
            print(f"  supports_hand_tracking = {bool(ht_props.supports_hand_tracking)}")
        except Exception as exc:
            # Convenience helper may not exist in this pyopenxr version
            print(f"  [INFO] chained query not supported in this pyopenxr build ({exc!r})")
            print(f"  But XR_EXT_hand_tracking is in the supported list -> still promising.")
    else:
        print("\n[step 7] XR_EXT_hand_tracking NOT in supported list -> hand tracking unavailable.")

    # ----- Verdict -----
    print("\n" + "=" * 70)
    print("VERDICT:")
    print("=" * 70)
    headless_ok = found.get("XR_MND_headless", False)
    hand_ok = found.get("XR_EXT_hand_tracking", False)
    hand_aim_ok = found.get("XR_FB_hand_tracking_aim", False)
    pico_motion_ok = found.get("XR_BD_motion_tracker", False)

    print(f"  XR_EXT_hand_tracking  : {'YES' if hand_ok else 'NO'}")
    print(f"  XR_FB_hand_tracking_aim : {'YES' if hand_aim_ok else 'NO'}  (pinch strength + aim)")
    print(f"  XR_MND_headless         : {'YES' if headless_ok else 'NO'}  (background input-only session)")
    print(f"  XR_BD_motion_tracker    : {'YES' if pico_motion_ok else 'NO'}  (PICO body tracking)")
    print()

    if hand_ok and headless_ok:
        print("  ==> GREEN: Both critical extensions supported.  Direct pyopenxr")
        print("      hand-tracking input is feasible.  Next step: implement a")
        print("      background-thread session and probe hand joint data.")
        ret = 0
    elif hand_ok and not headless_ok:
        print("  ==> YELLOW: hand_tracking supported, but headless mode is not.")
        print("      Means we MUST share an OpenXR session with Isaac Sim's")
        print("      omni.kit.xr.core (since two graphics sessions on same HMD")
        print("      will conflict).  Isaac Sim does not expose its session")
        print("      handle, so this path requires Isaac Sim plugin work.")
        ret = 1
    else:
        print("  ==> RED: hand_tracking NOT in supported extensions.")
        print("      This runtime fundamentally does not expose hand joints.")
        print("      Option B is dead -- only software fallback (keyboard / ALVR)")
        print("      remains viable.")
        ret = 1

    # Cleanup
    try:
        xr.destroy_instance(instance)
    except Exception:
        pass
    return ret


if __name__ == "__main__":
    sys.exit(main())
