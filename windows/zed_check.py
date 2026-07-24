"""Minimal ZED verification (NO cv2/GUI) — run on WINDOWS.

Validates exactly what our publisher needs: pyzed import, camera open in
LEFT_HANDED_Y_UP/METER, BODY_38 body tracking with fitting, per-joint
positions + orientations. Also prints SDK 5.x API specifics so the publisher
can be finalized for the installed SDK.

Run on Windows:  python zed_check.py
"""
import sys
import pyzed.sl as sl

print("=== pyzed import OK ===")
print("SDK version:", sl.Camera.get_sdk_version())

# 1) Verify the enums/attributes our publisher uses exist in this SDK
print("\n=== API checks ===")
print("COORDINATE_SYSTEM.LEFT_HANDED_Y_UP:", hasattr(sl.COORDINATE_SYSTEM, "LEFT_HANDED_Y_UP"))
print("BODY_FORMAT members:", [m for m in dir(sl.BODY_FORMAT) if m.isupper()])
print("BODY_TRACKING_MODEL members:", [m for m in dir(sl.BODY_TRACKING_MODEL) if m.isupper()])
# BODY_38 part enumeration (to verify our ZED38->SMPL24 index map)
for parts in ("BODY_38_PARTS", "BODY_34_PARTS"):
    if hasattr(sl, parts):
        members = [m for m in dir(getattr(sl, parts)) if m.isupper()]
        print(f"{parts} ({len(members)}):", members)

# 2) Open camera
cam = sl.Camera()
init = sl.InitParameters()
init.coordinate_units = sl.UNIT.METER
init.coordinate_system = sl.COORDINATE_SYSTEM.LEFT_HANDED_Y_UP
init.depth_mode = sl.DEPTH_MODE.NEURAL
init.camera_resolution = sl.RESOLUTION.HD1080
init.camera_fps = 60
st = cam.open(init)
if st != sl.ERROR_CODE.SUCCESS:
    print("\n[FAIL] camera open:", st, "(camera connected? USB3?)")
    sys.exit(1)
print("\n=== camera opened OK ===")

bp = sl.BodyTrackingParameters()
bp.enable_tracking = True
bp.enable_body_fitting = True
bp.body_format = sl.BODY_FORMAT.BODY_38
bp.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE
st = cam.enable_body_tracking(bp)
if st != sl.ERROR_CODE.SUCCESS:
    print("[FAIL] enable_body_tracking:", st); cam.close(); sys.exit(1)
print("=== body tracking enabled (BODY_38) ===")

bodies = sl.Bodies()
rt = sl.RuntimeParameters()
brt = sl.BodyTrackingRuntimeParameters()
brt.detection_confidence_threshold = 40

print("\nStand in front of the camera... grabbing 120 frames")
seen = 0
for i in range(120):
    if cam.grab(rt) != sl.ERROR_CODE.SUCCESS:
        continue
    cam.retrieve_bodies(bodies, brt)
    if bodies.body_list:
        seen += 1
        if seen == 1 or seen % 30 == 0:
            b = bodies.body_list[0]
            kp = b.keypoint
            print(f"  frame {i}: bodies={len(bodies.body_list)} conf={b.confidence:.0f} keypoints={len(kp)}")
            print(f"    keypoint[0] (pelvis?) = {kp[0]}")
            # which orientation attributes exist?
            for attr in ("local_orientation_per_joint", "global_root_orientation",
                         "keypoint_orientation", "global_orientation_per_joint"):
                v = getattr(b, attr, None)
                if v is not None:
                    try:
                        print(f"    has {attr}: len={len(v)} sample={v[0]}")
                    except Exception:
                        print(f"    has {attr}: {v}")

print(f"\n=== DONE. body-detected frames: {seen}/120 ===")
print("If keypoints printed with finite xyz, the front-end is GOOD.")
cam.disable_body_tracking()
cam.close()
