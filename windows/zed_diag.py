import sys
import pyzed.sl as sl

cam = sl.Camera()
init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.HD1080
init.camera_fps = 60
init.coordinate_units = sl.UNIT.METER
init.coordinate_system = sl.COORDINATE_SYSTEM.LEFT_HANDED_Y_UP
init.depth_mode = sl.DEPTH_MODE.NEURAL
if cam.open(init) != sl.ERROR_CODE.SUCCESS:
    sys.exit("open failed")

bp = sl.BodyTrackingParameters()
bp.enable_tracking = True
bp.enable_body_fitting = True
bp.body_format = sl.BODY_FORMAT.BODY_38
bp.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE
if cam.enable_body_tracking(bp) != sl.ERROR_CODE.SUCCESS:
    sys.exit("enable body tracking failed")

rt = sl.BodyTrackingRuntimeParameters()
rt.detection_confidence_threshold = 40
bodies = sl.Bodies()
print(">>> STAND IN FULL VIEW (2-3m back). Grabbing 300 frames (~10s)...", flush=True)
detected = 0
for i in range(300):
    if cam.grab() != sl.ERROR_CODE.SUCCESS:
        continue
    cam.retrieve_bodies(bodies, rt)
    n = len(bodies.body_list)
    if n > 0:
        detected += 1
        b = max(bodies.body_list, key=lambda x: x.confidence)
        if detected % 30 == 1:
            kp0 = b.keypoint[0]
            print(f"  frame {i}: DETECTED conf={b.confidence:.0f} pelvis={kp0}", flush=True)
    elif i % 60 == 0:
        print(f"  frame {i}: no body", flush=True)
print(f"\n>>> RESULT: {detected}/300 frames had a body", flush=True)
cam.disable_body_tracking(); cam.close()
