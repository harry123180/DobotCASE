from GripperHighLevel import GripperHighLevelAPI, GripperType
import time

gripper = GripperHighLevelAPI(GripperType.PGC)
gripper.connected = True  # 強制設置
gripper.initialized = True

print("測試 quick_close...")
result1 = gripper.quick_close()
print(f"結果: {result1}")

time.sleep(2)

print("測試 smart_release(470)...")
result2 = gripper.smart_release(470)
print(f"結果: {result2}")