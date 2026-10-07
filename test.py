import time
import cv2
# Test different thread counts
for threads in [1, 2, 4]:
    # Initialize your model with `threads`
    start_time = time.time()
    # Run inference for 50 frames
    for _ in range(50):
        # run_inference()
        pass
    elapsed = time.time() - start_time
    print(f"Threads: {threads} -> {50 / elapsed:.2f} FPS")