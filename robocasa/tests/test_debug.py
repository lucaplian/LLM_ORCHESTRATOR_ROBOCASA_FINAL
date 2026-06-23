import numpy as np

# Load the arrays from the file
loaded_arrays = np.load('/Volumes/Extreme SSD/robocasa/robocasa/models/assets/demonstrations_private/2026-03-13-10-33-27_Kitchen/episodes/ep_1773390820_500969/state_1773390824_813743.npz', allow_pickle=True)
print(loaded_arrays)
print(f"States shape: {loaded_arrays['states'].shape}")
print(f"Success status: {loaded_arrays['successful']}")
print(f"Environment status: {loaded_arrays['env']}")
print(f"Action_infos status: {loaded_arrays['action_infos']}")

print(f"States status: {loaded_arrays['states']}")


