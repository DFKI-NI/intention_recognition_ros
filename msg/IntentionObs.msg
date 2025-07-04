# 1. Type of assembly (e.g. hotel type)
int8 object_type # 1 = Hotel Type A, 2 = Hotel Type B
float32 object_type_accuracy

# 2. Worker activity (e.g. observe hotel progress)
# Detections come from a camera observing the worker
# Color id:
# 0. bright green (left wall)
# 1. dark green (back wall type 1)
# 2. magenta (left roof)
# 3. purple (floor)
# 4. red (right wall type 1)
# 5. yellow (right roof)
# 6. black (back wall type 2)
# 7. orange (right wall type 2

bool[] worker_parts # e.g. [true, false, true, true, false, false, true, false]
float32 worker_accuracy

# 3. Observe local storage, is within reach of the worker
# Detections come from a camera observing the local storage (parts cam)
bool[] storage_parts # [true, false, true, true, false, false, false, false]
float32 storage_accuracy
