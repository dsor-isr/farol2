import re
import time

def get_next_available_vehicle_id(vehicle_name):
  # Query current ROS graph and choose the first free numeric suffix for /<vehicle_name><id>
  try:
    import rclpy
  except Exception:
    return '0'

  node = None
  try:
    rclpy.init(args=[])
    node = rclpy.create_node('_vehicle_id_allocator')

    # Allow discovery to populate graph info.
    time.sleep(0.25)

    # Match any namespace of the form /<name><digits> — IDs are global across all vehicle names,
    # so /magicelectric0 and /mred0 share the same pool and the next vehicle gets id 1.
    pattern = re.compile(r'^/\D+(\d+)(?:/|$)')
    used_ids = set()
    for _, namespace in node.get_node_names_and_namespaces():
      match = pattern.match(namespace)
      if match:
        used_ids.add(int(match.group(1)))

    next_id = 0
    while next_id in used_ids:
      next_id += 1

    return str(next_id)
  except Exception:
    return '0'
  finally:
    if node is not None:
      node.destroy_node()
    try:
      rclpy.shutdown()
    except Exception:
      pass
