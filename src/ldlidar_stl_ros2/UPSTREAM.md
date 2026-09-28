# Vendored LDROBOT ROS 2 driver

This package is vendored from the official LDROBOT repository so a fresh drone
workspace clone contains the LD19 driver without a nested Git repository.

- Upstream: https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2
- Upstream commit: `bf668a89baf722a787dadc442860dcbf33a82f5a`
- Imported: 2026-09-28

Local changes are intentionally small: initialized LD19 defaults, rejection of
malformed scans with fewer than two points, and the missing POSIX mutex include
needed by current compilers. Real-drone parameters and the vertical mounting
transform live in `master_scripts`, outside this vendor tree.

Upstream lint targets are disabled because this pinned SDK predates the current
ament style rules; the package is still compiled in every integration build.
