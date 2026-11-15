"""STL19P / LD19 lidar -> nearest obstacle in each direction.

The lidar TX pin is LIDAR_RX in sketch/pins.h. sketch.ino decodes the packets.
"""

from dataclasses import dataclass

import hardware

INF = float("inf")


@dataclass
class Obstacles:
    front: float   # meters to nearest object (inf = nothing seen)
    back: float
    left: float
    right: float


class Lidar:
    @property
    def stats(self):
        return hardware.lidar_stats

    def read(self):
        """Return the nearest distance (m) in front / back / left / right."""
        s = hardware.lidar_stats
        return Obstacles(s["front"], s["back"], s["left"], s["right"])
