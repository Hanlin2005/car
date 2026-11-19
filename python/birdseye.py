"""Top-down (bird's-eye) view of the floor in front of the car, for the web page.

The floor is flat, so every image pixel below the horizon maps to one spot on
it. That mapping needs the camera's focal length, tilt, and height:

  focal length  from CAMERA_HFOV_DEG
  tilt          from the horizon row the lane detector learns (where the
                lane lines meet), or LANE_HORIZON_FRAC before it has one
  height        from the lane width: the tape LANE_WIDTH_M apart appears
                lane_width_px apart on the bottom row, which only fits one
                camera height

So the view needs no extra measurements, but its scale is only as right as
LANE_WIDTH_M. Straight tape should come out as two parallel lines; if they
splay apart or pinch together far away, the horizon (tilt) is off.
"""

import math

import config


def camera_pose(width, height, horizon_row, lane_px):
    """(focal_px, tilt_rad, camera_height_m), or None if the geometry is unusable.

    tilt is how far the optical axis points below horizontal.
    """
    focal = (width / 2) / math.tan(math.radians(config.CAMERA_HFOV_DEG) / 2)
    cy = height / 2
    tilt = math.atan2(cy - horizon_row, focal)
    bottom = height - 1
    # Angle below horizontal of the ray through the bottom row.
    below = tilt + math.atan2(bottom - cy, focal)
    if below <= 0.01 or lane_px <= 0:
        return None
    # The bottom row hits the floor this far ahead, per meter of camera height.
    ahead_per_h = 1 / math.tan(below)
    depth_per_h = math.sin(tilt) + ahead_per_h * math.cos(tilt)
    cam_h = focal * config.LANE_WIDTH_M / (lane_px * depth_per_h)
    return focal, tilt, cam_h


class BirdsEye:
    def __init__(self):
        import cv2
        import numpy as np
        self._cv2, self._np = cv2, np
        self.scale = config.BIRDSEYE_PX_PER_M
        self.half_w = config.BIRDSEYE_HALF_WIDTH_M
        self.ahead = config.BIRDSEYE_RANGE_M
        self.behind = 0.22    # room below the camera to draw the car
        self.size = (int(2 * self.half_w * self.scale),
                     int((self.ahead + self.behind) * self.scale))

    def _to_px(self, x_m, z_m):
        """Floor point (meters right, meters ahead of the camera) -> view pixel."""
        return (int(round(self.size[0] / 2 + x_m * self.scale)),
                int(round((self.ahead - z_m) * self.scale)))

    def render(self, frame, detector):
        """Warp the frame (with its lane overlay) onto the floor. Returns a BGR image."""
        cv2, np = self._cv2, self._np
        h, w = frame.shape[:2]
        horizon = detector.horizon_row
        if horizon is None:
            horizon = config.LANE_HORIZON_FRAC * h
        lane_px = detector.lane_width_px or config.LANE_DEFAULT_WIDTH_FRAC * w
        out = np.zeros((self.size[1], self.size[0], 3), np.uint8)
        pose = camera_pose(w, h, horizon, lane_px)
        if pose is None:
            self._text(out, "Horizon too low to map the floor", (0, 0, 255))
            return out
        focal, tilt, cam_h = pose

        # Floor (X right, Z ahead, 1) -> image pixel, for a camera cam_h above
        # the floor and tilted down by tilt.
        s, c = math.sin(tilt), math.cos(tilt)
        k = np.array([[focal, 0, w / 2], [0, focal, h / 2], [0, 0, 1]])
        floor_to_cam = np.array([[1, 0, 0], [0, -s, cam_h * c], [0, c, cam_h * s]])
        # View pixel -> floor point.
        view_to_floor = np.array([[1 / self.scale, 0, -self.half_w],
                                  [0, -1 / self.scale, self.ahead],
                                  [0, 0, 1]])
        m = k @ floor_to_cam @ view_to_floor
        warped = cv2.warpPerspective(frame, m, self.size,
                                     flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                                     borderMode=cv2.BORDER_CONSTANT)
        # The floor nearer than the bottom image row isn't visible. Behind the
        # camera the math mirrors the image, so blank all of it.
        nearest = cam_h / math.tan(tilt + math.atan2(h / 2 - 1, focal))
        _, near_row = self._to_px(0, nearest)
        warped[max(near_row, 0):] = 0
        out[:] = warped

        self._grid(out)
        self._car(out)
        self._text(out, f"cam {cam_h * 100:.0f} cm up, tilt {math.degrees(tilt):+.0f} deg",
                   (200, 200, 200))
        return out

    def _grid(self, img):
        cv2 = self._cv2
        grey = (70, 70, 70)
        step = 0.1
        x = 0.0
        while x <= self.half_w:
            for sx in ((x, -x) if x else (0.0,)):
                cv2.line(img, self._to_px(sx, -self.behind), self._to_px(sx, self.ahead), grey, 1)
            x += step
        z = 0.0
        while z <= self.ahead + 1e-6:
            y = self._to_px(0, z)[1]
            cv2.line(img, (0, y), (self.size[0], y), grey, 1)
            if z > 0 and abs(z * 10 - round(z * 10)) < 1e-6 and round(z * 10) % 5 == 0:
                cv2.putText(img, f"{z:.1f} m", (3, y - 3), cv2.FONT_HERSHEY_SIMPLEX,
                            0.35, (160, 160, 160), 1, cv2.LINE_AA)
            z += step

    def _car(self, img):
        cv2 = self._cv2
        half = config.WHEEL_BASE_M / 2 + 0.02
        cv2.rectangle(img, self._to_px(-half, -0.02), self._to_px(half, -self.behind + 0.01),
                      (0, 200, 0), 2)
        cv2.circle(img, self._to_px(0, 0), 3, (255, 255, 255), -1)

    def _text(self, img, text, color):
        cv2 = self._cv2
        cv2.rectangle(img, (0, 0), (img.shape[1], 16), (0, 0, 0), -1)
        cv2.putText(img, text, (3, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.36, color, 1,
                    cv2.LINE_AA)
