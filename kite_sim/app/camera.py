"""Fixed rider camera. Coordinates are source-frame x downwind, y right, z down."""
import math
import numpy as np


class Camera:
    def __init__(self, size, pitch_deg=40, focal_length=285):
        self.width, self.height = size
        self.focal = focal_length * self.height / 820
        pitch = math.radians(pitch_deg)
        self.forward = np.array([math.cos(pitch), 0, -math.sin(pitch)])
        self.up = np.array([math.sin(pitch), 0, math.cos(pitch)])
        self.center = np.array([self.width/2, self.height/2])
        self.horizon = self.height/2 + self.focal*math.tan(pitch)

    def project(self, point):
        depth = float(np.dot(point, self.forward))
        if depth <= 0.05:
            return None
        return (float(self.center[0]+self.focal*point[1]/depth),
                float(self.center[1]+self.focal*np.dot(point, self.up)/depth))

    def direction(self, azimuth_deg, elevation_deg):
        a, e = math.radians(azimuth_deg), math.radians(elevation_deg)
        return self.project(np.array([math.cos(e)*math.cos(a),
                                      math.cos(e)*math.sin(a), -math.sin(e)]))

    def project_object(self, center, points):
        """Angular-scale inset at the object's world-projected location.

        The fixed wide camera distorts objects off axis. Use a local camera
        looking along the actual tether to display the canopy with uniform
        pixels per visual angle. This is a perceptual wide-view correction,
        not a single pinhole photograph. Distance and foreshortening are retained.
        """
        anchor = self.project(center)
        if anchor is None:
            return [None for _ in points]
        radial = center / np.linalg.norm(center)
        right = np.cross(self.up, radial)
        norm = np.linalg.norm(right)
        if norm < 1e-8:
            return [None for _ in points]
        right /= norm
        down = np.cross(radial, right)
        # Fixed viewing magnification, independent of airflow or screen position.
        scale = 450 * self.height / 820
        projected = []
        for point in points:
            depth = float(np.dot(point, radial))
            offset = point-center
            projected.append(None if depth <= 0.05 else (
                anchor[0]+scale*float(np.dot(offset, right))/depth,
                anchor[1]+scale*float(np.dot(offset, down))/depth))
        return projected
