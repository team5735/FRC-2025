from typing import Tuple
import os, sys
import pprint
import math
import numpy as np
import cv2



def nearest_point_on_circle(ax: float, ay: float,
                            bx: float, by: float,
                            r: float) -> Tuple[float, float]:
    """
    Find the point on a circle centered at (bx, by) with radius r
    that is closest to point (ax, ay).

    Parameters:
        ax, ay : 2D coordinates (in meters) of robot position
        bx, by : 2D coordinates (in meters) of bin center
        r : radius of throwing circle, in meters

    Returns:
        tuple (x, y) — nearest point on the circle
    """
    dx = ax - bx
    dy = ay - by

    d = (dx*dx+dy*dy)**0.5

    # (degenerate case) the robot is at the exact center.
    # Choose a direction: we choose to the right
    if d == 0:
        return (bx + r, by)

    scale = r / d

    point = (bx + dx * scale, by + dy * scale)
    print(point)
    return point



def normalize_angle(theta):
    """Normalize angle to [0, 2π)."""
    return (theta + 2 * math.pi) % (2 * math.pi)

def angle_in_range(theta, start, end):
    """
    Check if theta lies in the CCW range [start, end].
    Handles wrap-around.
    """
    theta = normalize_angle(theta)
    start = normalize_angle(start)
    end = normalize_angle(end)

    if start <= end:
        return start <= theta <= end
    else:
        # wrapped interval
        return theta >= start or theta <= end

def point_on_circle(cx, cy, r, theta):
    """ returns the point on a circle defined by center point (cx, cy) and radius r
        at position theta radians around the circle (starting from 0==right)
    """
    return (cx + r * math.cos(theta), cy + r * math.sin(theta))

def nearest_point_on_circle_arc(ax: float, ay: float,
                                bx:float , by:float,
                                r: float,
                                theta_min: float, theta_max: float) -> Tuple[float, float]:
    """
    ax, ay     : robot location (ax, ay) (nominally in meters)
    bx, by     : circle center (bx, by) (nominally in meters)
    r          : radius of circle (nominally in meters)
    theta_min  : arc start angle (radians)
    theta_max  : arc end angle (radians)

    Returns the point on the arc closest to a.
    """
    dx = ax - bx
    dy = ay - by

    # (degenerate case) if we're exactly at the center of the circle,
    # move us a little to the left to choose a favored direction
    if dx == 0 and dy == 0:
        ax = bx-0.0001
        dx = ax-bx

    # Angle of the unconstrained closest point
    theta_a = math.atan2(dy, dx)

    # If projection lies on the arc, it's the answer
    if angle_in_range(theta_a, theta_min, theta_max):
        scale = r / math.hypot(dx, dy)
        return (bx + dx * scale, by + dy * scale)

    # Otherwise, closest endpoint wins
    p1 = point_on_circle(bx, by, r, theta_min)
    p2 = point_on_circle(bx, by, r, theta_max)

    d1 = math.hypot(p1[0] - ax, p1[1] - ay)
    d2 = math.hypot(p2[0] - ax, p2[1] - ay)

    return p1 if d1 <= d2 else p2

def nearest_point1():
    cx = 0
    cy = 0
    r = 2
    theta_min = math.radians(0)
    theta_max = math.radians(90)

    print(nearest_point_on_circle_arc(0,0, cx, cy, r, theta_min, theta_max))

def nearest_point_visualization():
    WIDTH, HEIGHT = 800, 600
    CENTER = (400, 300)
    RADIUS = 200

    THETA_MIN = math.radians(-45)
    THETA_MAX = math.radians(45)

    click_point = None

    def draw_arc(img, center, radius, t0, t1, color, thickness):
        steps = 200
        angles = np.linspace(t0, t1, steps)
        pts = [point_on_circle(center[0], center[1], radius, t) for t in angles]
        for i in range(len(pts) - 1):
            cv2.line(img, list(map(int,pts[i])), list(map(int,pts[i+1])), color, thickness)

    def mouse_callback(event, x, y, flags, param):
        nonlocal click_point
        if event == cv2.EVENT_LBUTTONDOWN:
            click_point = (x, y)

    cv2.namedWindow("Arc Nearest Point")
    cv2.setMouseCallback("Arc Nearest Point", mouse_callback)

    while True:
        img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)

        # Draw full circle (faint)
        cv2.circle(img, CENTER, RADIUS, (50, 50, 50), 1)

        # Draw arc
        draw_arc(img, CENTER, RADIUS, THETA_MIN, THETA_MAX, (0, 255, 0), 3)

        # Draw center
        cv2.circle(img, CENTER, 4, (255, 255, 255), -1)

        if click_point:
            # Draw clicked point
            cv2.circle(img, click_point, 5, (255, 0, 0), -1)

            # Compute closest point
            closest = nearest_point_on_circle_arc(
                click_point[0], click_point[1], CENTER[0], CENTER[1], RADIUS, THETA_MIN, THETA_MAX
            )

            if closest:
                closest = list(map(int,closest))

                # Draw closest point
                cv2.circle(img, closest, 6, (0, 0, 255), -1)

                # Draw connecting line
                cv2.line(img, click_point, closest, (255, 255, 0), 1)

        cv2.imshow("Arc Nearest Point", img)

        key = cv2.waitKey(16)
        if key == 27:  # ESC
            break

    cv2.destroyAllWindows()

def compute_heading(robot_x, robot_y, target_x, target_y):
    """
    computes the pose angle that the robot needs to be facing
    for it to be looking at the target

    Parameters:
    robot_x, robot_y: robot location (nominally in meters)
    target_x, target_y: the point of the thing we want to be looking at
    """
    dx = target_x - robot_x
    dy = target_y - robot_y
    theta = math.atan2(dy, dx)
    theta = normalize_angle(theta)
    return theta

def heading1():
    print(math.degrees(compute_heading(0,0,1,0)))
    print(math.degrees(compute_heading(0,0,0,1)))
    print(math.degrees(compute_heading(0,0,1,1)))
    print(math.degrees(compute_heading(0,0,-1,-1)))

def main():
    return heading1()
    return nearest_point_visualization()
    return nearest_point1()
    return nearest_point_on_circle(-5,0,0,0,2)
    return nearest_point_on_circle(4,3,1,1,2)


if __name__ == '__main__':
    main()
