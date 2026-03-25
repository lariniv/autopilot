import collections
import collections.abc

collections.MutableMapping = collections.abc.MutableMapping

from dronekit import connect, VehicleMode
import time
import math

CONNECTION_STRING = "udpin:0.0.0.0:14550"

def get_distance_metres(lat1, lon1, lat2, lon2):
    dlat = lat2 - lat1
    dlong = lon2 - lon1
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def get_bearing(lat1, lon1, lat2, lon2):
    off_x = lon2 - lon1
    off_y = lat2 - lat1
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def angle_error_deg(target, current):
    return (target - current + 180.0) % 360.0 - 180.0


def get_ne_error_m(lat1, lon1, lat2, lon2):
    r = 6378137.0
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    lat_avg = math.radians((lat1 + lat2) / 2.0)

    north = dlat * r
    east = dlon * r * math.cos(lat_avg)
    return north, east


def rotate_ne_to_body(north, east, heading_deg):
    yaw = math.radians(heading_deg)

    forward = north * math.cos(yaw) + east * math.sin(yaw)
    right = -north * math.sin(yaw) + east * math.cos(yaw)

    return forward, right

def active_flight_and_land(vehicle, target_lat, target_lon, target_alt):
    print("Preparing for takeoff and active navigation...")

    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("STABILIZE")
    while vehicle.mode.name != "STABILIZE":
        time.sleep(0.5)

    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)

    print("Motors armed! Engaging dynamic control loop...")

    # Tune this first: actual PWM that roughly holds hover in STABILIZE
    hover_throttle = 1500

    # XY controller
    kp_xy = 70.0
    kd_xy = 45.0

    # Altitude controller
    kp_alt = 80.0
    kd_alt = 40.0

    # Yaw hold controller
    kp_yaw = 4.0
    landing_heading = vehicle.heading

    # Precision landing logic
    descend_radius_m = 0.6
    center_hold_s = 1.0
    deadband_m = 0.20

    alpha = 0.25
    filt_north = 0.0
    filt_east = 0.0

    centered_since = None
    last_t = time.time()

    while True:
        now = time.time()
        dt = now - last_t
        last_t = now
        dt = max(dt, 0.01)

        loc = vehicle.location.global_relative_frame
        heading = vehicle.heading
        vn, ve, vd = vehicle.velocity  # m/s: north, east, down

        north_err, east_err = get_ne_error_m(
            loc.lat,
            loc.lon,
            target_lat,
            target_lon,
        )

        # Low-pass filter GPS position error
        filt_north = alpha * north_err + (1.0 - alpha) * filt_north
        filt_east = alpha * east_err + (1.0 - alpha) * filt_east

        horiz_dist = math.hypot(filt_north, filt_east)
        horiz_speed = math.hypot(vn, ve)

        # Convert position and velocity to body frame
        fwd_err, right_err = rotate_ne_to_body(filt_north, filt_east, heading)
        fwd_vel, right_vel = rotate_ne_to_body(vn, ve, heading)

        # Deadband near center so it does not chase GPS jitter
        if abs(fwd_err) < deadband_m:
            fwd_err = 0.0
        if abs(right_err) < deadband_m:
            right_err = 0.0

        # XY PD controller
        fwd_effort = kp_xy * fwd_err - kd_xy * fwd_vel
        right_effort = kp_xy * right_err - kd_xy * right_vel

        # Reduce tilt authority close to ground
        max_xy = 180 if loc.alt > 3.0 else 90
        fwd_effort = clamp(fwd_effort, -max_xy, max_xy)
        right_effort = clamp(right_effort, -max_xy, max_xy)

        # RC outputs
        pitch = int(clamp(1500 - fwd_effort, 1300, 1700))
        roll = int(clamp(1500 + right_effort, 1300, 1700))

        # Hold heading fixed instead of always yawing toward target
        yaw_err = angle_error_deg(landing_heading, heading)
        if abs(yaw_err) < 2.0:
            yaw = 1500
        else:
            yaw = int(clamp(1500 + kp_yaw * yaw_err, 1400, 1600))

        # Descend only when centered and slow
        if horiz_dist < descend_radius_m and horiz_speed < 0.20:
            if centered_since is None:
                centered_since = now
        else:
            centered_since = None

        if centered_since is not None and (now - centered_since) > center_hold_s:
            if loc.alt > 3.0:
                target_alt -= 0.30 * dt
            elif loc.alt > 1.0:
                target_alt -= 0.15 * dt
            else:
                target_alt -= 0.08 * dt

        target_alt = max(0.0, target_alt)

        # Altitude PD controller
        alt_err = target_alt - loc.alt
        climb_rate = -vd  # positive = climbing
        throttle_effort = kp_alt * alt_err - kd_alt * climb_rate
        throttle = int(
            clamp(hover_throttle + throttle_effort, 1300, 1700)
        )

        vehicle.channels.overrides = {
            "1": roll,
            "2": pitch,
            "3": throttle,
            "4": yaw,
        }

        print(
            f"D={horiz_dist:.2f}m "
            f"N={filt_north:.2f} "
            f"E={filt_east:.2f} "
            f"Alt={loc.alt:.2f}/{target_alt:.2f} "
            f"V={horiz_speed:.2f}"
        )

        if loc.alt <= 0.5 and target_alt <= 0.2 and horiz_speed < 0.15:
            print("--- TOUCHDOWN! Cutting throttle and disarming. ---")
            current_loc = vehicle.location.global_relative_frame
            print(f"Final coordinates - {current_loc.lat} {current_loc.lon}, with difference of {get_distance_metres(current_loc.lat, current_loc.lon, target_lat, target_lon):.1}m")
            vehicle.channels.overrides = {
                "1": 1500,
                "2": 1500,
                "3": 1000,
                "4": 1500,
            }
            time.sleep(1)
            vehicle.armed = False
            break

        time.sleep(0.05)

def main():
    print(f"Connecting to vehicle on: {CONNECTION_STRING}")
    vehicle = connect(CONNECTION_STRING, wait_ready=True)

    target_lat = 50.443326
    target_lon = 30.448078
    target_alt = 100.0

    active_flight_and_land(vehicle, target_lat, target_lon, target_alt)

    vehicle.close()
    print("Completed test flight.")

if __name__ == "__main__":
    main()
