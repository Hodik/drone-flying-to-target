from dronekit import connect, VehicleMode, Vehicle, LocationGlobal, Command
import time
import math
from pymavlink import mavutil


def arm_and_takeoff(vehicle: Vehicle, target_alt_meters: int):
    """
    Arms vehicle and fly to target_alt_meters.
    """

    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_alt_meters)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if (
            vehicle.location.global_relative_frame.alt
            >= float(target_alt_meters) * 0.99
        ):
            print("Reached target altitude")
            break
        time.sleep(1)


def switch_mode(vehicle: Vehicle, mode: str):
    print("Switching to", mode, "mode")
    vehicle.mode = VehicleMode(mode)
    while vehicle.mode.name != mode:
        print("Waiting for mode to change to", mode)
        time.sleep(0.1)


def get_distance_metres(location_1, location_2, /):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = location_2.lat - location_1.lat
    dlong = location_2.lon - location_1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def get_accurate_bearing(lat1, lon1, lat2, lon2):
    """
    Calculate bearing between two points using spherical Earth model (Haversine formula)
    """
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    y = math.sin(lon2 - lon1) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(
        lon2 - lon1
    )
    bearing = math.atan2(y, x)

    bearing = math.degrees(bearing)

    bearing = (bearing + 360) % 360

    return bearing


def goto(
    vehicle: Vehicle,
    location: LocationGlobal,
    airspeed: int = 15,
    groundspeed: int = 15,
):

    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    if not v.home_location:
        v.commands.download()
        v.commands.wait_ready()

    v._master.mav.mission_item_send(
        0,
        0,
        0,
        frame,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2,
        0,
        0,
        0,
        0,
        0,
        location.lat,
        location.lon,
        location.alt,
    )

    if airspeed is not None:
        v.airspeed = airspeed
    if groundspeed is not None:
        v.groundspeed = groundspeed

    targetDistance = get_distance_metres(vehicle.location.global_frame, location)

    while vehicle.mode.name == "GUIDED":
        remainingDistance = get_distance_metres(
            vehicle.location.global_relative_frame, location
        )
        print("Distance to target: ", remainingDistance)
        if remainingDistance <= targetDistance * 0.01 or remainingDistance < 1:
            print("Reached target")
            break
        time.sleep(2)


def goto_with_channel_overrides(
    vehicle: Vehicle, location: LocationGlobal, accuracy: int = 30
):
    """
    Navigate to target location using ONLY channel overrides in ALT_HOLD mode.
    """
    vehicle.channels.overrides = {"3": 1500}
    switch_mode(vehicle, "ALT_HOLD")

    print("ALT_HOLD engaged. Stabilizing...")
    time.sleep(5)
    vehicle.channels.overrides = {"2": 1100, "3": 1500}

    while True:
        current_location = vehicle.location.global_relative_frame

        distance = get_distance_metres(current_location, location)
        print(
            f"Distance to target: {distance:.1f}m, Altitude: {current_location.alt:.1f}m"
        )

        if distance <= accuracy * 3:
            vehicle.channels.overrides = {"2": 1200, "3": 1500}
        if distance <= accuracy:
            print(f"Target reached! Final distance: {distance:.1f}m")
            break

        time.sleep(1)

    print("Stopping movement")
    vehicle.channels.overrides = {"2": None, "3": 1500}
    print("Navigation complete!")


def yaw(v: Vehicle, heading: float, relative=False, speed: int = 10):
    """
    Yaw the vehicle to a specified heading.
    """
    print(f"Yawing to {heading}°")
    if relative:
        is_relative = 1
    else:
        is_relative = 0

    msg = v.message_factory.command_long_encode(
        0,
        0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,
        speed,
        1,
        is_relative,
        0,
        0,
        0,
    )
    v.send_mavlink(msg)


def add_mission_waypoint(vehicle, lat, lon, alt):
    """Add a waypoint to the current mission that will be visible in Mission Planner"""
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    cmd = Command(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,
        0,
        0,
        0,
        0,
        0,
        lat,
        lon,
        alt,
    )

    cmds.add(cmd)

    cmds.upload()
    print(f"Added waypoint at {lat}, {lon}, {alt}")


if __name__ == "__main__":

    connection_string = "tcp:127.0.0.1:5762"

    print("Connecting to vehicle on: %s" % (connection_string,))
    v = connect(connection_string, wait_ready=True)
    arm_and_takeoff(v, 100)

    b_location = LocationGlobal(50.443326, 30.448078, 100)
    add_mission_waypoint(v, b_location.lat, b_location.lon, b_location.alt)
    current_location = v.location.global_relative_frame
    v.simple_goto(v.location.global_frame)
    target_bearing = get_accurate_bearing(
        current_location.lat, current_location.lon, b_location.lat, b_location.lon
    )
    print(f"Current position: {current_location.lat:.6f}, {current_location.lon:.6f}")
    print(f"Target position: {b_location.lat:.6f}, {b_location.lon:.6f}")
    print(f"Calculated bearing to target: {target_bearing:.1f}°")

    print(f"Rotating drone to face bearing {target_bearing:.1f}°")

    yaw(v, target_bearing, speed=10)
    time.sleep(5)

    goto_with_channel_overrides(v, b_location, accuracy=30)

    switch_mode(v, "GUIDED")
    yaw(v, 350)

    v.close()

    print("Completed")
