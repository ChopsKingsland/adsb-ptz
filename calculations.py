"""
ADS-B Tracking Coordinate Transformations

Provides utilities for converting geodetic coordinates (lat, lon, altitude)
of an observer and a target aircraft into camera motor angles (Pan/Azimuth and Tilt/Elevation).
Supports (optional) 45º backward mount pitch (in the north direction) compensation for sky tracking
"""

from typing import Tuple
import math
import numpy as np

# define types
# 0: lat (deg), 1: lon (deg), 2: height (m)
TRIPLE_FLOAT_TUPLE = Tuple[float, float, float]
DOUBLE_FLOAT_TUPLE = Tuple[float, float]

# constants
SEMI_MAJOR_AXIS_METRES = 6_378_137
FIRST_ECCENTRICITY_SQUARED = 0.00669437999014

def _calculate_n_phi(phi: float) -> float:
    """Calculate the prime vertical radius of curvature for a given latitude."""
    return SEMI_MAJOR_AXIS_METRES / (math.sqrt(1 - (FIRST_ECCENTRICITY_SQUARED * (math.sin(phi) ** 2))))

def _geodetic_to_ecef(coordinates: TRIPLE_FLOAT_TUPLE) -> TRIPLE_FLOAT_TUPLE:
    """Convert Geodetic coordinates (lat, lon in rad; alt in meters) to ECEF XYZ (meters)."""
    phi, lam, h = coordinates
    
    n_phi = _calculate_n_phi(phi)
    
    X = (n_phi + h) * math.cos(phi) * math.cos(lam)
    Y = (n_phi + h) * math.cos(phi) * math.sin(lam)
    Z = (n_phi * (1 - FIRST_ECCENTRICITY_SQUARED) + h) * math.sin(phi)
    
    return (X, Y, Z)

def _calculate_vector_difference(receiver: TRIPLE_FLOAT_TUPLE, aircraft: TRIPLE_FLOAT_TUPLE) -> TRIPLE_FLOAT_TUPLE:
    """Calculate the relative delta vector (dX, dY, dZ) in ECEF frame from receiver to aircraft."""
    receiver_ECEF = _geodetic_to_ecef(receiver)
    aircraft_ECEF = _geodetic_to_ecef(aircraft)
    
    delta_X = aircraft_ECEF[0] - receiver_ECEF[0]
    delta_Y = aircraft_ECEF[1] - receiver_ECEF[1]
    delta_Z = aircraft_ECEF[2] - receiver_ECEF[2]
    
    return (delta_X, delta_Y, delta_Z)

def _rotate_enu_by_angle(ENU: TRIPLE_FLOAT_TUPLE, degrees: float) -> TRIPLE_FLOAT_TUPLE:
    """Apply a pitch rotation matrix around the local East-axis to compensate for mount tilt."""
    beta = np.radians(degrees)

    # matrix rotation
    R_x = np.array(
        [[1, 0, 0], [0, math.cos(beta), math.sin(beta)], [0, -math.sin(beta), math.cos(beta)]]
    )

    Ec, Nc, Uc = R_x @ ENU

    return (Ec, Nc, Uc)

def _rotate_to_observer_frame(receiver: TRIPLE_FLOAT_TUPLE, aircraft: TRIPLE_FLOAT_TUPLE, is_angled_back_45=True) -> TRIPLE_FLOAT_TUPLE:
    """Transform relative ECEF coordinates into the observer's local ENU (or tilted mount) frame."""
    phi, lam, h = receiver
    
    # rotate matrix
    R = np.array(
        [
            [-math.sin(lam), math.cos(lam), 0],
            [
                -math.sin(phi) * math.cos(lam),
                -math.sin(phi) * math.sin(lam),
                math.cos(phi),
            ],
            [math.cos(phi) * math.cos(lam), math.cos(phi) * math.sin(lam), math.sin(phi)],
        ]
    )

    # get the difference between ECEF
    delta_ecef = _calculate_vector_difference(receiver, aircraft)

    E, N, U = R @ delta_ecef

    if is_angled_back_45:
        E, N, U = _rotate_enu_by_angle((E, N, U), 45)

    return (E, N, U)

def calculate_azimuth_elevation(receiver: TRIPLE_FLOAT_TUPLE, aircraft: TRIPLE_FLOAT_TUPLE, is_angled_back_45=True) -> DOUBLE_FLOAT_TUPLE:
    """Calculate the Pan (Azimuth) and Tilt (Elevation) motor angles to track a given aircraft.

    Args:
        receiver: Tuple of (latitude_deg, longitude_deg, altitude_m) for the observer.
        aircraft: Tuple of (latitude_deg, longitude_deg, altitude_m) for the aircraft.
        is_angled_back_45: If True, applies a 45° pitch rotation around the North axis 
            to account for a mount physically tilted backward into the sky. Defaults to True.

    Returns:
        Tuple[float, float]: A tuple containing:
            - azimuth_deg (float): Required Pan motor angle in degrees [0.0, 360.0).
            - elevation_deg (float): Required Tilt motor angle in degrees relative to the camera base.
    """
    # convert deg to rad
    receiver = (math.radians(receiver[0]), math.radians(receiver[1]), receiver[2])
    aircraft = (math.radians(aircraft[0]), math.radians(aircraft[1]), aircraft[2])
    
    E, N, U = _rotate_to_observer_frame(receiver, aircraft, is_angled_back_45)
    
    azimuth_rad = math.atan2(E, N)
    azimuth_deg = math.degrees(azimuth_rad) % 360
    
    ground_elevation = math.sqrt((E ** 2) + (N ** 2))
    elevation_rad = math.atan2(U, ground_elevation)
    elevation_deg = math.degrees(elevation_rad)
    
    return (azimuth_deg, elevation_deg)
