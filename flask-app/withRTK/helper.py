import time
import numpy as np
import math

def is_gps_valid(gps_coords):
    """Same as original"""
    if not gps_coords:
        return False
    
    lat = gps_coords.get('latitude', 0)
    lon = gps_coords.get('longitude', 0)
    
    if (lat == 0.0 and lon == 0.0) or abs(lat) > 90 or abs(lon) > 180:
        return False
    
    return True

def latlon_to_xy(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    """Same as original"""
    R = 6371000  # Earth radius in meters
    
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    lat_ref_rad = math.radians(lat_ref)
    lon_ref_rad = math.radians(lon_ref)
    
    x = R * (lon_rad - lon_ref_rad) * math.cos(lat_ref_rad)
    y = R * (lat_rad - lat_ref_rad)
    
    return x, y