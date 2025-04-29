from pathplanning import haversine_distance
from math import atan2, degrees  # Tambahkan ini
import numpy as np

# Fungsi-fungsi untuk kontrol otomatis (tambahkan di sini)
def calculate_turn_angle(point1, point2, point3):
    """
    Menghitung sudut belokan (dalam derajat) antara tiga titik berurutan.
    Nilai positif = belok kanan, nilai negatif = belok kiri
    """
    # Vektor arah dari point1 ke point2
    vector1 = np.array([point2[0] - point1[0], point2[1] - point1[1]])
    # Vektor arah dari point2 ke point3
    vector2 = np.array([point3[0] - point2[0], point3[1] - point2[1]])
    
    # Normalisasi vektor
    vector1 = vector1 / np.linalg.norm(vector1)
    vector2 = vector2 / np.linalg.norm(vector2)
    
    # Produk silang untuk menentukan arah belokan
    cross_product = np.cross(vector1, vector2)
    
    # Produk dot untuk menentukan sudut
    dot_product = np.dot(vector1, vector2)
    
    # Sudut belokan dalam radian
    angle = atan2(cross_product, dot_product)
    
    # Konversi ke derajat
    return degrees(angle)

# Ambang batas sudut untuk menentukan belokan
TURN_THRESHOLD = 45  # derajat

def determine_turn_command(angle):
    """
    Menentukan perintah belok berdasarkan sudut belokan
    """
    if abs(angle) < TURN_THRESHOLD:
        return None  # Tidak perlu belok
    elif angle > 0:
        return "belok_kanan"
    else:
        return "belok_kiri"

def find_closest_point(position, path):
    """
    Mencari indeks titik terdekat di jalur dari posisi saat ini
    """
    min_distance = float('inf')
    closest_idx = 0
    
    for i, point in enumerate(path):
        dist = haversine_distance(position[0], position[1], point[0], point[1])
        if dist < min_distance:
            min_distance = dist
            closest_idx = i
    
    return closest_idx

def auto_control(current_position, path, distance_threshold=5.0):
    """
    Kontrol otomatis traktor berdasarkan posisi saat ini dan jalur yang direncanakan
    
    Parameters:
    -----------
    current_position : tuple (lat, lng)
        Posisi saat ini dari traktor (diambil dari EKF)
    path : list of tuples
        Jalur yang telah direncanakan
    distance_threshold : float
        Jarak threshold dalam meter untuk mendeteksi kedekatan dengan titik belokan
    
    Returns:
    --------
    command : str or None
        Perintah kontrol traktor
    """
    # Temukan titik terdekat di jalur
    closest_point_idx = find_closest_point(current_position, path)
    
    # Jika sudah mendekati akhir jalur, tidak perlu belok
    if closest_point_idx >= len(path) - 2:
        return None
    
    # Ambil tiga titik berurutan dari jalur (titik terdekat dan dua titik berikutnya)
    p1 = path[closest_point_idx]
    p2 = path[closest_point_idx + 1]
    p3 = path[closest_point_idx + 2] if closest_point_idx + 2 < len(path) else None
    
    if p3 is None:
        return None  # Sudah mendekati akhir jalur
    
    # Hitung jarak ke titik belokan
    distance_to_turn = haversine_distance(current_position[0], current_position[1], p2[0], p2[1])
    
    # Jika sudah mendekati titik belokan
    if distance_to_turn <= distance_threshold:
        # Hitung sudut belokan
        turn_angle = calculate_turn_angle(p1, p2, p3)
        
        # Tentukan perintah belok
        return determine_turn_command(turn_angle)
    
    return None