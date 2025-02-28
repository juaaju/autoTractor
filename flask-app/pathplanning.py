import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, sqrt, atan2, radians

def haversine_distance(lat1, lon1, lat2, lon2):
    """
    Menghitung jarak antara dua titik koordinat menggunakan formula Haversine
    """
    # Radius bumi dalam meter
    R = 6371000.0
    
    # Konversi koordinat ke radian
    lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
    
    # Perbedaan koordinat
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    # Formula Haversine
    a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    distance = R * c
    
    return distance

def latlon_to_meters(lat_center, lon_center, lat, lon):
    """
    Mengkonversi koordinat lat, lon ke posisi meter relatif terhadap titik pusat
    """
    dx = haversine_distance(lat_center, lon_center, lat_center, lon)
    # Jika longitude target lebih kecil dari pusat, jarak adalah negatif
    if lon < lon_center:
        dx = -dx
        
    dy = haversine_distance(lat_center, lon_center, lat, lon_center)
    # Jika latitude target lebih kecil dari pusat, jarak adalah negatif
    if lat < lat_center:
        dy = -dy
        
    return dx, dy

def meters_to_latlon(lat_center, lon_center, x, y):
    """
    Mengkonversi posisi meter relatif ke koordinat lat, lon
    """
    # Radius bumi dalam meter
    R = 6371000.0
    
    # Konversi ke radian
    lat_center, lon_center = map(radians, [lat_center, lon_center])
    
    # 1 derajat latitude ≈ 111,32 km = 111.320 m
    # 1 derajat longitude ≈ 111,32 km * cos(latitude) = 111.320 m * cos(latitude)
    lat_change = y / 111320.0
    lon_change = x / (111320.0 * cos(lat_center))
    
    new_lat = lat_center + lat_change
    new_lon = lon_center + lon_change
    
    return np.degrees(new_lat), np.degrees(new_lon)

def sort_vertices_counterclockwise(vertices):
    """
    Mengurutkan titik-titik dengan arah berlawanan jarum jam
    """
    # Menghitung pusat poligon
    centroid = np.mean(vertices, axis=0)
    
    # Menghitung sudut dari pusat ke setiap titik
    def get_angle(point):
        return np.arctan2(point[1] - centroid[1], point[0] - centroid[0])
    
    # Mengurutkan berdasarkan sudut
    return sorted(vertices, key=get_angle)

def generate_zigzag_path_90deg(corners, swath_width):
    """
    Menghasilkan jalur zigzag dengan sudut 90 derajat berdasarkan empat titik sudut.
    
    Parameters:
    -----------
    corners : list of tuples
        4 titik koordinat dalam format [(lat1, lon1), (lat2, lon2), (lat3, lon3), (lat4, lon4)]
    swath_width : float
        Lebar alur pembajakan dalam meter
        
    Returns:
    --------
    path : list of tuples
        Daftar titik-titik koordinat untuk jalur zigzag
    """
    # Menentukan pusat koordinat sebagai referensi
    center_lat = np.mean([corner[0] for corner in corners])
    center_lon = np.mean([corner[1] for corner in corners])
    
    # Konversi koordinat lat, lon ke meter
    corners_meter = []
    for lat, lon in corners:
        x, y = latlon_to_meters(center_lat, center_lon, lat, lon)
        corners_meter.append((x, y))
    
    # Mengubah ke array numpy
    corners_meter = np.array(corners_meter)
    
    # Mengurutkan titik-titik dengan arah berlawanan jarum jam
    corners_meter = sort_vertices_counterclockwise(corners_meter)
    
    # Mencari batas area
    x_min, y_min = np.min(corners_meter, axis=0)
    x_max, y_max = np.max(corners_meter, axis=0)
    
    # Menentukan arah pembajakan pada sumbu x (horizontal, 90 derajat terhadap utara)
    # Menghitung jumlah alur pembajakan
    num_passes = int(np.ceil((y_max - y_min) / swath_width))
    
    # Inisialisasi jalur zigzag
    zigzag_path_meter = []
    
    # Membuat jalur zigzag
    for i in range(num_passes):
        y = y_min + i * swath_width + swath_width / 2
        
        # Mencari titik perpotongan dengan tepi area
        intersections = []
        for j in range(len(corners_meter)):
            j_next = (j + 1) % len(corners_meter)
            
            x1, y1 = corners_meter[j]
            x2, y2 = corners_meter[j_next]
            
            # Jika garis horizontal memotong sisi poligon
            if (y1 <= y <= y2) or (y2 <= y <= y1):
                if y1 == y2:  # Sisi horizontal, abaikan
                    continue
                
                # Menghitung titik perpotongan
                x_intersect = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
                intersections.append(x_intersect)
        
        # Mengurutkan titik perpotongan berdasarkan koordinat x
        intersections.sort()
        
        # Memastikan ada perpotongan valid
        if len(intersections) >= 2:
            # Mengambil titik perpotongan terluar saja
            x_start = intersections[0]
            x_end = intersections[-1]
            
            # Menambahkan titik ke jalur (urutan bergantung pada indeks ganjil/genap)
            if i % 2 == 0:
                zigzag_path_meter.append((x_start, y))
                zigzag_path_meter.append((x_end, y))
            else:
                zigzag_path_meter.append((x_end, y))
                zigzag_path_meter.append((x_start, y))
    
    # Konversi kembali ke koordinat lat, lon
    zigzag_path = []
    for x, y in zigzag_path_meter:
        lat, lon = meters_to_latlon(center_lat, center_lon, x, y)
        zigzag_path.append((lat, lon))
    
    print(zigzag_path_meter)
    return zigzag_path

def visualize_path(corners, path, swath_width):
    """
    Visualisasi area sawah dan jalur pembajakan
    """
    # Konversi ke numpy array
    corners = np.array(corners)
    path = np.array(path)
    
    # Menambahkan titik pertama di akhir untuk menutup poligon
    corners_closed = np.vstack([corners, corners[0]])
    
    plt.figure(figsize=(12, 10))
    
    # Plot area sawah
    plt.plot(corners_closed[:, 1], corners_closed[:, 0], 'k-', linewidth=2, label='Batas Sawah')
    plt.fill(corners_closed[:, 1], corners_closed[:, 0], color='#90EE90', alpha=0.3)
    
    # Plot jalur pembajakan
    for i in range(0, len(path), 2):
        if i+1 < len(path):
            color = 'b-' if i % 4 == 0 else 'r-'
            plt.plot([path[i, 1], path[i+1, 1]], [path[i, 0], path[i+1, 0]], color, linewidth=1.5)
    
    # # Tambahkan arah jalur dengan panah
    # for i in range(0, len(path)-1, 2):
    #     if i+1 < len(path):
    #         mid_x = (path[i, 1] + path[i+1, 1]) / 2
    #         mid_y = (path[i, 0] + path[i+1, 0]) / 2
            
    #         dx = (path[i+1, 1] - path[i, 1]) / 20
    #         dy = (path[i+1, 0] - path[i, 0]) / 20
            
    #         plt.arrow(mid_x, mid_y, dx, dy, 
    #                  head_width=0.0001, head_length=0.0001, 
    #                  fc='red', ec='red')
    
    # Titik awal dan akhir
    plt.plot(path[0, 1], path[0, 0], 'go', markersize=8, label='Titik Awal')
    plt.plot(path[-1, 1], path[-1, 0], 'ro', markersize=8, label='Titik Akhir')
    
    # Menambahkan label untuk titik-titik jalur
    for i in range(0, min(len(path), 20), 2):
        plt.annotate(f'P{i+1}', (path[i, 1], path[i, 0]), 
                    textcoords="offset points", 
                    xytext=(0,5), 
                    ha='center',
                    fontsize=8)
    
    plt.title('Jalur Pembajakan Zigzag (90 Derajat)')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    
    # Simpan visualisasi sebagai gambar
    plt.savefig('zigzag_90_degrees.png', dpi=300)
    
    return plt

def export_waypoints_csv(path, filename='waypoints.csv'):
    """
    Menyimpan titik-titik jalur ke file CSV
    """
    with open(filename, 'w') as f:
        f.write('Point,Latitude,Longitude\n')
        for i, (lat, lon) in enumerate(path):
            f.write(f'{i+1},{lat:.8f},{lon:.8f}\n')
    print(f'Titik-titik jalur telah disimpan ke {filename}')

# Contoh penggunaan
if __name__ == "__main__":
    # Empat titik koordinat sawah (latitude, longitude)
    corners = [
        (-7.8051, 110.3653),  # Titik Barat Laut
        (-7.8051, 110.3658),  # Titik Timur Laut
        (-7.8056, 110.3658),  # Titik Tenggara
        (-7.8056, 110.3653)   # Titik Barat Daya
    ]
    
    # Lebar alur pembajakan (meter)
    swath_width = 2.0
    
    # Menghasilkan jalur zigzag dengan sudut 90 derajat
    zigzag_path = generate_zigzag_path_90deg(corners, swath_width)
    
    # Menghitung total panjang jalur
    path_length = 0
    for i in range(len(zigzag_path)-1):
        path_length += haversine_distance(
            zigzag_path[i][0], zigzag_path[i][1], 
            zigzag_path[i+1][0], zigzag_path[i+1][1]
        )
    
    # Cetak informasi jalur
    print("=== INFORMASI JALUR PEMBAJAKAN (90 DERAJAT) ===")
    print(f"Jumlah baris pembajakan: {len(zigzag_path)//2}")
    print(f"Total panjang jalur: {path_length:.2f} meter")
    
    # Cetak titik-titik jalur
    print("\n=== TITIK-TITIK JALUR PEMBAJAKAN ===")
    for i, (lat, lon) in enumerate(zigzag_path):
        print(f"Point {i+1}: Latitude = {lat:.8f}, Longitude = {lon:.8f}")
    
    # # Ekspor titik-titik jalur ke CSV
    # export_waypoints_csv(zigzag_path)
    
    # # Visualisasi
    # plt = visualize_path(corners, np.array(zigzag_path), swath_width)
    # plt.show()