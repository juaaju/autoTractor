c1 = -7.283564
x1 = 112.796641
x2 = 112.796465

array1 = []
epsilon = 1e-9  # toleransi untuk perbandingan float

while abs(x1 - x2) > epsilon:
    array1.append([c1, round(x1, 6)])  # Pasangkan x1 dan c1
    x1 -= 0.000001

print(array1, "##########")

x1 = -7.283564
x2 = -7.283510
c1 = 112.796465

array2 = []
epsilon = 1e-9  # toleransi untuk perbandingan float

while abs(x1 - x2) > epsilon:
    array2.append([round(x1, 6), c1])  # Pasangkan x1 dan c1
    x1 += 0.000001

print(array2, "#########")

x1 = 112.796465
x2 = 112.796641
c1 = -7.283510

array3 = []
epsilon = 1e-9  # toleransi untuk perbandingan float

while abs(x1 - x2) > epsilon:
    array3.append([c1, round(x1, 6)])  # Pasangkan x1 dan c1
    x1 += 0.000001

print(array3, "#########")

x1 = -7.283510
x2 = -7.283564
c1 = 112.796641

array4 = []
epsilon = 1e-9  # toleransi untuk perbandingan float

while abs(x1 - x2) > epsilon:
    array4.append([ round(x1, 6), c1])  # Pasangkan x1 dan c1
    x1 -= 0.000001

print(array4, "#########")

# Gabungkan semua array
array_gabung = array1 + array2 + array3 + array4

# Cetak hasil
print(array_gabung)