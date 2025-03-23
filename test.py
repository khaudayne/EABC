import math

x = 1  # Giá trị cần tính toán

# Tính arctan (trả về kết quả theo radian)
arctan_x = math.atan(x)

# Tính sin và cos
sin_x = math.sin(arctan_x)
cos_x = math.cos(arctan_x)

print(f"arctan({x}) = {arctan_x} rad")
print(f"sin({x}) = {sin_x * sin_x}")
print(f"cos({x}) = {cos_x}")
