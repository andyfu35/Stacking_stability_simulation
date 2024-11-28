file_name = "simple_sealed_cube.obj"
length, width, height = 1.0, 1.0, 1.0
resolution = 3

vertices = []
faces = []

# 生成顶点
vertex_map = {}
idx = 1
step_x = length / (resolution - 1)
step_y = width / (resolution - 1)
step_z = height / (resolution - 1)

for i in range(resolution):
    for j in range(resolution):
        for k in range(resolution):
            x = -length / 2 + i * step_x
            y = -width / 2 + j * step_y
            z = -height / 2 + k * step_z
            vertices.append(f"v {x:.5f} {y:.5f} {z:.5f}")
            vertex_map[(i, j, k)] = idx
            idx += 1

# 生成表面三角形面
for i in range(resolution - 1):
    for j in range(resolution - 1):
        # 前后表面
        faces.append(f"f {vertex_map[(i, j, 0)]} {vertex_map[(i + 1, j, 0)]} {vertex_map[(i, j + 1, 0)]}")
        faces.append(f"f {vertex_map[(i + 1, j + 1, 0)]} {vertex_map[(i, j + 1, 0)]} {vertex_map[(i + 1, j, 0)]}")

        faces.append(f"f {vertex_map[(i, j, resolution - 1)]} {vertex_map[(i + 1, j, resolution - 1)]} {vertex_map[(i, j + 1, resolution - 1)]}")
        faces.append(f"f {vertex_map[(i + 1, j + 1, resolution - 1)]} {vertex_map[(i, j + 1, resolution - 1)]} {vertex_map[(i + 1, j, resolution - 1)]}")

# 写入 .obj 文件
with open(file_name, "w") as obj_file:
    obj_file.write("\n".join(vertices) + "\n")
    obj_file.write("\n".join(faces) + "\n")

print(f"生成完成: {file_name}")
