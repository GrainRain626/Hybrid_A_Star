import random

def generate_random_map(width, height, obstacle_ratio=0.3, filename='map.txt'):
    with open(filename, 'w') as f:
        f.write(f"{width} {height}\n")
        for _ in range(height):
            row = []
            for _ in range(width):
                cell = 1 if random.random() < obstacle_ratio else 0
                row.append(str(cell))
            f.write(" ".join(row) + "\n")
    print(f"✅ 随机地图生成成功：{filename}，大小 {width}x{height}，障碍占比 {obstacle_ratio * 100:.0f}%")

if __name__ == "__main__":
    generate_random_map(width=50, height=50, obstacle_ratio=0.3)
