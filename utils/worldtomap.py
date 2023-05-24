resolution = 0.05000000074505806
width = 480
height = 480
origin_x = -12.2
origin_y = -12.2


def read_costmap():
    fpath = "./costmap.txt"

    with open(fpath, "r") as f:
        content = f.read()
        content = content.replace("[", "").replace("]", "")
        int_list = [int(num) for num in content.split(",")]
        return int_list


def world_to_map(x: float, y: float):
    t_x = x - origin_x
    t_y = y - origin_y
    map_x = int(t_x / resolution)
    map_y = int(t_y / resolution)
    return map_x, map_y


if __name__ == "__main__":
    map_list = read_costmap()

    world_coords = input("Input world coords separated by a comma (like 2.2, 4.1): ")
    [x, y] = list(map(lambda x: float(x), world_coords.split(",")))
    print(world_to_map(x, y))
