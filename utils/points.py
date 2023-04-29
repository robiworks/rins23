import sys


def parse_points(fname: str):
    output = ["vector<NavigatorPoint> interestPoints {"]

    with open(fname) as f:
        contents = f.read().strip()
        lst = list(filter(lambda l: l, contents.split("---")))

        for pt in lst:
            lines = list(map(lambda l: l, pt.strip().splitlines()))

            x = lines[-3].strip()[3:]
            y = lines[-2].strip()[3:]
            output.append(f"  {{{x}, {y}, true}},")

    output.append("};")
    print("\n".join(output))


if __name__ == "__main__":
    if len(sys.argv) > 1:
        fname = sys.argv[1]
        parse_points(fname)
    else:
        raise Exception("Not enough arguments. Provide filename")
