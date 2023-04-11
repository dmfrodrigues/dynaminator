#!/usr/bin/python3

import pathlib
import sys

desktop = pathlib.Path(".")
l = ["script/main.cpp"] # list(desktop.rglob("*.cpp")) + list(desktop.rglob("*.hpp"))

yaml = ""
paths = {}

for filename in l:
    with open(filename) as f:
        print(filename, file=sys.stderr)
        
        lines = [s.strip() for s in f.read().split("\n")]
        
        for l in range(len(lines)):
            if "/**yaml" in lines[l]:
                r = l+1
                while not "*/" in lines[r]:
                    r += 1
                if r == len(lines):
                    exit(-1)

                comment = [s[2:] for s in lines[l+1:r]]
                if lines[l] == "/**yaml":
                    yaml = yaml + "\n".join(comment)
                else:
                    _, method, url = lines[l].split(" ")
                    if url not in paths:
                        paths[url] = {}
                    paths[url][method] = "\n".join("      " + s for s in comment)

print(yaml)
print("paths:")
for url in paths:
    print(f"  {url}:")
    for method in paths[url]:
        print(f"    {method.lower()}:")
        print(paths[url][method])
