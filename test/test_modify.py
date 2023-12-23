import sys
from pathlib import Path
path_scripts = Path(__file__).resolve().parents[1]
sys.path.append(str(path_scripts))
from lib.kdtree import *
import pytest

# Example usage
kd_tree = KDTree()
kd_tree.insert(Point([1.0, 2.0, 3.0]))
kd_tree.insert(Point([4.0, 5.0, 6.0]))
kd_tree.insert(Point([7.0, 8.0, 9.0]))
kd_tree.insert(Point([10.0, 11.0, 12.0]))
kd_tree.insert(Point([13.0, 14.0, 15.0]))
kd_tree.insert(Point([16.0, 17.0, 18.0]))
kd_tree.insert(Point([19.0, 20.0, 21.0]))
kd_tree.insert(Point([22.0, 23.0, 24.0]))
kd_tree.insert(Point([25.0, 26.0, 27.0]))
kd_tree.insert(Point([28.0, 29.0, 30.0]))
near = kd_tree.nearest_neighbor(Point([30.0, 0.0, 30.0]))
for i in near.coordinates:
    print(i, end=' ')
print()
kd_tree.traverse_and_print()
# Call other functions as needed

