from shapely.strtree import STRtree
from shapely.geometry import Polygon, Point, LineString
# Danh sách polygon không lồi (vẫn hợp lệ)
polygons = [
    Polygon([(3, 3), (10, 3), (10, 10), (9, 10), (9, 4)])
]

tree = STRtree(polygons)
line = LineString([(3, 4), (9, 4)])
print(tree.query(line, predicate='intersects'))


