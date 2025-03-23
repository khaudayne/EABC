from shapely.geometry import LineString

def heuristic(a, b):
    DD = 1
    DD2 = 1.412
    d1 = abs(b.x - a.x)
    d2 = abs(b.y - a.y)
    return (DD * (d1 + d2)) + ((DD2 - (2 * DD)) * min(d1, d2))

class GridNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.f = 0
        self.g = 0
        self.h = 0
        self.visited = False
        self.closed = False
        self.parent = None

class BinaryHeap:
    def __init__(self):
        self.content = []

    def push(self, element):
        # Add the new element to the end of the array.
        self.content.append(element)

        # Allow it to sink down.
        self.sinkDown(len(self.content) - 1)

    def pop(self):
        # Store the first element so we can return it later.
        result = self.content[0]
        # Get the element at the end of the array.
        end = self.content.pop()
        # If there are any elements left, put the end element at the
        # start, and let it bubble up.
        if (len(self.content) > 0):
            self.content[0] = end
            self.bubbleUp(0)
    
        return result

    def remove(self, node):
        i = self.content.index(node)

        # When it is found, the process seen in 'pop' is repeated
        # to fill up the hole.
        end = self.content.pop()

        if i != len(self.content) - 1:
            self.content[i] = end
            if end.f < node.f:
                self.sinkDown(i)
            else:
                self.bubbleUp(i)
            
            

    def size(self):
        return len(self.content)

    def rescore_element(self, node):
        self.sinkDown(self.content.index(node))

    def sinkDown(self, n):
        # Fetch the element that has to be sunk.
        element = self.content[n]

        # When at 0, an element can not sink any further.
        while n > 0:
            # Compute the parent element's index, and fetch it.
            parentN = ((n + 1) >> 1) - 1
            parent = self.content[parentN]
            # Swap the elements if the parent is greater.
            if element.f < parent.f:
                self.content[parentN] = element
                self.content[n] = parent
                # Update 'n' to continue at the new position.
                n = parentN
                
            # Found a parent that is less, no need to sink any further.
            else:
                break
    
    def bubbleUp(self, n):
        # Look up the target element and its score.
        length = len(self.content)
        element = self.content[n]
        elemScore = element.f

        while True:
            # Compute the indices of the child elements.
            child2N = (n + 1) * 2
            child1N = child2N - 1
            # This is used to store the new position of the element, if any.
            swap = None
            child1Score = None
            # If the first child exists (is inside the array)...
            if child1N < length:
                # Look it up and compute its score.
                child1 = self.content[child1N]
                child1Score = child1.f
                # If the score is less than our element's, we need to swap.
                if child1Score < elemScore:
                    swap = child1N

            # Do the same checks for the other child.
            if child2N < length:
                child2 = self.content[child2N]
                child2Score = child2.f
                tmp = child1Score
                if swap == None:
                    tmp = elemScore
                if child2Score < tmp:
                    swap = child2N


            # If the element needs to be moved, swap it, and continue.
            if swap != None:
                self.content[n] = self.content[swap]
                self.content[swap] = element
                n = swap
            
            # Otherwise, we are done.
            else:
                break

### Class tìm đường đi bằng A-star
# Gồm các param:
    # start: tuple gồm tọa độ (x, y) của điểm bắt đầu
    # goal: tuple gồm tọa độ (x, y) của điểm kết thúc
    # map_size: tuple chứa thông tin kích cỡ của map (width, height)
    # str_tree: STRtree, gồm tree chứa bounding của các obstacle

# Tìm đường bằng hàm find_path():
    # Trả về đường nếu tìm thành không
    # Trả về None nếu không tìm được đường
class AStar:
    def __init__(self, start, goal, map_size, str_tree):
        self.graph = [[GridNode(i, j) for j in range(map_size[1] + 1)] for i in range(map_size[0] + 1)]
        self.start = self.graph[start[0]][start[1]]
        self.goal = self.graph[goal[0]][goal[1]]
        self.map_size = map_size
        self.str_tree = str_tree
        self.directions = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1), (1, 1, 1.412), (-1, -1, 1.412), (1, -1, 1.412), (-1, 1, 1.412)]
    
    def is_valid(self, currentNode, point):
        x, y = point
        if not (0 <= x <= self.map_size[0] and 0 <= y <= self.map_size[1]):
            return False
        if self.graph[x][y].closed:
            return False
        line = LineString([(currentNode.x, currentNode.y), (x, y)])
        candidates = self.str_tree.query(line)
        if len(candidates) > 0:
            return False
        return True
    
    
    def find_path(self):
        openHeap = BinaryHeap()
        self.start.h = heuristic(self.start, self.goal)
        openHeap.push(self.start)
        while openHeap.size() > 0:
            current = openHeap.pop()
            
            if current == self.goal:
                return self.reconstruct_path(current)
            current.closed = True
            for dx, dy, cost in self.directions:
                neighborPoint = (current.x + dx, current.y + dy)
                if not self.is_valid(current, neighborPoint):
                    continue
                neighborNode = self.graph[neighborPoint[0]][neighborPoint[1]]
                gScore = current.g + cost
                beenVisited = neighborNode.visited
                if (not beenVisited) or (gScore < neighborNode.g):
                    neighborNode.visited = True
                    neighborNode.parent = current
                    if neighborNode.h == 0:
                        neighborNode.h = heuristic(neighborNode, self.goal)
                    neighborNode.g = gScore
                    neighborNode.f = neighborNode.g + neighborNode.h
                    if not beenVisited:
                        openHeap.push(neighborNode)
                    else:
                        openHeap.rescore_element(neighborNode)
        
        return None
    
    def reconstruct_path(self, current):
        path = []
        while current.parent != None:
            path.append((current.x, current.y))
            current = current.parent
        return path[::-1]
