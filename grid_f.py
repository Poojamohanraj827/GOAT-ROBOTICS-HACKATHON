import pygame
import random
import time
import heapq

GRID_SIZE = 10
CELL_SIZE = 50
WINDOW_SIZE = GRID_SIZE * CELL_SIZE
WINDOW_HEIGHT = WINDOW_SIZE
OBSTACLE_COUNT = 30
MOVE_INTERVAL = 3  
STEP_DELAY = 0.5   

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
BLUE = (0, 0, 255)

pygame.init()
window = pygame.display.set_mode((WINDOW_SIZE, WINDOW_HEIGHT))
pygame.display.set_caption("Dynamic Robot Navigation")


def initialize_grid(start, end):
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    obstacles = []
    while len(obstacles) < OBSTACLE_COUNT:
        x, y = random.randint(0, GRID_SIZE - 1), random.randint(0, GRID_SIZE - 1)
        if grid[x][y] == 0 and (x, y) != start and (x, y) != end:
            grid[x][y] = 1
            obstacles.append((x, y))
    return grid, obstacles


def draw_grid(grid, robot, end, path, obstacles):
    window.fill(WHITE)

    for x in range(GRID_SIZE):
        for y in range(GRID_SIZE):
            color = WHITE if grid[x][y] == 0 else RED
            pygame.draw.rect(window, color, (y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE))
            pygame.draw.rect(window, BLACK, (y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE), 1)

    if robot:
        pygame.draw.circle(window, GREEN, (robot[1] * CELL_SIZE + CELL_SIZE // 2, robot[0] * CELL_SIZE + CELL_SIZE // 2), CELL_SIZE // 4)
    if end:
        pygame.draw.circle(window, BLUE, (end[1] * CELL_SIZE + CELL_SIZE // 2, end[0] * CELL_SIZE + CELL_SIZE // 2), CELL_SIZE // 4)

    for x, y in path:
        pygame.draw.rect(window, YELLOW, (y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    pygame.display.update()


def a_star(grid, start, end):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, end)}

    while open_list:
        _, current = heapq.heappop(open_list)
        if current == end:
            return reconstruct_path(came_from, current)
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE and grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, end)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
    return reconstruct_path(came_from, current)  # Return partial path to the last reachable node


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path


def find_nearest_reachable(grid, robot, end):
    """
    Find the nearest reachable cell that is closer to the goal.
    """
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    visited = set()
    priority_queue = []
    heapq.heappush(priority_queue, (heuristic(robot, end), 0, robot))  # (priority, distance, position)

    while priority_queue:
        _, dist, current = heapq.heappop(priority_queue)
        if current not in visited:
            visited.add(current)
            if grid[current[0]][current[1]] == 0:
                return current  # Return the nearest valid cell
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE and neighbor not in visited:
                    priority = dist + 1 + heuristic(neighbor, end)  # Prioritize cells closer to the goal
                    heapq.heappush(priority_queue, (priority, dist + 1, neighbor))

    return robot  # If no reachable point is found, stay at the current position


def main():
    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    robot = None
    end = None
    path = []
    obstacles = []
    last_move_time = time.time()

    running = True
    selecting_points = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN and selecting_points:
                mx, my = pygame.mouse.get_pos()
                x, y = my // CELL_SIZE, mx // CELL_SIZE
                if robot is None:
                    robot = (x, y)
                elif end is None:
                    end = (x, y)
                    grid, obstacles = initialize_grid(robot, end)
                    path = a_star(grid, robot, end)
                    selecting_points = False

        if not selecting_points and time.time() - last_move_time > MOVE_INTERVAL:
            for i in range(len(obstacles)):
                x, y = obstacles[i]
                grid[x][y] = 0
                while True:
                    nx, ny = x + random.choice([-1, 0, 1]), y + random.choice([-1, 0, 1])
                    if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and grid[nx][ny] == 0 and (nx, ny) != robot and (nx, ny) != end:
                        obstacles[i] = (nx, ny)
                        grid[nx][ny] = 1
                        break
            last_move_time = time.time()
            path = a_star(grid, robot, end)

        if not path and not selecting_points:
            # Find the nearest reachable point closer to the goal if the goal is blocked
            nearest = find_nearest_reachable(grid, robot, end)
            path = a_star(grid, robot, nearest)

        if path and not selecting_points:
            next_step = path.pop(0)
            robot = next_step
            if robot == end:
                running = False

        draw_grid(grid, robot, end, path, obstacles)
        time.sleep(STEP_DELAY)

    pygame.quit()


main()
