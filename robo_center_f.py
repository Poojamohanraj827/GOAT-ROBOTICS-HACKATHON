import pygame
import random
import heapq
import time

GRID_SIZE = 10
CELL_SIZE = 50
WINDOW_SIZE = GRID_SIZE * CELL_SIZE
MOVE_INTERVAL = 3  
STEP_DELAY = 0.5   

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
BLUE = (0, 0, 255)

DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]

def calculate_center(v1, v2, v3, v4):
    center_x = round((v1[0] + v2[0] + v3[0] + v4[0]) / 4)
    center_y = round((v1[1] + v2[1] + v3[1] + v4[1]) / 4)
    return (center_x, center_y)

def a_star(grid, start, goal):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  

    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_list:
        _, current = heapq.heappop(open_list)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for dx, dy in DIRECTIONS:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < GRID_SIZE and 0 <= neighbor[1] < GRID_SIZE and grid[neighbor[0]][neighbor[1]] == 0:
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_list, (f_score[neighbor], neighbor))
    return []

def update_grid(grid, obstacles):
    for x, y in obstacles:
        if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
            grid[x][y] = 1  

def move_obstacles(obstacles):
    new_obstacles = []
    for x, y in obstacles:
        nx, ny = x + random.choice([-1, 0, 1]), y + random.choice([-1, 0, 1])
        if 0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE:
            new_obstacles.append((nx, ny))
        else:
            new_obstacles.append((x, y))  
    return new_obstacles

def draw_grid(window, grid, robot, goal, path, obstacles, selections):
    window.fill(WHITE)

    for x in range(GRID_SIZE):
        for y in range(GRID_SIZE):
            color = WHITE if grid[x][y] == 0 else RED
            pygame.draw.rect(window, color, (y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE))
            pygame.draw.rect(window, BLACK, (y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE), 1)

    if robot:
        pygame.draw.circle(window, GREEN, (robot[1] * CELL_SIZE + CELL_SIZE // 2, robot[0] * CELL_SIZE + CELL_SIZE // 2), CELL_SIZE // 4)
    if goal:
        pygame.draw.circle(window, BLUE, (goal[1] * CELL_SIZE + CELL_SIZE // 2, goal[0] * CELL_SIZE + CELL_SIZE // 2), CELL_SIZE // 4)
    for x, y in path:
        pygame.draw.rect(window, YELLOW, (y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    for x, y in obstacles:
        pygame.draw.rect(window, RED, (y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE))
    
    for x, y in selections:
        pygame.draw.rect(window, GREEN, (y * CELL_SIZE, x * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    pygame.display.update()

def main():
    pygame.init()
    window = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    pygame.display.set_caption("Dynamic Robot Navigation")

    grid = [[0 for _ in range(GRID_SIZE)] for _ in range(GRID_SIZE)]
    robot = None
    goal = None
    path = []
    obstacles = []
    selections = []

    input_state = "start"  # Tracks input state: "start" -> "pillars" -> "navigate"
    running = True
    last_move_time = time.time()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                x, y = my // CELL_SIZE, mx // CELL_SIZE

                if input_state == "start":
                    robot = (x, y)
                    selections.append((x, y))
                    input_state = "pillars"
                elif input_state == "pillars" and len(obstacles) < 4:
                    obstacles.append((x, y))
                    selections.append((x, y))
                    if len(obstacles) == 4:
                        center = calculate_center(*obstacles)
                        goal = center
                        update_grid(grid, obstacles)
                        path = a_star(grid, robot, goal)
                        input_state = "navigate"
        
        if input_state == "navigate":
            if robot and goal and path:
                if time.time() - last_move_time > MOVE_INTERVAL:
                    obstacles = move_obstacles(obstacles)
                    update_grid(grid, obstacles)
                    path = a_star(grid, robot, goal)
                    last_move_time = time.time()

                if path:
                    next_step = path.pop(0)
                    robot = next_step
                    if robot == goal:
                        running = False  

        draw_grid(window, grid, robot, goal, path, obstacles, selections)
        time.sleep(STEP_DELAY)

    pygame.quit()

if __name__ == "__main__":
    main()
