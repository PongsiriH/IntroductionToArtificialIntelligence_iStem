import os
import numpy as np
from PIL import Image
import cv2
import matplotlib.pyplot as plt
from IPython.display import Video

def manhattan_distance(xy0, xy1):
    """We will use Manhattan distance as the heuristic function."""
    h = abs(xy0[0] - xy1[0]) + abs(xy0[1] - xy1[1])
    return h

def get_neighbor_nodes(xy, direction_moves):
    """
    xy: the coordinate (x,y)
    direction_moves: list of moves.
    """
    # np.random.shuffle(direction_moves)
    neighbors = np.add(xy, direction_moves)
    neighbors_tuple = [tuple(n) for n in neighbors]
    return neighbors_tuple

def is_out_of_bounds(xy, board_size):
    x, y = xy
    return not (
        0 <= x < board_size[0] 
        and 0 <= y < board_size[1]
    )
    
def load_board(path_to_board):
    """This function load board from image by the given path.
    """
    file = path_to_board
    board = Image.open(file)
    board = np.array(board, dtype='float').max(axis=2)
    board[board == 0] = np.inf
    board[board == 255] = 1e5
    return board

def reconstruct_path(parent_map, start, target):
    """
    Reconstructs the path by tracing the parent_map from the target node 
    back to the starting node. This function return optimal path when used 
    with both Dijkstra's and A* algorithms.
    """
    current = target
    path = []
    while current != start:
        path.append(current)
        current = parent_map.get(current, None)
        if current is None:
            break
    path.append(start)
    path.reverse() # because we built it backwards
    path.append(target)
    return path

def plot_board(board, xy_current=None, xy_starting=None, xy_target=None, frontier=None, parent_map=None, linePath=None, return_frame=None, figsize=(4, 4)):
    vmin, vmax = 0, 5e4
    cmap = plt.get_cmap('Pastel2').copy()
    cmap.set_over('white')  # unexplored nodes are set to 1e5 which is greater than 5e4
    cmap.set_under('pink') 
    pink = -1  # -1 as pink to plot frontier.
    cmap.set_bad((0.2, 0.2, 0.2))   # walls as gray.
    display_board = board.copy()

    if frontier and len(frontier.queue) > 0: # Highlight frontier nodes with pink color
        _, frontier_nodes = zip(*frontier.queue)
        rows, cols = zip(*frontier_nodes)
        display_board[rows, cols] = pink
    
    plt.figure(figsize=figsize)
    plt.imshow(display_board, cmap=cmap, vmin=vmin, vmax=vmax, interpolation='none')

    # Draw grid lines
    for i in range(display_board.shape[0] - 1):
        plt.axhline(i + 0.5, color='gray', linestyle='-', linewidth=0.5)
    for j in range(display_board.shape[1] - 1):
        plt.axvline(j + 0.5, color='gray', linestyle='-', linewidth=0.5)
        
    # Annotating positions and values on the board
    for (r_idx, row) in enumerate(board):
        for (c_idx, val) in enumerate(row):
            if not np.isnan(val):
                if val != 1e5 and val != np.inf:
                    plt.text(c_idx, r_idx, f'{int(val):.0f}', ha='center', va='center', color='black', fontsize=22)
    
    if xy_current: # Plot current position
        plt.text(xy_current[1], xy_current[0], 'C', color='blue', ha='center', va='center', fontsize=16, weight='bold')
    if xy_starting: # Plot starting position
        plt.text(xy_starting[1], xy_starting[0], 'S', color='red', ha='center', va='center', fontsize=16, weight='bold')
    if xy_target: # Plot target position
        plt.text(xy_target[1], xy_target[0], 'G', color='red', ha='center', va='center', fontsize=16, weight='bold')
    if parent_map:
        for (x0, y0), (x1, y1) in parent_map.items():
            plt.arrow(y0, x0, (y1-y0)/4, (x1-x0)/4, alpha=0.5, head_width=0.3, head_length=0.4, fc='blue', ec='blue')
        
    if linePath:
        y, x = zip(*linePath)
        plt.plot(x, y, marker='o', color='gold', linestyle='-', markersize=5, linewidth=2, alpha=1)
    
    plt.axis('off')
    plt.axis('equal')
    
    if return_frame:
        plt.gca().figure.canvas.draw()
        frame = np.frombuffer(plt.gcf().canvas.tostring_rgb(), dtype=np.uint8)
        width, height = plt.gcf().canvas.get_width_height()
        frame = frame.reshape((height, width, 3))
        plt.close()
        return frame

def create_video_from_frames(frames, output_filename, fps=2, fourcc_codec='mp4v'):
    if not frames:
        raise ValueError("The frame list is empty.")
    
    frame_size = frames[0].shape[:-1]
    fourcc = cv2.VideoWriter_fourcc(*fourcc_codec)
    
    out = cv2.VideoWriter(output_filename, fourcc, fps, frame_size)
    
    for frame in frames:
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        out.write(frame_bgr)
    out.release()
    
    # Display the video
    video_path = os.path.abspath(output_filename)
    return Video(video_path, width=640, height=360, embed=True)
