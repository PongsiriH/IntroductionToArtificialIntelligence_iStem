import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

def exceed_board(xy, board_size):
    x, y = xy
    return not (
        0 <= x < board_size[0] 
        and 0 <= y < board_size[1]
    )
    
def load_board(path_to_board):
    file = path_to_board
    board = Image.open(file)
    board = np.array(board, dtype='float').max(axis=2)
    board[board == 0] = np.inf
    board[board == 255] = 1e5
    return board

def plot_board(ax, board, xy_current=None, xy_starting=None, xy_target=None, frontier=None, linePath=None, return_frame=None):
    vmin, vmax = 0, 5e4
    cmap = plt.get_cmap('Pastel2').copy()
    cmap.set_over('white')  # unexplored nodes are set to 1e5 which is greater than vmax
    cmap.set_under('pink') 
    pink = -1  # Use pink to plot frontier nodes.
    cmap.set_bad((0.2, 0.2, 0.2))  # Set walls as gray.

    display_board = board.copy()

    if frontier and len(frontier.queue) > 0:  # Highlight frontier nodes with pink color
        _, frontier_nodes = zip(*frontier.queue)
        rows, cols = zip(*frontier_nodes)
        display_board[rows, cols] = pink
    
    im = ax.imshow(display_board, cmap=cmap, vmin=vmin, vmax=vmax, interpolation='none')

    # Draw grid lines
    for i in range(display_board.shape[0] - 1):
        ax.axhline(i + 0.5, 0, display_board.shape[1], color='gray', linestyle='-', linewidth=0.5)
    for j in range(display_board.shape[1] - 1):
        ax.axvline(j + 0.5, 0, display_board.shape[0], color='gray', linestyle='-', linewidth=0.5)
        
    # Annotating positions and values on the board
    for (r_idx, row) in enumerate(board):
        for (c_idx, val) in enumerate(row):
            if not np.isnan(val):
                if val != 1e5 and val != np.inf:
                    ax.text(c_idx, r_idx, f'{int(val):.0f}', ha='center', va='center', color='black', fontsize=16)
    
    if xy_current:  # Plot current position
        ax.text(xy_current[1], xy_current[0], 'C', color='red', ha='center', va='center', fontsize=20, weight='bold')
    if xy_starting:  # Plot starting position
        ax.text(xy_starting[1], xy_starting[0], 'S', color='green', ha='center', va='center', fontsize=20, weight='bold')
    if xy_target:  # Plot target position
        ax.text(xy_target[1], xy_target[0], 'G', color='blue', ha='center', va='center', fontsize=20, weight='bold')
    if linePath:
        y, x = zip(*linePath)
        ax.plot(x, y, marker='o', color='black', linestyle='-', markersize=5, linewidth=2)
    
    ax.axis('off')
    ax.axis('equal')
    
    if return_frame:
        ax.figure.canvas.draw()
        frame = np.frombuffer(ax.figure.canvas.tostring_rgb(), dtype=np.uint8)
        width, height = ax.figure.canvas.get_width_height()
        frame = frame.reshape((height, width, 3))
        plt.close(ax.figure)
        return frame
