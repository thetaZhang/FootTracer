import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import RBFInterpolator
from matplotlib.path import Path
import time


def init_dynamic_plot():
    
    points = np.array([[61.34, 224.12], [91.63, 218.88], [120.47, 206.35], [63.38, 175.77],
                       [82.02, 172.56], [102.99, 168.19], [125.13, 163.53], [122.28, 127.73],
                       [113.96, 81.78], [81.39, 150.53], [96.95, 111.45], [80.67, 72.73],
                       [98.26, 52.58], [96.84, 33.03], [94.63, 12.47]
                       ])      
    initial_values = np.zeros(len(points))                  
    grid_x, grid_y = np.mgrid[0:200:100j, 0:270:135j]
    grid_points = np.column_stack([grid_x.ravel(), grid_y.ravel()])
    grid_z = np.full(grid_x.shape, np.nan)

    
    boundary_points = np.array([[112.15, 250.66], [120.0, 240.0], [127.59, 225.81], [132.9, 213.26],
                                [137.49, 200.22], [140.87, 182.61], [141.35, 168.37], [140.0, 160.0],
                                [137.97, 147.85], [134.83, 133.86], [132.42, 118.17], [130.01, 102.0],
                                [128.8, 85.35], [126.63, 50.84], [125.42, 31.77], [123.49, 21.64],
                                [117.7, 11.5], [111.42, 5.22], [103.7, 2.09], [95.01, 0.88],
                                [87.53, 2.57], [80.53, 5.95], [73.53, 11.74], [70.4, 19.7],
                                [68.47, 27.67], [68.22, 39.98], [70.88, 65.08], [74.98, 99.35],
                                [75.71, 112.62], [75.22, 123.96], [73.05, 132.65], [69.67, 141.82],
                                [66.29, 149.06], [61.71, 155.82], [56.64, 163.3], [51.57, 173.44],
                                [48.92, 184.54], [48.19, 197.09], [48.92, 215.19], [52.54, 235.7],
                                [57.12, 250.18], [61.47, 257.18], [67.5, 262.73], [74.26, 265.87],
                                [82.46, 266.67], [92.84, 265.39], [100.1, 262.1], [107.14, 256.68],
                                [112.15, 250.66]
                                ])
    boundary_path = Path(boundary_points)
    mask = boundary_path.contains_points(grid_points)
    valid_grid_points = grid_points[mask]

    
    plt.ion()  
    fig, ax = plt.subplots()
    cax = ax.imshow(np.zeros_like(grid_x), extent=(0, 200, 0, 270),
                    origin='lower', cmap='turbo', vmin=0, vmax=4095)
    scatter = ax.scatter(points[:, 0], points[:, 1], c=initial_values,
                         cmap='turbo', edgecolors='black', s=40, vmin=0, vmax=4095)
    fig.colorbar(cax, ax=ax, label="Interpolated Value")

    ax.set_title("Dynamic 2D Interpolation with Boundary")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")

    ax.plot(*boundary_points.T, 'k-')

    return points, valid_grid_points, grid_x, grid_z, ax, cax, mask, scatter, fig

def update_dynamic_plot(points, values, valid_grid_points, grid_x, grid_z, ax, cax, mask, scatter, fig):
    
    rbf = RBFInterpolator(points, values, kernel='thin_plate_spline')
    interpolated = rbf(valid_grid_points)
    grid_z[:, :] = np.nan
    grid_z[mask.reshape(grid_x.shape)] = interpolated
    
    cax.set_array(grid_z.T)
    scatter.set_array(values)
    ax.set_title(f"Dynamic 2D Interpolation with Boundary") # 步数显示，可以去掉
    fig.canvas.draw()
    fig.canvas.flush_events()


# if __name__ == '__main__':
#     #进行初始化
#     points, valid_grid_points, grid_x, grid_z, ax, cax, mask, scatter, fig = init_dynamic_plot()
#     # 主循环
#     for step in range(1000):  # 模拟循环
#         pressure = np.random.rand(len(points))*4095
#         # 更新图像
#         update_dynamic_plot(points, pressure, valid_grid_points, grid_x, grid_z, ax, cax, mask, scatter, fig)
#         # 模拟主循环延时
#         time.sleep(0.04)