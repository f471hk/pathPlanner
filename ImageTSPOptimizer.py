import cv2
import numpy as np
from scipy.spatial.distance import pdist, squareform
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import matplotlib.pyplot as plt
from matplotlib import cm
import os

class ImageTSPOptimizer:
    def __init__(self, image_path, output_dir="output"):
        self.image_path = image_path
        self.output_dir = output_dir
        self.img = None
        self.binary = None
        self.labels = None
        self.num_labels = 0
        self.all_paths = []
        os.makedirs(self.output_dir, exist_ok=True)

    def load_image(self):
        self.img = cv2.imread(self.image_path, cv2.IMREAD_GRAYSCALE)
        if self.img is None:
            raise FileNotFoundError(f"Image at {self.image_path} could not be loaded.")
        print(f"Image {self.image_path} loaded successfully.")

    def threshold_image(self):
        _, self.binary = cv2.threshold(self.img, 128, 255, cv2.THRESH_BINARY)

    def identify_shapes(self):
        self.num_labels, self.labels = cv2.connectedComponents(self.binary)
        print(f"Number of connected components (shapes): {self.num_labels - 1}")

    def solve_tsp(self, coords):
        """Solve TSP for a given set of coordinates."""
        manager = pywrapcp.RoutingIndexManager(len(coords), 1, 0)
        routing = pywrapcp.RoutingModel(manager)

        # Compute pairwise distances
        distances = squareform(pdist(coords))

        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return int(distances[from_node, to_node])

        # Register the distance callback
        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Define search parameters
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
        search_parameters.time_limit.seconds = 60

        # Solve
        solution = routing.SolveWithParameters(search_parameters)
        if solution:
            index = routing.Start(0)
            route = []
            while not routing.IsEnd(index):
                route.append(manager.IndexToNode(index))
                index = solution.Value(routing.NextVar(index))
            return [coords[i] for i in route]
        else:
            print("No solution found for the current shape.")
            return None

    def process_shapes(self):
        for label in range(1, self.num_labels):  # Skip background (label 0)
            shape_coords = np.column_stack(np.where(self.labels == label))
            print(f"Processing shape {label} with {len(shape_coords)} points...")
            optimized_path = self.solve_tsp(shape_coords)
            if optimized_path:
                self.all_paths.extend(optimized_path)

    def save_optimized_path(self):
        output_file = f"{self.output_dir}/optimized_path_sequential.txt"
        with open(output_file, "w") as f:
            for point in self.all_paths:
                scaled_x = point[0] 
                scaled_y = point[1] 
                f.write(f"{scaled_x:.3f},{scaled_y:.3f}\n")  # Save with 3 decimal places

                #f.write(f"{point[0]},{point[1]}\n")
        print(f"Optimized sequential path saved to {output_file}")

    def visualize_optimized_path(self):
        x_coords = [point[1] for point in self.all_paths]
        y_coords = [point[0] for point in self.all_paths]

        plt.figure(figsize=(8, 8))
        plt.scatter(x_coords, y_coords, c=range(len(self.all_paths)), cmap='viridis', s=50)
        plt.colorbar(label='Order of Visit')
        plt.plot(x_coords, y_coords, linestyle='--', linewidth=1, alpha=0.7, color='gray')
        plt.gca().invert_yaxis()
        plt.title('Sequential Optimized Printing Path')
        plt.xlabel('X Coordinate')
        plt.ylabel('Y Coordinate')

        plot_file = f"{self.output_dir}/optimized_path_plot.png"
        plt.savefig(plot_file, dpi=300)
        print(f"Plot saved to {plot_file}")
        plt.show()

    def save_optimized_image(self):
        optimized_img = np.zeros((self.img.shape[0], self.img.shape[1], 3), dtype=np.uint8)

        color_map = cm.gist_rainbow
        num_points = len(self.all_paths)

        for idx, point in enumerate(self.all_paths):
            color = color_map(idx / num_points)
            color_rgb = (int(color[0] * 255), int(color[1] * 255), int(color[2] * 255))
            optimized_img[point[0], point[1]] = color_rgb[::-1]  # Convert RGB to BGR for OpenCV

        optimized_img[self.binary == 0] = (0, 0, 0)

        output_image_file = f"{self.output_dir}/optimized_path_image.png"
        cv2.imwrite(output_image_file, optimized_img)
        print(f"Optimized path image saved to {output_image_file}")

    def run(self):
        self.load_image()
        self.threshold_image()
        self.identify_shapes()
        self.process_shapes()
        self.save_optimized_path()
        self.save_optimized_image()


if __name__ == "__main__":
    image_path = "inputimage/bioirl.jpg"
    optimizer = ImageTSPOptimizer(image_path)
    optimizer.run()
