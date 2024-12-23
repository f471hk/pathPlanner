# pathPlanner
This project implements a tool to optimize sequential paths for 2D printing shapes in a binary image using the Traveling Salesman Problem (TSP). The solution processes connected components in an image, computes an optimized traversal path for each shape, and visualizes the result.

Requirements
	•	Python 3.8+
	•	OpenCV
	•	NumPy
	•	Matplotlib
	•	SciPy
	•	OR-Tools

How to Use
	1.	Place your grayscale input image in the inputimage/ directory.
	2.	Update the image_path variable in the script to point to your image.
	3.	Run the script to generate:
	•	An optimized path saved as a .txt file in the output/ directory.
	•	A scatter plot and color-coded image of the optimized path.

Example

Input image: bioirl.jpg
Output:
	•	output/optimized_path_sequential.txt: Text file with scaled coordinates.
	•	output/optimized_path_plot.png: Scatter plot of the traversal path.
	•	output/optimized_path_image.png: Color-coded image of the traversal path.
