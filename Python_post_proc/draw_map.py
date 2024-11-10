import numpy as np
import math
import cv2
import yaml
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Point


class drawMap():
    """
    A class to handle the drawing of a map based on sensor measurements and robot coordinates.

    Attributes:
        image (ndarray): The image of the map loaded from a file.
        origin_x (float): The x-coordinate of the origin of the map.
        origin_y (float): The y-coordinate of the origin of the map.
        resolution (float): The resolution of the map.
        map_height (int): The height of the map in pixels.
        measurements (dict): Sensor measurements and robot poses loaded from a YAML file.

    Methods:
        transform(x_bot, y_bot, angle, ss, bs, fs): Transforms robot's base coordinates and orientation into two points.
        process_measurements(): Processes measurements to compute lines and their pixel coordinates.
        remove_duplicates(lines, tolerance): Removes duplicate lines based on a given tolerance.
        draw_lines(filtered_points): Draws lines between given points.
        map(): Main mapping function that processes measurements and draws the map.
    """

    def __init__(self, map_img_file_path, map_yaml_file_path, measurements_yaml_file_path):

        self.image = cv2.imread(map_img_file_path, cv2.IMREAD_GRAYSCALE)

        with open(map_yaml_file_path, 'r') as file:
            map_metadata = yaml.safe_load(file)

        self.origin_x = map_metadata["origin"][0]
        self.origin_y = map_metadata["origin"][1]
        self.resolution = map_metadata["resolution"]
        self.map_height, _ = self.image.shape

        with open(measurements_yaml_file_path, 'r') as file1:
            self.measurements = yaml.safe_load(file1)

    def transform(self, x_bot, y_bot, angle, ss, bs, fs):
        """
        Transforms robot's base coordinates and orientation into two points based on sensor measurements.

        Returns:
        - tuple: Coordinates of two points (x1, y1, x2, y2).
        """
        x_robot = x_bot
        y_robot = y_bot
        theta_robot = np.radians(angle)

        x1 = round(x_robot + bs * math.cos(theta_robot) -
                   ss * math.sin(theta_robot), 2)
        y1 = round(y_robot + bs * math.sin(theta_robot) +
                   ss * math.cos(theta_robot), 2)

        x2 = round(x_robot + fs * math.cos(theta_robot) -
                   ss * math.sin(theta_robot), 2)
        y2 = round(y_robot + fs * math.sin(theta_robot) +
                   ss * math.cos(theta_robot), 2)

        return x1, y1, x2, y2


    def process_measurements(self):
            self.lines = []
            self.lengths = []

            for measure in self.measurements:
                if not measure:
                    continue
                fs = measure["x1"]
                bs = measure["x2"]
                ss = measure["y1"]
                x = measure["robot_pose"]["x"]
                y = measure["robot_pose"]["y"]
                angle = measure["robot_pose"]["z"]

                x1, y1, x2, y2 = self.transform(x, y, angle, ss, bs, fs)
                length = fs - bs

                # Ignore points out of map bounds
                if not self.is_within_bounds(x1, y1) or not self.is_within_bounds(x2, y2):
                    continue

                if all(not (math.isinf(c) or math.isnan(c)) for c in [x1, y1, x2, y2]):
                    self.lines.append(((x1, y1), (x2, y2)))
                    self.lengths.append(length)

            return self.lines, self.lengths

    def is_within_bounds(self, x, y):
        pixel_x = int((x - self.origin_x) / self.resolution)
        pixel_y = int((self.map_height - (y - self.origin_y) / self.resolution))
        return 0 <= pixel_x < self.image.shape[1] and 0 <= pixel_y < self.image.shape[0]

    def remove_duplicates(self, lines, tolerance=0.05):
        unique_lines = []
        for line in lines:
                line1 = LineString([line[0], line[1]])
                if not any(line1.almost_equals(LineString([l[0], l[1]]), decimal=3) for l in unique_lines):
                    unique_lines.append(line) 
        return unique_lines 

    def draw_lines(self, filtered_points):
            plt.figure(figsize=(12, 8))
            for ((x1, y1), (x2, y2)) in filtered_points:
                pixel_x1, pixel_y1 = int((x1 - self.origin_x) / self.resolution), int(self.map_height - (y1 - self.origin_y) / self.resolution)
                pixel_x2, pixel_y2 = int((x2 - self.origin_x) / self.resolution), int(self.map_height - (y2 - self.origin_y) / self.resolution)
                plt.plot([pixel_x1, pixel_x2], [pixel_y1, pixel_y2], 'k-', linewidth=1.5)
                plt.scatter([pixel_x1, pixel_x2], [pixel_y1, pixel_y2], color='b', s=30)

            plt.gca().set_aspect('equal', adjustable='box')
            plt.grid(True, linestyle='--', alpha=0.6)
            plt.title('Processed Map Lines')
            plt.xlabel('X (meters)')
            plt.ylabel('Y (meters)')
            plt.show()
 


    def map(self):
        """
        Main mapping function that processes measurements and draws the map.

        Returns:
        - list of lists of tuples: Final processed lines ready for display or further processing.
        """

        lines, _ = self.process_measurements()
        unique_lines = self.remove_duplicates(lines)
        self.draw_lines(unique_lines)

        # ------------------------------------------------------------------------------------------------
        # TO DO: Create your own methods and add them to this function for further processing and cleaning  
        # up the output as much as you can
        # ------------------------------------------------------------------------------------------------

        return unique_lines


def main():

    mapper = drawMap(map_img_file_path="SLAM/slam_map.pgm", 
                     map_yaml_file_path="SLAM/slam_map.yaml", 
                     measurements_yaml_file_path="measurements.yaml")
    
    lines = mapper.map()


if __name__ == "__main__":
    main()
